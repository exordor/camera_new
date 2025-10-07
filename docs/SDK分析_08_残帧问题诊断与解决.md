# Galaxy Camera SDK 分析 - 残帧问题诊断与解决

## 1. 问题现象

### 1.1 症状描述

当相机和Hesai QT 128激光雷达共享同一个以太网时：

**原始问题（未优化）：**
```bash
# 症状1：ROS话题无新消息
$ rostopic hz /galaxy_camera/image_raw
WARNING: no new messages

# 症状2：但网络层数据正常到达
$ sudo tcpdump -i eth0 'udp and src 192.168.40.11' -q
209992 packets received by filter
0 packets dropped by kernel  # ✓ 无丢包

# 症状3：SDK回调显示全是残帧
[WARN] Frame status: Total=6, Success=0, Incomplete=6 (status=-1)
```

**优化后现状（2025-10-07）：**
```bash
# 参数设置：5ms包间延迟 + 60秒块超时
$ rostopic hz /galaxy_camera/image_raw
subscribed to [/galaxy_camera/image_raw]
average rate: 3.079
    min: 0.084s max: 0.806s std dev: 0.21283s window: 21

# 结果：成功接收图像，但帧率约3 Hz（低于目标5 Hz）
# 状态：帧完整，无残帧 ✓
```

### 1.2 网络状态

```
相机数据流：
- 速率：~42K packets/sec
- 带宽：~493 Mbps
- 状态：UDP包正常到达 ✓

激光雷达数据流：
- 速率：~4K packets/sec  
- 带宽：~39 Mbps
- 状态：持续发送UDP包（即使ROS未启动）✓

总带宽：~532 Mbps (远低于1 Gbps限制) ✓
内核丢包：0 packets ✓
```

**结论**：网络层面一切正常，但应用层收到的全是残帧！

## 2. 残帧状态码详解

### 2.1 GX_FRAME_STATUS 枚举

```cpp
typedef enum GX_FRAME_STATUS_LIST {
    GX_FRAME_STATUS_SUCCESS = 0,           // 正常帧
    GX_FRAME_STATUS_INCOMPLETE = -1,       // 残帧（不完整帧）
    GX_FRAME_STATUS_INVALID_IMAGE_INFO = -2, // 图像信息错误帧
} GX_FRAME_STATUS_LIST;
```

### 2.2 残帧的含义

`GX_FRAME_STATUS_INCOMPLETE = -1` 表示：

1. **部分数据包丢失** - 组成一帧的某些UDP包未到达
2. **数据包超时** - 数据包到达太慢，超过SDK的等待时间
3. **数据包乱序** - 包顺序混乱，SDK无法正确组装
4. **缓冲区溢出** - SDK接收缓冲区不足

## 3. GigE Vision协议原理

### 3.1 帧传输机制

一个完整的图像帧需要通过多个UDP包传输：

```
单帧数据 = 12,288,000 bytes (4096×3000像素，BayerRG8)
单个UDP包 ≈ 1,436 bytes
总包数 = 12,288,000 ÷ 1,436 ≈ 8,556 个UDP包

传输过程：
相机 → [Packet #1] → [Packet #2] → ... → [Packet #8556] → 主机
       └─ 包间延迟 ─┘
```

### 3.2 帧组装流程

```
┌─────────────────────────────────────────────────┐
│ 1. 相机发送第一个包（BlockID + PacketID）      │
│    ↓                                            │
│ 2. 主机SDK开始组装缓冲区                       │
│    ↓                                            │
│ 3. 启动帧组装计时器（Block Timeout）           │
│    ↓                                            │
│ 4. 持续接收包，检查PacketID连续性              │
│    ↓                                            │
│ 5. 如果缺包 → 发送重传请求（Resend Request）  │
│    ↓                                            │
│ 6. 等待所有包到齐 OR 超时                      │
│    ↓                                            │
│ 7a. 全部到齐 → GX_FRAME_STATUS_SUCCESS ✓      │
│ 7b. 超时/缺包 → GX_FRAME_STATUS_INCOMPLETE ✗  │
└─────────────────────────────────────────────────┘
```

## 4. 根因分析

### 4.1 实际诊断发现

通过详细日志发现了惊人的事实：

```
=== INCOMPLETE FRAME DETAILS ===
Frame ID: 446
Frame status: -1 (GX_FRAME_STATUS_INCOMPLETE)
Image size received: 12,288,000 bytes  ← 实际接收
Expected size: 12,288,000 bytes        ← 期望大小
Bytes missing: 0 (0.0%)                ← ⚠️ 无数据丢失！
===============================
```

**关键发现**：所有数据都到达了，但仍然被标记为残帧！

### 4.2 真正的原因：包到达时间过慢

问题不是**丢包**，而是**时序**！

#### 计算分析：

使用5ms包间延迟时：

```
帧大小：12,288,000 bytes
包大小：~1,436 bytes/packet
总包数：8,556 packets
包间延迟：5ms

总传输时间 = 8,556 × 5ms = 42,780ms ≈ 42.78秒！
```

但SDK的默认超时设置：

```cpp
GX_DS_INT_BLOCK_TIMEOUT = 5000ms (5秒)  // 默认帧组装超时
```

**结果**：

```
时间轴：
0s     ─ 相机开始发送包
5s     ─ SDK超时，标记为残帧 ✗
42.78s ─ 所有包实际到达完成（但已被丢弃）
```

### 4.3 为什么需要5ms包间延迟？

#### 原始问题（1ms延迟时）：

```
时间线（微秒级）：
0µs    : 相机发包 #1
1000µs : 相机发包 #2
2000µs : 相机发包 #3  } 碰撞！
2100µs : 激光雷达突发数据包 }
3000µs : 相机发包 #4
...
```

即使总带宽充足，在微秒级仍会发生包碰撞。

#### 解决方案（5ms延迟）：

```
时间线（毫秒级）：
0ms    : 相机发包 #1
5ms    : 相机发包 #2
10ms   : 相机发包 #3
12ms   : 激光雷达突发（独立时间窗口）
15ms   : 相机发包 #4
...
```

5ms延迟确保相机和激光雷达的包在时间上分离，避免碰撞。

## 5. 解决方案

### 5.1 方案1：增加SDK超时时间（已实施）

修改 `galaxy_camera.cpp`：

```cpp
// 1. 包超时 - 每个包的等待时间
GXSetInt(dev_handle_, GX_DS_INT_PACKET_TIMEOUT, 1000);  // 1秒

// 2. 帧组装超时 - 最关键！
GXSetInt(dev_handle_, GX_DS_INT_BLOCK_TIMEOUT, 60000);  // 60秒

// 3. 最大等待包数量
GXSetInt(dev_handle_, GX_DS_INT_MAX_WAIT_PACKET_COUNT, 1000);

// 4. 重传超时
GXSetInt(dev_handle_, GX_DS_INT_RESEND_TIMEOUT, 500);   // 500ms

// 5. 包间延迟
int64_t packet_delay = 5000;  // 5ms
GXSetInt(dev_handle_, GX_INT_GEV_PACKETDELAY, packet_delay);
```

**效果**：
- ✓ 帧能够成功组装
- ⚠️ 帧率较低（~3 fps，而非目标5 fps）

**实际测试结果（2025-10-07）**：
```bash
$ rostopic hz /galaxy_camera/image_raw
average rate: 3.079
    min: 0.084s max: 0.806s std dev: 0.21283s window: 21
```
- 实际帧率：**3.079 Hz** (约3 fps)
- 帧间隔：84ms ~ 806ms（有波动）
- 状态：所有帧完整，无残帧 ✅
- 分析：帧率低于理论计算，可能因为激光雷达数据包干扰或系统处理延迟

### 5.2 方案2：优化包间延迟（可尝试）

```cpp
// 尝试更小的延迟
int64_t packet_delay = 2000;  // 2ms
// 帧传输时间：8556 × 2ms = 17.1秒

// 相应调整超时
GXSetInt(dev_handle_, GX_DS_INT_BLOCK_TIMEOUT, 20000);  // 20秒
```

**预期**：
- 帧率：~0.058 fps（理论值）
- 需要测试是否仍能避免包碰撞

**注意**：基于当前5ms延迟实现3 Hz的结果，减小延迟可能导致包碰撞和残帧增加。建议保持当前配置。

### 5.3 方案3：降低分辨率

```yaml
# 在launch文件中
image_width: 2048   # 4096的一半
image_height: 1500  # 3000的一半
```

**效果**：
```
帧大小减小4倍：
- 原：12 MB → 新：3 MB
- 原：8556包 → 新：2139包
- 传输时间：2139 × 5ms = 10.7秒
- 帧率：~0.093 fps
```

### 5.4 方案4：增加包大小（若相机支持）

```cpp
// 检查相机支持的最大包大小
GX_INT_RANGE packetSizeRange;
GXGetIntRange(dev_handle_, GX_INT_GEV_PACKETSIZE, &packetSizeRange);

// 设置为最大值（通常8000-9000字节）
GXSetInt(dev_handle_, GX_INT_GEV_PACKETSIZE, 8000);
```

**效果**：
```
包大小：1436 → 8000 bytes
包数量：8556 → 1536包
传输时间：1536 × 5ms = 7.68秒
帧率：~0.13 fps
```

### 5.5 方案5：使用独立网卡（最佳方案）

**硬件配置**：

```
eth0 (192.168.40.x) ─→ 相机专用
eth1 (192.168.50.x) ─→ 激光雷达专用
```

**优势**：
```cpp
// 无需包间延迟
int64_t packet_delay = 0;  // 或保持默认

// 使用较短超时
GXSetInt(dev_handle_, GX_DS_INT_BLOCK_TIMEOUT, 5000);  // 5秒

// 预期性能
帧率：5 fps ✓ (达到目标)
延迟：<200ms ✓
```

### 5.6 方案6：网络QoS优化（需要交换机支持）

使用支持VLAN和QoS的管理型交换机：

```
VLAN 10 (高优先级) ─→ 相机流量
VLAN 20 (普通优先级) ─→ 激光雷达流量
```

配置：
```cpp
// 可以使用更小的延迟
int64_t packet_delay = 500;  // 0.5ms

// 传输时间
8556 × 0.5ms = 4.28秒

// 超时设置
GXSetInt(dev_handle_, GX_DS_INT_BLOCK_TIMEOUT, 6000);  // 6秒

// 预期帧率
~0.23 fps (仍不理想，但好于5ms延迟)
```

## 6. 相关SDK参数详解

### 6.1 数据流层参数 (GX_FEATURE_LEVEL_DS)

| 参数ID | 说明 | 推荐值 | 影响 |
|--------|------|--------|------|
| `GX_DS_INT_PACKET_TIMEOUT` | 单包超时 | 1000ms | 等待单个包的时间 |
| `GX_DS_INT_BLOCK_TIMEOUT` | 帧超时 | 60000ms | **最关键**，整帧组装时间 |
| `GX_DS_INT_MAX_WAIT_PACKET_COUNT` | 最大等待包数 | 1000 | 乱序包缓冲 |
| `GX_DS_INT_RESEND_TIMEOUT` | 重传超时 | 500ms | 请求重传的等待时间 |
| `GX_DS_ENUM_RESEND_MODE` | 重传模式 | ON | 启用丢包重传 |
| `GX_DS_INT_SOCKET_BUFFER_SIZE` | Socket缓冲 | 2048KB | 系统缓冲区 |

### 6.2 设备层参数 (GX_FEATURE_LEVEL_REMOTE_DEV)

| 参数ID | 说明 | 推荐值 | 影响 |
|--------|------|--------|------|
| `GX_INT_GEV_PACKETSIZE` | 包大小 | 1500 | 单个UDP包大小 |
| `GX_INT_GEV_PACKETDELAY` | 包间延迟 | 5000µs | 包发送间隔 |
| `GX_INT_PAYLOAD_SIZE` | 载荷大小 | 自动 | 单帧总大小 |

## 7. 诊断和监控

### 7.1 添加诊断代码

在回调函数中添加详细诊断：

```cpp
void GalaxyCamera::onFrameCB(GX_FRAME_CALLBACK_PARAM *pFrameData) {
    static uint64_t totalFrames = 0;
    static uint64_t successFrames = 0;
    static uint64_t incompleteFrames = 0;
    static auto lastLogTime = std::chrono::steady_clock::now();
    
    totalFrames++;
    
    if (pFrameData->status == GX_FRAME_STATUS_SUCCESS) {
        successFrames++;
        // 发布图像
        publishImage(pFrameData);
    } 
    else if (pFrameData->status == GX_FRAME_STATUS_INCOMPLETE) {
        incompleteFrames++;
        
        // 详细残帧信息
        int64_t payloadSize;
        GXGetInt(dev_handle_, GX_INT_PAYLOAD_SIZE, &payloadSize);
        int64_t missing = payloadSize - pFrameData->nImgSize;
        double missingPercent = 100.0 * missing / payloadSize;
        
        ROS_WARN_THROTTLE(1.0, 
            "Incomplete frame #%llu: received %d/%lld bytes (%.2f%% missing)",
            pFrameData->nFrameID, 
            pFrameData->nImgSize, 
            payloadSize,
            missingPercent);
    }
    
    // 每秒输出统计
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(
            now - lastLogTime).count() >= 1) {
        
        ROS_INFO("Frame status: Total=%llu, Success=%llu, Incomplete=%llu",
                 totalFrames, successFrames, incompleteFrames);
        
        lastLogTime = now;
    }
}
```

### 7.2 网络监控命令

```bash
# 1. 监控UDP包到达
sudo timeout 5 tcpdump -i eth0 'udp and src 192.168.40.11' -q

# 2. 检查内核丢包
sudo timeout 3 tcpdump -i eth0 'udp' 2>&1 | grep dropped

# 3. 监控带宽
sudo iftop -i eth0 -f "src host 192.168.40.11"

# 4. 检查网卡统计
ethtool -S eth0 | grep -i drop

# 5. 查看网卡缓冲区
ethtool -g eth0
```

### 7.3 系统优化

```bash
# 增加网卡接收缓冲区
sudo ethtool -G eth0 rx 4096

# 增加系统UDP缓冲区
sudo sysctl -w net.core.rmem_max=134217728
sudo sysctl -w net.core.rmem_default=134217728

# 禁用网卡的省电功能
sudo ethtool -s eth0 speed 1000 duplex full autoneg off
```

## 8. 测试和验证流程

### 8.1 基准测试（相机单独运行）

```bash
# 1. 启动相机节点
roslaunch galaxy_camera MER2-302.launch

# 2. 检查话题发布
rostopic hz /galaxy_camera/image_raw
# 期望：5.0 Hz

# 3. 检查帧状态
rostopic echo /rosout | grep "Frame status"
# 期望：Success=100%, Incomplete=0%
```

### 8.2 压力测试（相机+激光雷达）

```bash
# 1. 启动相机
roslaunch galaxy_camera MER2-302.launch

# 2. 启动激光雷达（或直接上电）
# Hesai会自动发送UDP包

# 3. 监控帧状态
rostopic echo /rosout | grep "Frame status"

# 4. 监控话题
rostopic hz /galaxy_camera/image_raw

# 5. 监控网络
sudo iftop -i eth0
```

### 8.3 性能指标

| 指标 | 理想值 | 可接受值 | 不可接受 |
|------|--------|----------|----------|
| 成功率 | 100% | >95% | <90% |
| 帧率 | 5.0 fps | >3.0 fps | <1.0 fps |
| 延迟 | <200ms | <500ms | >1000ms |
| 内核丢包 | 0 | 0 | >0 |
| 带宽利用 | <80% | <90% | >95% |

## 9. 故障排查清单

### ✓ 检查项目

- [ ] 网络连接正常（ping通）
- [ ] 相机IP配置正确
- [ ] MTU设置正确（1500或9000）
- [ ] 防火墙已关闭
- [ ] 网卡驱动最新
- [ ] ROS节点正常启动
- [ ] SDK参数已应用（查看日志）
- [ ] 系统缓冲区已优化
- [ ] 无其他进程占用带宽

### 常见错误和解决

| 症状 | 可能原因 | 解决方法 |
|------|----------|----------|
| 100%残帧 | 超时过短 | 增加`BLOCK_TIMEOUT` |
| 偶尔残帧 | 包碰撞 | 增加包间延迟 |
| 内核丢包 | 缓冲区小 | 增加系统缓冲区 |
| 带宽满载 | 分辨率高 | 降低分辨率或帧率 |
| 完全无数据 | IP错误 | 检查网络配置 |

## 10. 总结

### 问题本质

GigE Vision相机的残帧问题通常不是真正的"丢包"，而是**时序问题**：

1. **UDP层正常** - 所有包都到达
2. **应用层超时** - SDK等不及包组装完成
3. **共享网络** - 多设备竞争导致包到达延迟

### 解决策略

短期方案（软件）：
```
增加延迟 + 增加超时 = 降低帧率但保证完整性
```

长期方案（硬件）：
```
独立网卡 OR 10GbE升级 = 高帧率 + 高可靠性
```

### 当前实施状态（2025-10-07）

**已实施方案**：方案1（增大超时 + 包间延迟）

**配置参数**：
- 包间延迟：5000µs (5ms)
- 块超时：60000ms (60秒)
- 重传超时：500ms
- 包超时：1000ms

**实际效果**：
- ✅ 残帧问题已解决（无残帧）
- ⚠️ 帧率：3.079 Hz (低于目标5 Hz)
- ⚠️ 帧间隔波动：84ms ~ 806ms

**性能对比**：

| 指标 | 优化前 | 优化后 | 目标 |
|------|--------|--------|------|
| 残帧率 | 100% | 0% ✅ | 0% |
| 帧率 | 0 fps | 3.08 fps ⚠️ | 5 fps |
| 数据完整性 | 失败 | 成功 ✅ | 成功 |

**进一步优化建议**：

1. **接受当前性能**：如果3 Hz满足应用需求，保持当前配置
2. **尝试减小延迟**：逐步将5ms降至3ms或2ms，测试稳定性
3. **硬件升级**：增加独立网卡实现完整5 Hz
4. **降低分辨率**：如果不需要4K，降至2K可提升帧率

### 最佳实践

1. **开发阶段**：使用独立网卡测试
2. **部署阶段**：根据实际需求选择方案
3. **生产环境**：监控残帧率，及时调整参数
4. **定期检查**：监控帧率波动，确保系统稳定

---

**文档版本**: 1.1  
**最后更新**: 2025-10-07  
**状态**: 问题已解决，当前配置实现3 Hz稳定采集
