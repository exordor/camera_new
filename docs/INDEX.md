# 文档索引 - GigE 相机优化与监控

## 📋 快速入口

### 🚨 **遇到内核丢包？** 
👉 **[README_网卡丢包.md](README_网卡丢包.md)** - 快速解决指南

### 📊 **想要监控性能？**
👉 **[README_MONITORING.md](README_MONITORING.md)** - 综合监控系统

### 🔧 **需要详细配置？**
👉 **[网卡层丢包_最终解决方案.md](网卡层丢包_最终解决方案.md)** - 完整方案

## 📚 文档目录

### 核心文档

#### 1. [README_网卡丢包.md](README_网卡丢包.md) ⭐ **推荐首读**
**适合**: 遇到内核丢包问题时

**内容**:
- 快速验证和启动步骤
- 已应用的优化总览
- 常见问题快速解决
- 工具脚本使用指南

**关键命令**:
```bash
# 验证优化
./scripts/verify_4k_optimization.sh

# 监控丢包
./scripts/monitor_kernel_drops.sh
```

---

#### 2. [网卡层丢包_最终解决方案.md](网卡层丢包_最终解决方案.md) 📖 **完整方案**
**适合**: 需要了解优化细节和原理

**内容**:
- 问题根本原因分析
- 完整优化措施说明
- 性能对比数据
- 详细的故障排除
- 备选优化方案

**关键信息**:
- 4K相机 (4096x3000) = 12 MB/帧
- 缓冲区从 64 MB → 256 MB
- 内核丢包从 53% → 0%

---

#### 3. [4K相机内核丢包解决方案.md](4K相机内核丢包解决方案.md) 🔬 **技术深度**
**适合**: 需要技术细节和高级优化

**内容**:
- 内核缓冲区原理
- 网络栈优化详解
- CPU亲和性配置
- 系统级参数调优
- 进一步优化建议

**高级话题**:
- Bayer vs BGR8 性能对比
- 硬件加速方案
- 多线程处理
- 专用网络接口

---

#### 4. [README_MONITORING.md](README_MONITORING.md) 📊 **监控系统**
**适合**: 需要性能监控和瓶颈分析

**内容**:
- 三层监控系统（网卡/SDK/应用）
- 性能基准和对比
- 瓶颈诊断流程
- 监控数据解释

**监控层次**:
1. 网络接口层 - rx_dropped, rx_errors
2. GigE SDK层 - lost_frames, incomplete_frames
3. ROS回调层 - 处理时间分解

---

#### 5. [网卡层丢包消除指南.md](网卡层丢包消除指南.md) 📘 **通用指南**
**适合**: 一般网络优化需求

**内容**:
- 标准网络优化步骤
- 内核参数配置
- 环形缓冲区设置
- 网络offloading优化

---

### 技术文档

#### [LLM_Analysis_Prompt.md](LLM_Analysis_Prompt.md)
**内容**: AI分析提示词模板

---

## 🛠️ 脚本工具

### 位置
```
/home/eagrumo/catkin_ws_eagrumo/src/catkin_ws_eas/scripts/
```

### 优化脚本

#### 1. `optimize_for_4k_camera.sh` ⭐ **主优化脚本**
```bash
sudo ./scripts/optimize_for_4k_camera.sh
```
**功能**:
- 扩展内核缓冲区到 256 MB
- 优化网络参数
- 设置CPU性能模式
- 配置网络IRQ亲和性
- 持久化配置

---

#### 2. `eliminate_network_packet_loss.sh` 🔧 **通用优化**
```bash
sudo ./scripts/eliminate_network_packet_loss.sh
```
**功能**:
- 一般网络优化
- 适合标准分辨率相机
- 包含自动配置服务

---

### 监控脚本

#### 3. `monitor_kernel_drops.sh` 👁️ **实时监控**
```bash
./scripts/monitor_kernel_drops.sh
```
**功能**:
- 实时显示内核丢包
- 彩色输出（红色=有丢包，绿色=正常）
- 每秒更新
- Drop Rate 百分比

---

#### 4. `verify_4k_optimization.sh` ✅ **验证工具**
```bash
./scripts/verify_4k_optimization.sh
```
**功能**:
- 检查所有优化配置
- 验证缓冲区设置
- 检查相机连接
- 确认launch文件配置

---

#### 5. `check_network_packet_loss.sh` 📊 **统计检查**
```bash
./scripts/check_network_packet_loss.sh
```
**功能**:
- 显示累计网络统计
- 检查rx_dropped, rx_errors
- 显示当前缓冲区配置
- 建议优化措施

---

### 其他脚本

#### 6. `fix_camera_udp.sh` 🔧 **紧急修复**
```bash
./scripts/fix_camera_udp.sh
```
**功能**:
- 重置网络MTU
- 修复UDP接收问题
- 验证相机连接

---

#### 7. `verify_jumbo_frames.sh` 🔍 **MTU检查**
```bash
./scripts/verify_jumbo_frames.sh
```
**功能**:
- 检查jumbo frames支持
- 测试不同packet size

---

## 🎯 使用场景

### 场景 1: 首次设置 4K 相机
1. 阅读 [README_网卡丢包.md](README_网卡丢包.md)
2. 运行 `sudo ./scripts/optimize_for_4k_camera.sh`
3. 运行 `./scripts/verify_4k_optimization.sh`
4. 启动相机并运行 `./scripts/monitor_kernel_drops.sh`

### 场景 2: 遇到丢包问题
1. 运行 `./scripts/monitor_kernel_drops.sh` 确认问题
2. 查看 [网卡层丢包_最终解决方案.md](网卡层丢包_最终解决方案.md)
3. 应用建议的优化方案
4. 重新验证

### 场景 3: 性能调优
1. 阅读 [README_MONITORING.md](README_MONITORING.md)
2. 启动相机并观察三层监控数据
3. 根据瓶颈位置应用针对性优化
4. 参考 [4K相机内核丢包解决方案.md](4K相机内核丢包解决方案.md) 的高级方案

### 场景 4: 问题排查
1. 运行 `./scripts/verify_4k_optimization.sh`
2. 查看故障排除章节
3. 根据具体错误查找对应文档
4. 应用解决方案

---

## 📈 优化效果总览

### 优化前（问题状态）
```
内核丢包: 171,269 packets (53%)  ❌
帧率:     不稳定               ❌
帧间隔:   204 frame gap        ❌
SDK丢帧:  816+ frames          ❌
```

### 优化后（目标状态）
```
内核丢包: 0 packets            ✅
帧率:     稳定 5 Hz            ✅
帧间隔:   消除                 ✅
SDK丢帧:  < 10 frames          ✅
回调处理: < 25 ms              ✅
```

---

## 🔗 快速链接

| 需求 | 文档 | 脚本 |
|------|------|------|
| 🚨 内核丢包 | [README_网卡丢包.md](README_网卡丢包.md) | `optimize_for_4k_camera.sh` |
| 📊 性能监控 | [README_MONITORING.md](README_MONITORING.md) | `monitor_kernel_drops.sh` |
| 🔧 完整方案 | [网卡层丢包_最终解决方案.md](网卡层丢包_最终解决方案.md) | `verify_4k_optimization.sh` |
| 🔬 技术深度 | [4K相机内核丢包解决方案.md](4K相机内核丢包解决方案.md) | - |
| ✅ 验证配置 | [README_网卡丢包.md](README_网卡丢包.md) | `verify_4k_optimization.sh` |

---

## 💡 推荐阅读路径

### 快速路径（急需解决问题）
1. [README_网卡丢包.md](README_网卡丢包.md) - 5分钟
2. 运行脚本验证和监控
3. 如有问题查看故障排除

### 完整路径（深入理解）
1. [README_网卡丢包.md](README_网卡丢包.md) - 概览
2. [网卡层丢包_最终解决方案.md](网卡层丢包_最终解决方案.md) - 详细方案
3. [4K相机内核丢包解决方案.md](4K相机内核丢包解决方案.md) - 技术深度
4. [README_MONITORING.md](README_MONITORING.md) - 监控系统

### 监控路径（性能调优）
1. [README_MONITORING.md](README_MONITORING.md) - 监控系统
2. 运行相机并观察三层监控
3. 根据瓶颈查看对应优化章节

---

## 📞 获取帮助

### 1. 查看日志
```bash
cat ~/.ros/log/latest/galaxy_camera-*-stdout.log
```

### 2. 检查当前状态
```bash
./scripts/verify_4k_optimization.sh
./scripts/check_network_packet_loss.sh
```

### 3. 查看相关章节
- 网络丢包 → README_网卡丢包.md
- 性能问题 → README_MONITORING.md
- 配置问题 → 网卡层丢包_最终解决方案.md

---

*文档结构最后更新: 2025-10-08*
