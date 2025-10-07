# Galaxy Camera SDK 分析 - 图像采集接口

## 1. 采集控制接口

### 1.1 GXStreamOn - 开始采集

```cpp
GX_STATUS GXStreamOn(GX_DEV_HANDLE hDevice);
```

**功能**: 开始图像采集，包括流采集和设备采集

**返回值**:
- `GX_STATUS_SUCCESS`: 操作成功
- `GX_STATUS_NOT_INIT_API`: 未初始化
- `GX_STATUS_INVALID_HANDLE`: 句柄无效
- `GX_STATUS_INVALID_ACCESS`: 设备访问模式错误

**使用说明**: 
- 必须在配置好相机参数后调用
- 调用后才能获取图像

**示例**:
```cpp
GX_STATUS status = GXStreamOn(hDevice);
if (status != GX_STATUS_SUCCESS) {
    printf("开始采集失败\n");
}
```

### 1.2 GXStreamOff - 停止采集

```cpp
GX_STATUS GXStreamOff(GX_DEV_HANDLE hDevice);
```

**功能**: 停止图像采集，包括停止流采集和设备采集

**返回值**:
- `GX_STATUS_SUCCESS`: 操作成功
- `GX_STATUS_NOT_INIT_API`: 未初始化
- `GX_STATUS_INVALID_HANDLE`: 句柄无效
- `GX_STATUS_INVALID_ACCESS`: 设备访问模式错误
- `GX_STATUS_INVALID_CALL`: 未开始采集或已注册回调

**示例**:
```cpp
GXStreamOff(hDevice);
```

### 1.3 GXFlushQueue - 清空图像队列

```cpp
GX_STATUS GXFlushQueue(GX_DEV_HANDLE hDevice);
```

**功能**: 清空图像输出队列中的缓存图像

**使用场景**: 
- 触发模式下，发送触发信号前清空旧图像
- 避免获取到上一次采集的残留图像

**示例**:
```cpp
// 清空队列
GXFlushQueue(hDevice);
// 发送触发信号
GXSendCommand(hDevice, GX_COMMAND_TRIGGER_SOFTWARE);
// 获取新图像
GXGetImage(hDevice, &frameData, 1000);
```

## 2. 缓冲区管理

### 2.1 GXSetAcqusitionBufferNumber - 设置缓冲区数量

```cpp
GX_STATUS GXSetAcqusitionBufferNumber(
    GX_DEV_HANDLE hDevice,      // [in] 设备句柄
    uint64_t nBufferNum         // [in] 缓冲区数量
);
```

**功能**: 设置采集缓冲区的数量

**参数**: 
- `nBufferNum`: 缓冲区数量，建议值3-10

**使用说明**:
- 缓冲区越多，丢帧概率越低
- 但会占用更多内存
- 建议在GXStreamOn前设置

**示例**:
```cpp
// 设置5个缓冲区
GXSetAcqusitionBufferNumber(hDevice, 5);
```

### 2.2 GXGetAcqusitionBufferNumber - 获取缓冲区数量

```cpp
GX_STATUS GXGetAcqusitionBufferNumber(
    GX_DEV_HANDLE hDevice,      // [in] 设备句柄
    uint64_t *pBufferNum        // [out] 返回缓冲区数量
);
```

**功能**: 获取当前采集缓冲区的数量

**示例**:
```cpp
uint64_t bufferNum;
GXGetAcqusitionBufferNumber(hDevice, &bufferNum);
printf("当前缓冲区数量: %llu\n", bufferNum);
```

## 3. 采集模式

SDK提供三种图像采集模式：

### 模式1: 同步采集模式 (GXGetImage)

**特点**:
- 主动拉取图像
- 阻塞等待
- 有内存拷贝，性能稍低
- 使用简单

**适用场景**: 低速采集、简单应用

### 模式2: 异步回调模式 (GXRegisterCaptureCallback)

**特点**:
- 被动接收图像
- 回调函数触发
- 无内存拷贝，性能高
- 需要注意线程安全

**适用场景**: 高速采集、实时处理

### 模式3: 零拷贝模式 (GXDQBuf/GXQBuf)

**特点**:
- 零拷贝机制
- 性能最高
- 需要手动管理缓冲区
- 使用复杂

**适用场景**: 超高速采集、性能要求极高

## 4. 同步采集接口 (模式1)

### 4.1 GXGetImage - 获取图像

```cpp
GX_STATUS GXGetImage(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FRAME_DATA *pFrameData,      // [in,out] 图像数据结构
    uint32_t nTimeout               // [in] 超时时间(毫秒)
);
```

**功能**: 获取一帧图像（阻塞等待）

**参数**:
- `pFrameData`: 图像数据结构，需提前分配pImgBuf
- `nTimeout`: 超时时间（毫秒），INFINITE表示无限等待

**返回值**:
- `GX_STATUS_SUCCESS`: 获取成功
- `GX_STATUS_TIMEOUT`: 超时
- `GX_STATUS_INVALID_CALL`: 已注册回调，不允许调用

**注意事项**:
- 不能与回调模式混用
- 需要用户分配和管理图像缓冲区
- 有内存拷贝开销

**完整示例**:
```cpp
// 1. 分配图像缓冲区
GX_FRAME_DATA frameData;
memset(&frameData, 0, sizeof(frameData));

int64_t payloadSize;
GXGetInt(hDevice, GX_INT_PAYLOAD_SIZE, &payloadSize);
frameData.pImgBuf = malloc(payloadSize);

// 2. 开始采集
GXStreamOn(hDevice);

// 3. 循环获取图像
for (int i = 0; i < 100; i++) {
    GX_STATUS status = GXGetImage(hDevice, &frameData, 1000);
    
    if (status == GX_STATUS_SUCCESS) {
        if (frameData.nStatus == GX_FRAME_STATUS_SUCCESS) {
            printf("获取第%d帧，大小: %d字节\n", 
                i, frameData.nImgSize);
            
            // 处理图像数据 frameData.pImgBuf
            // ...
        } else {
            printf("帧状态错误: %d\n", frameData.nStatus);
        }
    } else if (status == GX_STATUS_TIMEOUT) {
        printf("获取图像超时\n");
    }
}

// 4. 停止采集
GXStreamOff(hDevice);

// 5. 释放缓冲区
free(frameData.pImgBuf);
```

## 5. 异步回调接口 (模式2)

### 5.1 GXRegisterCaptureCallback - 注册回调

```cpp
GX_STATUS GXRegisterCaptureCallback(
    GX_DEV_HANDLE hDevice,              // [in] 设备句柄
    void *pUserParam,                   // [in] 用户私有参数
    GXCaptureCallBack callBackFun       // [in] 回调函数
);
```

**功能**: 注册图像采集回调函数

**参数**:
- `pUserParam`: 用户私有数据，将传递给回调函数
- `callBackFun`: 回调函数指针

**回调函数原型**:
```cpp
void MyCallback(GX_FRAME_CALLBACK_PARAM *pFrameData);
```

**使用说明**:
- 必须在GXStreamOn之前注册
- 回调函数中不要执行耗时操作
- 注意线程安全

**完整示例**:
```cpp
// 1. 定义回调函数
void OnImageCallback(GX_FRAME_CALLBACK_PARAM *pFrameData) {
    if (pFrameData->status == GX_FRAME_STATUS_SUCCESS) {
        printf("收到图像: %dx%d, 帧号: %llu\n",
            pFrameData->nWidth,
            pFrameData->nHeight,
            pFrameData->nFrameID);
        
        // 处理图像数据 pFrameData->pImgBuf
        // 注意：不要在回调中执行耗时操作
        // 建议：拷贝数据到队列，由其他线程处理
    }
}

// 2. 注册回调
GXRegisterCaptureCallback(hDevice, NULL, OnImageCallback);

// 3. 开始采集
GXStreamOn(hDevice);

// 4. 等待采集（主线程可以做其他事情）
sleep(10);  // 采集10秒

// 5. 停止采集
GXStreamOff(hDevice);

// 6. 注销回调
GXUnregisterCaptureCallback(hDevice);
```

### 5.2 GXUnregisterCaptureCallback - 注销回调

```cpp
GX_STATUS GXUnregisterCaptureCallback(GX_DEV_HANDLE hDevice);
```

**功能**: 注销图像采集回调函数

**使用说明**: 必须在GXStreamOff之后调用

**示例**:
```cpp
GXStreamOff(hDevice);
GXUnregisterCaptureCallback(hDevice);
```

### 回调函数最佳实践

```cpp
// 使用队列传递图像数据
#include <queue>
#include <mutex>

std::queue<cv::Mat> imageQueue;
std::mutex queueMutex;

void OnImageCallback(GX_FRAME_CALLBACK_PARAM *pFrameData) {
    if (pFrameData->status == GX_FRAME_STATUS_SUCCESS) {
        // 1. 快速拷贝图像数据
        cv::Mat image(pFrameData->nHeight, 
                     pFrameData->nWidth,
                     CV_8UC1, 
                     (void*)pFrameData->pImgBuf);
        
        // 2. 深拷贝并加入队列
        std::lock_guard<std::mutex> lock(queueMutex);
        imageQueue.push(image.clone());
    }
}

// 处理线程
void ProcessThread() {
    while (running) {
        cv::Mat image;
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            if (!imageQueue.empty()) {
                image = imageQueue.front();
                imageQueue.pop();
            }
        }
        
        if (!image.empty()) {
            // 耗时的图像处理
            processImage(image);
        }
    }
}
```

## 6. 零拷贝接口 (模式3)

### 6.1 GXDQBuf - 取出缓冲区

```cpp
GX_STATUS GXDQBuf(
    GX_DEV_HANDLE hDevice,              // [in] 设备句柄
    PGX_FRAME_BUFFER *ppFrameBuffer,    // [out] 图像缓冲区指针
    uint32_t nTimeOut                   // [in] 超时时间(毫秒)
);
```

**功能**: 从队列中取出图像缓冲区（零拷贝）

**参数**:
- `ppFrameBuffer`: 返回图像缓冲区指针
- `nTimeOut`: 超时时间（毫秒）

**返回值**:
- `GX_STATUS_SUCCESS`: 成功
- `GX_STATUS_TIMEOUT`: 超时

**注意**: 使用完必须调用GXQBuf归还缓冲区

### 6.2 GXQBuf - 归还缓冲区

```cpp
GX_STATUS GXQBuf(
    GX_DEV_HANDLE hDevice,              // [in] 设备句柄
    PGX_FRAME_BUFFER pFrameBuffer       // [in] 图像缓冲区指针
);
```

**功能**: 将图像缓冲区归还到队列

**使用说明**: 每次GXDQBuf后必须调用GXQBuf

### 6.3 GXDQAllBufs - 取出所有缓冲区

```cpp
GX_STATUS GXDQAllBufs(
    GX_DEV_HANDLE hDevice,                  // [in] 设备句柄
    PGX_FRAME_BUFFER *ppFrameBufferArray,   // [out] 缓冲区数组
    uint32_t nFrameBufferArraySize,         // [in] 数组大小
    uint32_t *pnFrameCount,                 // [out] 实际帧数
    uint32_t nTimeOut                       // [in] 超时时间
);
```

**功能**: 一次性取出所有可用的图像缓冲区

**使用说明**: 
- 按时间顺序排列，[0]是最旧的，[n-1]是最新的
- 使用完必须调用GXQAllBufs归还

### 6.4 GXQAllBufs - 归还所有缓冲区

```cpp
GX_STATUS GXQAllBufs(GX_DEV_HANDLE hDevice);
```

**功能**: 将所有取出的图像缓冲区归还

### 零拷贝完整示例

```cpp
// 1. 设置缓冲区数量
GXSetAcqusitionBufferNumber(hDevice, 5);

// 2. 开始采集
GXStreamOn(hDevice);

// 3. 循环获取图像
for (int i = 0; i < 100; i++) {
    PGX_FRAME_BUFFER pFrameBuffer = NULL;
    
    // 取出缓冲区
    GX_STATUS status = GXDQBuf(hDevice, &pFrameBuffer, 1000);
    
    if (status == GX_STATUS_SUCCESS) {
        if (pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS) {
            printf("获取第%d帧，缓冲区ID: %llu\n",
                i, pFrameBuffer->nBufID);
            
            // 直接使用 pFrameBuffer->pImgBuf
            // 无内存拷贝，性能最高
            processImage(pFrameBuffer->pImgBuf, 
                        pFrameBuffer->nWidth,
                        pFrameBuffer->nHeight);
        }
        
        // 必须归还缓冲区
        GXQBuf(hDevice, pFrameBuffer);
    }
}

// 4. 停止采集
GXStreamOff(hDevice);
```

## 7. 触发采集

### 7.1 软件触发示例

```cpp
// 1. 设置触发模式
GXSetEnum(hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
GXSetEnum(hDevice, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_SOFTWARE);

// 2. 开始采集
GXStreamOn(hDevice);

// 3. 触发采集
for (int i = 0; i < 10; i++) {
    // 清空旧图像
    GXFlushQueue(hDevice);
    
    // 发送软件触发
    GXSendCommand(hDevice, GX_COMMAND_TRIGGER_SOFTWARE);
    
    // 获取图像
    GXGetImage(hDevice, &frameData, 1000);
    
    printf("触发采集第%d帧\n", i);
}

// 4. 停止采集
GXStreamOff(hDevice);
```

### 7.2 硬件触发示例

```cpp
// 1. 设置触发模式
GXSetEnum(hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
GXSetEnum(hDevice, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_LINE0);
GXSetEnum(hDevice, GX_ENUM_TRIGGER_ACTIVATION, GX_TRIGGER_ACTIVATION_RISINGEDGE);
GXSetFloat(hDevice, GX_FLOAT_TRIGGER_DELAY, 100.0);  // 延迟100us

// 2. 开始采集（等待外部触发信号）
GXStreamOn(hDevice);

// 3. 持续接收图像
while (running) {
    GXGetImage(hDevice, &frameData, 5000);  // 长超时
}

// 4. 停止采集
GXStreamOff(hDevice);
```

## 8. 采集模式对比

| 特性 | GXGetImage | 回调模式 | 零拷贝模式 |
|------|-----------|---------|-----------|
| 内存拷贝 | 有 | 无 | 无 |
| 性能 | 中 | 高 | 最高 |
| 使用难度 | 简单 | 中等 | 复杂 |
| 线程安全 | 自动 | 需注意 | 需注意 |
| 适用场景 | 低速采集 | 高速采集 | 超高速采集 |
| 推荐度 | ★★★ | ★★★★★ | ★★★★ |

## 9. 性能优化建议

### 9.1 网络优化（GigE相机）

```cpp
// 设置包大小
GXSetInt(hDevice, GX_INT_GEV_PACKETSIZE, 9000);  // 巨型帧

// 设置包延迟
GXSetInt(hDevice, GX_INT_GEV_PACKETDELAY, 0);

// 启用重传
GXSetEnum(hDevice, GX_DS_ENUM_RESEND_MODE, GX_DS_RESEND_MODE_ON);
```

### 9.2 缓冲区优化

```cpp
// 增加缓冲区数量
GXSetAcqusitionBufferNumber(hDevice, 10);

// 增加系统socket缓冲区
GXSetInt(hDevice, GX_DS_INT_SOCKET_BUFFER_SIZE, 2048);  // 2MB
```

### 9.3 采集模式选择

```cpp
// 低速(<30fps): 使用GXGetImage
if (frameRate < 30) {
    // 同步模式
}

// 中速(30-100fps): 使用回调模式
else if (frameRate < 100) {
    GXRegisterCaptureCallback(hDevice, NULL, callback);
}

// 高速(>100fps): 使用零拷贝
else {
    GXDQBuf / GXQBuf
}
```

## 10. 错误处理和丢帧检测

```cpp
uint64_t lastFrameID = 0;
uint64_t totalFrames = 0;
uint64_t lostFrames = 0;

void OnImageCallback(GX_FRAME_CALLBACK_PARAM *pFrameData) {
    if (pFrameData->status == GX_FRAME_STATUS_SUCCESS) {
        totalFrames++;
        
        // 检测丢帧
        if (lastFrameID > 0) {
            uint64_t expectedID = lastFrameID + 1;
            if (pFrameData->nFrameID != expectedID) {
                lostFrames += (pFrameData->nFrameID - expectedID);
                printf("检测到丢帧! 丢失%llu帧\n", 
                    pFrameData->nFrameID - expectedID);
            }
        }
        lastFrameID = pFrameData->nFrameID;
    }
    else if (pFrameData->status == GX_FRAME_STATUS_INCOMPLETE) {
        printf("收到残帧\n");
    }
}

// 定期输出统计
void PrintStatistics() {
    printf("总帧数: %llu, 丢帧数: %llu, 丢帧率: %.2f%%\n",
        totalFrames, lostFrames, 
        100.0 * lostFrames / (totalFrames + lostFrames));
}
```

## 相关文档

- [SDK分析_05_参数控制接口](./SDK分析_05_参数控制接口.md)
- [SDK分析_07_事件回调机制](./SDK分析_07_事件回调机制.md)
