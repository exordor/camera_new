# Galaxy Camera SDK 分析 - 事件回调机制

## 1. 事件机制概述

Galaxy SDK提供了完善的事件回调机制，允许应用程序在特定事件发生时得到通知。主要包括：

1. **图像采集回调** - 收到新图像时触发
2. **设备离线回调** - 设备断开连接时触发  
3. **特性更新回调** - 设备参数变化时触发
4. **设备事件回调** - 设备内部事件（曝光结束、丢帧等）

## 2. 图像采集回调

### 2.1 回调函数类型定义

```cpp
typedef void (GX_STDC *GXCaptureCallBack)(GX_FRAME_CALLBACK_PARAM *pFrameData);
```

### 2.2 回调参数结构

```cpp
typedef struct GX_FRAME_CALLBACK_PARAM {
    void *pUserParam;                // 用户私有数据指针
    GX_FRAME_STATUS status;          // 图像状态
    const void *pImgBuf;             // 图像数据地址（只读）
    int32_t nImgSize;                // 数据大小（字节）
    int32_t nWidth;                  // 图像宽度
    int32_t nHeight;                 // 图像高度
    int32_t nPixelFormat;            // 像素格式
    uint64_t nFrameID;               // 帧ID
    uint64_t nTimestamp;             // 时间戳
    int32_t nOffsetX;                // X方向偏移
    int32_t nOffsetY;                // Y方向偏移
    int32_t reserved[1];             // 保留字段
} GX_FRAME_CALLBACK_PARAM;
```

### 2.3 注册图像采集回调

```cpp
GX_STATUS GXRegisterCaptureCallback(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    void *pUserParam,               // [in] 用户私有参数
    GXCaptureCallBack callBackFun   // [in] 回调函数指针
);
```

**功能**: 注册图像采集回调函数

**使用时机**: 必须在`GXStreamOn`之前调册

**完整示例**:

```cpp
// 1. 定义用户数据结构
struct UserData {
    int frameCount;
    std::string savePath;
};

// 2. 定义回调函数
void OnFrameCallback(GX_FRAME_CALLBACK_PARAM *pFrameData) {
    // 获取用户数据
    UserData* userData = (UserData*)pFrameData->pUserParam;
    
    if (pFrameData->status == GX_FRAME_STATUS_SUCCESS) {
        userData->frameCount++;
        
        printf("收到第%d帧图像\n", userData->frameCount);
        printf("  尺寸: %dx%d\n", pFrameData->nWidth, pFrameData->nHeight);
        printf("  帧号: %llu\n", pFrameData->nFrameID);
        printf("  时间戳: %llu\n", pFrameData->nTimestamp);
        
        // 处理图像数据
        // 注意: pImgBuf指向的内存由SDK管理，回调返回后可能失效
        // 如需保存，必须立即拷贝
        
    } else if (pFrameData->status == GX_FRAME_STATUS_INCOMPLETE) {
        printf("收到残帧\n");
    }
}

// 3. 使用示例
int main() {
    GX_DEV_HANDLE hDevice;
    // ... 打开设备 ...
    
    UserData userData = {0, "/home/images"};
    
    // 注册回调
    GXRegisterCaptureCallback(hDevice, &userData, OnFrameCallback);
    
    // 开始采集
    GXStreamOn(hDevice);
    
    // 等待采集
    sleep(10);
    
    // 停止采集
    GXStreamOff(hDevice);
    
    // 注销回调
    GXUnregisterCaptureCallback(hDevice);
    
    printf("总共收到 %d 帧\n", userData.frameCount);
    
    return 0;
}
```

### 2.4 注销图像采集回调

```cpp
GX_STATUS GXUnregisterCaptureCallback(GX_DEV_HANDLE hDevice);
```

**功能**: 注销图像采集回调函数

**使用时机**: 必须在`GXStreamOff`之后调用

### 2.5 回调函数注意事项

#### ⚠️ 重要事项

1. **不要执行耗时操作**
   ```cpp
   void OnFrameCallback(GX_FRAME_CALLBACK_PARAM *pFrameData) {
       // ❌ 错误: 在回调中执行耗时操作
       cv::imwrite("image.jpg", image);  // 写文件很慢
       
       // ✅ 正确: 快速拷贝，异步处理
       {
           std::lock_guard<std::mutex> lock(queueMutex);
           imageQueue.push(image.clone());
       }
   }
   ```

2. **注意内存生命周期**
   ```cpp
   void OnFrameCallback(GX_FRAME_CALLBACK_PARAM *pFrameData) {
       // ❌ 错误: 直接保存指针
       savedPtr = pFrameData->pImgBuf;  // 回调返回后失效!
       
       // ✅ 正确: 立即拷贝数据
       memcpy(myBuffer, pFrameData->pImgBuf, pFrameData->nImgSize);
   }
   ```

3. **线程安全**
   ```cpp
   std::mutex dataMutex;
   int frameCount = 0;
   
   void OnFrameCallback(GX_FRAME_CALLBACK_PARAM *pFrameData) {
       // ✅ 正确: 使用互斥锁保护共享数据
       std::lock_guard<std::mutex> lock(dataMutex);
       frameCount++;
   }
   ```

4. **不要抛出异常**
   ```cpp
   void OnFrameCallback(GX_FRAME_CALLBACK_PARAM *pFrameData) {
       try {
           // 处理可能抛异常的代码
           processImage(pFrameData->pImgBuf);
       } catch (...) {
           // 捕获所有异常，不要让异常传出回调
           printf("处理图像时发生错误\n");
       }
   }
   ```

### 2.6 高级示例：生产者-消费者模式

```cpp
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

class ImageProcessor {
private:
    std::queue<cv::Mat> imageQueue;
    std::mutex queueMutex;
    std::condition_variable queueCV;
    bool running;
    std::thread processThread;
    
public:
    ImageProcessor() : running(false) {}
    
    // 回调函数（生产者）
    static void OnFrameCallback(GX_FRAME_CALLBACK_PARAM *pFrameData) {
        ImageProcessor* processor = (ImageProcessor*)pFrameData->pUserParam;
        processor->AddImage(pFrameData);
    }
    
    void AddImage(GX_FRAME_CALLBACK_PARAM *pFrameData) {
        if (pFrameData->status == GX_FRAME_STATUS_SUCCESS) {
            // 转换为OpenCV Mat
            cv::Mat image(pFrameData->nHeight, 
                         pFrameData->nWidth,
                         CV_8UC1,
                         (void*)pFrameData->pImgBuf);
            
            // 加入队列
            std::unique_lock<std::mutex> lock(queueMutex);
            imageQueue.push(image.clone());  // 深拷贝
            queueCV.notify_one();
        }
    }
    
    // 处理线程（消费者）
    void ProcessLoop() {
        while (running) {
            cv::Mat image;
            
            // 从队列取图像
            {
                std::unique_lock<std::mutex> lock(queueMutex);
                queueCV.wait(lock, [this] { 
                    return !imageQueue.empty() || !running; 
                });
                
                if (!running) break;
                
                if (!imageQueue.empty()) {
                    image = imageQueue.front();
                    imageQueue.pop();
                }
            }
            
            // 处理图像（耗时操作）
            if (!image.empty()) {
                processImage(image);
            }
        }
    }
    
    void Start() {
        running = true;
        processThread = std::thread(&ImageProcessor::ProcessLoop, this);
    }
    
    void Stop() {
        running = false;
        queueCV.notify_all();
        if (processThread.joinable()) {
            processThread.join();
        }
    }
    
    void processImage(const cv::Mat& image) {
        // 执行耗时的图像处理
        // 例如：滤波、检测、识别、保存等
        cv::imwrite("image.jpg", image);
    }
};

// 使用示例
int main() {
    GX_DEV_HANDLE hDevice;
    // ... 打开设备 ...
    
    ImageProcessor processor;
    processor.Start();
    
    // 注册回调
    GXRegisterCaptureCallback(hDevice, &processor, 
        ImageProcessor::OnFrameCallback);
    
    // 开始采集
    GXStreamOn(hDevice);
    
    // 运行一段时间
    sleep(60);
    
    // 停止采集
    GXStreamOff(hDevice);
    GXUnregisterCaptureCallback(hDevice);
    
    processor.Stop();
    
    return 0;
}
```

## 3. 设备离线回调

### 3.1 回调函数类型定义

```cpp
typedef void (GX_STDC *GXDeviceOfflineCallBack)(void *pUserParam);
```

### 3.2 注册设备离线回调

```cpp
GX_STATUS GXRegisterDeviceOfflineCallback(
    GX_DEV_HANDLE hDevice,                  // [in] 设备句柄
    void *pUserParam,                       // [in] 用户私有参数
    GXDeviceOfflineCallBack callBackFun,    // [in] 回调函数
    GX_EVENT_CALLBACK_HANDLE *pHCallBack    // [out] 返回回调句柄
);
```

**功能**: 注册设备离线通知回调

**使用场景**: 
- 检测网线断开
- 检测相机断电
- 实现自动重连

**示例**:

```cpp
// 1. 定义回调函数
void OnDeviceOffline(void *pUserParam) {
    printf("设备离线！\n");
    
    // 执行清理或重连逻辑
    bool* deviceOnline = (bool*)pUserParam;
    *deviceOnline = false;
}

// 2. 使用示例
int main() {
    GX_DEV_HANDLE hDevice;
    GX_EVENT_CALLBACK_HANDLE hOfflineCallback;
    bool deviceOnline = true;
    
    // ... 打开设备 ...
    
    // 注册离线回调
    GXRegisterDeviceOfflineCallback(hDevice, 
        &deviceOnline,
        OnDeviceOffline,
        &hOfflineCallback);
    
    // 开始采集
    GXStreamOn(hDevice);
    
    // 监控设备状态
    while (deviceOnline) {
        // 正常采集
        sleep(1);
    }
    
    printf("检测到设备离线，尝试重连...\n");
    
    // 清理
    GXUnregisterDeviceOfflineCallback(hDevice, hOfflineCallback);
    
    return 0;
}
```

### 3.3 注销设备离线回调

```cpp
GX_STATUS GXUnregisterDeviceOfflineCallback(
    GX_DEV_HANDLE hDevice,                  // [in] 设备句柄
    GX_EVENT_CALLBACK_HANDLE hCallBack      // [in] 回调句柄
);
```

**功能**: 注销设备离线回调

### 3.4 设备自动重连示例

```cpp
class DeviceManager {
private:
    GX_DEV_HANDLE hDevice;
    GX_EVENT_CALLBACK_HANDLE hOfflineCallback;
    bool deviceOnline;
    std::string deviceSN;
    
public:
    static void OnDeviceOffline(void *pUserParam) {
        DeviceManager* manager = (DeviceManager*)pUserParam;
        manager->HandleOffline();
    }
    
    void HandleOffline() {
        printf("设备离线，开始自动重连...\n");
        deviceOnline = false;
        
        // 关闭设备
        if (hDevice) {
            GXStreamOff(hDevice);
            GXUnregisterDeviceOfflineCallback(hDevice, hOfflineCallback);
            GXCloseDevice(hDevice);
            hDevice = NULL;
        }
        
        // 尝试重连
        Reconnect();
    }
    
    void Reconnect() {
        int retryCount = 0;
        const int maxRetry = 10;
        
        while (retryCount < maxRetry && !deviceOnline) {
            printf("重连尝试 %d/%d...\n", retryCount + 1, maxRetry);
            
            // 枚举设备
            uint32_t deviceNum = 0;
            GXUpdateDeviceList(&deviceNum, 1000);
            
            if (deviceNum > 0) {
                // 尝试打开设备
                GX_OPEN_PARAM openParam;
                openParam.pszContent = (char*)deviceSN.c_str();
                openParam.openMode = GX_OPEN_SN;
                openParam.accessMode = GX_ACCESS_EXCLUSIVE;
                
                if (GXOpenDevice(&openParam, &hDevice) == GX_STATUS_SUCCESS) {
                    printf("设备重连成功！\n");
                    
                    // 重新注册离线回调
                    GXRegisterDeviceOfflineCallback(hDevice, 
                        this, OnDeviceOffline, &hOfflineCallback);
                    
                    // 恢复采集
                    InitCamera();
                    GXStreamOn(hDevice);
                    deviceOnline = true;
                    break;
                }
            }
            
            retryCount++;
            sleep(2);  // 等待2秒后重试
        }
        
        if (!deviceOnline) {
            printf("重连失败，已达到最大重试次数\n");
        }
    }
    
    void InitCamera() {
        // 配置相机参数
        GXSetEnum(hDevice, GX_ENUM_PIXEL_FORMAT, GX_PIXEL_FORMAT_MONO8);
        GXSetInt(hDevice, GX_INT_WIDTH, 1920);
        GXSetInt(hDevice, GX_INT_HEIGHT, 1080);
        // ...
    }
};
```

## 4. 特性更新回调

### 4.1 回调函数类型定义

```cpp
typedef void (GX_STDC *GXFeatureCallBack)(
    GX_FEATURE_ID_CMD nFeatureID,   // 发生变化的特性ID
    void *pUserParam                // 用户私有参数
);
```

### 4.2 注册特性更新回调

```cpp
GX_STATUS GXRegisterFeatureCallback(
    GX_DEV_HANDLE hDevice,                      // [in] 设备句柄
    void *pUserParam,                           // [in] 用户私有参数
    GXFeatureCallBack callBackFun,              // [in] 回调函数
    GX_FEATURE_ID_CMD featureID,                // [in] 要监听的特性ID
    GX_FEATURE_CALLBACK_HANDLE *pHCallBack      // [out] 返回回调句柄
);
```

**功能**: 注册设备属性更新回调

**使用场景**:
- 监控曝光时间变化
- 监控增益变化
- 监控ROI变化
- 实现参数联动

**示例**:

```cpp
// 1. 定义回调函数
void OnExposureChanged(GX_FEATURE_ID_CMD nFeatureID, void *pUserParam) {
    GX_DEV_HANDLE hDevice = *(GX_DEV_HANDLE*)pUserParam;
    
    if (nFeatureID == GX_FLOAT_EXPOSURE_TIME) {
        double exposureTime;
        GXGetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, &exposureTime);
        printf("曝光时间已更改为: %.2f us\n", exposureTime);
        
        // 可以根据曝光时间调整其他参数
        // 例如：调整增益
    }
}

// 2. 使用示例
int main() {
    GX_DEV_HANDLE hDevice;
    GX_FEATURE_CALLBACK_HANDLE hFeatureCallback;
    
    // ... 打开设备 ...
    
    // 注册特性回调
    GXRegisterFeatureCallback(hDevice,
        &hDevice,
        OnExposureChanged,
        GX_FLOAT_EXPOSURE_TIME,
        &hFeatureCallback);
    
    // 修改曝光时间会触发回调
    GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, 10000.0);
    
    // ... 其他操作 ...
    
    // 注销回调
    GXUnregisterFeatureCallback(hDevice, 
        GX_FLOAT_EXPOSURE_TIME, 
        hFeatureCallback);
    
    return 0;
}
```

### 4.3 注销特性更新回调

```cpp
GX_STATUS GXUnregisterFeatureCallback(
    GX_DEV_HANDLE hDevice,                      // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,                // [in] 特性ID
    GX_FEATURE_CALLBACK_HANDLE hCallBack        // [in] 回调句柄
);
```

**功能**: 注销特性更新回调

### 4.4 高级示例：参数联动

```cpp
class ParameterController {
private:
    GX_DEV_HANDLE hDevice;
    GX_FEATURE_CALLBACK_HANDLE hExposureCallback;
    GX_FEATURE_CALLBACK_HANDLE hGainCallback;
    
public:
    // 曝光时间变化回调
    static void OnExposureChanged(GX_FEATURE_ID_CMD nFeatureID, 
                                  void *pUserParam) {
        ParameterController* controller = (ParameterController*)pUserParam;
        controller->AdjustGain();
    }
    
    // 增益变化回调
    static void OnGainChanged(GX_FEATURE_ID_CMD nFeatureID, 
                              void *pUserParam) {
        ParameterController* controller = (ParameterController*)pUserParam;
        controller->AdjustExposure();
    }
    
    void AdjustGain() {
        double exposureTime;
        GXGetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, &exposureTime);
        
        // 根据曝光时间自动调整增益
        if (exposureTime < 5000) {
            // 曝光时间短，提高增益
            GXSetFloat(hDevice, GX_FLOAT_GAIN, 10.0);
        } else {
            // 曝光时间长，降低增益
            GXSetFloat(hDevice, GX_FLOAT_GAIN, 0.0);
        }
    }
    
    void AdjustExposure() {
        double gain;
        GXGetFloat(hDevice, GX_FLOAT_GAIN, &gain);
        
        // 根据增益自动调整曝光时间
        if (gain > 15.0) {
            // 增益高，缩短曝光时间
            GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, 3000.0);
        }
    }
    
    void RegisterCallbacks() {
        GXRegisterFeatureCallback(hDevice, this, OnExposureChanged,
            GX_FLOAT_EXPOSURE_TIME, &hExposureCallback);
        
        GXRegisterFeatureCallback(hDevice, this, OnGainChanged,
            GX_FLOAT_GAIN, &hGainCallback);
    }
    
    void UnregisterCallbacks() {
        GXUnregisterFeatureCallback(hDevice, 
            GX_FLOAT_EXPOSURE_TIME, hExposureCallback);
        GXUnregisterFeatureCallback(hDevice, 
            GX_FLOAT_GAIN, hGainCallback);
    }
};
```

## 5. 设备事件

### 5.1 清空事件队列

```cpp
GX_STATUS GXFlushEvent(GX_DEV_HANDLE hDevice);
```

**功能**: 清空设备事件数据队列

**使用场景**: 获取实时事件前清空历史事件

### 5.2 获取事件队列长度

```cpp
GX_STATUS GXGetEventNumInQueue(
    GX_DEV_HANDLE hDevice,      // [in] 设备句柄
    uint32_t *pnEventNum        // [out] 返回事件数量
);
```

**功能**: 获取当前事件队列中的事件数量

**示例**:

```cpp
uint32_t eventNum;
GXGetEventNumInQueue(hDevice, &eventNum);
printf("事件队列中有 %d 个事件\n", eventNum);
```

## 6. 最佳实践总结

### 6.1 回调函数设计原则

1. **快速返回** - 不执行耗时操作
2. **数据拷贝** - 立即拷贝需要的数据
3. **线程安全** - 使用互斥锁保护共享数据
4. **异常处理** - 捕获所有异常
5. **异步处理** - 使用队列+处理线程

### 6.2 性能优化建议

```cpp
// ❌ 低效：在回调中直接处理
void OnFrameCallback(GX_FRAME_CALLBACK_PARAM *pFrameData) {
    cv::Mat image = convertToMat(pFrameData);
    cv::imwrite("image.jpg", image);  // 慢
    processImage(image);  // 慢
}

// ✅ 高效：使用队列异步处理
void OnFrameCallback(GX_FRAME_CALLBACK_PARAM *pFrameData) {
    // 快速拷贝
    ImageData data;
    data.width = pFrameData->nWidth;
    data.height = pFrameData->nHeight;
    data.buffer = copyBuffer(pFrameData->pImgBuf);
    
    // 加入队列
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        imageQueue.push(data);
    }
    // 快速返回
}
```

### 6.3 错误处理

```cpp
void OnFrameCallback(GX_FRAME_CALLBACK_PARAM *pFrameData) {
    try {
        // 检查状态
        if (pFrameData->status != GX_FRAME_STATUS_SUCCESS) {
            handleError(pFrameData->status);
            return;
        }
        
        // 检查数据有效性
        if (!pFrameData->pImgBuf || pFrameData->nImgSize <= 0) {
            printf("无效的图像数据\n");
            return;
        }
        
        // 处理图像
        processFrame(pFrameData);
        
    } catch (const std::exception& e) {
        printf("回调异常: %s\n", e.what());
    } catch (...) {
        printf("未知异常\n");
    }
}
```

## 相关文档

- [SDK分析_06_图像采集接口](./SDK分析_06_图像采集接口.md)
- [SDK分析_05_参数控制接口](./SDK分析_05_参数控制接口.md)
