# Galaxy Camera SDK 分析 - 数据结构定义

## 1. 句柄类型定义

SDK使用void指针类型定义各种句柄：

```cpp
typedef void* GX_DEV_HANDLE;                // 设备句柄
typedef void* GX_EVENT_CALLBACK_HANDLE;     // 事件回调句柄
typedef void* GX_FEATURE_CALLBACK_HANDLE;   // 特性回调句柄
```

**说明**:
- `GX_DEV_HANDLE`: 通过GXOpenDevice获取，用于控制和采集
- `GX_EVENT_CALLBACK_HANDLE`: 用于注册事件回调函数
- `GX_FEATURE_CALLBACK_HANDLE`: 用于注册设备属性更新回调

## 2. 设备信息结构

### 2.1 设备基础信息 (GX_DEVICE_BASE_INFO)

```cpp
typedef struct GX_DEVICE_BASE_INFO {
    char szVendorName[32];           // 厂商名称，32字节
    char szModelName[32];            // 型号名称，32字节
    char szSN[32];                   // 设备序列号，32字节
    char szDisplayName[132];         // 设备显示名称，128+4字节
    char szDeviceID[68];             // 设备唯一标识符，64+4字节
    char szUserID[68];               // 用户自定义名称，64+4字节
    GX_ACCESS_STATUS_CMD accessStatus;  // 设备当前访问状态，4字节
    GX_DEVICE_CLASS deviceClass;     // 设备类型（USB2.0/GEV等），4字节
    char reserved[300];              // 保留字段，300字节
} GX_DEVICE_BASE_INFO;
```

**字段说明**:
- `szVendorName`: 相机制造商名称
- `szModelName`: 相机型号
- `szSN`: 唯一序列号，用于识别特定设备
- `szDisplayName`: 友好的显示名称
- `szDeviceID`: 系统生成的唯一ID
- `szUserID`: 用户可自定义的设备名称
- `accessStatus`: 当前可访问状态（读写/只读/不可访问）
- `deviceClass`: 设备连接类型

### 2.2 设备IP信息 (GX_DEVICE_IP_INFO)

```cpp
typedef struct GX_DEVICE_IP_INFO {
    char szDeviceID[68];             // 设备唯一标识符，64+4字节
    char szMAC[32];                  // MAC地址，32字节
    char szIP[32];                   // IP地址，32字节
    char szSubNetMask[32];           // 子网掩码，32字节
    char szGateWay[32];              // 网关，32字节
    char szNICMAC[32];               // 对应网卡的MAC地址，32字节
    char szNICIP[32];                // 对应网卡的IP地址，32字节
    char szNICSubNetMask[32];        // 对应网卡的子网掩码，32字节
    char szNICGateWay[32];           // 对应网卡的网关，32字节
    char szNICDescription[132];      // 对应网卡的描述，128+4字节
    char reserved[512];              // 保留字段，512字节
} GX_DEVICE_IP_INFO;
```

**字段说明**:
- **设备网络信息**: szMAC, szIP, szSubNetMask, szGateWay
- **主机网卡信息**: szNICMAC, szNICIP, szNICSubNetMask, szNICGateWay
- 用于GigE相机的网络配置和诊断

### 2.3 设备打开参数 (GX_OPEN_PARAM)

```cpp
typedef struct GX_OPEN_PARAM {
    char* pszContent;                // 由openMode决定的内容字符串
    GX_OPEN_MODE_CMD openMode;       // 设备打开方式
    GX_ACCESS_MODE_CMD accessMode;   // 设备访问模式
} GX_OPEN_PARAM;
```

**使用示例**:
```cpp
GX_OPEN_PARAM openParam;
openParam.pszContent = "192.168.1.100";      // IP地址
openParam.openMode = GX_OPEN_IP;             // 通过IP打开
openParam.accessMode = GX_ACCESS_EXCLUSIVE;  // 独占模式
```

## 3. 图像数据结构

### 3.1 图像帧数据 (GX_FRAME_DATA)

```cpp
typedef struct GX_FRAME_DATA {
    GX_FRAME_STATUS nStatus;         // 图像状态
    void* pImgBuf;                   // 图像数据地址
    int32_t nWidth;                  // 图像宽度
    int32_t nHeight;                 // 图像高度
    int32_t nPixelFormat;            // 图像像素格式
    int32_t nImgSize;                // 数据大小（字节）
    uint64_t nFrameID;               // 帧ID
    uint64_t nTimestamp;             // 时间戳
    int32_t nOffsetX;                // X方向偏移
    int32_t nOffsetY;                // Y方向偏移
    int32_t reserved[1];             // 保留字段，4字节
} GX_FRAME_DATA;
```

**字段说明**:
- `nStatus`: 指示帧是否完整（成功/残帧/错误）
- `pImgBuf`: 指向实际图像数据的指针
- `nWidth/nHeight`: 图像尺寸
- `nPixelFormat`: 像素格式（Mono8, Bayer, RGB等）
- `nImgSize`: 图像数据总大小
- `nFrameID`: 帧序号，用于检测丢帧
- `nTimestamp`: 相机硬件时间戳（详见下方说明）

#### nTimestamp 详解

**时间戳来源**: 相机内部硬件计数器

**生成位置**: 
- 由相机FPGA/芯片内部的时钟计数器产生
- 在图像传感器**曝光结束时刻**记录
- 独立于PC系统时钟运行

**时间戳单位**: 
- 取决于相机型号的时钟频率（`GX_INT_TIMESTAMP_TICK_FREQUENCY`）
- 常见值: 1 GHz (1 tick = 1 纳秒) 或 125 MHz (1 tick = 8 纳秒)
- 需要查询 `GX_INT_TIMESTAMP_TICK_FREQUENCY` 获取准确频率

**时间戳起点**:
- 从相机上电/复位时开始计数（tick = 0）
- **不是** Unix时间戳（不基于1970-01-01）
- 可通过 `GX_COMMAND_TIMESTAMP_RESET` 命令重置为0

**典型值示例**:
```
假设时钟频率 = 125 MHz (8ns/tick)
nTimestamp = 625,000,000  // 625M ticks
实际时间 = 625,000,000 × 8ns = 5,000,000,000ns = 5秒
含义: 相机启动后第5秒拍摄的图像
```

**与ROS时间戳的区别**:

| 特性 | nTimestamp (硬件) | ROS header.stamp (软件) |
|------|------------------|------------------------|
| 生成位置 | 相机FPGA | PC系统 |
| 时间基准 | 相机上电时刻 | Unix Epoch (1970) |
| 精度 | 纳秒级 (取决于时钟) | 纳秒级 |
| 延迟 | 0ms (曝光结束) | ~50-150ms (包含传输和处理) |
| 用途 | 精确帧时序、触发同步 | ROS节点间同步 |

**如何使用硬件时间戳**:
```cpp
// 获取时钟频率
int64_t tick_frequency;
GXGetInt(hDevice, GX_INT_TIMESTAMP_TICK_FREQUENCY, &tick_frequency);

// 转换为秒
double timestamp_seconds = (double)pFrame->nTimestamp / tick_frequency;

// 转换为ROS时间（需要记录起始时间）
static ros::Time start_ros_time = ros::Time::now();
static uint64_t start_camera_timestamp = first_frame->nTimestamp;

ros::Time frame_time = start_ros_time + 
    ros::Duration((pFrame->nTimestamp - start_camera_timestamp) / (double)tick_frequency);
```

**优点**:
- ✅ 精确反映图像实际捕获时刻
- ✅ 不受PC系统负载影响
- ✅ 适合多相机同步、外部触发场景

**缺点**:
- ❌ 需要转换才能与ROS系统时间对应
- ❌ 相机重启会重置计数器
- ❌ 不同相机的时间戳不同步（除非使用硬件同步）

### 3.2 图像缓冲区 (GX_FRAME_BUFFER)

```cpp
typedef struct GX_FRAME_BUFFER {
    GX_FRAME_STATUS nStatus;         // 图像状态
    void* pImgBuf;                   // 图像数据指针
    int32_t nWidth;                  // 图像宽度
    int32_t nHeight;                 // 图像高度
    int32_t nPixelFormat;            // 像素格式
    int32_t nImgSize;                // 数据大小（字节）
    uint64_t nFrameID;               // 帧ID
    uint64_t nTimestamp;             // 时间戳
    uint64_t nBufID;                 // 缓冲区ID
    int32_t nOffsetX;                // X方向偏移
    int32_t nOffsetY;                // Y方向偏移
    int32_t reserved[16];            // 保留字段，64字节
} GX_FRAME_BUFFER;
```

**与GX_FRAME_DATA的区别**:
- 增加了`nBufID`字段用于缓冲区管理
- 用于零拷贝(zero-copy)采集模式
- 需要通过GXQBuf归还缓冲区

### 3.3 回调函数参数 (GX_FRAME_CALLBACK_PARAM)

```cpp
typedef struct GX_FRAME_CALLBACK_PARAM {
    void* pUserParam;                // 用户私有数据指针
    GX_FRAME_STATUS status;          // 图像状态
    const void* pImgBuf;             // 图像数据地址（只读）
    int32_t nImgSize;                // 数据大小（字节）
    int32_t nWidth;                  // 图像宽度
    int32_t nHeight;                 // 图像高度
    int32_t nPixelFormat;            // 像素格式
    uint64_t nFrameID;               // 帧ID
    uint64_t nTimestamp;             // 时间戳
    int32_t nOffsetX;                // X方向偏移
    int32_t nOffsetY;                // Y方向偏移
    int32_t reserved[1];             // 保留字段，4字节
} GX_FRAME_CALLBACK_PARAM;
```

**使用场景**: 在注册的回调函数中接收此结构体，用于回调采集模式

## 4. 参数范围结构

### 4.1 整型参数范围 (GX_INT_RANGE)

```cpp
typedef struct GX_INT_RANGE {
    int64_t nMin;                    // 最小值
    int64_t nMax;                    // 最大值
    int64_t nInc;                    // 步长
    int32_t reserved[8];             // 保留字段，32字节
} GX_INT_RANGE;
```

**使用示例**:
```cpp
GX_INT_RANGE widthRange;
GXGetIntRange(hDevice, GX_INT_WIDTH, &widthRange);
// widthRange.nMin: 最小宽度
// widthRange.nMax: 最大宽度
// widthRange.nInc: 宽度增量（通常为1）
```

### 4.2 浮点型参数范围 (GX_FLOAT_RANGE)

```cpp
typedef struct GX_FLOAT_RANGE {
    double dMin;                     // 最小值
    double dMax;                     // 最大值
    double dInc;                     // 步长
    char szUnit[8];                  // 单位，8字节
    bool bIncIsValid;                // 步长是否有效，1字节
    int8_t reserved[31];             // 保留字段，31字节
} GX_FLOAT_RANGE;
```

**使用示例**:
```cpp
GX_FLOAT_RANGE exposureRange;
GXGetFloatRange(hDevice, GX_FLOAT_EXPOSURE_TIME, &exposureRange);
// exposureRange.dMin: 最小曝光时间
// exposureRange.dMax: 最大曝光时间
// exposureRange.szUnit: "us" (微秒)
```

### 4.3 枚举型参数描述 (GX_ENUM_DESCRIPTION)

```cpp
typedef struct GX_ENUM_DESCRIPTION {
    int64_t nValue;                  // 枚举项的值
    char szSymbolic[64];             // 枚举项的字符描述，64字节
    int32_t reserved[8];             // 保留字节，32字节
} GX_ENUM_DESCRIPTION;
```

**使用场景**: 获取枚举类型参数的所有可选项

## 5. 回调函数类型定义

### 5.1 图像采集回调函数

```cpp
typedef void (GX_STDC *GXCaptureCallBack)(GX_FRAME_CALLBACK_PARAM *pFrameData);
```

**函数原型**:
```cpp
void MyImageCallback(GX_FRAME_CALLBACK_PARAM *pFrameData) {
    // 处理图像数据
    if (pFrameData->status == GX_FRAME_STATUS_SUCCESS) {
        // pFrameData->pImgBuf 指向图像数据
        // pFrameData->nWidth, nHeight 是图像尺寸
    }
}
```

### 5.2 设备离线回调函数

```cpp
typedef void (GX_STDC *GXDeviceOfflineCallBack)(void *pUserParam);
```

**函数原型**:
```cpp
void MyOfflineCallback(void *pUserParam) {
    // 设备离线时被调用
    printf("设备离线！\n");
}
```

### 5.3 特性更新回调函数

```cpp
typedef void (GX_STDC *GXFeatureCallBack)(GX_FEATURE_ID_CMD nFeatureID, void *pUserParam);
```

**函数原型**:
```cpp
void MyFeatureCallback(GX_FEATURE_ID_CMD nFeatureID, void *pUserParam) {
    // 当设备属性更新时被调用
    // nFeatureID 指示哪个属性发生了变化
}
```

## 6. 信息长度常量

```cpp
#define GX_INFO_LENGTH_8_BYTE    (8)      // 8字节
#define GX_INFO_LENGTH_32_BYTE   (32)     // 32字节
#define GX_INFO_LENGTH_64_BYTE   (64)     // 64字节
#define GX_INFO_LENGTH_128_BYTE  (128)    // 128字节
```

这些常量定义了各种信息字符串的缓冲区大小。

## 7. 数据结构使用流程示例

```cpp
// 1. 获取设备信息
GX_DEVICE_BASE_INFO baseInfo;
size_t size = sizeof(GX_DEVICE_BASE_INFO);
GXGetAllDeviceBaseInfo(&baseInfo, &size);

// 2. 打开设备
GX_DEV_HANDLE hDevice;
GX_OPEN_PARAM openParam;
openParam.pszContent = baseInfo.szSN;
openParam.openMode = GX_OPEN_SN;
openParam.accessMode = GX_ACCESS_EXCLUSIVE;
GXOpenDevice(&openParam, &hDevice);

// 3. 获取参数范围
GX_FLOAT_RANGE exposureRange;
GXGetFloatRange(hDevice, GX_FLOAT_EXPOSURE_TIME, &exposureRange);

// 4. 采集图像
GX_FRAME_DATA frameData;
frameData.pImgBuf = malloc(width * height);
GXGetImage(hDevice, &frameData, 1000);

// 5. 处理图像
if (frameData.nStatus == GX_FRAME_STATUS_SUCCESS) {
    // 使用 frameData.pImgBuf 处理图像
}
```

## 8. 内存管理注意事项

1. **图像缓冲区**: 
   - GXGetImage模式: 用户分配和释放缓冲区
   - 回调模式: SDK管理缓冲区
   - GXDQBuf/GXQBuf模式: 零拷贝，需要归还缓冲区

2. **字符串缓冲区**: 
   - 使用前需要分配足够大小
   - 建议使用GX_INFO_LENGTH_*常量

3. **结构体初始化**: 
   - 建议使用memset清零
   - 特别注意reserved字段

## 相关文档

- [SDK分析_04_图像采集接口](./SDK分析_04_图像采集接口.md)
- [SDK分析_05_参数控制接口](./SDK分析_05_参数控制接口.md)
