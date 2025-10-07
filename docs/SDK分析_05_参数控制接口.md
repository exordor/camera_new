# Galaxy Camera SDK 分析 - 参数控制接口

## 1. 参数特性查询

### 1.1 GXGetFeatureName - 获取特性名称

```cpp
GX_STATUS GXGetFeatureName(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    char *pszName,                  // [out] 返回名称字符串
    size_t *pnSize                  // [in,out] 缓冲区大小
);
```

**功能**: 获取特性码对应的字符串描述

**示例**:
```cpp
char name[128];
size_t size = 128;
GXGetFeatureName(hDevice, GX_FLOAT_EXPOSURE_TIME, name, &size);
printf("特性名称: %s\n", name);  // 输出: ExposureTime
```

### 1.2 GXIsImplemented - 查询是否支持

```cpp
GX_STATUS GXIsImplemented(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    bool *pbIsImplemented           // [out] 是否支持
);
```

**功能**: 查询相机是否支持某个特性

**使用场景**: 在使用某功能前先检查是否支持

**示例**:
```cpp
bool isImplemented = false;
GXIsImplemented(hDevice, GX_BOOL_GAMMA_ENABLE, &isImplemented);
if (isImplemented) {
    printf("设备支持Gamma功能\n");
}
```

### 1.3 GXIsReadable - 查询是否可读

```cpp
GX_STATUS GXIsReadable(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    bool *pbIsReadable              // [out] 是否可读
);
```

**功能**: 查询特性码当前是否可读

**示例**:
```cpp
bool isReadable = false;
GXIsReadable(hDevice, GX_FLOAT_EXPOSURE_TIME, &isReadable);
if (isReadable) {
    double exposureTime;
    GXGetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, &exposureTime);
}
```

### 1.4 GXIsWritable - 查询是否可写

```cpp
GX_STATUS GXIsWritable(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    bool *pbIsWritable              // [out] 是否可写
);
```

**功能**: 查询特性码当前是否可写

**示例**:
```cpp
bool isWritable = false;
GXIsWritable(hDevice, GX_FLOAT_EXPOSURE_TIME, &isWritable);
if (isWritable) {
    GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, 10000.0);
}
```

## 2. 整型参数控制

### 2.1 GXGetIntRange - 获取整型范围

```cpp
GX_STATUS GXGetIntRange(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    GX_INT_RANGE *pIntRange         // [out] 整型范围结构
);
```

**功能**: 获取整型参数的最小值、最大值和步长

**示例**:
```cpp
GX_INT_RANGE widthRange;
GXGetIntRange(hDevice, GX_INT_WIDTH, &widthRange);
printf("宽度范围: %lld - %lld, 步长: %lld\n",
    widthRange.nMin, widthRange.nMax, widthRange.nInc);
```

### 2.2 GXGetInt - 获取整型值

```cpp
GX_STATUS GXGetInt(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    int64_t *pnValue                // [out] 返回值
);
```

**功能**: 获取整型参数的当前值

**示例**:
```cpp
int64_t width, height;
GXGetInt(hDevice, GX_INT_WIDTH, &width);
GXGetInt(hDevice, GX_INT_HEIGHT, &height);
printf("图像尺寸: %lld x %lld\n", width, height);
```

### 2.3 GXSetInt - 设置整型值

```cpp
GX_STATUS GXSetInt(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    int64_t nValue                  // [in] 要设置的值
);
```

**功能**: 设置整型参数的值

**注意**: 
- 值必须在范围内
- 值必须是步长的整数倍
- 否则返回`GX_STATUS_OUT_OF_RANGE`

**示例**:
```cpp
// 设置图像宽度为1920
GXSetInt(hDevice, GX_INT_WIDTH, 1920);

// 设置图像高度为1080
GXSetInt(hDevice, GX_INT_HEIGHT, 1080);

// 设置ROI偏移
GXSetInt(hDevice, GX_INT_OFFSET_X, 100);
GXSetInt(hDevice, GX_INT_OFFSET_Y, 50);
```

### 常用整型参数

| 参数ID | 说明 | 单位 |
|--------|------|------|
| `GX_INT_WIDTH` | 图像宽度 | 像素 |
| `GX_INT_HEIGHT` | 图像高度 | 像素 |
| `GX_INT_OFFSET_X` | X方向偏移 | 像素 |
| `GX_INT_OFFSET_Y` | Y方向偏移 | 像素 |
| `GX_INT_SENSOR_WIDTH` | 传感器宽度 | 像素 |
| `GX_INT_SENSOR_HEIGHT` | 传感器高度 | 像素 |
| `GX_INT_BINNING_HORIZONTAL` | 水平合并 | - |
| `GX_INT_BINNING_VERTICAL` | 垂直合并 | - |
| `GX_INT_PAYLOAD_SIZE` | 图像数据大小 | 字节 |

## 3. 浮点型参数控制

### 3.1 GXGetFloatRange - 获取浮点范围

```cpp
GX_STATUS GXGetFloatRange(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    GX_FLOAT_RANGE *pFloatRange     // [out] 浮点范围结构
);
```

**功能**: 获取浮点型参数的范围、步长和单位

**示例**:
```cpp
GX_FLOAT_RANGE exposureRange;
GXGetFloatRange(hDevice, GX_FLOAT_EXPOSURE_TIME, &exposureRange);
printf("曝光时间范围: %.2f - %.2f %s\n",
    exposureRange.dMin, exposureRange.dMax, exposureRange.szUnit);
```

### 3.2 GXGetFloat - 获取浮点值

```cpp
GX_STATUS GXGetFloat(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    double *pdValue                 // [out] 返回值
);
```

**功能**: 获取浮点型参数的当前值

**示例**:
```cpp
double exposureTime, gain;
GXGetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, &exposureTime);
GXGetFloat(hDevice, GX_FLOAT_GAIN, &gain);
printf("曝光: %.2f us, 增益: %.2f dB\n", exposureTime, gain);
```

### 3.3 GXSetFloat - 设置浮点值

```cpp
GX_STATUS GXSetFloat(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    double dValue                   // [in] 要设置的值
);
```

**功能**: 设置浮点型参数的值

**示例**:
```cpp
// 设置曝光时间为10毫秒（10000微秒）
GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, 10000.0);

// 设置增益为10dB
GXSetFloat(hDevice, GX_FLOAT_GAIN, 10.0);

// 设置帧率为30fps
GXSetFloat(hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, 30.0);
```

### 常用浮点型参数

| 参数ID | 说明 | 单位 |
|--------|------|------|
| `GX_FLOAT_EXPOSURE_TIME` | 曝光时间 | 微秒(us) |
| `GX_FLOAT_GAIN` | 增益 | dB |
| `GX_FLOAT_BLACKLEVEL` | 黑电平 | - |
| `GX_FLOAT_BALANCE_RATIO` | 白平衡比率 | - |
| `GX_FLOAT_GAMMA` | Gamma值 | - |
| `GX_FLOAT_ACQUISITION_FRAME_RATE` | 帧率 | fps |
| `GX_FLOAT_TRIGGER_DELAY` | 触发延迟 | 微秒(us) |

## 4. 枚举型参数控制

### 4.1 GXGetEnumEntryNums - 获取枚举项数量

```cpp
GX_STATUS GXGetEnumEntryNums(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    uint32_t *pnEntryNums           // [out] 枚举项数量
);
```

**功能**: 获取枚举类型参数有多少个选项

**示例**:
```cpp
uint32_t entryNums;
GXGetEnumEntryNums(hDevice, GX_ENUM_PIXEL_FORMAT, &entryNums);
printf("支持 %d 种像素格式\n", entryNums);
```

### 4.2 GXGetEnumDescription - 获取枚举描述

```cpp
GX_STATUS GXGetEnumDescription(
    GX_DEV_HANDLE hDevice,              // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,        // [in] 特性码ID
    GX_ENUM_DESCRIPTION *pEnumDescription, // [out] 枚举描述数组
    size_t *pBufferSize                 // [in,out] 缓冲区大小
);
```

**功能**: 获取枚举类型的所有选项及其描述

**示例**:
```cpp
uint32_t entryNums;
GXGetEnumEntryNums(hDevice, GX_ENUM_PIXEL_FORMAT, &entryNums);

size_t size = entryNums * sizeof(GX_ENUM_DESCRIPTION);
GX_ENUM_DESCRIPTION* pEnumDesc = 
    (GX_ENUM_DESCRIPTION*)malloc(size);

GXGetEnumDescription(hDevice, GX_ENUM_PIXEL_FORMAT, 
    pEnumDesc, &size);

for (uint32_t i = 0; i < entryNums; i++) {
    printf("%d: %s (值: %lld)\n", i, 
        pEnumDesc[i].szSymbolic, pEnumDesc[i].nValue);
}

free(pEnumDesc);
```

### 4.3 GXGetEnum - 获取枚举值

```cpp
GX_STATUS GXGetEnum(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    int64_t *pnValue                // [out] 返回枚举值
);
```

**功能**: 获取枚举型参数的当前值

**示例**:
```cpp
int64_t pixelFormat;
GXGetEnum(hDevice, GX_ENUM_PIXEL_FORMAT, &pixelFormat);

if (pixelFormat == GX_PIXEL_FORMAT_MONO8) {
    printf("当前像素格式: Mono8\n");
} else if (pixelFormat == GX_PIXEL_FORMAT_BAYER_RG8) {
    printf("当前像素格式: Bayer RG8\n");
}
```

### 4.4 GXSetEnum - 设置枚举值

```cpp
GX_STATUS GXSetEnum(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    int64_t nValue                  // [in] 要设置的枚举值
);
```

**功能**: 设置枚举型参数的值

**示例**:
```cpp
// 设置像素格式为Mono8
GXSetEnum(hDevice, GX_ENUM_PIXEL_FORMAT, GX_PIXEL_FORMAT_MONO8);

// 设置采集模式为连续模式
GXSetEnum(hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);

// 打开触发模式
GXSetEnum(hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);

// 设置触发源为软件触发
GXSetEnum(hDevice, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_SOFTWARE);
```

### 常用枚举型参数

| 参数ID | 说明 | 常用值 |
|--------|------|--------|
| `GX_ENUM_PIXEL_FORMAT` | 像素格式 | MONO8, BAYER_RG8, RGB8 |
| `GX_ENUM_ACQUISITION_MODE` | 采集模式 | SINGLE_FRAME, CONTINUOUS |
| `GX_ENUM_TRIGGER_MODE` | 触发模式 | OFF, ON |
| `GX_ENUM_TRIGGER_SOURCE` | 触发源 | SOFTWARE, LINE0-3 |
| `GX_ENUM_TRIGGER_ACTIVATION` | 触发激活方式 | RISINGEDGE, FALLINGEDGE |
| `GX_ENUM_EXPOSURE_MODE` | 曝光模式 | TIMED, TRIGGERWIDTH |
| `GX_ENUM_EXPOSURE_AUTO` | 自动曝光 | OFF, CONTINUOUS, ONCE |
| `GX_ENUM_GAIN_AUTO` | 自动增益 | OFF, CONTINUOUS, ONCE |

## 5. 布尔型参数控制

### 5.1 GXGetBool - 获取布尔值

```cpp
GX_STATUS GXGetBool(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    bool *pbValue                   // [out] 返回布尔值
);
```

**功能**: 获取布尔型参数的当前值

**示例**:
```cpp
bool gammaEnable;
GXGetBool(hDevice, GX_BOOL_GAMMA_ENABLE, &gammaEnable);
printf("Gamma校正: %s\n", gammaEnable ? "开" : "关");
```

### 5.2 GXSetBool - 设置布尔值

```cpp
GX_STATUS GXSetBool(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    bool bValue                     // [in] 要设置的布尔值
);
```

**功能**: 设置布尔型参数的值

**示例**:
```cpp
// 使能Gamma校正
GXSetBool(hDevice, GX_BOOL_GAMMA_ENABLE, true);

// 禁用图像水平翻转
GXSetBool(hDevice, GX_BOOL_REVERSE_X, false);

// 使能垂直翻转
GXSetBool(hDevice, GX_BOOL_REVERSE_Y, true);

// 使能LUT
GXSetBool(hDevice, GX_BOOL_LUT_ENABLE, true);
```

### 常用布尔型参数

| 参数ID | 说明 |
|--------|------|
| `GX_BOOL_GAMMA_ENABLE` | Gamma校正使能 |
| `GX_BOOL_REVERSE_X` | 水平翻转 |
| `GX_BOOL_REVERSE_Y` | 垂直翻转 |
| `GX_BOOL_LUT_ENABLE` | 查找表使能 |
| `GX_BOOL_COLOR_TRANSFORMATION_ENABLE` | 颜色转换使能 |

## 6. 字符串型参数控制

### 6.1 GXGetStringLength - 获取字符串长度

```cpp
GX_STATUS GXGetStringLength(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    size_t *pnSize                  // [out] 返回长度（含'\0'）
);
```

**功能**: 获取字符串参数的当前长度

**示例**:
```cpp
size_t length;
GXGetStringLength(hDevice, GX_STRING_DEVICE_SERIAL_NUMBER, &length);
char* serialNumber = (char*)malloc(length);
```

### 6.2 GXGetStringMaxLength - 获取最大长度

```cpp
GX_STATUS GXGetStringMaxLength(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    size_t *pnSize                  // [out] 返回最大长度
);
```

**功能**: 获取字符串参数的最大可能长度

### 6.3 GXGetString - 获取字符串

```cpp
GX_STATUS GXGetString(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    char *pszContent,               // [out] 返回字符串
    size_t *pnSize                  // [in,out] 缓冲区大小
);
```

**功能**: 获取字符串型参数的内容

**示例**:
```cpp
char vendorName[128];
char modelName[128];
char serialNumber[128];
size_t size;

size = 128;
GXGetString(hDevice, GX_STRING_DEVICE_VENDOR_NAME, 
    vendorName, &size);

size = 128;
GXGetString(hDevice, GX_STRING_DEVICE_MODEL_NAME, 
    modelName, &size);

size = 128;
GXGetString(hDevice, GX_STRING_DEVICE_SERIAL_NUMBER, 
    serialNumber, &size);

printf("厂商: %s\n", vendorName);
printf("型号: %s\n", modelName);
printf("序列号: %s\n", serialNumber);
```

### 6.4 GXSetString - 设置字符串

```cpp
GX_STATUS GXSetString(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    char *pszContent                // [in] 字符串内容（以'\0'结尾）
);
```

**功能**: 设置字符串型参数的内容

**示例**:
```cpp
// 设置用户自定义ID
GXSetString(hDevice, GX_STRING_DEVICE_USERID, "MyCamera01");
```

### 常用字符串型参数

| 参数ID | 说明 | 权限 |
|--------|------|------|
| `GX_STRING_DEVICE_VENDOR_NAME` | 厂商名称 | 只读 |
| `GX_STRING_DEVICE_MODEL_NAME` | 型号名称 | 只读 |
| `GX_STRING_DEVICE_SERIAL_NUMBER` | 序列号 | 只读 |
| `GX_STRING_DEVICE_VERSION` | 设备版本 | 只读 |
| `GX_STRING_DEVICE_FIRMWARE_VERSION` | 固件版本 | 只读 |
| `GX_STRING_DEVICE_USERID` | 用户ID | 读写 |

## 7. 缓冲区型参数控制

### 7.1 GXGetBufferLength - 获取缓冲区长度

```cpp
GX_STATUS GXGetBufferLength(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    size_t *pnSize                  // [out] 返回长度（字节）
);
```

**功能**: 获取块数据的长度

### 7.2 GXGetBuffer - 获取缓冲区数据

```cpp
GX_STATUS GXGetBuffer(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    uint8_t *pBuffer,               // [out] 数据缓冲区
    size_t *pnSize                  // [in,out] 缓冲区大小
);
```

**功能**: 获取块数据内容

**示例**:
```cpp
size_t lutSize;
GXGetBufferLength(hDevice, GX_BUFFER_LUT_VALUEALL, &lutSize);

uint8_t* lutData = (uint8_t*)malloc(lutSize);
GXGetBuffer(hDevice, GX_BUFFER_LUT_VALUEALL, lutData, &lutSize);

// 处理LUT数据...

free(lutData);
```

### 7.3 GXSetBuffer - 设置缓冲区数据

```cpp
GX_STATUS GXSetBuffer(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID,    // [in] 特性码ID
    uint8_t *pBuffer,               // [in] 数据缓冲区
    size_t nSize                    // [in] 数据大小
);
```

**功能**: 设置块数据内容

**示例**:
```cpp
// 设置LUT数据
uint8_t lutData[256];
for (int i = 0; i < 256; i++) {
    lutData[i] = i;  // 线性LUT
}
GXSetBuffer(hDevice, GX_BUFFER_LUT_VALUEALL, lutData, 256);
```

### 常用缓冲区型参数

| 参数ID | 说明 |
|--------|------|
| `GX_BUFFER_LUT_VALUEALL` | LUT全部数据 |
| `GX_BUFFER_FRAME_INFORMATION` | 帧信息 |
| `GX_BUFFER_USER_DATA` | 用户数据 |

## 8. 命令型参数

### 8.1 GXSendCommand - 发送命令

```cpp
GX_STATUS GXSendCommand(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    GX_FEATURE_ID_CMD featureID     // [in] 命令ID
);
```

**功能**: 发送控制命令到相机

**示例**:
```cpp
// 发送软件触发命令
GXSendCommand(hDevice, GX_COMMAND_TRIGGER_SOFTWARE);

// 开始采集
GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);

// 停止采集
GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_STOP);

// 加载用户参数集
GXSendCommand(hDevice, GX_COMMAND_USER_SET_LOAD);

// 保存用户参数集
GXSendCommand(hDevice, GX_COMMAND_USER_SET_SAVE);

// 复位时间戳
GXSendCommand(hDevice, GX_COMMAND_TIMESTAMP_RESET);
```

### 常用命令

| 命令ID | 说明 |
|--------|------|
| `GX_COMMAND_ACQUISITION_START` | 开始采集 |
| `GX_COMMAND_ACQUISITION_STOP` | 停止采集 |
| `GX_COMMAND_TRIGGER_SOFTWARE` | 软件触发 |
| `GX_COMMAND_USER_SET_LOAD` | 加载参数集 |
| `GX_COMMAND_USER_SET_SAVE` | 保存参数集 |
| `GX_COMMAND_DEVICE_RESET` | 设备复位 |
| `GX_COMMAND_TIMESTAMP_RESET` | 时间戳复位 |
| `GX_COMMAND_TIMESTAMP_LATCH` | 锁存时间戳 |

## 9. 寄存器读写

### 9.1 GXReadRemoteDevicePort - 读寄存器

```cpp
GX_STATUS GXReadRemoteDevicePort(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    uint64_t ui64Address,           // [in] 寄存器地址
    void *pBuffer,                  // [out] 返回数据
    size_t *piSize                  // [in,out] 数据大小
);
```

**功能**: 读取指定寄存器的值

**使用场景**: 高级用户直接访问底层寄存器

### 9.2 GXWriteRemoteDevicePort - 写寄存器

```cpp
GX_STATUS GXWriteRemoteDevicePort(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    uint64_t ui64Address,           // [in] 寄存器地址
    const void *pBuffer,            // [in] 要写入的数据
    size_t *piSize                  // [in,out] 数据大小
);
```

**功能**: 向指定寄存器写入值

**警告**: 不正确的寄存器操作可能导致相机工作异常

## 10. 完整参数配置示例

```cpp
// 设置图像格式和ROI
GXSetEnum(hDevice, GX_ENUM_PIXEL_FORMAT, GX_PIXEL_FORMAT_MONO8);
GXSetInt(hDevice, GX_INT_WIDTH, 1920);
GXSetInt(hDevice, GX_INT_HEIGHT, 1080);
GXSetInt(hDevice, GX_INT_OFFSET_X, 0);
GXSetInt(hDevice, GX_INT_OFFSET_Y, 0);

// 设置曝光和增益
GXSetEnum(hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, 10000.0);  // 10ms
GXSetEnum(hDevice, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
GXSetFloat(hDevice, GX_FLOAT_GAIN, 0.0);  // 0dB

// 设置触发模式
GXSetEnum(hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
GXSetEnum(hDevice, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_SOFTWARE);
GXSetEnum(hDevice, GX_ENUM_TRIGGER_ACTIVATION, GX_TRIGGER_ACTIVATION_RISINGEDGE);

// 使能图像处理功能
GXSetBool(hDevice, GX_BOOL_GAMMA_ENABLE, true);
GXSetEnum(hDevice, GX_ENUM_GAMMA_MODE, GX_GAMMA_SELECTOR_SRGB);
```

## 相关文档

- [SDK分析_04_设备管理接口](./SDK分析_04_设备管理接口.md)
- [SDK分析_06_图像采集接口](./SDK分析_06_图像采集接口.md)
