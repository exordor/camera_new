# Galaxy Camera SDK 分析 - 数据类型和枚举定义

## 1. 基础数据类型定义

SDK使用标准C语言的整数类型定义：

```cpp
int8_t, int16_t, int32_t, int64_t       // 有符号整型
uint8_t, uint16_t, uint32_t, uint64_t   // 无符号整型
```

## 2. 状态码枚举 (GX_STATUS_LIST)

### 2.1 成功状态
```cpp
GX_STATUS_SUCCESS = 0                    // 操作成功
```

### 2.2 错误状态
```cpp
GX_STATUS_ERROR = -1                     // 内部未知错误
GX_STATUS_NOT_FOUND_TL = -2             // 未找到传输层库
GX_STATUS_NOT_FOUND_DEVICE = -3         // 未找到设备
GX_STATUS_OFFLINE = -4                   // 设备离线
GX_STATUS_INVALID_PARAMETER = -5         // 参数无效（空指针、格式错误等）
GX_STATUS_INVALID_HANDLE = -6            // 句柄无效
GX_STATUS_INVALID_CALL = -7              // 接口调用逻辑错误
GX_STATUS_INVALID_ACCESS = -8            // 当前不可访问或设备访问模式错误
GX_STATUS_NEED_MORE_BUFFER = -9          // 缓冲区不足
GX_STATUS_ERROR_TYPE = -10               // FeatureID类型错误
GX_STATUS_OUT_OF_RANGE = -11             // 参数超出范围
GX_STATUS_NOT_IMPLEMENTED = -12          // 功能未实现
GX_STATUS_NOT_INIT_API = -13             // 未调用初始化接口
GX_STATUS_TIMEOUT = -14                  // 超时错误
```

## 3. 帧状态枚举 (GX_FRAME_STATUS_LIST)

```cpp
GX_FRAME_STATUS_SUCCESS = 0              // 正常帧
GX_FRAME_STATUS_INCOMPLETE = -1          // 残帧
GX_FRAME_STATUS_INVALID_IMAGE_INFO = -2  // 图像信息错误帧
```

## 4. 设备类型枚举 (GX_DEVICE_CLASS_LIST)

```cpp
GX_DEVICE_CLASS_UNKNOWN = 0              // 未知设备类型
GX_DEVICE_CLASS_USB2 = 1                 // USB2.0 Vision设备
GX_DEVICE_CLASS_GEV = 2                  // GigE Vision设备
GX_DEVICE_CLASS_U3V = 3                  // USB3 Vision设备
GX_DEVICE_CLASS_SMART = 4                // 智能相机设备
```

## 5. 特性掩码 (GX_FEATURE_MASK)

```cpp
GX_FEATURE_TYPE_MASK = 0xF0000000        // 特性类型掩码
GX_FEATURE_LEVEL_MASK = 0x0F000000       // 特性级别掩码
```

## 6. 特性类型 (GX_FEATURE_TYPE)

```cpp
GX_FEATURE_INT = 0x10000000              // 整型
GX_FEATURE_FLOAT = 0X20000000            // 浮点型
GX_FEATURE_ENUM = 0x30000000             // 枚举型
GX_FEATURE_BOOL = 0x40000000             // 布尔型
GX_FEATURE_STRING = 0x50000000           // 字符串型
GX_FEATURE_BUFFER = 0x60000000           // 块数据型
GX_FEATURE_COMMAND = 0x70000000          // 命令型
```

## 7. 特性级别 (GX_FEATURE_LEVEL)

```cpp
GX_FEATURE_LEVEL_REMOTE_DEV = 0x00000000 // 远程设备层
GX_FEATURE_LEVEL_TL = 0x01000000         // 传输层
GX_FEATURE_LEVEL_IF = 0x02000000         // 接口层
GX_FEATURE_LEVEL_DEV = 0x03000000        // 设备层
GX_FEATURE_LEVEL_DS = 0x04000000         // 数据流层
```

## 8. 设备访问模式 (GX_ACCESS_MODE)

```cpp
GX_ACCESS_READONLY = 2                   // 只读模式打开设备
GX_ACCESS_CONTROL = 3                    // 可控模式打开设备
GX_ACCESS_EXCLUSIVE = 4                  // 独占模式打开设备
```

## 9. 设备访问状态 (GX_ACCESS_STATUS)

```cpp
GX_ACCESS_STATUS_UNKNOWN = 0             // 设备当前状态未知
GX_ACCESS_STATUS_READWRITE = 1           // 设备当前支持读写
GX_ACCESS_STATUS_READONLY = 2            // 设备当前只支持读
GX_ACCESS_STATUS_NOACCESS = 3            // 设备当前不可读写
```

## 10. 设备打开模式 (GX_OPEN_MODE)

```cpp
GX_OPEN_SN = 0                           // 通过序列号打开
GX_OPEN_IP = 1                           // 通过IP地址打开
GX_OPEN_MAC = 2                          // 通过MAC地址打开
GX_OPEN_INDEX = 3                        // 通过索引打开（1, 2, 3, 4...）
GX_OPEN_USERID = 4                       // 通过用户自定义ID打开
```

## 11. IP配置模式 (GX_IP_CONFIGURE_MODE_LIST)

```cpp
GX_IP_CONFIGURE_DHCP = 0x6               // 通过DHCP服务器分配IP
GX_IP_CONFIGURE_LLA = 0x4                // 通过LLA模式分配IP
GX_IP_CONFIGURE_STATIC_IP = 0x5          // 静态IP模式配置IP
GX_IP_CONFIGURE_DEFAULT = 0x7            // 默认模式配置IP
```

## 12. 像素格式相关枚举

### 12.1 像素大小 (GX_PIXEL_SIZE_ENTRY)
```cpp
GX_PIXEL_SIZE_BPP8 = 8                   // 8位
GX_PIXEL_SIZE_BPP10 = 10                 // 10位
GX_PIXEL_SIZE_BPP12 = 12                 // 12位
GX_PIXEL_SIZE_BPP16 = 16                 // 16位
GX_PIXEL_SIZE_BPP24 = 24                 // 24位
GX_PIXEL_SIZE_BPP30 = 30                 // 30位
GX_PIXEL_SIZE_BPP32 = 32                 // 32位
GX_PIXEL_SIZE_BPP36 = 36                 // 36位
GX_PIXEL_SIZE_BPP48 = 48                 // 48位
GX_PIXEL_SIZE_BPP64 = 64                 // 64位
```

### 12.2 像素颜色滤波器 (GX_PIXEL_COLOR_FILTER_ENTRY)
```cpp
GX_COLOR_FILTER_NONE = 0                 // 无
GX_COLOR_FILTER_BAYER_RG = 1             // RG格式
GX_COLOR_FILTER_BAYER_GB = 2             // GB格式
GX_COLOR_FILTER_BAYER_GR = 3             // GR格式
GX_COLOR_FILTER_BAYER_BG = 4             // BG格式
```

### 12.3 像素格式定义
```cpp
#define GX_PIXEL_MONO   (0x01000000)     // 单色
#define GX_PIXEL_COLOR  (0x02000000)     // 彩色

#define GX_PIXEL_8BIT   (0x00080000)     // 8位
#define GX_PIXEL_10BIT  (0x000A0000)     // 10位
#define GX_PIXEL_12BIT  (0x000C0000)     // 12位
#define GX_PIXEL_16BIT  (0x00100000)     // 16位
#define GX_PIXEL_24BIT  (0x00180000)     // 24位
#define GX_PIXEL_30BIT  (0x001E0000)     // 30位
#define GX_PIXEL_32BIT  (0x00200000)     // 32位
#define GX_PIXEL_36BIT  (0x00240000)     // 36位
#define GX_PIXEL_48BIT  (0x00300000)     // 48位
#define GX_PIXEL_64BIT  (0x00400000)     // 64位
```

### 12.4 像素格式枚举 (GX_PIXEL_FORMAT_ENTRY)
```cpp
// 单色格式
GX_PIXEL_FORMAT_MONO8                    // 8位单色
GX_PIXEL_FORMAT_MONO10                   // 10位单色
GX_PIXEL_FORMAT_MONO12                   // 12位单色
GX_PIXEL_FORMAT_MONO16                   // 16位单色

// Bayer格式（需要插值转换为RGB）
GX_PIXEL_FORMAT_BAYER_GR8                // 8位Bayer GR
GX_PIXEL_FORMAT_BAYER_RG8                // 8位Bayer RG
GX_PIXEL_FORMAT_BAYER_GB8                // 8位Bayer GB
GX_PIXEL_FORMAT_BAYER_BG8                // 8位Bayer BG
GX_PIXEL_FORMAT_BAYER_GR10               // 10位Bayer GR
GX_PIXEL_FORMAT_BAYER_RG10               // 10位Bayer RG
// ... 更多Bayer格式

// RGB平面格式
GX_PIXEL_FORMAT_RGB8_PLANAR              // 8位RGB平面
GX_PIXEL_FORMAT_RGB10_PLANAR             // 10位RGB平面
GX_PIXEL_FORMAT_RGB12_PLANAR             // 12位RGB平面
GX_PIXEL_FORMAT_RGB16_PLANAR             // 16位RGB平面
```

## 13. 采集模式枚举 (GX_ACQUISITION_MODE_ENTRY)

```cpp
GX_ACQ_MODE_SINGLE_FRAME = 0             // 单帧模式
GX_ACQ_MODE_MULITI_FRAME = 1             // 多帧模式
GX_ACQ_MODE_CONTINUOUS = 2               // 连续模式
```

## 14. 触发相关枚举

### 14.1 触发模式 (GX_TRIGGER_MODE_ENTRY)
```cpp
GX_TRIGGER_MODE_OFF = 0                  // 关闭触发模式
GX_TRIGGER_MODE_ON = 1                   // 打开触发模式
```

### 14.2 触发源 (GX_TRIGGER_SOURCE_ENTRY)
```cpp
GX_TRIGGER_SOURCE_SOFTWARE = 0           // 软件触发
GX_TRIGGER_SOURCE_LINE0 = 1              // 触发源0
GX_TRIGGER_SOURCE_LINE1 = 2              // 触发源1
GX_TRIGGER_SOURCE_LINE2 = 3              // 触发源2
GX_TRIGGER_SOURCE_LINE3 = 4              // 触发源3
```

### 14.3 触发激活方式 (GX_TRIGGER_ACTIVATION_ENTRY)
```cpp
GX_TRIGGER_ACTIVATION_FALLINGEDGE = 0    // 下降沿触发
GX_TRIGGER_ACTIVATION_RISINGEDGE = 1     // 上升沿触发
```

### 14.4 触发开关 (GX_TRIGGER_SWITCH_ENTRY)
```cpp
GX_TRIGGER_SWITCH_OFF = 0                // 关闭外部触发
GX_TRIGGER_SWITCH_ON = 1                 // 打开外部触发
```

### 14.5 触发选择器 (GX_TRIGGER_SELECTOR_ENTRY)
```cpp
GX_ENUM_TRIGGER_SELECTOR_FRAME_START = 1          // 捕获单帧
GX_ENUM_TRIGGER_SELECTOR_FRAME_BURST_START = 2    // 高速连拍
```

## 15. 曝光相关枚举

### 15.1 曝光模式 (GX_EXPOSURE_MODE_ENTRY)
```cpp
GX_EXPOSURE_MODE_TIMED = 1               // 通过曝光时间寄存器控制
GX_EXPOSURE_MODE_TRIGGERWIDTH = 2        // 通过触发信号宽度控制
```

### 15.2 自动曝光模式 (GX_EXPOSURE_AUTO_ENTRY)
```cpp
GX_EXPOSURE_AUTO_OFF = 0                 // 关闭自动曝光
GX_EXPOSURE_AUTO_CONTINUOUS = 1          // 连续自动曝光
GX_EXPOSURE_AUTO_ONCE = 2                // 单次自动曝光
```

## 16. 增益相关枚举

### 16.1 自动增益模式 (GX_GAIN_AUTO_ENTRY)
```cpp
GX_GAIN_AUTO_OFF = 0                     // 关闭自动增益
GX_GAIN_AUTO_CONTINUOUS = 1              // 连续自动增益
GX_GAIN_AUTO_ONCE = 2                    // 单次自动增益
```

### 16.2 增益选择器 (GX_GAIN_SELECTOR_ENTRY)
```cpp
GX_GAIN_SELECTOR_ALL = 0                 // 所有增益通道
GX_GAIN_SELECTOR_RED = 1                 // 红色通道增益
GX_GAIN_SELECTOR_GREEN = 2               // 绿色通道增益
GX_GAIN_SELECTOR_BLUE = 3                // 蓝色通道增益
```

## 17. 白平衡相关枚举

### 17.1 自动白平衡 (GX_BALANCE_WHITE_AUTO_ENTRY)
```cpp
GX_BALANCE_WHITE_AUTO_OFF = 0            // 关闭自动白平衡
GX_BALANCE_WHITE_AUTO_CONTINUOUS = 1     // 连续自动白平衡
GX_BALANCE_WHITE_AUTO_ONCE = 2           // 单次自动白平衡
```

### 17.2 白平衡比率选择器 (GX_BALANCE_RATIO_SELECTOR_ENTRY)
```cpp
GX_BALANCE_RATIO_SELECTOR_RED = 0        // 红色通道
GX_BALANCE_RATIO_SELECTOR_GREEN = 1      // 绿色通道
GX_BALANCE_RATIO_SELECTOR_BLUE = 2       // 蓝色通道
```

### 17.3 白平衡光源 (GX_AWB_LAMP_HOUSE_ENTRY)
```cpp
GX_AWB_LAMP_HOUSE_ADAPTIVE = 0           // 自适应光源
GX_AWB_LAMP_HOUSE_D65 = 1                // 色温6500k
GX_AWB_LAMP_HOUSE_FLUORESCENCE = 2       // 荧光灯
GX_AWB_LAMP_HOUSE_INCANDESCENT = 3       // 白炽灯
GX_AWB_LAMP_HOUSE_D75 = 4                // 色温7500k
GX_AWB_LAMP_HOUSE_D50 = 5                // 色温5000k
GX_AWB_LAMP_HOUSE_U30 = 6                // 色温3000k
```

## 18. 图像处理相关枚举

### 18.1 Gamma模式 (GX_GAMMA_MODE_ENTRY)
```cpp
GX_GAMMA_SELECTOR_SRGB = 0               // 默认Gamma校正
GX_GAMMA_SELECTOR_USER = 1               // 用户自定义Gamma校正
```

### 18.2 颜色校正 (GX_COLOR_CORRECT_ENTRY)
```cpp
GX_COLOR_CORRECT_OFF = 0                 // 关闭颜色校正
GX_COLOR_CORRECT_ON = 1                  // 打开颜色校正
```

### 18.3 坏点校正 (GX_DEAD_PIXEL_CORRECT_ENTRY)
```cpp
GX_DEAD_PIXEL_CORRECT_OFF = 0            // 关闭坏点校正
GX_DEAD_PIXEL_CORRECT_ON = 1             // 打开坏点校正
```

### 18.4 锐化模式 (GX_SHARPNESS_MODE_ENTRY)
```cpp
GX_SHARPNESS_MODE_OFF = 0                // 关闭锐化
GX_SHARPNESS_MODE_ON = 1                 // 打开锐化
```

## 使用说明

这些枚举类型在SDK中广泛使用，用于：
1. 设置相机参数
2. 查询相机状态
3. 配置采集模式
4. 处理图像数据

通过组合使用这些枚举类型，可以实现对相机的完整控制。
