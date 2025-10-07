# Galaxy Camera SDK 分析 - 概述

## 文档信息
- **SDK名称**: GxIAPI (Galaxy Interface API)
- **版本**: 1.1.1905.9141
- **日期**: 2019-5-14
- **开发商**: Software Department

## SDK简介

GxIAPI是大恒图像（Galaxy）相机的SDK开发接口，提供了完整的相机控制和图像采集功能。该SDK采用C语言接口设计，支持跨平台使用（Windows和Linux）。

## SDK架构层次

SDK采用分层架构设计，包含以下几个层次：

### 1. **远程设备层** (Remote Device Feature)
- 最核心的控制层
- 直接控制相机硬件参数
- 包含设备信息、图像格式、传输层、采集触发、数字IO、模拟控制等

### 2. **传输层** (TL - Transport Layer)
- 负责底层数据传输
- 处理网络通信（GigE Vision）
- 管理USB通信（USB3 Vision, USB2）

### 3. **接口层** (IF - Interface Layer)
- 网卡接口管理
- 设备枚举和发现

### 4. **设备层** (DEV - Device Layer)
- 设备句柄管理
- 命令超时和重试控制

### 5. **数据流层** (DS - DataStream Layer)
- 图像数据流管理
- 缓冲区管理
- 重传机制控制

## 主要功能模块

### 1. 设备管理
- 设备枚举和发现
- 设备打开/关闭
- 设备配置导入/导出

### 2. 图像采集
- 连续采集模式
- 单帧采集模式
- 多帧采集模式
- 触发采集模式

### 3. 参数控制
- 曝光时间控制
- 增益控制
- 白平衡控制
- ROI（感兴趣区域）设置

### 4. 图像处理
- Gamma校正
- 颜色转换
- 查找表（LUT）
- 坏点校正

### 5. 事件机制
- 设备离线事件
- 帧采集事件
- 参数更新回调

## 支持的设备类型

```cpp
typedef enum GX_DEVICE_CLASS_LIST {
    GX_DEVICE_CLASS_UNKNOWN = 0,    // 未知设备
    GX_DEVICE_CLASS_USB2 = 1,       // USB2.0 Vision设备
    GX_DEVICE_CLASS_GEV = 2,        // GigE Vision设备
    GX_DEVICE_CLASS_U3V = 3,        // USB3 Vision设备
    GX_DEVICE_CLASS_SMART = 4,      // 智能相机设备
} GX_DEVICE_CLASS_LIST;
```

## SDK使用流程

```
1. GXInitLib()              // 初始化库
2. GXUpdateDeviceList()     // 枚举设备
3. GXOpenDevice()           // 打开设备
4. 设置相机参数             // 配置采集参数
5. GXStreamOn()            // 开始采集
6. 获取图像数据             // 采集图像
7. GXStreamOff()           // 停止采集
8. GXCloseDevice()         // 关闭设备
9. GXCloseLib()            // 关闭库
```

## 关键数据结构

### 设备信息
- `GX_DEVICE_BASE_INFO`: 基本设备信息
- `GX_DEVICE_IP_INFO`: 网络设备信息

### 图像数据
- `GX_FRAME_DATA`: 图像帧数据
- `GX_FRAME_BUFFER`: 图像缓冲区
- `GX_FRAME_CALLBACK_PARAM`: 回调函数参数

### 参数范围
- `GX_INT_RANGE`: 整型参数范围
- `GX_FLOAT_RANGE`: 浮点型参数范围
- `GX_ENUM_DESCRIPTION`: 枚举参数描述

## 错误处理

SDK提供完善的错误码机制，所有函数返回`GX_STATUS`类型的错误码：

- `GX_STATUS_SUCCESS = 0`: 成功
- `GX_STATUS_ERROR = -1`: 内部错误
- `GX_STATUS_NOT_FOUND_TL = -2`: 未找到传输层库
- `GX_STATUS_NOT_FOUND_DEVICE = -3`: 未找到设备
- `GX_STATUS_TIMEOUT = -14`: 超时错误
- 等等...

## 下一步阅读

- [02_数据类型和枚举定义](./SDK分析_02_数据类型和枚举定义.md)
- [03_设备管理接口](./SDK分析_03_设备管理接口.md)
- [04_图像采集接口](./SDK分析_04_图像采集接口.md)
- [05_参数控制接口](./SDK分析_05_参数控制接口.md)
- [06_事件回调机制](./SDK分析_06_事件回调机制.md)
