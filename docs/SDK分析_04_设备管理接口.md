# Galaxy Camera SDK 分析 - 设备管理接口

## 1. 库初始化和关闭

### 1.1 GXInitLib - 初始化库

```cpp
GX_STATUS GXInitLib();
```

**功能**: 初始化设备库，进行资源申请操作

**返回值**:
- `GX_STATUS_SUCCESS`: 操作成功
- `GX_STATUS_NOT_FOUND_TL`: 找不到传输层库

**使用说明**:
- 必须在使用其他接口前调用（除了GXCloseLib和GXGetLastError）
- 否则返回`GX_STATUS_NOT_INIT_API`错误

**示例**:
```cpp
GX_STATUS status = GXInitLib();
if (status != GX_STATUS_SUCCESS) {
    printf("初始化库失败\n");
    return -1;
}
```

### 1.2 GXCloseLib - 关闭库

```cpp
GX_STATUS GXCloseLib();
```

**功能**: 关闭设备库，释放所有资源

**返回值**:
- `GX_STATUS_SUCCESS`: 操作成功

**使用说明**:
- 程序结束前必须调用
- 与GXInitLib配对使用

**示例**:
```cpp
GXCloseLib();
```

### 1.3 GXGetLibVersion - 获取库版本

```cpp
const char* GXGetLibVersion();
```

**功能**: 获取库的版本号字符串

**返回值**: 版本号字符串指针

**示例**:
```cpp
const char* version = GXGetLibVersion();
printf("SDK版本: %s\n", version);
```

### 1.4 GXGetLastError - 获取最后错误

```cpp
GX_STATUS GXGetLastError(
    GX_STATUS *pErrorCode,     // [out] 返回错误码
    char *pszErrText,          // [out] 返回错误描述
    size_t *pSize              // [in,out] 缓冲区大小
);
```

**功能**: 获取程序最近的错误描述信息

**参数**:
- `pErrorCode`: 返回错误码（可以为NULL）
- `pszErrText`: 返回错误描述文本
- `pSize`: 缓冲区大小（字节）

**返回值**:
- `GX_STATUS_SUCCESS`: 操作成功
- `GX_STATUS_INVALID_PARAMETER`: 参数为NULL
- `GX_STATUS_NEED_MORE_BUFFER`: 缓冲区太小

**示例**:
```cpp
GX_STATUS errorCode;
char errorText[1024];
size_t size = sizeof(errorText);
GXGetLastError(&errorCode, errorText, &size);
printf("错误码: %d, 描述: %s\n", errorCode, errorText);
```

## 2. 设备枚举和信息获取

### 2.1 GXUpdateDeviceList - 枚举子网设备

```cpp
GX_STATUS GXUpdateDeviceList(
    uint32_t *punNumDevices,   // [out] 返回设备数量
    uint32_t nTimeOut          // [in] 超时时间（毫秒）
);
```

**功能**: 枚举当前子网内所有可用设备

**参数**:
- `punNumDevices`: 返回发现的设备数量（不能为NULL）
- `nTimeOut`: 枚举超时时间（毫秒）

**返回值**:
- `GX_STATUS_SUCCESS`: 操作成功
- `GX_STATUS_NOT_INIT_API`: 未调用GXInitLib
- `GX_STATUS_INVALID_PARAMETER`: 指针为NULL

**示例**:
```cpp
uint32_t deviceNum = 0;
GX_STATUS status = GXUpdateDeviceList(&deviceNum, 1000);
if (status == GX_STATUS_SUCCESS) {
    printf("发现 %d 个设备\n", deviceNum);
}
```

### 2.2 GXUpdateAllDeviceList - 枚举全网设备

```cpp
GX_STATUS GXUpdateAllDeviceList(
    uint32_t *punNumDevices,   // [out] 返回设备数量
    uint32_t nTimeOut          // [in] 超时时间（毫秒）
);
```

**功能**: 枚举整个网络中所有可用设备（包括不在同一子网的设备）

**参数**: 与GXUpdateDeviceList相同

**使用说明**: 用于发现不在同一子网的设备

### 2.3 GXGetAllDeviceBaseInfo - 获取所有设备基础信息

```cpp
GX_STATUS GXGetAllDeviceBaseInfo(
    GX_DEVICE_BASE_INFO *pDeviceInfo,  // [out] 设备信息结构指针
    size_t *pBufferSize                // [in,out] 缓冲区大小
);
```

**功能**: 获取所有设备的基础信息

**参数**:
- `pDeviceInfo`: 设备信息数组
- `pBufferSize`: 缓冲区大小（字节）

**返回值**:
- `GX_STATUS_SUCCESS`: 操作成功
- `GX_STATUS_NOT_INIT_API`: 未初始化
- `GX_STATUS_INVALID_PARAMETER`: 参数无效

**示例**:
```cpp
uint32_t deviceNum = 0;
GXUpdateDeviceList(&deviceNum, 1000);

size_t size = deviceNum * sizeof(GX_DEVICE_BASE_INFO);
GX_DEVICE_BASE_INFO* pDeviceInfo = 
    (GX_DEVICE_BASE_INFO*)malloc(size);

GXGetAllDeviceBaseInfo(pDeviceInfo, &size);

for (uint32_t i = 0; i < deviceNum; i++) {
    printf("设备 %d:\n", i+1);
    printf("  厂商: %s\n", pDeviceInfo[i].szVendorName);
    printf("  型号: %s\n", pDeviceInfo[i].szModelName);
    printf("  序列号: %s\n", pDeviceInfo[i].szSN);
}

free(pDeviceInfo);
```

### 2.4 GXGetDeviceIPInfo - 获取设备网络信息

```cpp
GX_STATUS GXGetDeviceIPInfo(
    uint32_t nIndex,                // [in] 设备索引
    GX_DEVICE_IP_INFO *pstDeviceIPInfo  // [out] IP信息结构
);
```

**功能**: 获取指定设备的网络信息

**参数**:
- `nIndex`: 设备索引（从1开始）
- `pstDeviceIPInfo`: 返回IP信息

**返回值**:
- `GX_STATUS_SUCCESS`: 操作成功
- `GX_STATUS_NOT_INIT_API`: 未初始化
- `GX_STATUS_INVALID_PARAMETER`: 索引越界

**示例**:
```cpp
GX_DEVICE_IP_INFO ipInfo;
GXGetDeviceIPInfo(1, &ipInfo);
printf("设备IP: %s\n", ipInfo.szIP);
printf("设备MAC: %s\n", ipInfo.szMAC);
printf("子网掩码: %s\n", ipInfo.szSubNetMask);
```

## 3. 设备打开和关闭

### 3.1 GXOpenDeviceByIndex - 通过索引打开设备

```cpp
GX_STATUS GXOpenDeviceByIndex(
    uint32_t nDeviceIndex,      // [in] 设备索引（从1开始）
    GX_DEV_HANDLE *phDevice     // [out] 返回设备句柄
);
```

**功能**: 通过索引号打开设备

**参数**:
- `nDeviceIndex`: 设备索引（1, 2, 3, ...）
- `phDevice`: 返回的设备句柄

**返回值**:
- `GX_STATUS_SUCCESS`: 操作成功
- `GX_STATUS_NOT_INIT_API`: 未初始化
- `GX_STATUS_INVALID_PARAMETER`: 指针为NULL
- `GX_STATUS_OUT_OF_RANGE`: 索引超出范围

**示例**:
```cpp
GX_DEV_HANDLE hDevice = NULL;
GX_STATUS status = GXOpenDeviceByIndex(1, &hDevice);
if (status == GX_STATUS_SUCCESS) {
    printf("设备打开成功\n");
}
```

### 3.2 GXOpenDevice - 通过特定标识打开设备

```cpp
GX_STATUS GXOpenDevice(
    GX_OPEN_PARAM *pOpenParam,  // [in] 打开参数
    GX_DEV_HANDLE *phDevice     // [out] 返回设备句柄
);
```

**功能**: 通过特定的唯一标识（SN、IP、MAC等）打开设备

**参数**:
- `pOpenParam`: 打开参数结构
- `phDevice`: 返回的设备句柄

**返回值**:
- `GX_STATUS_SUCCESS`: 操作成功
- `GX_STATUS_NOT_INIT_API`: 未初始化
- `GX_STATUS_INVALID_PARAMETER`: 参数为NULL
- `GX_STATUS_NOT_FOUND_DEVICE`: 未找到匹配的设备
- `GX_STATUS_INVALID_ACCESS`: 当前访问模式无法打开

**示例1 - 通过IP打开**:
```cpp
GX_DEV_HANDLE hDevice = NULL;
GX_OPEN_PARAM openParam;
openParam.pszContent = "192.168.1.100";
openParam.openMode = GX_OPEN_IP;
openParam.accessMode = GX_ACCESS_EXCLUSIVE;

GX_STATUS status = GXOpenDevice(&openParam, &hDevice);
```

**示例2 - 通过序列号打开**:
```cpp
GX_DEV_HANDLE hDevice = NULL;
GX_OPEN_PARAM openParam;
openParam.pszContent = "KJ0180090001";
openParam.openMode = GX_OPEN_SN;
openParam.accessMode = GX_ACCESS_EXCLUSIVE;

GX_STATUS status = GXOpenDevice(&openParam, &hDevice);
```

**示例3 - 通过MAC地址打开**:
```cpp
GX_DEV_HANDLE hDevice = NULL;
GX_OPEN_PARAM openParam;
openParam.pszContent = "00-11-1C-XX-XX-XX";
openParam.openMode = GX_OPEN_MAC;
openParam.accessMode = GX_ACCESS_CONTROL;

GX_STATUS status = GXOpenDevice(&openParam, &hDevice);
```

### 3.3 GXCloseDevice - 关闭设备

```cpp
GX_STATUS GXCloseDevice(
    GX_DEV_HANDLE hDevice       // [in] 设备句柄
);
```

**功能**: 关闭指定的设备句柄

**参数**:
- `hDevice`: 通过GXOpenDevice获取的设备句柄

**返回值**:
- `GX_STATUS_SUCCESS`: 操作成功
- `GX_STATUS_NOT_INIT_API`: 未初始化
- `GX_STATUS_INVALID_HANDLE`: 非法句柄或重复关闭

**使用说明**: 关闭已关闭的设备会返回错误

**示例**:
```cpp
GXCloseDevice(hDevice);
hDevice = NULL;  // 避免重复关闭
```

## 4. IP配置接口

### 4.1 GXGetDevicePersistentIpAddress - 获取静态IP

```cpp
GX_STATUS GXGetDevicePersistentIpAddress(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    char *pszIP,                    // [out] IP地址
    size_t *pnIPLength,             // [in,out] IP缓冲区大小
    char *pszSubNetMask,            // [out] 子网掩码
    size_t *pnSubNetMaskLength,     // [in,out] 子网掩码缓冲区大小
    char *pszDefaultGateWay,        // [out] 默认网关
    size_t *pnDefaultGateWayLength  // [in,out] 网关缓冲区大小
);
```

**功能**: 获取设备的静态IP信息

**示例**:
```cpp
char ip[32], mask[32], gateway[32];
size_t ipLen = 32, maskLen = 32, gatewayLen = 32;

GXGetDevicePersistentIpAddress(hDevice, 
    ip, &ipLen,
    mask, &maskLen,
    gateway, &gatewayLen);

printf("静态IP: %s\n", ip);
printf("子网掩码: %s\n", mask);
printf("网关: %s\n", gateway);
```

### 4.2 GXSetDevicePersistentIpAddress - 设置静态IP

```cpp
GX_STATUS GXSetDevicePersistentIpAddress(
    GX_DEV_HANDLE hDevice,          // [in] 设备句柄
    const char *pszIP,              // [in] IP地址
    const char *pszSubNetMask,      // [in] 子网掩码
    const char *pszDefaultGateWay   // [in] 默认网关
);
```

**功能**: 设置设备的静态IP信息

**示例**:
```cpp
GXSetDevicePersistentIpAddress(hDevice,
    "192.168.1.100",
    "255.255.255.0",
    "192.168.1.1");
```

### 4.3 GXGigEIpConfiguration - 配置IP

```cpp
GX_STATUS GXGigEIpConfiguration(
    const char *pszDeviceMacAddress,    // [in] 设备MAC地址
    GX_IP_CONFIGURE_MODE emIpConfigMode, // [in] IP配置模式
    const char *pszIpAddress,           // [in] IP地址
    const char *pszSubnetMask,          // [in] 子网掩码
    const char *pszDefaultGateway,      // [in] 默认网关
    const char *pszUserID               // [in] 用户自定义名称
);
```

**功能**: 配置GigE设备的IP地址

**IP配置模式**:
- `GX_IP_CONFIGURE_DHCP`: DHCP模式
- `GX_IP_CONFIGURE_LLA`: LLA模式
- `GX_IP_CONFIGURE_STATIC_IP`: 静态IP模式
- `GX_IP_CONFIGURE_DEFAULT`: 默认模式

**示例**:
```cpp
GXGigEIpConfiguration(
    "00-11-1C-XX-XX-XX",
    GX_IP_CONFIGURE_STATIC_IP,
    "192.168.1.100",
    "255.255.255.0",
    "192.168.1.1",
    "MyCameraName");
```

### 4.4 GXGigEForceIp - 强制IP

```cpp
GX_STATUS GXGigEForceIp(
    const char *pszDeviceMacAddress,  // [in] 设备MAC地址
    const char *pszIpAddress,         // [in] IP地址
    const char *pszSubnetMask,        // [in] 子网掩码
    const char *pszDefaultGateway     // [in] 默认网关
);
```

**功能**: 强制设置设备IP（临时IP，重启后失效）

**使用场景**: 设备IP与主机不在同一网段时使用

**示例**:
```cpp
GXGigEForceIp(
    "00-11-1C-XX-XX-XX",
    "192.168.1.100",
    "255.255.255.0",
    "192.168.1.1");
```

### 4.5 GXGigEResetDevice - 重启设备

```cpp
GX_STATUS GXGigEResetDevice(
    const char *pszDeviceMacAddress,      // [in] 设备MAC地址
    GX_RESET_DEVICE_MODE ui32FeatureInfo  // [in] 重启模式
);
```

**功能**: 重启或重连设备

**重启模式**:
- `GX_MANUFACTURER_SPECIFIC_RECONNECT = 0x1`: 重连设备
- `GX_MANUFACTURER_SPECIFIC_RESET = 0x2`: 重启设备

**示例**:
```cpp
GXGigEResetDevice("00-11-1C-XX-XX-XX", 
    GX_MANUFACTURER_SPECIFIC_RESET);
```

## 5. 配置文件管理

### 5.1 GXExportConfigFile - 导出配置

```cpp
GX_STATUS GXExportConfigFile(
    GX_DEV_HANDLE hDevice,      // [in] 设备句柄
    const char *pszFilePath     // [in] 配置文件路径
);
```

**功能**: 将当前相机参数导出到配置文件

**示例**:
```cpp
GXExportConfigFile(hDevice, "/home/user/camera_config.txt");
```

### 5.2 GXImportConfigFile - 导入配置

```cpp
GX_STATUS GXImportConfigFile(
    GX_DEV_HANDLE hDevice,      // [in] 设备句柄
    const char *pszFilePath,    // [in] 配置文件路径
    bool bVerify = false        // [in] 是否验证
);
```

**功能**: 从配置文件导入相机参数

**参数**:
- `bVerify`: 如果为true，会读取所有导入的值以验证一致性

**示例**:
```cpp
GXImportConfigFile(hDevice, "/home/user/camera_config.txt", true);
```

## 6. 完整使用示例

```cpp
// 1. 初始化
GXInitLib();

// 2. 枚举设备
uint32_t deviceNum = 0;
GXUpdateDeviceList(&deviceNum, 1000);

// 3. 获取设备信息
GX_DEVICE_BASE_INFO baseInfo;
size_t size = sizeof(GX_DEVICE_BASE_INFO);
GXGetAllDeviceBaseInfo(&baseInfo, &size);

// 4. 打开设备
GX_DEV_HANDLE hDevice = NULL;
GXOpenDeviceByIndex(1, &hDevice);

// 5. 使用设备...
// ...

// 6. 关闭设备
GXCloseDevice(hDevice);

// 7. 关闭库
GXCloseLib();
```

## 相关文档

- [SDK分析_05_参数控制接口](./SDK分析_05_参数控制接口.md)
- [SDK分析_04_图像采集接口](./SDK分析_04_图像采集接口.md)
