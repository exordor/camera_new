# DxGetLut

用途
- 计算 8-bit 图像处理使用的综合查找表（对比度、伽玛、亮度联合 LUT）。

参数
- `VxInt32 nContrastParam`：对比度参数（`-50~100`）。
- `double dGamma`：伽玛参数（`0.1~10`）。
- `VxInt32 nLightness`：亮度参数（`-150~150`）。
- `VxUint8 *pLut`：输出 LUT 缓冲（调用者分配）。
- `VxUint16 *pLutLength`：输入/输出 LUT 长度。

返回值
- `DX_OK` 成功，否则错误码。

示例
```c
VxUint8 lut[256];
VxUint16 len = 256;
DxGetLut(10, 2.2, 5, lut, &len);
```