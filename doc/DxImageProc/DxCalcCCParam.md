# DxCalcCCParam

用途
- 计算颜色校正（Color Correction）参数数组，供图像增强使用。

参数
- `VxInt64 nColorCorrectionParam`：相机提供的颜色校正参数地址或值。
- `VxInt16 nSaturation`：饱和度（`0~128`）。
- `VxInt16 *parrCC`：输出数组地址（调用者分配，大小为 `sizeof(VxInt16)*9`）。
- `VxUint8 nLength`：输出数组长度（字节）。

返回值
- `DX_OK` 成功，否则错误码。

示例
```c
VxInt16 cc[9];
DxCalcCCParam(colorParam, 80, cc, sizeof(cc));
```