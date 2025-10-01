# DxGetContrastLut

用途
- 计算对比度查找表（LUT）用于 RGB24 图像的对比度调整。

参数
- `int nContrastParam`：对比度参数，范围 `-50~100`。
- `void *pContrastLut`：输出 LUT 缓冲地址（调用方分配）。
- `int *pLutLength`：返回 LUT 长度（字节）。

返回值
- `DX_OK` 成功，否则错误码。

注意事项
- 调用此函数后 `pContrastLut` 可传入到 `DxImageImprovment` 等函数中进行加速处理。

示例
```c
unsigned char lut[256];
int len;
DxGetContrastLut(20, lut, &len);
```