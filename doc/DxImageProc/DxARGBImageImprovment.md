# DxARGBImageImprovment

用途
- 对 ARGB32 图像执行图像质量增强（与 `DxImageImprovment` 类似，但针对 ARGB32）。

参数
- `void *pInputBuffer`, `void *pOutputBuffer`：ARGB32 缓冲。
- `VxUint32 nWidth, VxUint32 nHeight`：图像尺寸。
- `VxInt64 nColorCorrectionParam`：颜色校正参数。
- `void *pContrastLut`, `void *pGammaLut`：查找表。

返回值
- `DX_OK` 成功，否则错误码。

示例
```c
DxARGBImageImprovment(inArgb, outArgb, width, height, colorParam, contrastLut, gammaLut);
```