# DxImageImprovment

用途
- 对 RGB24 图像进行图像质量增强（颜色校正、对比度、伽玛等），利用色彩校正参数与 LUT。

参数
- `void *pInputBuffer`、`void *pOutputBuffer`：输入/输出 RGB24 缓冲。
- `VxUint32 nWidth, VxUint32 nHeight`：图像尺寸。
- `VxInt64 nColorCorrectionParam`：从相机获取的颜色校正参数地址/值。
- `void *pContrastLut`：对比度查找表。
- `void *pGammaLut`：伽玛查找表。

返回值
- `DX_OK` 成功，否则错误码。

注意事项
- 确保 `pContrastLut` 与 `pGammaLut` 已通过 `DxGetContrastLut` 与 `DxGetGammatLut` 正确生成。

示例
```c
DxImageImprovment(inRgb, outRgb, width, height, colorParam, contrastLut, gammaLut);
```