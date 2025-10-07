# DxImageImprovmentEx

用途
- 对 RGB24/BGR24 图像进行增强，并可指定输出通道顺序（RGB 或 BGR）。

参数
- `void *pInputBuffer`, `void *pOutputBuffer`：输入/输出缓冲。
- `VxUint32 nWidth, VxUint32 nHeight`：图像尺寸。
- `VxInt64 nColorCorrectionParam`：颜色校正参数。
- `void *pContrastLut`, `void *pGammaLut`：查找表。
- `DX_RGB_CHANNEL_ORDER emChannelOrder`：输出通道顺序。

返回值
- `DX_OK` 成功，否则错误码。

示例
```c
DxImageImprovmentEx(inBuf, outBuf, width, height, colorParam, contrastLut, gammaLut, DX_ORDER_BGR);
```