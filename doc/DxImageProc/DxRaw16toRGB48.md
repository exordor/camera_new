# DxRaw16toRGB48

用途
- 将 Raw16（单通道 16-bit）转换为 RGB48（每通道 16-bit）的彩色图像，需指定 Bayer 转换算法与排列。

参数
- `void *pInputBuffer`：输入 Raw16（size = width*height*2）。
- `void *pOutputBuffer`：输出 RGB48（size = width*height*3*2）。
- `VxUint32 nWidth, VxUint32 nHeight`：图像尺寸。
- `DX_ACTUAL_BITS nActualBits`：图像实际位深（10/12/14/16）。
- `DX_BAYER_CONVERT_TYPE cvtype`：去马赛克算法。
- `DX_PIXEL_COLOR_FILTER nBayerType`：Bayer 排列。
- `bool bFlip`：是否翻转输出。

返回值
- `DX_OK` 成功，否则错误码。

注意事项
- 输出缓冲需预分配充足空间。

示例
```c
DxRaw16toRGB48(inRaw16, outRgb48, width, height, DX_ACTUAL_BITS_12, RAW2RGB_ADAPTIVE, BAYERRG, false);
```