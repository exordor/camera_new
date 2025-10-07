# DxRaw8toRGB24

用途
- 将 Raw8（Bayer）图像去马赛克并输出为 24 位 RGB（默认通道顺序）。

参数
- `void *pInputBuffer`：输入 Raw8 缓冲区，大小 `nWidth * nHeight`。
- `void *pOutputBuffer`：输出缓冲区，需 `nWidth * nHeight * 3` 字节。
- `VxUint32 nWidth`、`VxUint32 nHeight`：图像尺寸。
- `DX_BAYER_CONVERT_TYPE cvtype`：去马赛克算法。
- `DX_PIXEL_COLOR_FILTER nBayerType`：Bayer 排列类型。
- `bool bFlip`：是否翻转输出图像。

返回值
- 成功返回 `DX_OK`，否则返回错误码。

注意事项与边界情况
- 同 `DxRaw8toRGB24Ex`，但不提供通道顺序选择。若需 BGR，请使用 Ex 版本或后处理交换通道。

示例调用
```c
unsigned char *in = ...; // width * height
unsigned char *out = malloc(width * height * 3);
VxInt32 ret = DxRaw8toRGB24(in, out, width, height, RAW2RGB_NEIGHBOUR, BAYERGB, true);
```