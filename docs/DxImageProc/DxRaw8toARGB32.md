# DxRaw8toARGB32

用途
- 将 Raw8 转换为 ARGB32（带 Alpha 通道）的图像，适用于 Android surface 等需要 stride 的场景。

参数
- `void *pInputBuffer`：输入 Raw8 缓冲，大小 `width*height`。
- `void *pOutputBuffer`：输出 ARGB32 缓冲，大小 `width*height*4`（或根据 stride 有变化）。
- `VxUint32 nWidth, VxUint32 nHeight`：图像尺寸。
- `int nStride`：Android surface 的行跨度（以像素或字节根据实现决定，注意头文件描述）。
- `DX_BAYER_CONVERT_TYPE cvtype`：去马赛克算法。
- `DX_PIXEL_COLOR_FILTER nBayerType`：Bayer 排列。
- `bool bFlip`：是否翻转输出。
- `VxUint8 nAlpha`：Alpha 通道值（0~255）。

返回值
- `DX_OK` 成功，否则错误码。

注意事项
- 注意 `nStride` 的单位与目标平台接口的期望一致。

示例
```c
DxRaw8toARGB32(inRaw, outArgb, width, height, width, RAW2RGB_NEIGHBOUR, BAYERGB, false, 255);
```