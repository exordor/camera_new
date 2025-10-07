# DxRGB48toRGB24

用途
- 将 48-bit（RGB48，每通道 16-bit）图像降级为 24-bit（每通道 8-bit）。

参数
- `void *pInputBuffer`：输入 RGB48 缓冲，大小 `width*height*3*2`。
- `void *pOutputBuffer`：输出 RGB24 缓冲，大小 `width*height*3`。
- `VxUint32 nWidth, VxUint32 nHeight`：图像尺寸。
- `DX_VALID_BIT nValidBits`：有效位选择，用以正确缩放 16-bit 到 8-bit。

返回值
- `DX_OK` 成功，否则错误码。

注意事项
- 确保 `nValidBits` 与输入图像的实际位对齐以避免失真。

示例
```c
DxRGB48toRGB24(in48, out24, width, height, DX_BIT_2_9);
```