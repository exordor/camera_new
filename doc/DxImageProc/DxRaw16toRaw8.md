# DxRaw16toRaw8

用途
- 将 Raw16（每像素 2 字节）按有效位（valid bits）转换为 8 位图像（Raw8）。

参数
- `void *pInputBuffer`：输入 Raw16 缓冲，大小 `width*height*2`。
- `void *pOutputBuffer`：输出 Raw8 缓冲，大小 `width*height`。
- `VxUint32 nWidth, VxUint32 nHeight`：图像尺寸。
- `DX_VALID_BIT nValidBits`：指定有效位（例如 `DX_BIT_0_7`, `DX_BIT_3_10` 等）。

返回值
- `DX_OK` 成功，否则错误码。

注意事项
- 选择合适的 `nValidBits`，以正确缩放位深到 8 位，否则会丢失亮度信息或溢出。

示例
```c
DxRaw16toRaw8(inRaw16, outRaw8, width, height, DX_BIT_3_10);
```