# DxRaw10PackedToRaw16

用途
- 将 10-bit packed 格式的 Raw 数据转换为 Raw16（每像素 16-bit）。

参数
- `void *pInputBuffer`：输入 packed 数据。
- `void *pOutputBuffer`：输出 Raw16 缓冲，大小 `nWidth * nHeight * 2` 字节。
- `VxUint32 nWidth, VxUint32 nHeight`：图像尺寸。

返回值
- `DX_OK` 成功，否则错误码。

注意事项
- 确认输入为 10-bit packed 格式，输出缓冲够大。

示例
```c
DxRaw10PackedToRaw16(inPacked, outRaw16, width, height);
```