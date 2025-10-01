# DxRaw12PackedToRaw16

用途
- 将 12-bit packed 格式的 Raw 图像展开成每像素 16-bit（Raw16）格式，方便后续处理。

参数
- `void *pInputBuffer`：输入缓冲，12bit packed 数据，大小按相机输出。
- `void *pOutputBuffer`：输出缓冲，需分配 `nWidth * nHeight * 2` 字节。
- `VxUint32 nWidth, VxUint32 nHeight`：图像尺寸。

返回值
- 返回状态码：`DX_OK` 表示成功，其他为错误码。

注意事项
- 输入数据格式必须是相机的 12-bit packed（库未内置检测），否则结果错误。
- 输出缓冲需有足够空间。

示例
```c
DxRaw12PackedToRaw16(inPacked, outRaw16, width, height);
```