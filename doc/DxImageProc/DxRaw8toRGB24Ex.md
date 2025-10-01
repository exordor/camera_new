# DxRaw8toRGB24Ex

用途
- 将单通道 Raw8（Bayer）图像去马赛克（demosaic）并输出为 24 位 RGB 或 BGR 图像。

参数
- `void *pInputBuffer`：输入缓冲区，Raw8 数据，大小应为 `nWidth * nHeight` 字节。
- `void *pOutputBuffer`：输出缓冲区，需预分配 `nWidth * nHeight * 3` 字节，用于接收 RGB24/BGR24 数据。
- `VxUint32 nWidth`：图像宽度（像素）。
- `VxUint32 nHeight`：图像高度（像素）。
- `DX_BAYER_CONVERT_TYPE cvtype`：去马赛克算法（`RAW2RGB_NEIGHBOUR`, `RAW2RGB_ADAPTIVE`, `RAW2RGB_NEIGHBOUR3`）。
- `DX_PIXEL_COLOR_FILTER nBayerType`：输入 Bayer 排列（`BAYERRG`, `BAYERGB`, `BAYERGR`, `BAYERBG`）。
- `bool bFlip`：是否翻转输出图像（实现可能为垂直翻转或镜像）。
- `DX_RGB_CHANNEL_ORDER emChannelOrder`：输出通道顺序（`DX_ORDER_RGB` 或 `DX_ORDER_BGR`）。

返回值
- 返回 `VxInt32` 状态码：`DX_OK (0)` 表示成功；负值表示各类错误（`DX_PARAMETER_INVALID`, `DX_PARAMETER_OUT_OF_BOUND`, `DX_NOT_ENOUGH_SYSTEM_MEMORY`, `DX_STATUS_NOT_SUPPORTED` 等）。

调用注意事项
- 确保 `pInputBuffer` 和 `pOutputBuffer` 非空且已分配足够内存。
- `nBayerType` 必须与相机实际输出一致，否则颜色会不正确。
- 根据实时性需求选择 `cvtype`（`ADAPTIVE` 质量更好但更慢）。
- 线程并发调用时请自行管理缓冲区与同步。

常见边界情况
- 输入或输出指针为 NULL
- `nWidth` 或 `nHeight` 为 0
- 输出缓冲区长度不足导致写越界
- 使用了不支持的 `cvtype` 或 `emChannelOrder`

示例（伪 C 代码）
```c
unsigned char *in = ...; // width * height
unsigned char *out = malloc(width * height * 3);
VxInt32 ret = DxRaw8toRGB24Ex(in, out, width, height, RAW2RGB_ADAPTIVE, BAYERRG, false, DX_ORDER_RGB);
if (ret != DX_OK) {
    // 处理错误
}
```