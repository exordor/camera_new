# DxBrightness

用途
- 对 RGB24 或 8-bit 灰度图像执行亮度调整。

参数
- `void *pInputBuffer`：输入缓冲（RGB24 或 单通道 8-bit）。
- `void *pOutputBuffer`：输出缓冲，大小与输入相同。
- `VxUint32 nImagesize`：图像字节大小（RGB24 为 `width*height*3`，灰度为 `width*height`）。
- `VxInt32 nFactor`：亮度因子，范围 `-150~150`。

返回值
- `DX_OK` 成功，否则错误码。

注意事项
- 确认 `nImagesize` 与实际缓冲大小一致。过大或过小都会导致错误或未定义行为。

示例
```c
DxBrightness(inRgb, outRgb, width*height*3, 20); // 增加亮度
```