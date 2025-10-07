# DxContrast

用途
- 对 RGB24 或 8-bit 灰度图像执行对比度调整。

参数
- `void *pInputBuffer`：输入缓冲。
- `void *pOutputBuffer`：输出缓冲。
- `VxUint32 nImagesize`：图像字节大小（RGB: width*height*3）。
- `VxInt32 nFactor`：对比度因子，范围 `-50~100`。

返回值
- `DX_OK` 成功，否则错误码。

注意事项与边界
- 确保 `nFactor` 在有效范围内。

示例
```c
DxContrast(inRgb, outRgb, width*height*3, 10);
```