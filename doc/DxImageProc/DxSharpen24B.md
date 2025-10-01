# DxSharpen24B

用途
- 对 RGB24 图像执行锐化处理。

参数
- `void *pInputBuffer`：输入 RGB24 缓冲（size = width*height*3）。
- `void *pOutputBuffer`：输出缓冲。
- `VxUint32 nWidth, VxUint32 nHeight`：图像尺寸。
- `float fFactor`：锐化因子，范围 `0.1~5.0`。

返回值
- `DX_OK` 成功，否则错误码。

注意事项
- 过高的 `fFactor` 可能导致图像产生边缘伪影。

示例
```c
DxSharpen24B(inRgb, outRgb, width, height, 1.5f);
```