# DxImageMirror

用途
- 对 Raw8 或 8-bit 图像进行镜像（水平或垂直）。

参数
- `void *pInputBuffer`, `void *pOutputBuffer`：输入/输出缓冲。
- `VxUint32 nWidth, VxUint32 nHeight`：图像尺寸。
- `DX_IMAGE_MIRROR_MODE emMirrorMode`：`HORIZONTAL_MIRROR` 或 `VERTICAL_MIRROR`。

返回值
- `DX_OK` 成功，否则错误码。

注意事项
- 输出缓冲需同大小。

示例
```c
DxImageMirror(inBuf, outBuf, width, height, HORIZONTAL_MIRROR);
```