# DxRotate90CW8B

用途
- 将 8 位图像顺时针旋转 90 度。

参数
- `void *pInputBuffer`：输入 8-bit 图像缓冲，大小 `nWidth * nHeight`。
- `void *pOutputBuffer`：输出缓冲，大小 `nWidth * nHeight`（旋转后尺寸为 `nHeight * nWidth`，但总像素数相同）。
- `VxUint32 nWidth, VxUint32 nHeight`：输入图像尺寸。

返回值
- `DX_OK` 成功，否则错误码。

注意事项
- 输出缓冲需足够，旋转后图像的宽高互换（注意后续使用者应知道新宽高为原高度/宽度）。

示例
```c
DxRotate90CW8B(in8, out8, width, height);
// out has dimensions height x width
```