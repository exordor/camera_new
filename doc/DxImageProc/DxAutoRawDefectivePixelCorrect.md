# DxAutoRawDefectivePixelCorrect

用途
- 对 Raw（8~16 bit）图像进行自动坏像素校正（defective pixel correct），支持 Raw8 到 Raw16，需在每帧调用以去除坏点。

参数
- `void *pRawImgBuf`：输入/输出原始图像缓冲（会在原缓冲上修改）。
- `VxUint32 nWidth, VxUint32 nHeight`：图像尺寸。
- `VxInt32 nBitNum`：图像实际位数（8 ~ 16，例如 10 或 12）。

返回值
- `DX_OK` 成功，否则错误码。

注意事项
- 若输入为 packed 格式（如 Raw12Packed），需先转换为 Raw16（例如使用 `DxRaw12PackedToRaw16`)。
- 该函数会修改传入缓冲（in-place）。

示例
```c
DxAutoRawDefectivePixelCorrect(rawBuf, width, height, 12);
```