# DxRaw8ImgProcess

用途
- 对 Raw8 图像执行综合处理：去坏点、去噪、锐化、色彩校正与转为 RGB24，使用 `COLOR_IMG_PROCESS` 结构传参。

参数
- `void *pRaw8Buf`：输入 Raw8 缓冲。
- `void *pRgb24Buf`：输出 RGB24 缓冲，需 `width*height*3` 字节。
- `VxUint32 nWidth, VxUint32 nHeight`：图像尺寸。
- `COLOR_IMG_PROCESS *pstClrImgProc`：处理选项结构，包含是否去坏点、去噪、锐化、色彩校正矩阵指针、LUT 等。

返回值
- `DX_OK` 成功，否则错误码。

注意事项
- `pstClrImgProc` 中 `parrCC`、`pProLut` 等指针若被使用，需确保其指向有效内存并且长度字段正确。

示例
```c
COLOR_IMG_PROCESS proc = { .bDefectivePixelCorrect = true, .bDenoise=false, .bSharpness=true, .fSharpFactor=1.2f, .cvType=RAW2RGB_ADAPTIVE, .emLayOut=BAYERRG };
DxRaw8ImgProcess(raw8, rgb24, width, height, &proc);
```