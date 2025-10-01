# DxMono8ImgProcess

用途
- 对单通道 8-bit（Mono8）图像进行处理：坏点纠正、锐化、加速等，使用 `MONO_IMG_PROCESS` 结构传参。

参数
- `void *pInputBuf`, `void *pOutputBuf`：输入/输出单通道缓冲。
- `VxUint32 nWidth, VxUint32 nHeight`：图像尺寸。
- `MONO_IMG_PROCESS *pstGrayImgProc`：处理参数结构。

返回值
- `DX_OK` 成功，否则错误码。

示例
```c
MONO_IMG_PROCESS mproc = { .bDefectivePixelCorrect=true, .bSharpness=true, .fSharpFactor=1.0f };
DxMono8ImgProcess(inMono, outMono, width, height, &mproc);
```