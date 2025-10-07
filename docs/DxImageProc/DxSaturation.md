# DxSaturation

用途
- 对 RGB24 图像的饱和度进行调整。

参数
- `void *pInputBuffer`：输入 RGB24 缓冲。
- `void *pOutputBuffer`：输出缓冲。
- `VxUint32 nImageSize`：图像像素数（本函数注释为 width * height）。
- `VxInt32 nFactor`：饱和度因子，范围 `0~128`。

返回值
- `DX_OK` 成功，否则错误码。

注意事项
- 注意 `nImageSize` 是像素数（非字节数），文档中有歧义，调用时请按头文件注释传入正确值。

示例
```c
DxSaturation(inRgb, outRgb, width*height, 80);
```