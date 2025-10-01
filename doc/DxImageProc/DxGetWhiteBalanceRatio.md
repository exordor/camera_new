# DxGetWhiteBalanceRatio

用途
- 计算 RGB24 图像的白平衡比例（R:G:B），用于自动白平衡估计。

参数
- `void *pInputBuffer`：输入 RGB24 缓冲。
- `VxUint32 nWidth, VxUint32 nHeight`：图像尺寸。
- `double *dRatioR, *dRatioG, *dRatioB`：输出的三个比率指针。

返回值
- `DX_OK` 成功，否则错误码。

注意事项
- 为了准确计算，应当提供以“白色”区域为主的图像，或让相机对准白色参考板。

示例
```c
double r,g,b;
DxGetWhiteBalanceRatio(rgbBuf, width, height, &r, &g, &b);
```