# DxGetGammatLut

用途
- 计算伽玛（Gamma）查找表（LUT），用于图像伽玛校正。

参数
- `double dGammaParam`：伽玛参数，范围 `0.1~10`。
- `void *pGammaLut`：输出 LUT 缓冲（调用方分配）。
- `int *pLutLength`：返回 LUT 长度（字节）。

返回值
- `DX_OK` 成功，否则错误码。

示例
```c
double gamma = 2.2;
unsigned char lut[256];
int len;
DxGetGammatLut(gamma, lut, &len);
```