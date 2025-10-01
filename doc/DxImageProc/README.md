# DxImageProc 函数说明汇总

本目录包含 `DxImageProc.h` 中各函数的中文说明文档。每个函数的文档包括用途、参数、返回值、常见注意事项及示例调用。

常用函数列表
- `DxRaw8toRGB24Ex.md` - Raw8 -> RGB24/BGR24（可选通道顺序）
- `DxRaw8toRGB24.md` - Raw8 -> RGB24
- `DxRaw8ImgProcess.md` - Raw8 综合处理（去坏点、去噪、锐化）
- `DxRaw16toRaw8.md` - Raw16 -> Raw8
- `DxRaw16toRGB48.md` - Raw16 -> RGB48
- `DxRaw12PackedToRaw16.md` - Raw12Packed -> Raw16
- `DxRaw10PackedToRaw16.md` - Raw10Packed -> Raw16
- `DxRaw8toARGB32.md` - Raw8 -> ARGB32
- `DxImageImprovment.md` / `DxImageImprovmentEx.md` - 图像质量增强
- `DxGetContrastLut.md`, `DxGetGammatLut.md`, `DxGetLut.md` - LUT 生成
- 其它函数详见对应文件。

使用建议
- 调用前请确保输入/输出缓冲尺寸正确并已分配内存。
- 去马赛克与增强相关函数对性能敏感，实时流建议选择轻量算法或在独立线程处理。
- 若需要，我可以把示例代码集成到 `galaxy_camera_ros_driver` 中作为 helper 函数或测试节点。

---

如果你希望我把这些文档移到项目的 `docs/` 根目录，或生成 HTML 格式的文档（例如使用 Doxygen 或 MkDocs），告诉我你的偏好。