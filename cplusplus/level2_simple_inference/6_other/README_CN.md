中文|[English](README.md)

# 其它样例

#### 介绍
本仓包含多种样例，以供用户参考。目录结构和具体说明如下。
| 样例  | 说明  | 支持芯片 |
|---|---|---|
| [DeRain](./DeRain)  | 通过读取本地雨天退化图像数据，对场景中的雨线、雨雾进行去除，实现图像增强效果 | Ascend310 |
| [DeblurGAN_GOPRO_Blur2Sharp](./DeblurGAN_GOPRO_Blur2Sharp)  | 输入一张模糊图片，使用DeblurGAN将其变清晰 | Ascend310 |
| [colorization](./colorization)  | 使用colorization模型对输入的黑白图片进行上色推理 | Ascend310 |
| [colorization_video](./colorization_video)  | 使用黑白图像上色模型对输入的黑白视频进行推理 | Ascend310 |
| [super_resolution_dynamic](./super_resolution_dynamic)  | 使用FSRCNN对输入图片进行图像超分辨率处理 | Ascend310 |
| [acl_execute_conv2d](./acl_execute_conv2d)  | 以om+aclopExecute的方式进行Conv2d算子运行 | Ascend310 |
| [acl_compile_and_execute_conv2d](./acl_compile_and_execute_conv2d)  | 以aclopCompileAndExecute的方式进行Conv2d算子运行 | Ascend310 |
