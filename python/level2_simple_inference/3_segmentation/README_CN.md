中文|[English](README.md)

# 语义分割样例

#### 介绍
本目录为语义分割样例，以供用户参考。目录结构和具体说明如下。

- **deeplabv3-MindSpore系列样例**
  | 样例名称  | 样例说明  | 特性解析  | 支持芯片 |
  |---|---|---|---|
  | [deeplabv3](./deeplabv3)  | 图片分割样例  | 输入和输出均为图片，存在两套运行代码，主要介绍opencv处理和dvpp处理。  | Ascend310 |
  | [deeplabv3_postprocess_optimization](./googlenet_imagenet_multi_batch)  | 图片分割样例  | 输入和输出均为图片，存在两套运行代码，主要介绍后处理优化和python多进程处理  | Ascend310 |

- **其它样例**
  | 样例  | 说明  | 支持芯片 |
  |---|---|---|
  | [deeplabv3_pascal_pic](./deeplabv3_pascal_pic) | 使用deeplabv3+模型对输入图片进行语义分割 | Ascend310 |