# AMCT TensorFlow

AMCT TensorFlow 包含训练后量化，`convert_model` 接口，量化感知训练，QAT 模型转 Ascend 模型，自动量化和张量分解功能，功能示例具体如下。

## 量化

| 网络 | 演示功能 |
| :-: | :-- |
| [MobileNet V2](./mobilenet_v2/README_CN.md) | 训练后量化（均匀量化），`convert_model` 接口，QAT 模型转 Ascend 模型，自动量化（基于精度） |
| [ResNet-50 V1](./resnet-50_v1/README_CN.md) | 训练后量化（非均匀量化），量化感知训练，通道稀疏，自动通道稀疏搜索 |
| [YOLO V3](./yolo_v3/README_CN.md) | 训练后量化（均匀量化） |
| [cmd](./cmd/README_CN.md) | 命令行 |

## 张量分解

请详见 [AMCT_TensorFlow 张量分解样例](./tensor_decompose/README_CN.md)

**在执行以上用例之前，请先根据 requirements 安装必要的环境依赖。**
