# AMCT PyTorch

AMCT PyTorch 包含训练后量化，量化感知训练， QAT 模型转 Ascend 模型和张量分解功能，功能示例具体如下。

## 量化

| 网络 | 演示功能 |
| :-: | :-: |
| [MobileNet V2](mobilenet_v2/README_CN.md) | 训练后量化（基于精度的自动量化） |
| [ResNet-101](./resnet-101/README_CN.md) | 训练后量化（均匀量化，非均匀量化），量化感知训练 |

## 张量分解

请详见 [AMCT_PyTorch 张量分解样例](./tensor_decompose/README_CN.md)

**在执行以上用例之前，请先根据 requirements 安装必要的环境依赖。**
