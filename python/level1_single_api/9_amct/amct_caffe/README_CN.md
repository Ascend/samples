# AMCT Caffe

AMCT Caffe 包含训练后量化，量化感知训练和张量分解功能，具体示例如下表所示：

| 网络 | 演示功能 |
| :-: | :-: |
| [cmd](./cmd/README_CN.md) | 命令行工具（量化） |
| [ResNet-50](./resnet50/README_CN.md) | 训练后量化（均匀量化，静态非均匀量化，自动非均匀量化） |
| [FastRCNN](./faster_rcnn/README_CN.md) | 训练后量化（均匀量化） |
| [MobileNetV2](./mobilenetV2/README_CN.md) | 训练后量化（基于精度的自动量化） |
| [MNIST](./mnist/README_CN.md) | 训练后量化（均匀量化） |
| [tensor_decompose](./tensor_decompose/README_CN.md) | 张量分解 |

**在执行以上用例之前，请先根据 requirements 安装必要的环境依赖。**
