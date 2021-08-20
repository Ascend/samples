# ONNX 框架示例

ONNX ResNet-101 分类网络模型量化

## 1. 准备模型

请下载 [ResNet-101](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/amct_acl/resnet101_v11.onnx)。下载后将 resnet101_v11.onnx 放到当前目录的 [model](./model/) 子目录中。

## 2. 准备校准数据集

计算量化因子的过程被称为“校准 (calibration)”。校准过程需要使用一部分测试图片来执行模型推理，在推理过程中计算量化因子。请下载[校准图片](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/amct_acl/classification/imagenet_calibration.tar.gz)，并解压到 [data](./data/) 文件夹中。

## 3. 执行量化

```none
bash ./scripts/run_calibration.sh
```

# MobileNet V2

## 1. QAT 模型转 Ascend 模型

如果 TensorFlow 框架的原始模型已经做了量化功能（以下简称 QAT 模型），但是该模型无法使用 ATC 工具转成适配昇腾 AI 处理器的离线模型，则需要借助 昇腾模型压缩工具提供的 `convert_qat_model` 接口，将该 QAT 模型转成 Ascend 量化模型格式，然后才能使用 ATC 工具，将 Ascend 量化模型转成适配昇腾 AI 处理器的离线模型。

### 1.1 量化前提

+ **模型准备**  
请下载 [MobileNetV2 QAT](https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mobilenetv2_convert_qat/mobilenetv2_qat.onnx) 模型文件到 [model](./model/) 目录。

```none
bash ./scripts/run_convert_qat.sh
```

