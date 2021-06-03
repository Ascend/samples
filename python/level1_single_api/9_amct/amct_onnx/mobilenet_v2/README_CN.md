# MobileNet V2

## 1. QAT 模型转 Ascend 模型

如果 TensorFlow 框架的原始模型已经做了量化功能（以下简称 QAT 模型），但是该模型无法使用 ATC 工具转成适配昇腾 AI 处理器的离线模型，则需要借助 昇腾模型压缩工具提供的 `convert_qat_model` 接口，将该 QAT 模型转成 Ascend 量化模型格式，然后才能使用 ATC 工具，将 Ascend 量化模型转成适配昇腾 AI 处理器的离线模型。

### 1.1 量化前提

+ **模型准备**  
请下载 [MobileNetV2 QAT](https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mobilenetv2_convert_qat/mobilenetv2_qat.onnx) 模型文件到 [model](./model/) 目录。

+ **数据集准备**  
使用昇腾模型压缩工具对模型完成量化后，需要对模型进行推理，以测试量化数据的精度。推理过程中需要使用和模型相匹配的数据集。请下载[测试图片](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/mobilenet_v2_calibration/classification.jpg)并将该图片放到 [data](./data/) 目录下。

### 1.2 量化示例

执行量化示例前，请先检查当前目录下是否包含以下文件及目录：

+ [data](./data/)
  + [classification.jpg](./data/classification.jpg)
+ [model](./model/)
  + [mobilenetv2_qat.onnx](./model/mobilenetv2_qat.onnx)
+ [src](./src/)
  + [mobilenet_v2_convert_qat.py](./src/mobilenet_v2_convert_qat.py)

在当前目录执行如下命令运行示例程序：

```none
python ./src/mobilenet_v2_convert_qat.py
```

若出现如下信息则说明模型量化成功：

```none
INFO - [AMCT]:[Utils]: The model file is saved in ./outputs/calibration/mobilenet_v2_deploy_model.onnx
INFO - [AMCT]:[Utils]: The model file is saved in ./outputs/calibration/mobilenet_v2_fake_quant_model.onnx
Origin Model Prediction:
        category index: 699
        category prob: 0.543
Quantized Model Prediction:
        category index: 699
        category prob: 0.398
```

### 1.3 量化结果

量化成功后，在当前目录会生成量化日志文件 [./amct_log/amct_onnx.log](./amct_log/amct_onnx.log) 和 [outputs](./outputs/) 文件夹，该文件夹内包含以下内容：

+ [convert_qat](./outputs/convert_qat/)
  + [record.txt](./outputs/convert_qat/record.txt): 量化因子记录文件记录量化因子。关于该文件的原型定义请参见
[量化因子记录文件说明](https://support.huaweicloud.com/content/dam/cloudbu-site/archive/china/zh-cn/support/docs/auxiliarydevtool-cann330alphaXinfer/atlasamcttf_16_0014.html)。
  + [mobilenet_v2_fake_quant_model.onnx](./outputs/convert_qat/mobilenet_v2_fake_quant_model.onnx): 量化仿真模型，即可在 ONNX 环境进行精度仿真的模型。
  + [mobilenet_v2_deploy_model.onnx](./outputs/convert_qat/mobilenet_v2_deploy_model.onnx): 量化部署模型，即可在昇腾 AI 处理器部署的模型。

> 对该模型重新进行量化时，在量化后模型的同级目录下生成的上述结果文件将会被覆盖。

## 2. 基于精度的自动量化

该接口会根据模型在数据集测试的精度和用户设定的精度损失要求来自动决定每一层是否进行量化，最终生成量化的混合模型；精度目标如果设置的比较苛刻，所有层均不量化才能达到目标则不会生成量化模型，可以适当降低精度损失的目标，继续调用该接口进行模型的量化。

### 2.1 量化前提

+ **模型准备**  
请下载 [MobileNet-V2](https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/acc_based_auto_calibration/mobilenetv2_v11.onnx) 模型文件到 [model](./model/) 目录。

+ **数据集准备**  
使用昇腾模型压缩工具对模型完成量化后，需要对模型进行推理，以测试量化数据的精度。推理过程中需要使用和模型相匹配的数据集。请下载[测试图片](https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet-101_nuq/images.zip)，解压后将 “images” 文件夹放到 [data](./data/) 目录下。

+ **校准集准备**  
校准集用来产生量化因子，保证精度。本 sample 校准集与数据集相同。

## 2.2 量化示例

在当前目录执行如下命令运行示例程序：

```bash
python ./src/mobilenet_v2_accuracy_based_auto_calibration.py
```

### 2.3 量化结果

量化成功后会生成量化 fake quant 和 deploy 的 onnx 模型文件和量化层敏感度的信息文件, 量化因子记录文件等；

| 目录     | 文件或目录                                        | 说明                                                         |
| -------- | ------------------------------------------------- | ------------------------------------------------------------ |
| amct_log | amct_onnx.log                                     | amct_onnx 工具在执行过程中相关日志文件                    |
|          | accuracy_based_auto_calibration_record.json       | 基于精度的自动量化回退过程中的量化配置记录文件               |
|          | tempxxxxx/ 目录                                   | 里面保存了隐藏层的 feature map 和其他临时性模型文件，当执行完毕后就可以删除 |
| results  | mobilenet_v2_fake_quant_model.onnx                | 生成的 fake quant onnx 模型文件                              |
|          | mobilenet_v2_deploy_model.onnx                    | 生成的 deploy onnx 模型文件                                  |
|          | accuracy_based_auto_calibration_final_config.json | 基于精度的自动量化回退最终搜索得到的量化配置文件             |
|          | accuracy_based_auto_calibration_ranking_info.json | 基于精度的自动量化回退过程中记录的每层量化敏感度信息文件     |
| tmp      | config.json                                       | 量化过程中的量化配置文件                                     |
|          | scale_offset_record.txt                           | 量化因子记录文件                                             |
