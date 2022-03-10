# ONNX 框架示例

ONNX ResNet-101 分类网络模型量化

### 1.1 量化前提

+ **模型准备**  
请下载 [ResNet-101](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/amct_acl/resnet101_v11.onnx) 模型文件到 [model](./model/) 目录。

+ **数据集准备**  
使用昇腾模型压缩工具对模型完成量化后，需要对模型进行推理，以测试量化数据的精度。推理过程中需要使用和模型相匹配的数据集。请下载[测试图片](https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet-101_nuq/images.zip)，解压后将 “images” 文件夹放到 [data](./data/) 目录下。

+ **校准集准备**  
校准集用来产生量化因子，保证精度。本 sample 校准集与数据集相同。

### 1.2 量化示例

执行量化示例前，请先检查当前目录下是否包含以下文件及目录，其中 [images](./data/images/) 文件夹内部包含有 160 张用于校准和测试的图片：

+ [data](./data/)
  + [images](./data/images/)
+ [model](./model/)
  + [resnet101_v11.onnx](./model/resnet101_v11.onnx)

请在当前目录执行如下命令运行示例程序：
```none
bash ./scripts/run_calibration.sh
```
执行成功会在当前目录生成output_deploy_model.onnx和output_fake_quant_model.onnx两个文件

### 3.1 量化结果

量化成功后，在当前目录会生成量化日志文件 [./amct_log/amct_onnx.log](./amct_log/amct_onnx.log) ，在当前目录下生成以下内容：

+ [date](./date/): 临时文件夹，date是随机数加下划线加当前日期组成，如29978_20210930135210
  + [config.json](./date/config.json): 量化配置文件描述了如何对模型中的每一层进行量化。
  + [record.txt](./date/record.txt): 量化因子记录文件记录量化因子。关于该文件的原型定义请参见[量化因子记录文件说明](https://support.huaweicloud.com/content/dam/cloudbu-site/archive/china/zh-cn/support/docs/auxiliarydevtool-cann330alphaXinfer/atlasamcttf_16_0014.html)。
  + [modified_model.onnx](./date/modified_model.onnx): 临时模型文件，即原始模型 BN 融合后导出的 ONNX 模型文件。
+ [output_deploy_model.onnx](./output_deploy_model.onnx): 量化部署模型，即量化后的可在昇腾 AI 处理器部署的模型文件。
+ [output_fake_quant_model.onnx](./output_fake_quant_model.onnx): 量化仿真模型，即量化后的可在 ONNX 执行框架 ONNX Runtime 进行精度仿真的模型文件。
+ [output_quant.json](./outputs/calibration/resnet-101_quant.json)：融合信息文件。

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

在当前目录执行如下命令运行示例程序：
```none
bash ./scripts/run_convert_qat.sh
```

### 1.2 量化结果

量化成功后，在当前目录会生成量化日志文件 [./amct_log/amct_onnx.log](./amct_log/amct_onnx.log) 和 [output](./output/) 文件夹，该文件夹内包含以下内容：
  + [result_fake_quant_model.onnx](./output/result_fake_quant_model.onnx): 量化仿真模型，即可在 ONNX 环境进行精度仿真的模型。
  + [result_deploy_model.onnx](./output/result_deploy_model.onnx): 量化部署模型，即可在昇腾 AI 处理器部署的模型。

