# 训练后量化

## 量化前提

+ **模型准备**  
  + 请下载 [ResNet-101](https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet-101_nuq/resnet-101.onnx) 模型文件到 [model](./model/) 目录。

+ **数据集准备**
使用昇腾模型压缩工具对模型完成量化后，需要对模型进行推理，以测试量化数据的精度。推理过程中需要使用和模型相匹配的数据集。请下载[测试图片](https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet-101_nuq/images.zip)，解压后将 “images” 文件夹放到 [data](./data/) 目录下。

+ **校准集准备**
校准集用来产生量化因子，保证精度。本 sample 校准集与数据集相同。

## 量化示例

执行量化示例前，请先检查当前目录下是否包含以下文件及目录，其中 images 文件夹内部包含有 160 张用于校准和测试的图片：

+ [data](./data/)
  + [images](./data/images/)
+ [model](./model/)
  + [resnet-101.onnx](./model/resnet-101.onnx)
+ [src](./src/)
  + [resnet-101_calibration.py](./src/resnet-101_calibration.py)

请在当前目录执行如下命令运行示例程序：

```none
python ./src/resnet-101_calibration.py
```

若出现如下信息，则说明量化成功：

```none
INFO - [AMCT]:[Utils]: The model file is saved in ./outputs/calibration/resnet-101_deploy_model.onnx
INFO - [AMCT]:[Utils]: The model file is saved in ./outputs/calibration/resnet-101_fake_quant_model.onnx
[INFO] ResNet101 before quantize top1:    0.8875 top5:    0.9625
[INFO] ResNet101 after quantize  top1:     0.875 top5:    0.9625
```

## 量化结果

量化成功后，在当前目录会生成量化日志文件 [amct_onnx.log](./amct_log/amct_onnx.log) 和 [outputs](./outputs/) 文件夹，该文件夹内包含以下内容：

+ 训练后量化 [calibration](./outputs/calibration/)
  + 临时文件夹 [tmp](./outputs/calibration/tmp/)
    + 量化配置文件 [config.json](./outputs/calibration/tmp/config.json)
    + 量化因子记录文件 [record.txt](./outputs/calibration/tmp/record.txt)
    + 临时模型文件 [modified_model.onnx](./outputs/calibration/tmp/modified_model.onnx)
  + 量化部署模型 [resnet-101_deploy_model.onnx](./outputs/calibration/resnet-101_deploy_model.onnx)
  + 量化仿真模型 [resnet-101_fake_quant_model.onnx](./outputs/calibration/resnet-101_fake_quant_model.onnx)

量化配置文件描述了如何对模型中的每一层进行量化。如果量化脚本所在目录下已经存在量化配置文件，则再次调用 create_quant_config 接口时，如果新生成的量化配置文件与已有的文件同名，则会覆盖已有的量化配置文件，否则生成新的量化配置文件。

量化日志文件记录了量化过程的日志信息。

量化因子记录文件记录量化因子。关于该文件的原型定义请参见[量化因子记录文件说明](https://support.huaweicloud.com/content/dam/cloudbu-site/archive/china/zh-cn/support/docs/auxiliarydevtool-cann330alphaXinfer/atlasamcttf_16_0014.html)。

临时模型文件为原始模型 BN 融合后导出的 ONNX 模型文件。

resnet-101_deploy_model.onnx 为量化后的可在昇腾 AI 处理器部署的模型文件。

resnet-101_fake_quant_model.onnx 为量化后的可在 ONNX 执行框架 ONNX Runtime 进行精度仿真的模型文件。
