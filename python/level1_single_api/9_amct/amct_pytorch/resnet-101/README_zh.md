# 训练后量化（非均匀量化）

## 量化前提

+ **模型准备**
请下载
[ResNet-101](https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Model/AE/ATC%20Model/resnet-101_nuq/resnet101-5d3b4d8f.pth)
模型文件到 [model](./model/) 目录。

+ **数据集准备**
使用昇腾模型压缩工具对模型完成量化后，需要对模型进行推理，以测试量化数据的精度。推理过程中需要使用和模型相匹配的数据集。请下载
[测试图片](https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Model/AE/ATC%20Model/resnet-101_nuq/images.zip)，解压后将 “images” 文件夹放到 [data](./data/) 目录下。

+ **校准集准备**
校准集用来产生量化因子，保证精度。本 sample 校准集与数据集相同。

## 量化示例

执行量化示例前，请先检查当前目录下是否包含以下文件及目录，其中 images 文件夹内部包含有 160 张用于校准和测试的图片：

+ [data](./data/)
  + [images](./data/images/)
+ [model](./model/)
  + [\_\_init__.py](./model/__init__.py)
  + [resnet.py](./model/resnet.py)
  + [resnet101-5d3b4d8f.pth](./model/resnet101-5d3b4d8f.pth)
+ [src](./src/)
  + [nuq_files](./src/nuq_files/)
    + [nuq_quant.cfg](./src/nuq_files/nuq_quant.cfg)
    + [resnet-101_quantized.json](src/nuq_files/resnet-101_quantized.json)

并根据 requirements 安装必要的环境依赖。

在当前目录执行如下命令运行示例程序：

```none
CUDA_VISIBLE_DEVICES=0 python ./src/resnet-101_sample.py
```

其中 ```CUDA_VISIBLE_DEVICES``` 是必填参数，表示使用 CPU 还是 GPU 进行量化，参数取值为：

+ -1：使用 CPU 进行量化。
+ 其他 Device ID使用 GPU 进行量化，具体 ID 请以用户实际环境为准。当前仅支持配置单 Device。

若出现如下信息，则说明量化成功：

```none
******final top1:0.8625
******final top5:0.9625      //量化后的精度仿真模型在ONNX Runtime环境中top1、top5的推理精度
[INFO] ResNet101 before quantize top1:    0.8875 top5:    0.9625     //原始模型的推理结果，推理结果根据用户环境会有所不同，请以实际显示的为准
[INFO] ResNet101 after quantize  top1:   0.8625 top5:    0.9625     //量化后精度仿真模型的推理结果，推理结果根据用户环境会有所不同，请以实际显示的为准
```

## 量化结果

量化成功后，在当前目录下会生成以下文件

+ 量化配置文件 [config.json](./outputs/config.json)
+ 量化日志文件 [amct_pytorch.log](./amct_log/amct_pytorch.log)
+ 量化因子记录文件 [record.txt](./outputs/record.txt)
+ 临时模型文件 [modified_model.onnx](./outputs/modified_model.onnx)
+ 量化模型
  + [resnet-101_deploy_model.onnx](./outputs/resnet-101_deploy_model.onnx)
  + [resnet-101_fake_quant_model.onnx](./outputs/resnet-101_fake_quant_model.onnx)

量化配置文件描述了如何对模型中的每一层进行量化。如果量化脚本所在目录下已经存在量化配置文件，则再次调用 create_quant_config 接口时，如果新生成的量化配置文件与已有的文件同名，则会覆盖已有的量化配置文件，否则生成新的量化配置文件。

量化日志文件记录了量化过程的日志信息。

量化因子记录文件记录量化因子。关于该文件的原型定义请参见
[量化因子记录文件说明](https://support.huaweicloud.com/content/dam/cloudbu-site/archive/china/zh-cn/support/docs/auxiliarydevtool-cann330alphaXinfer/atlasamcttf_16_0014.html)。

临时模型文件为原始的 PyTorch 模型 BN 融合后导出的 ONNX 模型文件。

ResNet101_deploy_model.onnx 为量化后的可在昇腾 AI 处理器部署的模型文件。
ResNet101_fake_quant_model.onnx 为量化后的可在 ONNX 执行框架 ONNX Runtime 进行精度仿真的模型文件。