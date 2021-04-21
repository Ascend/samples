# 基于 calibration 的量化

## 量化前提

+ **模型准备**  
请至
[昇腾社区-ModelZoo](https://ascend.huawei.com/zh/#/software/modelzoo/detail/1/2a3e8d64cf7e48249246140ddbe4135f)
下载 MobileNetV2 模型文件。解压并将其中的 mobilenetv2_tf.pb 文件放到 [model](./model/) 目录下。

+ **数据集准备**  
使用昇腾模型压缩工具对模型完成量化后，需要对模型进行推理，以测试量化数据的精度。推理过程中需要使用和模型相匹配的数据集。请下载
[测试图片](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/mobilenet_v2_calibration/classification.jpg)，并将该图片放到 [data](./data/) 目录下。

+ **校准集准备**  
校准集用来产生量化因子，保证精度。  
计算量化参数的过程被称为“校准(calibration)”。校准过程需要使用一部分测试图片来针对性计算量化参数，使用一个或多个 batch 对量化后的网络模型进行推理即可完成校准。请下载
[校准集](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/mobilenet_v2_calibration/calibration.rar)，解压后将 “calibration” 文件夹放到 [data](./data/) 目录下。

## 量化示例

执行量化示例前，请先检查当前目录下是否包含以下文件及目录，其中 calibration 文件夹内部包含有 32 张用于校准的图片：

+ [data](./data/)
  + [calibration/](./data/calibration/)
  + [classification.jpg](./data/classification.jpg)
+ [model](./model/)
  + [mobilenetv2_tf.pb](./model/mobilenetv2_tf.pb)
+ [src](./src/)
  + [mobilenet_v2_calibration.py](./src/mobilenet_v2_calibration.py)

并根据 requirements 安装必要的环境依赖。

在当前目录执行如下命令运行示例程序：

```none
python ./src/mobilenet_v2_calibration.py
```

若出现如下信息则说明模型量化成功：

```none
INFO - [AMCT]:[save_model]: The model is saved in ./outputs/mobilenet_v2_quantized.pb
Origin Model Prediction:
        category index: 699
        category prob: 0.560
Quantized Model Prediction:
        category index: 699
        category prob: 0.568
```

## 量化结果

量化成功后，在当前目录下会生成以下文件

+ 量化配置文件 [config.json](./outputs/config.json)
+ 量化日志文件 [amct_tensorflow.log](./amct_log/amct_tensorflow.log)
+ 量化因子记录文件 [record.txt](./outputs/record.txt)
+ 量化信息文件 [mobilenet_v2_quant.json](./outputs/mobilenet_v2_quant.json)
+ 量化模型 [mobilenet_v2_quantized.pb](./outputs/mobilenet_v2_quantized.pb)

量化配置文件描述了如何对模型中的每一层进行量化。如果量化脚本所在目录下已经存在量化配置文件，则再次调用create_quant_config接口时，如果新生成的量化配置文件与已有的文件同名，则会覆盖已有的量化配置文件，否则生成新的量化配置文件。

量化日志文件记录了量化过程的日志信息。

量化因子记录文件记录量化因子。关于该文件的原型定义请参见
[量化因子记录文件说明](https://support.huaweicloud.com/content/dam/cloudbu-site/archive/china/zh-cn/support/docs/auxiliarydevtool-cann330alphaXinfer/atlasamcttf_16_0014.html)。

量化信息文件记录了量化模型同原始模型节点的映射关系，用于量化后模型同原始模型精度比对使用。

量化模型即可在 TensorFlow 环境进行精度仿真并可在昇腾 AI 处理器部署的模型。
对该模型重新进行量化时，在量化后模型的同级目录下生成的上述结果文件将会被覆盖。
