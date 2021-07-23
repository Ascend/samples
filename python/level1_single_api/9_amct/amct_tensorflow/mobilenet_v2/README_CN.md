# MobileNet V2

## 1. 均匀量化

### 1.1 量化前提

+ **模型准备**  
请点击下载 [MobileNet V2](https://storage.googleapis.com/mobilenet_v2/checkpoints/mobilenet_v2_1.0_224.tgz) 模型文件。解压并将其中的 mobilenet_v2_1.0_224_frozen.pb 文件放到 [model](./model/) 目录下。

+ **数据集准备**  
使用昇腾模型压缩工具对模型完成量化后，需要对模型进行推理，以测试量化数据的精度。推理过程中需要使用和模型相匹配的数据集。请下载测试图片 [classification.jpg](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/mobilenet_v2_calibration/classification.jpg)，并将该图片放到 [data](./data/) 目录下。

+ **校准集准备**  
校准集用来产生量化因子，保证精度。  
计算量化参数的过程被称为“校准 (calibration)”。校准过程需要使用一部分测试图片来针对性计算量化参数，使用一个或多个 batch 对量化后的网络模型进行推理即可完成校准。请下载[校准集](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/mobilenet_v2_calibration/calibration.rar)，解压后将 calibration 文件夹放到 [data](./data/) 目录下。

### 1.2 量化示例

执行量化示例前，请先检查当前目录下是否包含以下文件及目录，其中 calibration 文件夹内部包含有 32 张用于校准的图片：

+ [data](./data/)
  + [calibration/](./data/calibration/)
  + [classification.jpg](./data/classification.jpg)
+ [model](./model/)
  + [mobilenetv2_tf.pb](./model/mobilenetv2_tf.pb)
+ [src](./src/)
  + [mobilenet_v2_calibration.py](./src/mobilenet_v2_calibration.py)

在当前目录执行如下命令运行示例程序：

```none
python ./src/mobilenet_v2_calibration.py
```

若出现如下信息则说明模型量化成功：

```none
INFO - [AMCT]:[save_model]: The model is saved in ./outputs/calibration/mobilenet_v2_quantized.pb
Origin Model Prediction:
        category index: 443
        category prob: 0.375
Quantized Model Prediction:
        category index: 443
        category prob: 0.478
```

### 1.3 量化结果

量化成功后，在当前目录会生成量化日志文件 [./amct_log/amct_tensorflow.log](./amct_log/amct_tensorflow.log) 和 [./outputs/calibration](./outputs/calibration/) 文件夹，文件夹内包含以下内容：

+ [config.json](./outputs/calibration/config.json): 量化配置文件，描述了如何对模型中的每一层进行量化。
+ [record.txt](./outputs/calibration/record.txt): 量化因子记录文件，记录量化因子。关于该文件的原型定义请参见
[量化因子记录文件说明](https://support.huaweicloud.com/content/dam/cloudbu-site/archive/china/zh-cn/support/docs/auxiliarydevtool-cann330alphaXinfer/atlasamcttf_16_0014.html)。
+ [mobilenet_v2_quant.json](./outputs/calibration/mobilenet_v2_quant.json): 量化信息文件，记录了量化模型同原始模型节点的映射关系，用于量化后模型同原始模型精度比对使用。
+ [mobilenet_v2_quantized.pb](./outputs/calibration/mobilenet_v2_quantized.pb): 量化模型，可在 TensorFlow 环境进行精度仿真并可在昇腾 AI 处理器部署。

> 对该模型重新进行量化时，在量化后模型的同级目录下生成的上述结果文件将会被覆盖。

## 2. `convert_model` 接口

### 2.1 量化前提

+ **模型准备**  
请点击下载 [MobileNet V2](https://storage.googleapis.com/mobilenet_v2/checkpoints/mobilenet_v2_1.0_224.tgz) 模型文件。解压并将其中的 mobilenet_v2_1.0_224_frozen.pb 文件放到 [model](./model/) 目录下。

+ **数据集准备**  
使用昇腾模型压缩工具对模型完成量化后，需要对模型进行推理，以测试量化数据的精度。推理过程中需要使用和模型相匹配的数据集。请下载测试图片 [classification.jpg](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/mobilenet_v2_calibration/classification.jpg)，并将该图片放到 [data](./data/) 目录下。

### 2.2 量化示例

执行量化示例前，请先检查当前目录下是否包含以下文件及目录：

+ [data](./data/)
  + [classification.jpg](./data/classification.jpg)
  + [record_quantized.txt](./data/record_quantized.txt)
+ [model](./model/)
  + [mobilenetv2_tf.pb](./model/mobilenetv2_tf.pb)
+ [src](./src/)
  + [mobilenet_v2_convert_model.py](./src/mobilenet_v2_convert_model.py)

在当前目录执行如下命令运行示例程序：

```bash
python ./src/mobilenet_v2_convert_model.py
```

若出现如下信息则说明模型量化成功：

```none
INFO - [AMCT]:[save_model]: The model is saved in ./outputs/convert_model/mobilenet_v2_quantized.pb
Origin Model Prediction:
        category index: 443
        category prob: 0.375
Quantized Model Prediction:
        category index: 443
        category prob: 0.517
```

### 2.3 量化结果

量化成功后，在当前目录会生成量化日志文件 [./amct_log/amct_tensorflow.log](./amct_log/amct_tensorflow.log) 和 [./outputs/convert_model](./outputs/convert_model/) 文件夹，文件夹内包含以下内容：

+ [mobilenet_v2_quant.json](./outputs/convert_model/mobilenet_v2_quant.json): 量化信息文件，记录了量化模型同原始模型节点的映射关系，用于量化后模型同原始模型精度比对使用。
+ [mobilenet_v2_quantized.pb](./outputs/convert_model/mobilenet_v2_quantized.pb): 量化模型，可在 TensorFlow 环境进行精度仿真并可在昇腾 AI 处理器部署。

> 对该模型重新进行量化时，在量化后模型的同级目录下生成的上述结果文件将会被覆盖。

## 3. QAT 模型转 Ascend 模型

### 3.1 量化前提

+ **模型准备**  
请下载 [MobileNetV2 QAT](https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mobilenetv2_convert_qat/mobilenetv2_qat.pb) 模型文件到 [model](./model/) 目录。

+ **数据集准备**  
使用昇腾模型压缩工具对模型完成量化后，需要对模型进行推理，以测试量化数据的精度。推理过程中需要使用和模型相匹配的数据集。请下载测试图片 [convert_qat.jpg](https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mobilenetv2_convert_qat/convert_qat.jpg)，并将该图片放到 [data](./data/) 目录下。

### 3.2 量化示例

执行量化示例前，请先检查当前目录下是否包含以下文件及目录：

+ [data](./data/)
  + [convert_qat.jpg](./data/convert_qat.jpg)
+ [model](./model/)
  + [mobilenetv2_qat.pb](./model/mobilenetv2_qat.pb)
+ [src](./src/)
  + [mobilenet_v2_convert_qat.py](./src/mobilenet_v2_convert_qat.py)

在当前目录执行如下命令运行示例程序：

```bash
python ./src/mobilenet_v2_convert_qat.py
```

若出现如下信息则说明模型量化成功：

```none
INFO - [AMCT]:[save_model]: The model is saved in ./outputs/convert_qat/mobilenet_v2_quantized.pb
Origin Model Prediction:
        category index: 346
        category prob: 0.650
Quantized Model Prediction:
        category index: 346
        category prob: 0.246
```

### 3.3 量化结果

量化成功后，在当前目录会生成量化日志文件 [./amct_log/amct_tensorflow.log](./amct_log/amct_tensorflow.log) 和 [./outputs/convert_qat](./outputs/convert_qat/) 文件夹，文件夹内包含以下内容：

+ [record.txt](./outputs/convert_qat/record.txt): 量化因子记录文件，记录量化因子。关于该文件的原型定义请参见
[量化因子记录文件说明](https://support.huaweicloud.com/content/dam/cloudbu-site/archive/china/zh-cn/support/docs/auxiliarydevtool-cann330alphaXinfer/atlasamcttf_16_0014.html)。
+ [mobilenet_v2_quant.json](./outputs/convert_qat/mobilenet_v2_quant.json): 量化信息文件，记录了量化模型同原始模型节点的映射关系，用于量化后模型同原始模型精度比对使用。
+ [mobilenet_v2_quantized.pb](./outputs/convert_qat/mobilenet_v2_quantized.pb): 量化模型，可在 TensorFlow 环境进行精度仿真并可在昇腾 AI 处理器部署。

> 对该模型重新进行量化时，在量化后模型的同级目录下生成的上述结果文件将会被覆盖。

## 4. 基于精度的自动量化

### 4.1 量化前提

+ **模型准备**  
请点击下载 [MobileNet V2](https://storage.googleapis.com/mobilenet_v2/checkpoints/mobilenet_v2_1.0_224.tgz) 模型文件。解压并将其中的 mobilenet_v2_1.0_224_frozen.pb 文件放到 [model](./model/) 目录下。

+ **数据集准备**  
自动量化回退过程中，需要不断的对模型进行校准和测试，因此需要用户准备数据集，本示例所采用的数据集为标准 TFRecord 格式的 ImageNet 的 子集 ILSVRC-2012-CLS 的验证集，共有 50000 张图片，如果采用其他数据集，则需要用户自行修改 sample 文件中的数据预处理部分以匹配模型输入。

### 4.2 量化示例

执行量化示例前，请先检查当前目录下是否包含以下文件及目录：

+ [model](./model/)
  + [mobilenetv2_tf.pb](./model/mobilenetv2_tf.pb)
+ [src](./src/)
  + [mobilenet_v2_accuracy_based_auto_calibration.py](./src/mobilenet_v2_accuracy_based_auto_calibration.py)

在当前目录执行如下命令运行示例程序：

```bash
python ./src/mobilenet_v2_accuracy_based_auto_calibration.py --dataset DATASET
```

上述命令只给出了常用的参数，不常用参数以及各个参数解释请参见如下表格：

| 参数 | 必填项 | 数据类型 | 默认值 | 参数解释 |
| :-- | :-: | :-: | :-: | :-- |
| -h | 否 | / | / | 显示帮助信息。 |
| --dataset | 是 | string | None | 标准 TFRecord 格式的 ImageNet 的子集 ILSVRC-2012-CLS 的验证部分。 |
| --num_parallel_reads | 否 | int | 4 | 用于读取数据集的线程数，根据硬件运算能力酌情调整。 |
| --batch_size | 否 | int | 32 | TensorFlow 运行一次所使用的样本数量，根据内存或显存大小酌情调整。 |
| --model | 否 | string | ./model/mobilenetv2_tf.pb | 自动量化回退时使用的原始模型。 |

若出现如下信息则说明模型量化成功：

```none
INFO - [AMCT]:[AMCT]: Accuracy of original model is         71.84
INFO - [AMCT]:[AMCT]: Accuracy of global quantized model is 70.85
INFO - [AMCT]:[AMCT]: Accuracy of saved model is            71.476
INFO - [AMCT]:[AMCT]: The generated model is stored in dir: ./outputs/accuracy_based_auto_calibration
INFO - [AMCT]:[AMCT]: The records is stored in dir: ./outputs/accuracy_based_auto_calibration
```

### 4.3 量化结果

量化成功后，在当前目录会生成如下文件：

+ [amct_log](./amct_log/): 量化日志文件夹
  + [amct_tensorflow.log](./amct_log/amct_tensorflow.log): 量化日志文件。
  + [accuracy_based_auto_calibration_record.json](./amct_log/accuracy_based_auto_calibration_record.json): 基于精度的自动量化回退历史记录文件。
+ [./outputs/accuracy_based_auto_calibration](./outputs/accuracy_based_auto_calibration/): 输出文件夹
  + [accuracy_based_auto_calibration_final_config.json](./outputs/accuracy_based_auto_calibration/accuracy_based_auto_calibration_final_config.json): 回退后的量化配置文件，描述了如何对模型中的每一层进行量化。
  + [accuracy_based_auto_calibration_ranking_information.json](./outputs/accuracy_based_auto_calibration/accuracy_based_auto_calibration_ranking_information.json): 量化层量化敏感信息。
  + [config.json](./outputs/calibration/config.json): 回退前的量化配置文件，描述了如何对模型中的每一层进行量化。
  + [mobilenet_v2_quantized.pb](./outputs/accuracy_based_auto_calibration/mobilenet_v2_quantized.pb): 量化模型，可在 TensorFlow 环境进行精度仿真并可在昇腾 AI 处理器部署。
  + [mobilenet_v2_quant.json](./outputs/accuracy_based_auto_calibration/mobilenet_v2_quant.json): 量化信息文件，记录了量化模型同原始模型节点的映射关系，用于量化后模型同原始模型精度比对使用。
  + [record.txt](./outputs/accuracy_based_auto_calibration/record.txt): 量化因子记录文件，记录量化因子。关于该文件的原型定义请参见[量化因子记录文件说明](https://support.huaweicloud.com/content/dam/cloudbu-site/archive/china/zh-cn/support/docs/auxiliarydevtool-cann330alphaXinfer/atlasamcttf_16_0014.html)。

> 对该模型重新进行量化时，在量化后模型的同级目录下生成的上述结果文件将会被覆盖。
