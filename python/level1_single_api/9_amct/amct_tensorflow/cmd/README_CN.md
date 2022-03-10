# MobileNet V2

## 1. 均匀量化

### 1.1 量化前提

+ **模型准备**  
请点击下载 [MobileNet V2](https://storage.googleapis.com/mobilenet_v2/checkpoints/mobilenet_v2_1.0_224.tgz) 模型文件。解压并将其中的 mobilenet_v2_1.0_224_frozen.pb 文件放到 [model](./model/) 目录下。

+ **准备校准数据集**
计算量化因子的过程被称为“校准 (calibration)”。校准过程需要使用一部分测试图片来执行模型推理，在推理过程中计算量化因子。请下载[校准图片](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/amct_acl/classification/calibration.rar)，并放到 [data](./data/) 文件夹中。

### 1.2 量化示例

在当前目录执行如下命令运行示例程序：

```none
sh ./scripts/run_calibration.sh
```

若出现如下信息则说明模型量化成功：

```none
INFO - [AMCT]:[save_model]: The model is saved in ./outputs/calibration/mobilenet_v2_quantized.pb
```

### 1.3 量化结果

量化成功后，在当前目录会生成量化日志文件 [./amct_log/amct_tensorflow.log](./amct_log/amct_tensorflow.log) 和 [./date]文件夹，在当前目录下生成以下内容：

+ [output_quant.json](./outputs/calibration/mobilenet_v2_quant.json): 量化信息文件，记录了量化模型同原始模型节点的映射关系，用于量化后模型同原始模型精度比对使用。
+ [output_quantized.pb](./outputs/calibration/mobilenet_v2_quantized.pb): 量化模型，可在 TensorFlow 环境进行精度仿真并可在昇腾 AI 处理器部署。
## 2. QAT 模型转 Ascend 模型

### 2.1 量化前提

+ **模型准备**  
请下载 [MobileNetV2 QAT](https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mobilenetv2_convert_qat/mobilenetv2_qat.pb) 模型文件到 [model](./model/) 目录。

### 2.2 量化示例

```bash
sh ./scripts/run_convert_qat.sh
```
量化成功后，在当前目录会生成量化日志文件 [./amct_log/amct_tensorflow.log](./amct_log/amct_tensorflow.log) 和 [./output/](./output/) 文件夹，文件夹内包含以下内容：
+ [result_quant.json](./output/result_quant.json): 量化信息文件，记录了量化模型同原始模型节点的映射关系，用于量化后模型同原始模型精度比对使用。
+ [result_quantized.pb](./output/result_quantized.pb): 量化模型，可在 TensorFlow 环境进行精度仿真并可在昇腾 AI 处理器部署。
