# 量化

## 量化前提

+ **模型准备**  
  + 请至 [昇腾社区-ModelZoo](https://ascend.huawei.com/zh/#/software/modelzoo/detail/1/2a3e8d64cf7e48249246140ddbe4135f) 下载 MobileNetV2 模型文件。解压并将其中的 mobilenetv2_tf.pb 文件放到 [model](./model/) 目录下。
  + 请下载 [MobileNetV2 QAT](https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mobilenetv2_convert_qat/mobilenetv2_qat.pb) 模型文件到 [model](./model/) 目录。

+ **数据集准备**  
  + 使用昇腾模型压缩工具对模型完成量化后，需要对模型进行推理，以测试量化数据的精度。推理过程中需要使用和模型相匹配的数据集。请下载测试图片 [classification.jpg](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/mobilenet_v2_calibration/classification.jpg) 和 [convert_qat.jpg](https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mobilenetv2_convert_qat/convert_qat.jpg)，并将该图片放到 [data](./data/) 目录下。
  + 自动量化回退过程中，需要不断的对模型进行校准和测试，因此需要用户准备数据集，本示例所采用的数据集为标准 TFRecord 格式的 ImageNet的 子集 ILSVRC-2012-CLS 的验证集，共有 50000 张图片，如果采用其他数据集，则需要用户自行修改 sample 文件中的数据预处理部分以匹配模型输入。

+ **校准集准备**  
校准集用来产生量化因子，保证精度。  
计算量化参数的过程被称为“校准(calibration)”。校准过程需要使用一部分测试图片来针对性计算量化参数，使用一个或多个 batch 对量化后的网络模型进行推理即可完成校准。请下载[校准集](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/mobilenet_v2_calibration/calibration.rar)，解压后将 calibration 文件夹放到 [data](./data/) 目录下。

## 量化示例

执行量化示例前，请先检查当前目录下是否包含以下文件及目录，其中 calibration 文件夹内部包含有 32 张用于校准的图片：

+ [data](./data/)
  + [calibration/](./data/calibration/)
  + [classification.jpg](./data/classification.jpg)
  + [convert_qat.jpg](./data/convert_qat.jpg)
  + [record_quantized.txt](./data/record_quantized.txt)
+ [model](./model/)
  + [mobilenetv2_qat.pb](./model/mobilenetv2_qat.pb)
  + [mobilenetv2_tf.pb](./model/mobilenetv2_tf.pb)
+ [src](./src/)
  + [mobilenet_v2_accuracy_based_auto_calibration.py](./src/mobilenet_v2_accuracy_based_auto_calibration.py)
  + [mobilenet_v2_calibration.py](./src/mobilenet_v2_calibration.py)
  + [mobilenet_v2_convert_model.py](./src/mobilenet_v2_convert_model.py)
  + [mobilenet_v2_convert_qat.py](./src/mobilenet_v2_convert_qat.py)
  + [mobilenet_v2_perf_based_auto_calibration.py](./src/mobilenet_v2_perf_based_auto_calibration.py)

在当前目录执行如下命令运行示例程序：

+ 训练后量化

  ```none
  python ./src/mobilenet_v2_calibration.py
  ```

+ convert model 接口

  ```none
  python ./src/mobilenet_v2_convert_model.py
  ```

+ QAT 模型转 Ascend 模型

  ```none
  python ./src/mobilenet_v2_convert_qat.py
  ```

+ 基于精度的自动量化回退

  ```none
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

+ 基于性能的自动量化回退

  ```none
  python ./src/mobilenet_v2_perf_based_auto_calibration.py
  ```

  上述命令只给出了常用的参数，不常用参数以及各个参数解释请参见如下表格：

  | 参数 | 必填项 | 数据类型 | 默认值 | 参数解释 |
  | :-- | :-: | :-: | :-: | :-- |
  | -h | 否 | / | / | 显示帮助信息。 |
  | --sampler_config_file | 否 | string | ./src/perf_conf/mobilenet_v2_sampler_config.cfg | 性能采样配置文件路径。 |
  | --cfg_define | 否 | string | None | 建议配置文件路径。 |

若出现如下信息则说明模型量化成功：

+ 训练后量化

  ```none
  INFO - [AMCT]:[save_model]: The model is saved in ./outputs/calibration/mobilenet_v2_quantized.pb
  Origin Model Prediction:
          category index: 699
          category prob: 0.560
  Quantized Model Prediction:
          category index: 699
          category prob: 0.568
  ```

+ convert model 接口

  ```none
  INFO - [AMCT]:[save_model]: The model is saved in ./outputs/convert_model/mobilenet_v2_quantized.pb
  Origin Model Prediction:
          category index: 699
          category prob: 0.560
  Quantized Model Prediction:
          category index: 699
          category prob: 0.569
  ```

+ QAT 模型转 Ascend 模型

  ```none
  INFO - [AMCT]:[save_model]: The model is saved in ./outputs/convert_qat/mobilenet_v2_quantized.pb
  Origin Model Prediction:
          category index: 346
          category prob: 0.650
  Quantized Model Prediction:
          category index: 346
          category prob: 0.246
  ```

+ 基于精度的自动量化回退

  ```none
  INFO - [AMCT]:[AMCT]: Accuracy of original model is         74.97
  INFO - [AMCT]:[AMCT]: Accuracy of global quantized model is 73.974
  INFO - [AMCT]:[AMCT]: Accuracy of saved model is            74.646
  INFO - [AMCT]:[AMCT]: The generated model is stored in dir: ./outputs/accuracy_based_auto_calibration
  INFO - [AMCT]:[AMCT]: The records is stored in dir: ./outputs/accuracy_based_auto_calibration
  ```

+ 基于性能的自动量化回退

  ```none
  INFO - [AMCT]:[AMCT]: The generated model is stored in dir: ./outputs/perf_based_auto_calibration
  INFO - [AMCT]:[AMCT]: The records is stored in dir: ./outputs/perf_based_auto_calibration
  Origin Model Model Prediction:
          category index: 699
          category prob: 0.560
  Quantized Model Model Prediction:
          category index: 699
          category prob: 0.595
  ```

## 量化结果

量化成功后，在当前目录会生成量化日志文件 [amct_tensorflow.log](./amct_log/amct_tensorflow.log) 和 [outputs](./outputs/) 文件夹，该文件夹内包含以下内容：

+ 训练后量化 [calibration](./outputs/calibration/)
  + 量化配置文件 [config.json](./outputs/calibration/config.json)
  + 量化因子记录文件 [record.txt](./outputs/calibration/record.txt)
  + 量化信息文件 [mobilenet_v2_quant.json](./outputs/calibration/mobilenet_v2_quant.json)
  + 量化模型 [mobilenet_v2_quantized.pb](./outputs/calibration/mobilenet_v2_quantized.pb)
+ QAT 模型转 Ascend 模型 [convert_qat](./outputs/convert_qat/)
  + 量化因子记录文件 [record.txt](./outputs/convert_qat/record.txt)
  + 量化信息文件 [mobilenet_v2_quant.json](./outputs/convert_qat/mobilenet_v2_quant.json)
  + 量化模型 [mobilenet_v2_quantized.pb](./outputs/convert_qat/mobilenet_v2_quantized.pb)

量化日志文件记录了量化过程的日志信息。

量化配置文件描述了如何对模型中的每一层进行量化。如果量化脚本所在目录下已经存在量化配置文件，则再次调用 create_quant_config 接口时，如果新生成的量化配置文件与已有的文件同名，则会覆盖已有的量化配置文件，否则生成新的量化配置文件。

量化因子记录文件记录量化因子。关于该文件的原型定义请参见
[量化因子记录文件说明](https://support.huaweicloud.com/content/dam/cloudbu-site/archive/china/zh-cn/support/docs/auxiliarydevtool-cann330alphaXinfer/atlasamcttf_16_0014.html)。

量化信息文件记录了量化模型同原始模型节点的映射关系，用于量化后模型同原始模型精度比对使用。

量化模型即可在 TensorFlow 环境进行精度仿真并可在昇腾 AI 处理器部署的模型。

对该模型重新进行量化时，在量化后模型的同级目录下生成的上述结果文件将会被覆盖。
