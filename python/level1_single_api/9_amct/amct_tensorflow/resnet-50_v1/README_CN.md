# ResNet-50 V1

## 1. 非均匀量化

### 1.1 量化前提

+ **模型准备**  
请至 [昇腾社区-ModelZoo](https://www.hiascend.com/zh/software/modelzoo/detail/1/7548422b6b9c4a809114435f6b128bb6) 下载 ResNet-50 V1 模型文件。解压并将其中的 resnet_v1_50.pb 文件放到 [model](./model/) 目录下。

+ **数据集准备**  
使用昇腾模型压缩工具对模型完成量化后，需要对模型进行推理，以测试量化数据的精度。推理过程中需要使用和模型相匹配的数据集。请下载测试图片 [classification.jpg](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/mobilenet_v2_calibration/classification.jpg)，并将该图片放到 [data](./data/) 目录下。

+ **校准集准备**  
校准集用来产生量化因子，保证精度。
计算量化参数的过程被称为“校准 (calibration)”。校准过程需要使用一部分测试图片来针对性计算量化参数，使用一个或多个 batch 对量化后的网络模型进行推理即可完成校准。请下载[校准集](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/mobilenet_v2_calibration/calibration.rar)，解压后将 calibration 文件夹放到 [data](./data/) 目录下。

### 1.2 量化示例

执行量化示例前，请先检查当前目录下是否包含以下文件及目录：

+ [data](./data/)
  + [calibration](./data/calibration/)
  + [classification.jpg](./data/classification.jpg)
+ [model/](./model/)
  + [resnet_v1_50.pb](./model/resnet_v1_50.pb)
+ [src](./src/)
  + [nuq_conf](./src/nuq_conf/)
    + [nuq_quant.cfg](src/nuq_conf/nuq_quant.cfg)
    + [resnet-50_v1_quantized.json](./src/nuq_conf/resnet-50_v1_quantized.json)
  + [resnet-50_v1_nuq.py](./src/resnet-50_v1_nuq.py)

在当前目录执行如下命令

```bash
python ./src/resnet-50_v1_nuq.py
```

若出现如下信息则说明模型量化成功：

```none
INFO - [AMCT]:[save_model]: The model is saved in ./outputs/nuq/resnet-50_v1_quantized.pb
Origin Model Prediction:
        category index: 111
        category prob: 0.118
Quantized Model Prediction:
        category index: 111
        category prob: 0.062
```

### 1.3 量化结果

量化成功后，在当前目录会生成量化日志文件 [./amct_log/amct_tensorflow.log](./amct_log/amct_tensorflow.log) 和 [./outputs/nuq](./outputs/nuq/) 文件夹，该文件夹内包含以下内容：

+ [amct_tensorflow_nuq_record.txt](./outputs/nuq/amct_tensorflow_nuq_record.txt): NUQ 量化层。
+ [config.json](./outputs/nuq/config.json): 量化配置文件，描述了如何对模型中的每一层进行量化。
+ [record.txt](./outputs/nuq/record.txt): 量化因子记录文件，记录量化因子。关于该文件的原型定义请参见[量化因子记录文件说明](https://support.huaweicloud.com/content/dam/cloudbu-site/archive/china/zh-cn/support/docs/auxiliarydevtool-cann330alphaXinfer/atlasamcttf_16_0014.html)。
+ [resnet-50_v1_quant.json](./outputs/nuq/resnet-50_v1_quant.json): 量化信息文件。
+ [resnet-50_v1_quantized.pb](./outputs/nuq/resnet-50_v1_quantized.pb): 量化模型，可在 TensorFlow 环境进行精度仿真并可在昇腾 AI 处理器部署。

> 对该模型重新进行量化时，在量化后模型的同级目录下生成的上述结果文件将会被覆盖。

## 2. 量化感知训练

### 2.1 量化前提

+ **模型准备**  
请下载 [ResNet-50](https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet-50_v1_retrain/pre_model.zip) 模型文件。解压并将 pre_model 文件夹内的文件放到 [model](./model/) 目录。其中 ResNet50_train.meta 用于重训练，ResNet50_eval.meta 用于验证。

+ **数据集准备**  
由于重训练需要使用大量数据对量化参数进行进一步优化，因此重训练数据需要与模型训练数据一致。ResNet-50 的数据集是在 ImageNet 的子集 ILSVRC-2012-CLS 上训练而来，因此需要用户自己准备 TFRecord 格式的数据集。

> 如果更换其他数据集，则需要自己进行数据预处理。

### 2.2 量化示例

执行量化示例前，请先检查当前目录下是否包含以下文件及目录：

+ [model/](./model/)
  + [resnet_v1_50.data-00000-of-00001](./model/resnet_v1_50.data-00000-of-00001)
  + [resnet_v1_50.index](./model/resnet_v1_50.index)
  + [resnet_v1_50_eval.meta](./model/resnet_v1_50_eval.meta)
  + [resnet_v1_50_train.meta](./model/resnet_v1_50_train.meta)
+ [src](./src/)
  + [retrain_conf](./src/retrain_conf/)
    + [sample_int8.cfg](./src/retrain_conf/sample_int8.cfg)
  + [resnet-50_v1_retrain.py](./src/resnet-50_v1_retrain.py)

在当前目录执行如下命令：

```bash
python ./src/resnet-50_v1_retrain.py --train_set TRAIN_SET --eval_set EVAL_SET --config_defination CONFIG
```

上述命令只给出了常用的参数，不常用参数以及各个参数解释请参见如下表格：

| 参数 | 必填项 | 数据类型 | 默认值 | 参数解释 |
| :-- | :-: | :-: | :-: | :-- |
| -h | 否 | / | / | 显示帮助信息。 |
| --config_defination CONFIG_DEFINATION | 是 | string | None | 量化的简易配置文件路径(```./src/retrain_conf/sample_int8.cfg```)。|
| --batch_num BATCH_NUM | 否 | int| 2 | retrain 量化推理阶段的 batch 数。 |
| --train_set TRAIN_SET | 是 | string | None | 测试数据集路径。 |
| --train_keyword TRAIN_KEYWORD | 否 | string | None | 用于筛选训练集路径下包含该关键词的文件，若未定义，则默认训练集路径下所有文件作为训练集。 |
| --eval_set EVAL_SET | 是 | string | None | 验证数据集路径。 |
| --eval_keyword EVAL_KEYWORD | 否 | string | None | 用于筛选训练集路径下包含该关键词的文件，若未定义，则默认验证集路径下所有文件作为验证集。 |
| --train_model TRAIN_MODEL | 是 | string | ./model/resnet_v1_50_train.meta | 训练用模型路径。 |
| --eval_model EVAL_MODEL | 是 | string | ./model/resnet_v1_50_eval.meta | 验证模型路径。 |
| --num_parallel_reads NUM_PARALLEL_READS | 否 | int | 4 | 用于读取数据集的线程数，根据硬件运算能力酌情调整。 |
| --buffer_size BUFFER_SIZE | 否 | int | 1000 | 数据集乱序的缓存大小，根据内存空间酌情调整。 |
| --repeat_count REPEAT_COUNT | 否 | int | 0 | 数据集重复次数，若为0则无限循环。 |
| --batch_size BATCH_SIZE | 否 | int | 32 | TensorFlow 运行一次所使用的样本数量，根据内存或显存大小酌情调整。 |
| --ckpt CKPT_PATH | 否 | string | ./model/resnet_v1_50 | ResNet-50 V1 模型的官方权重 checkpoint 文件路径。 |
| --learning_rate LEARNING_RATE | 否 | float | 1e-6 | 学习率。 |
| --save_interval SAVE_INTERVAL | 否 | int | 500 | 重训练保存间隔。 |
| --momentum MOMENTUM | 否 | float | 0.9 | RMSPropOptimizer优化器的动量。 |
| --train_iter TRAIN_ITER | 否 | int | 100 | 训练迭代次数。 |

> **注意**：如果测试数据集和验证数据集位于同一路径下，为确保量化过程中使用了正确的数据集，该场景下量化命令中需要追加 `--train_keyword TRAIN_KEYWORD` 和 `--eval_keyword EVAL_KEYWORD` 参数，根据上述两个参数过滤相关文件名，确保 `--train_set` 参数使用的是测试数据集，`--eval_set` 使用的是验证数据集。

若出现如下信息则说明模型量化成功：

```none
INFO - [AMCT]:[save_model]: The model is saved in ./outputs/retrain/resnet_v1_50_quantized.pb
The origin model top 1 accuracy = 66.2%
The origin model top 5 accuracy = 87.8%
The model after retrain top 1 accuracy = 46.4%.
The model after retrain top 5 accuracy = 71.0%.
```

> 本示例脚本仅用于展示量化感知训练流程，可以增大迭代训练次数 (`--train_iter`) 来缩小量化后精度损失。

### 2.3 量化结果

量化成功后，在当前目录会生成量化日志文件 [./amct_log/amct_tensorflow.log](./amct_log/amct_tensorflow.log) 和 [./outputs/retrain](./outputs/retrain/) 文件夹，该文件夹内包含以下内容：

+ [checkpoint](./outputs/retrain/checkpoint): 训练检查点。
+ [config.json](./outputs/retrain/config.json): 量化配置文件，描述了如何对模型中的每一层进行重训练。
+ [events.out.tfevents.xxxxxxxxxx.xxx](./outputs/retrain/events.out.tfevents.xxxxxxxxxx.xxx): 训练记录，包含重训练时的 loss 信息，可使用 TensorBoard 查看。
+ [record.txt](./outputs/retrain/record.txt): 量化因子记录文件，记录量化因子。关于该文件的原型定义请参见[量化因子记录文件说明](https://support.huaweicloud.com/content/dam/cloudbu-site/archive/china/zh-cn/support/docs/auxiliarydevtool-cann330alphaXinfer/atlasamcttf_16_0014.html)。
+ [resnet_v1_50.pb](./outputs/retrain/resnet_v1_50.pb): 训练后模型。
+ [resnet_v1_50_quantized.pb](./outputs/retrain/resnet_v1_50_quantized.pb): 量化模型，可在 TensorFlow 环境进行精度仿真并可在昇腾 AI 处理器部署。
+ [resnet_v1_50_retrain-0.data-00000-of-00001](./outputs/retrain/resnet_v1_50_retrain-0.data-00000-of-00001)
+ [resnet_v1_50_retrain-0.index](./outputs/retrain/resnet_v1_50_retrain-0.index)
+ [resnet_v1_50_retrain-0.meta](./outputs/retrain/resnet_v1_50_retrain-0.meta)
+ [resnet_v1_50_retrain-100.data-00000-of-00001](./outputs/retrain/resnet_v1_50_retrain-1.data-00000-of-00001)
+ [resnet_v1_50_retrain-100.index](./outputs/retrain/resnet_v1_50_retrain-1.index)
+ [resnet_v1_50_retrain-100.meta](./outputs/retrain/resnet_v1_50_retrain-1.meta)

> 对该模型重新进行量化感知训练时，上述结果文件将会被覆盖。
