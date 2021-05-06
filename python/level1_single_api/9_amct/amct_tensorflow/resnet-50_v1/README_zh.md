# 基于 量化感知训练 的量化

## 量化前提

+ **模型准备**
请下载
[ResNet-50](https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet-50_v1_retrain/pre_model.zip)
模型文件。解压并将 pre_model 文件夹内的文件放到 [model](./model/) 目录。其中
ResNet50_train.meta 用于重训练，ResNet50_eval.meta 用于验证。

+ **数据集准备**
由于重训练需要使用大量数据对量化参数进行进一步优化，因此重训练数据需要与模型训练数据一致。ResNet-50 的数据集是在 ImageNet 的子集 ILSVRC-2012-CLS 上训练而来，因此需要用户自己准备 TFRecord 格式的数据集。如果更换其他数据集，则需要自己进行数据预处理。

## 量化示例

执行量化示例前，请先检查当前目录下是否包含以下文件及目录：

+ [model/](./model/)
  + [resnet_v1_50.data-00000-of-00001](./model/resnet_v1_50.data-00000-of-00001)
  + [resnet_v1_50.index](./model/resnet_v1_50.index)
  + [resnet_v1_50_eval.meta](./model/resnet_v1_50_eval.meta)
  + [resnet_v1_50_train.meta](./model/resnet_v1_50_train.meta)
+ [resnet-50_v1_retrain.py](./src/resnet-50_v1_retrain.py)

并根据 requirements 安装必要的环境依赖。

在当前目录执行如下命令重训练 resnet_v1_50 网络模型。

```none
python3.7.5 ./src/resnet-50_v1_retrain.py --config_defination CONFIG_DEFINATION --batch_num BATCH_NUM --train_set TRAIN_SET --train_keyword TRAIN_KEYWORD --eval_set EVAL_SET --eval_keyword EVAL_KEYWORD --train_model TRAIN_MODEL --eval_model EVAL_MODEL
```

上述命令只给出了常用的参数，不常用参数以及各个参数解释请参见如下表格：

| 参数 | 必填项 | 数据类型 | 默认值 | 参数解释 |
| :-- | :-: | :-: | :-: | :-- |
| -h | 否 | / | / | 显示帮助信息。 |
| --config_defination CONFIG_DEFINATION | 否 | string | None | 量化的简易配置文件路径。可使用 './src/sample.cfg'，ResNet网络样例首尾层INT8，中间层INT4量化配置。|
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

如果测试数据集和验证数据集位于同一路径下，为确保量化过程中使用了正确的数据集，该场景下量化命令中需要追加 --train_keyword TRAIN_KEYWORD 和 --eval_keyword EVAL_KEYWORD 参数，根据上述两个参数过滤相关文件名，确保 --train_set 参数使用的是测试数据集，--eval_set 使用的是验证数据集。

本示例脚本仅用于展示量化感知训练流程，可以增大迭代训练次数（参数--train_iter）来缩小量化后精度损失。若出现如下信息则说明重训练成功。

```none
The model after retrain top 1 accuracy = 52.0%.
The model after retrain top 5 accuracy = 77.4%.
```

## 量化结果

重训练成功后，在当前目录会生成以下文件：

+ [amct_tensorflow.log](./amct_log/amct_tensorflow.log)：记录了工具的日志信息，包括重训练过程的日志信息。
+ [resnet_v1_50_quantized.pb](./results/resnet_v1_50_quantized.pb)：重训练完成最后生成的量化模型。该模型即可在 TensorFlow 环境进行精度仿真又可在昇腾AI处理器部署。
+ [tmp/](./tmp/)：重训练过程中产生的文件，包括：
  + [checkpoint](./tmp/checkpoint)：TensorFlow 训练过程中保存的检查点。
  + [config.json](./tmp/config.json)：重训练量化配置文件，描述了如何对模型中的每一层进行重训练。如果重训练脚本所在目录下已经存在重训练配置文件，则再次调用 create_quant_retrain_config 接口时，如果新生成的重训练配置文件与已有的文件同名，则会覆盖已有的重训练配置文件，否则生成新的重训练配置文件。
  + [events.out.tfevents.xxxxxxxxxx.xxx](./tmp/events.out.tfevents.xxxxxxxxxx.xxx)：包含重训练时的 loss 信息，可使用 TensorBoard 查看。
  + [record.txt](./tmp/record.txt)：量化因子记录文件。关于该文件的原型定义请参见量化因子记录文件说明。
  + [resnet_v1_50.pb](./tmp/resnet_v1_50.pb)：重训练结束后生成的固化模型。
  + [resnet_v1_50_retrain-0.data-00000-of-00001](./tmp/resnet_v1_50_retrain-0.data-00000-of-00001)：重训练过程生成的权重文件。
  + [resnet_v1_50_retrain-0.index](./tmp/resnet_v1_50_retrain-0.index)：重训练过程生成的权重文件索引。
  + [resnet_v1_50_retrain-0.meta](./tmp/resnet_v1_50_retrain-0.meta)：用于重训练的模型文件。
  + [resnet_v1_50_retrain-100.data-00000-of-00001](./tmp/resnet_v1_50_retrain-1.data-00000-of-00001)：重训练过程生成的权重文件。
  + [resnet_v1_50_retrain-100.index](./tmp/resnet_v1_50_retrain-1.index)：重训练过程生成的权重文件索引。
  + [resnet_v1_50_retrain-100.meta](./tmp/resnet_v1_50_retrain-1.meta)：用于重训练的模型文件。

对该模型重新进行重训练时，上述结果文件将会被覆盖。
