# 量化

## 量化前提

+ **模型准备**
请下载 [ResNet-101](https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet-101_nuq/resnet101-5d3b4d8f.pth) 模型文件到 [model](./model/) 目录。

+ **数据集准备**
使用昇腾模型压缩工具对模型完成量化后，需要对模型进行推理，以测试量化数据的精度。推理过程中需要使用和模型相匹配的数据集。请下载[测试图片](https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet-101_nuq/images.zip)，解压后将 “images” 文件夹放到 [data](./data/) 目录下。
  > 由于重训练需要使用大量数据对量化参数进行进一步优化，因此重训练数据需要与模型训练数据一致。ResNet-101的数据集是在 ImageNet的子集 ILSVRC-2012-CLS 上训练而来，因此需要用户自己准备 ImagenetPytorch 格式的数据集（获取方式请参见[此处](https://github.com/pytorch/examples/tree/master/imagenet)）。如果更换其他数据集，则需要自己进行数据预处理。

+ **校准集准备**
校准集用来产生量化因子，保证精度。本 sample 校准集与数据集相同。

## 量化示例

执行量化示例前，请先检查当前目录下是否包含以下文件及目录，其中 images 文件夹内部包含有 160 张用于校准和测试的图片：

+ [data](./data/)
  + [images](./data/images/)
+ [model](./model/)
  + [resnet101-5d3b4d8f.pth](./model/resnet101-5d3b4d8f.pth)
+ [src](./src/)
  + [nuq_files](./src/nuq_files/)
    + [nuq_quant.cfg](./src/nuq_files/nuq_quant.cfg)
    + [resnet-101_quantized.json](./src/nuq_files/resnet-101_quantized.json)
  + [retrain_conf](./src/retrain_conf/)
    + [retrain.cfg](./src/retrain_conf/retrain.cfg)
  + [\_\_init__.py](./src/__init__.py)
  + [resnet-101_calibration.py](./src/resnet-101_calibration.py)
  + [resnet-101_retrain.py](./src/resnet101_retrain.py)
  + [resnet.py](./src/resnet.py)

并根据 requirements 安装必要的环境依赖。

请在当前目录执行如下命令运行示例程序：

+ 训练后量化
  + 均匀量化

    ```none
    CUDA_VISIBLE_DEVICES=0 python ./src/resnet-101_calibration.py
    ```

  + 非均匀量化

    ```none
    CUDA_VISIBLE_DEVICES=0 python ./src/resnet-101_calibration.py --nuq
    ```

  > 其中 ```CUDA_VISIBLE_DEVICES``` 是必填参数，表示使用 CPU 还是 GPU 进行量化，参数取值为：
  >
  > + -1：使用 CPU 进行量化。
  > + 其他 Device ID使用 GPU 进行量化，具体 ID 请以用户实际环境为准。当前仅支持配置单 Device。

+ 量化感知训练
  + 单卡量化感知训练

    ```none
    CUDA_VISIBLE_DEVICES=0 python ./src/resnet-101_retrain.py --train_set TRAIN_SET --eval_set EVAL_SET --config_defination ./src/retrain_conf/retrain.cfg
    ```

  + 多卡量化感知训练

    ```none
    CUDA_VISIBLE_DEVICES=0,1,2,3,4,5,6,7 python ./src/resnet-101_retrain.py --train_set TRAIN_SET --eval_set EVAL_SET --config_defination ./src/retrain_conf/retrain.cfg
    ```

  > 仅支持 distribution 模式的多卡训练，不支持 DataParallel 模式的多卡训练。如果使用 DataParallel 模式的多卡训练，会出现如下错误信息：
  >
  > ```none
  > RuntimeError: Output 54 of BroadcastBackward is a view and its base or another view of its base has been modified inplace. This view is the output of a function that returns multiple views. Such functions do not allow the output view to be modified inplace. You should replace the inplace operation by an out-of-place one.
  > ```

  上述命令只给出了常用的参数，不常用参数以及各个参数解释请参见如下表格：

  | 参数 | 必填项 | 数据类型 | 默认值 | 参数解释 |
  | :-- | :-: | :-: | :-: | :-- |
  | -h | 否 | / | / | 显示帮助信息。 |
  | --config_defination CONFIG_DEFINATION | 否 | string | None | 量化的简易配置文件路径。 |
  | --batch_num BATCH_NUM | 否 | int | 2 | retrain 量化推理阶段的 batch 数。 |
  | --train_set TRAIN_SET | 是 | string | None | 测试数据集路径。 |
  | --eval_set EVAL_SET | 是 | string | None | 验证数据集路径。 |
  |  --num_parallel_reads NUM_PARALLEL_READS | 否 | int | 4 | 用于读取数据集的线程数，根据硬件运算能力酌情调整。 |
  | --batch_size BATCH_SIZE | 否 | int | 25 | PyTorch 执行一次前向推理所使用的样本数量，根据内存或显存大小酌情调整。 |
  | --learning_rate LEARNING_RATE | 否 | float | 1e-5 | 学习率。 |
  | --train_iter TRAIN_ITER | 否 | int | 2000 | 训练迭代次数。 |
  | --print_freq PRINT_FREQ | 否 | int | 10 | 训练及测试信息的打印频率。 |
  | --dist_url DIST_URL | 否 | string | tcp://127.0.0.1:50011 | 初始化多卡训练通信进程的方法。 |
  | --distributed | 否 | / | / | 使用该参数表示进行多卡训练，否则不进行多卡训练。 |

若出现如下信息，则说明量化成功：

+ 训练后量化
  + 均匀量化

    ```none
    INFO - [AMCT]:[Utils]: The model file is saved in ./outputs/calibration/resnet-101_deploy_model.onnx
    INFO - [AMCT]:[Utils]: The model file is saved in ./outputs/calibration/resnet-101_fake_quant_model.onnx
    [INFO] ResNet101 before quantize top1:    0.8875 top5:    0.9625
    [INFO] ResNet101 after quantize  top1:     0.875 top5:    0.9625
    ```

  + 非均匀量化

    ```none
    INFO - [AMCT]:[Utils]: The model file is saved in ./outputs/nuq/resnet-101_deploy_model.onnx
    INFO - [AMCT]:[Utils]: The model file is saved in ./outputs/nuq/resnet-101_fake_quant_model.onnx
    [INFO] ResNet101 before quantize top1:    0.8875 top5:    0.9625
    [INFO] ResNet101 after quantize  top1:   0.88125 top5:    0.9625
    ```

+ 量化感知训练

  ```none
  [INFO] ResNet101 before retrain top1:77.37% top5:93.55%
  [INFO] ResNet101 after retrain top1:77.42% top5:93.56%
  ```

## 量化结果

量化成功后，在当前目录会生成量化日志文件 [./amct_log/amct_pytorch.log](./amct_log/amct_pytorch.log) 和 [outputs](./outputs/) 文件夹，该文件夹内包含以下内容：

+ 均匀量化 [calibration](./outputs/calibration/)
  + 临时文件夹 [tmp](./outputs/calibration/tmp/)
    + 量化配置文件 [config.json](./outputs/calibration/tmp/config.json)
    + 量化因子记录文件 [record.txt](./outputs/calibration/tmp/record.txt)
    + 临时模型文件 [modified_model.onnx](./outputs/calibration/tmp/modified_model.onnx)
  + 量化模型
    + [resnet-101_deploy_model.onnx](./outputs/calibration/resnet-101_deploy_model.onnx)
    + [resnet-101_fake_quant_model.onnx](./outputs/calibration/resnet-101_fake_quant_model.onnx)
+ 非均匀量化 [nuq](./outputs/nuq/)
  + 临时文件夹 [tmp](./outputs/nuq/tmp/)
    + 量化配置文件 [config.json](./outputs/nuq/tmp/config.json)
    + 量化因子记录文件 [record.txt](./outputs/nuq/tmp/record.txt)
    + 临时模型文件 [modified_model.onnx](./outputs/nuq/tmp/modified_model.onnx)
  + 量化模型
    + [resnet-101_deploy_model.onnx](./outputs/nuq/resnet-101_deploy_model.onnx)
    + [resnet-101_fake_quant_model.onnx](./outputs/nuq/resnet-101_fake_quant_model.onnx)
+ 量化感知训练 [retrain](./outputs/retrain/)
  + 临时文件夹 [tmp](./outputs/retrain/tmp/)
    + 量化配置文件 [config.json](./outputs/retrain/tmp/config.json)
    + 量化因子记录文件 [record.txt](./outputs/retrain/tmp/record.txt)
    + 临时模型文件 [modified_model.onnx](./outputs/retrain/tmp/modified_model.onnx)
  + 量化模型
    + [resnet-101_deploy_model.onnx](./outputs/retrain/resnet-101_deploy_model.onnx)
    + [resnet-101_fake_quant_model.onnx](./outputs/retrain/resnet-101_fake_quant_model.onnx)

量化配置文件描述了如何对模型中的每一层进行量化。如果量化脚本所在目录下已经存在量化配置文件，则再次调用 create_quant_config 接口时，如果新生成的量化配置文件与已有的文件同名，则会覆盖已有的量化配置文件，否则生成新的量化配置文件。

量化日志文件记录了量化过程的日志信息。

量化因子记录文件记录量化因子。关于该文件的原型定义请参见[量化因子记录文件说明](https://support.huaweicloud.com/content/dam/cloudbu-site/archive/china/zh-cn/support/docs/auxiliarydevtool-cann330alphaXinfer/atlasamcttf_16_0014.html)。

临时模型文件为原始的 PyTorch 模型 BN 融合后导出的 ONNX 模型文件。

resnet-101_deploy_model.onnx 为量化后的可在昇腾 AI 处理器部署的模型文件。

resnet-101_fake_quant_model.onnx 为量化后的可在 ONNX 执行框架 ONNX Runtime 进行精度仿真的模型文件。
