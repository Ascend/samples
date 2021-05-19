# ResNet-50 量化 sample 指导

amct_mindspore 工具 sample 运行指导

### 环境要求
1. Ascend 910 host 环境；

2. 正确安装了 mindspore ascend 版本和与之配套的 amct_mindspore 工具；


## Usage

### 1. 准备 resnet50 预训练 checkpoint 

你可以在mindspore的官网下载 ResNet50 的预训练 checkpoint 文件，训练基于 CIFAR-10 dataset；

[resnet50 mindspore pretrain ckpt download page](https://www.mindspore.cn/resources/hub/details?MindSpore/ascend/0.7/resnet50_v1.5_cifar10)

将 resnet50.ckpt 文件放到 model 目录；

### 2. 准备 CIFAR-10 数据集

Dataset used: [CIFAR-10](http://www.cs.toronto.edu/~kriz/cifar.html)

- 数据集大小：60,000 32*32 10种类的彩色图片
  - Train：50,000 images
  - Test： 10,000 images
  
- 数据集格式：二进制文件

  
- 下载了 cifar-10 数据集后，进行一下的处理：

  ```shell
  tar -zxvf cifar-10-binary.tar.gz
  mkdir cifar-10-verify-bin
  cp cifar-10-batches-bin/batches.meta.txt ./cifar-10-verify-bin/
  cp cifar-10-batches-bin/readme.html ./cifar-10-verify-bin/
  cp cifar-10-batches-bin/test_batch.bin ./cifar-10-verify-bin/
  ```

  处理后的文件目录结构如下：

```
├─cifar-10-batches-bin
│      batches.meta.txt
│      data_batch_1.bin
│      data_batch_2.bin
│      data_batch_3.bin
│      data_batch_4.bin
│      data_batch_5.bin
│      readme.html
│
└─cifar-10-verify-bin
        batches.meta.txt
        readme.html
        test_batch.bin
```

**文件说明：**

- scripts/download_files.py：用于下载模型定义文件及工具类脚本的处理脚本。
- scripts/run_standalone_train_sample.sh：单device训练场景下，量化感知训练脚本。
- scripts/run_distribute_train_sample.sh：分布式训练场景下，量化感知训练的脚本。
- src/resnet50_sample.py ：训练后量化脚本。
- src/resnet50_retrain_sample.py：量化感知训练脚本。
- src/retrain_config.py：量化感知训练超参。
- README.md：量化示例使用说明文件。


### 3. 下载模型定义文件

  切换到 scripts 目录下，执行如下的命令下载模型定义文件：

  `python3.7.5 download_files.py --close_certificate_verify`

其中 --close_certificate_verify 参数可选，用于关闭证书验证，推荐在不关闭情况下下载，如果提示证书认证失败再添加该参数重新下载。

若出现如下的信息则说明下载成功：

```
# resnet50 模型定义文件
[INFO]Download file to "../src/resnet.py" success.
# 数据与处理脚本，执行量化脚本时，会调用该脚本进行数据集的预处理
[INFO]Download file to "../src/dataset.py" success.
# 控制学习率脚本，量化感知训练过程会调用该文件的方法，控制学习率
[INFO]Download file to "../src/lr_generator.py" success.
# 分布式训练场景使用的脚本，用于生成 hccl 配置文件
[INFO]Download file to "../src/hccl_tools.py" success.
```

### 4. 训练后量化示例

1. 执行训练后量化：

```shell
cd src
python3 resnet50_sample.py --dataset_path your_dataset_path/cifar-10-verify-bin  --checkpoint_path your_resnet50_checkpoint_file_path

注：上述命令中的 your_dataset_path your_resnet50_checkpoint_file_path 分别对应实际环境的 cifar10 数据集路径和预训练 ckpt 文件路径；

```

参数解释：

表1 量化脚本所用参数说明

| 参数              | 说明                                   | 是否必填 |
| ----------------- | -------------------------------------- | -------- |
| -h                | 显示帮助信息                           | 否       |
| --dataset_path    | 数据集路径： 如 ../cifar-10-verify-bin | 是       |
| --checkpoint_path | MindSpore 权重文件 resnet50.ckpt 路径  | 是       |

2. 提示如下信息则说明量化成功

```shell
INFO - [AMCT]:[QuantizeTool]：Generate AIR file : ../src/results/resnet50_quant.air success!
[INFO] the quantized AIR file has been stored at:
results/resnet50_quant.air
```

3. 量化结果说明

   量化成功后，在 Resne50/src 目录下生成如下文件：

   - config.json: 量化配置文件，描述了如何对模型中的每一层进行量化。如果量化脚本所在目录下已经存在量化配置文件，则再次调用 create_quant_config 接口时，如果新生成的量化配置文件和已有的文件重名会覆盖已有的量化配置文件，否则重新生成量化配置文件。
   - amct_log/acmt_mindspore.log ：量化日志文件，记录了量化过程的日志信息。
   - results/resnet50_quant.air: 量化结果文件，可以通过 ATC 生成昇腾AI处理器上的可部署的模型文件。
   - kernel_meta: 算子编译生成的文件目录
   - （可选）amct_dump/calibration_record.txt : 如果执行量化时，设置了 export DUMP_AMCT_RECORD=1的环境变量，则在量化脚本的同级目录还会生成量化因子目录，该目录下的量化因子记录文件 calibration_record.txt，记录了每个量化层的量化因子。



### 5. 单 device 量化感知训练示例

1. 量化感知训练有两种方式，一是使用 resnet50_retrain_sample.py 脚本量化，该方式需要配置多个参数，另一种是使用该脚本的封装脚本 run_standalone_train_sample.sh 进行量化，该方式配置参数较少，用户根据实际情况选择一种方式进行量化。

```shell
单卡量化感知训练：
cd src
python3.7.5 resnet50_retrain_sample.py [-h][--net NET][--dataset DATASET][--run_distribute RUN_DISTRIBUTE][--device_num DEVICE_NUM][--device_target DEVICE_TARGET][--eval_dataset EVAL_DATASET][--train_dataset TRAIN_DATASET][--pre_trained PRE_TRAIN][--air AIR]
```

2. 量化脚本所用参数：

| 参数             | 说明                                             |
| ---------------- | --------------------------------------------------|
| -h               | 是否必填：否                                     |
|                  | 参数解释：显示帮助信息                             |
| --net            | 是否必填：否                                       |
|                  | 数据类型：string                                   |
|                  | 默认值: resnet50                                   |
|                  | 参数解释：选定网络为 resnet50，目前仅支持resnet50  |
| --dataset        | 是否必填：否                                       |
|                  | 数据类型：string                                   |
|                  | 默认值：cifar10                                    |
|                  | 参数解释：指定数据集，目前只支持 cifar10           |
| --run_distribute | 是否必填：否                                       |
|                  | 数据类型： bool                                    |
|                  | 默认是：False                                      |
|                  | 参数解释：是否采用多卡训练                         |
| --device_num     | 是否必填：否                                       |
|                  | 数据类型： int                                     |
|                  | 默认值: 1                                          |
|                  | 参数解释：当前host对应device 数量                  |
| --device_target  | 是否必填：否                                       |
|                  | 数据类型：string                                   |
|                  | 默认值: Ascend                                     |
|                  | 参数解释：指定执行的设备为Ascend后端               |
| --pre_trained    | 是否必填：是                                       |
|                  | 数据类型：string                                   |
|                  | 默认值：None                                       |
|                  | 参数解释：量化感知训练使用到的预训练 ckpt 文件路径 |
| --eval_dataset   | 是否必填：是                                       |
|                  | 数据类型：string                                   |
|                  | 默认值：None                                       |
|                  | 参数解释：测试集路径                               |
| --train_dataset  | 是否必填：是                                       |
|                  | 数据类型: string                                   |
|                  | 默认值: None                                       |
|                  | 参数解释：训练集路径                               |
| --air            | 是否必填：否                                       |
|                  | 数据类型: string                                   |
|                  | 默认值： resnet50_quant_retrain                    |
|                  | 参数解释：最终导出的 AIR 格式文件名                |
| --original       | 是否必填：否                                       |
|                  | 数据类型：无                                       |
|                  | 默认值：无                                         |
|                  | 参数解释：若指定该参数则执行原始模型的精度测试流程 |

使用样例如下：

```shell
cd src
python3.7.5 resnet50_retrain_sample.py --train_dataset ../cifar-10-batches-bin --eval_dataset ../cifar-10-verify-bin --pre_trained ../resnet50.ckpt
```

- run_standalone_train_sample.sh 封装脚本进行量化

  ```shell
  cd scripts
  sh run_standalone_train_sample.sh [resnet50] [cifar10] [TRAIN_DATASET_PATH] [EVAL_DATASET_PATH] [PRETRAINED_CKPT_PATH]
  ```

  参数说明：
| 参数                   | 说明                                               |
| ---------------------- | -------------------------------------------------- |
| -h                     | 是否必填：否                                       |
|                        | 参数解释：显示帮助信息                             |
| [resnet50]             | 是否必填：否                                       |
|                        | 数据类型：string                                   |
|                        | 默认值: resnet50                                   |
|                        | 参数解释：选定网络为 resnet50，目前仅支持resnet50  |
| [cifar10]              | 是否必填：否                                       |
|                        | 数据类型：string                                   |
|                        | 默认值：cifar10                                    |
|                        | 参数解释：指定数据集，目前只支持 cifar10           |
| [TRAIN_DATASET_PATH]   | 是否必填：是                                       |
|                        | 数据类型：string                                   |
|                        | 默认值：None                                       |
|                        | 参数解释：训练集路径                               |
| [EVAL_DATASET_PATH]    | 是否必填：是                                       |
|                        | 数据类型: string                                   |
|                        | 默认值: None                                       |
|                        | 参数解释：测试集路径                               |
| [PRETRAINED_CKPT_PATH] | 是否必填：是                                       |
|                        | 数据类型：string                                   |
|                        | 默认值：None                                       |
|                        | 参数解释：量化感知训练使用到的预训练 ckpt 文件路径 |
使用示例如下：

```shell
bash run_standalone_train_sample.sh resnet50 cifar10 ../cifar-10-batches-bin  ../cifar-10-verify-bin ../resnet50.ckpt
```

若出现以下信息，则说明量化感知训练成功：

```
INFO-[AMCT]:[QuantizeTool]: Generate AIR file: xxx/resnet50_quant_geir.air success
```

3. 结果展示：

执行量化感知训练成功后，在执行目录下生成如下的文件：

- resnet50_quant_retrain.air: 执行量化感知训练完成后可以在昇腾AI芯片部署的量化模型。
- retrain_quant_config.json： 量化配置文件，描述了对每一层进行量化感知训练参数的配置。如果量化感知训练脚本所在目录下已经存在量化配置文件，则再次调用 create_quant_retrain_config 接口时，如果新生成的量化配置文件和已有的文件重名会覆盖已有的量化配置文件，否则重新生成量化配置文件。如果精度不满足要求，用户可以修改 retrain_quant_config.json 中的量化参数配置，具体的量化配置项解释可以参考amct_mindspore工具使用说明书。
- amct_log/amct_mindspore.log: 记录量化感知训练过程的日志信息。
- retrain_result: 执行量化感知训练过程中产生的文件。
- kernel_meta: 算子编译生成的文件目录。

由于量化感知训练过程中也会进行训练后量化，故量化感知训练后，也会同时产生训练后量化的结果文件，比如量化配置文件config.josn. amct_dump，关于文件介绍参考 训练后量化示例的介绍。

### 6. 多 device 量化感知训练示例

1. 生成hccl 分布式配置文件。

   hccl分布式配置文件依赖训练环境device侧 /etc 目录下的 hccn.conf 配置文件，所以生成 hccl 分布式配置文件之前，请确保训练环境已经搭建好，并且存在 hccn.conf 配置文件。

   切换到src 目录下，执行如下命令:

   ```shell
   python3.7.5 hccl_tools.py --device_num DEVICE_NUM --visible_devices VISIBLE_DEVICES --server_ip SERVER_IP
   ```

   参数说明：

   | 参数              | 说明                                                         |
   | ----------------- | ------------------------------------------------------------ |
   | -h                | 是否必填：否                                                 |
   |                   | 参数解释：显示帮助信息                                       |
   | --device_num      | 是否必填：是                                                 |
   |                   | 数据类型：string                                             |
   |                   | 默认值： “[0,8)”                                             |
   |                   | 参数解释：训练场景使用device个数，取值范围是[0,8)，最多支持8个device。取值必须连续，例如[0,4)，则表示使用0，1，2，3四个device，不允许跨device使用，例如[3,6)，取值需放在双引号内 |
   | --visible_devices | 是否必填：否                                                 |
   |                   | 数据类型： string                                            |
   |                   | 默认值： "0,1,2,3,4,5,6,7"                                   |
   |                   | 参数解释：按顺序的可用设备                                   |
   | --server_ip       | 是否必填：否                                                 |
   |                   | 数据类型: string                                             |
   |                   | 默认值： None                                                |
   |                   | 参数解释：训练host服务器的ip                                 |

   使用示例：

   ```
   python3.7.5 hccl_tools.py --device_num "[0,8)"
   ```

   执行上述命令后，在 src 当前目录生成 hccl_xxx.json 文件，示例，hccl_1p_0_127.0.0.1.json文件，该文件记录了device0的IP地址，以及device ID等信息，用于决定后续执行训练使用的device。

2. 分布式训练场景下执行量化感知训练：

   切换到 scripts 目录，执行如下命令：

   ```shell
   sh run_distribute_train_sample.sh [resnet50] [cifar10] [RANK_TABLE_FILE] [TRAIN_DATASET_PATH] [EVAL_DATASET_PATH] [PRETRAINED_CKPT_PATH]
   ```

   分布式量化感知训练脚本所使用的参数说明：

| 参数                   | 说明                                                |
| ---------------------- | --------------------------------------------------- |
| -h                     | 是否必填：否                                        |
|                        | 参数解释：显示帮助信息                              |
| [resnet50]             | 是否必填：是                                        |
|                        | 数据类型：string                                    |
|                        | 默认值： resnet50                                   |
|                        | 参数解释：执行训练用的网络，目前仅支持 resnet50     |
| [cifar10]              | 是否必填：是                                        |
|                        | 数据类型：string                                    |
|                        | 默认值：cifar10                                     |
|                        | 参数解释： 指定训练使用的数据集。目前仅支持cifar 10 |
| [RANK_TABLE_FILE]      | 是否必填：是                                        |
|                        | 数据类型：string                                    |
|                        | 默认值：None                                        |
|                        | 参数解释：通过hccl_tools.py生成的hccl配置文件       |
| [TRAIN_DATASET_PATH]   | 是否必填：是                                        |
|                        | 数据类型：string                                    |
|                        | 默认值：None                                        |
|                        | 参数解释：训练集路径                                |
| [EVAL_DATASET_PATH]    | 是否必填：是                                        |
|                        | 数据类型：string                                    |
|                        | 默认值：None                                        |
|                        | 参数解释：测试集路径                                |
| [PRETRAINED_CKPT_PATH] | 是否必填：是                                        |
|                        | 数据类型：string                                    |
|                        | 默认值：None                                        |
|                        | 参数解释：量化感知训练使用到的预训练 ckpt 文件路径  |

使用示例如下：

```
cd scripts
bash run_distribute_train_sample.sh resnet50 cifar10 hccl_1p_0_127.0.0.1.json ../cifar-10-batches-bin  ../cifar-10-verify-bin ../resnet50.ckpt
```

3. 分布式量化感知训练结果展示

   执行量化感知训练完成后，在 scripts 目录下生成针对每个 device 的量化感知训练结果，例如train_parallel0, train_parallel1, train_parallel2....

   进入train_parallel0可以查看到结果文件：

   - resnet50_quant_retrain.air: 执行量化感知训练完成后可以在昇腾AI芯片部署的量化模型。
   - retrain_quant_config.json： 量化配置文件，描述了对每一层进行量化感知训练参数的配置。如果量化感知训练脚本所在目录下已经存在量化配置文件，则再次调用 create_quant_retrain_config 接口时，如果新生成的量化配置文件和已有的文件重名会覆盖已有的量化配置文件，否则重新生成量化配置文件。如果精度不满足要求，用户可以修改 retrain_quant_config.json 中的量化参数配置，具体的量化配置项解释可以参考amct_mindspore工具使用说明书。
   - amct_log/amct_mindspore.log: 记录量化感知训练过程的日志信息。
   - retrain_result: 执行量化感知训练过程中产生的文件。
   - kernel_meta: 算子编译生成的文件目录。
   - retrain_resultckpt_0: 保存训练过程中的checkpoint 文件。
   - log: 分布式训练的打屏信息，重定向到文件中的内容。
   - env.log: MindSpore 相关的日志信息。