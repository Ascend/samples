# AMCT_Caffe MobileNet-V2 基于精度的自动量化回退样例

## 1， 准备工作

### 1.1 AMCT_Caffe 环境

执行本用例需要配置 AMCT_Caffe python 环境，详细环境准备流程可以参照[昇腾社区开发者文档](https://ascend.huawei.com/zh/#/document?tag=developer)页面下“全流程开发工具链” 下的 “模型压缩 (Caffe)” 文档进行配置。

### 1.2 原始模型 (Pre-trained model)

本样例依赖指定的原始模型定义文件 (.prototxt) 与预训练权重文件 (.caffemodel)。通过执行以下命令进行下载：

```bash
python3.7.5 ./src/download_prototxt.py \
    --caffe_dir CAFFE_DIR \
    --close_certificate_verify 
```

入参说明：

* `caffe_dir`: 必填 Caffe 源代码路径，支持相对路径和绝对路径。
* `close_certificate_verify`: 可选 关闭证书验证参数，确保模型正常下载

若执行成功，将会在该样例根目录下创建 model 路径，并下载 `mobilenet_v2_deploy.prototxt` 与 `mobilenet_v2.caffemodel` 到该路径下。

若用户环境无法连接网络。则请先手动下载相应文件上传到创建的 model 路径下：

```bash
wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mobilenetV2/mobilenet_v2_deploy.prototxt
wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mobilenetV2/mobilenet_v2.caffemodel
```

### 1.3 数据集

#### 1.3.1 准备 ImageNet LMDB 数据集

自动量化回退过程中，需要不断对模型进行校准和测试，因此需要准备数据集，本示例所使用的数据集为 LMDB 格式的 ImageNet 数据集，关于数据集的下载以及制作请参见 Caffe 工程caffe-master/examples/imagenet/readme.md文件或者参见 [Caffe 官方 Github 链接](https://github.com/BVLC/caffe/tree/master/examples/imagenet)。

#### 1.3.2 准备量化校准集

该示例提供了一组样例校准集用于量化校准，可切换到该样例根目录下执行以下命令获取校准数据与标签:

```bash
cd data
wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/amct_caffe/imagenet_calibration.tar.gz
tar -xvf imagenet_calibration.tar.gz
```

## 2， 执行样例

切换到该样例根目录下，执行如下命令进行基于精度的自动量化回退，默认目标精度标准为 0.2%，即量化后精度下降不大于 0.2%。

如果使用支持 GPU 的设备：

```bash
python3.7.5 ./src/auto_calibration_mobilenet_v2_sample.py \
    --model_file model/mobilenet_v2_deploy.prototxt \
    --weights_file model/mobilenet_v2.caffemodel \
    --gpu 0 \
    --caffe_dir path/to/caffe-master \
    --dataset /data/Datasets/imagenet/ilsvrc12_val_lmdb
```

或者仅使用 CPU 执行：

```bash
python3.7.5 ./src/auto_calibration_mobilenet_v2_sample.py \
    --model_file model/mobilenet_v2_deploy.prototxt \
    --weights_file model/mobilenet_v2.caffemodel \
    --caffe_dir path/to/caffe-master \
    --dataset /data/Datasets/imagenet/ilsvrc12_val_lmdb
```

入参说明:

* `model_file`: 必填。Caffe 模型文件 (.prototxt) 路径。
* `weights_file`: 必填。Caffe 权重文件 (.caffemodel) 路径。
* `caffe_dir`: 必填。Caffe 源代码路径，支持相对路径和绝对路径。
* `gpu`/`cpu`: 可选。指定推理时使用 CPU 或则使用的 GPU 设备 ID。
* `dataset`: 必填。用于验证精度的 lmdb 格式 ImageNet validation 集路径。

若出现如下信息则说明量化成功（如下推理精度只是样例，请以实际环境量化结果为准）：

```none
2021-01-06 11:24:14,809 - INFO - [AMCT]:[AMCT]: Accuracy of original model is         0.7116875
2021-01-06 11:24:14,810 - INFO - [AMCT]:[AMCT]: Accuracy of global quantized model is 0.70678125
2021-01-06 11:24:14,810 - INFO - [AMCT]:[AMCT]: Accuracy of saved model is            0.710125
2021-01-06 11:24:14,810 - INFO - [AMCT]:[AMCT]: The generated model is stored in dir: xxx/sample/mobilenetV2/results
2021-01-06 11:24:14,810 - INFO - [AMCT]:[AMCT]: The records file is stored in dir:    xxx/sample/mobilenetV2/tmp
2021-01-06 11:24:14,810 - INFO - [AMCT]:[AMCT]: *****************************************************
2021-01-06 11:24:14,810 - INFO - [AMCT]:[AMCT]: #>Func name: accuracy_based_auto_calibration
2021-01-06 11:24:14,811 - INFO - [AMCT]:[AMCT]: #>Cost time: 4 hours, 3 minutes, 23.26 seconds
2021-01-06 11:24:14,811 - INFO - [AMCT]:[AMCT]: *****************************************************
```
