# AMCT_Caffe ResNet-50示例

该示例包含 ResNet-50的均匀量化，静态非均匀量化，自动非均匀量化，以及自动非均匀量化示例。

## 1， 准备工作

### 1.1 AMCT_Caffe环境
执行本用例需要配置AMCT_Caffe python环境，详细环境准备流程可以参照[昇腾社区开发者文档](https://ascend.huawei.com/zh/#/document?tag=developer)页面下“全流程开发工具链” 下的 “模型压缩 (Caffe)” 文档进行配置。

### 1.2 原始模型(Pre-trained model)

本样例依赖指定的原始模型定义文件(.prototxt) 与预训练权重文件(.caffemodel)。通过执行以下命令下载.prototxt模型文件：
```bash
python3.7.5 ./src/download_prototxt.py  \
--caffe_dir CAFFE_DIR \ # 必填。Caffe源代码路径，支持相对路径和绝对路径。
--close_certificate_verify #可选。关闭证书验证参数，确保模型正常下载。

```
然后通过以下命令获取与预训练权重文件(.caffemodel):
```bash
cd model
wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet_50/ResNet-50-model.caffemodel
```
若执行成功，将会下载 `ResNet-50-deploy.prototxt`, `ResNet_50_train_val.prototxt` 与 `ResNet-50-model.caffemodel` 到该路径下。

若用户环境无法连接网络。则请先在可连通网络的服务器，下载相应文件上传到 `model` 路径下。



### 1.3 示例数据集下载

校准集用来产生量化因子，保证精度。计算量化参数的过程被称为“校准(calibration)”。校准过程需要使用一部分图片来针对性计算量化参数，使用一个或多个batch对量化后的网络模型进行推理即可完成校准。为了保证量化精度，校准集与测试精度的数据集来源应一致。

该示例提供了一组样例校准集用于量化校准，可切换到该样例根目录下执行以下命令获取校准数据与标签:
```bash
cd data
wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/amct_caffe/imagenet_calibration.tar.gz
tar -xvf imagenet_calibration.tar.gz
```

## 2， 执行样例

### 2.1 原始网络验证

在执行量化前，可对原始网络模型进行预测试，检测原始模型是否可以在Caffe环境中正常运行。以避免数据集和模型不匹配、模型无法在Caffe环境中执行等问题:

```bash
python3.7.5 ./src/ResNet50_sample.py \
--model_file model/ResNet-50-deploy.prototxt \
--weights_file model/ResNet-50-model.caffemodel \
--caffe_dir caffe-master \
--gpu 0 \
--pre_test 
```
入参说明:

* `model_file`: 必填。Caffe模型文件（.prototxt）路径。
* `weights_file`: 必填。Caffe权重文件（.caffemodel）路径。    
* `caffe_dir`: 必填。Caffe源代码路径，支持相对路径和绝对路径。    
* `gpu`: 可选。指定推理时使用GPU设备ID; 若没有可以使用的gpu, 可忽略此项。  
* `pre_test`: 可选。对量化之前的模型进行预测试，并给出推理结果，用于测试原始模型是否可以在Caffe环境中正常运行。


若出现如下信息，则说明原始模型在Caffe环境中运行正常
```
[AMCT][INFO]Run ResNet-50 without quantize success!
```


### 2.2 执行均匀量化

首先建议确保2.1章节中的原始网络推理验证成功。

如果使用支持GPU的设备：
```bash
python3.7.5 ./src/ResNet50_sample.py \
--model_file model/ResNet-50-deploy.prototxt \
--weights_file model/ResNet-50-model.caffemodel \
--caffe_dir caffe-master \
--gpu 0 
```
或者仅使用CPU执行：

```bash
python3.7.5 ./src/ResNet50_sample.py \
--model_file model/ResNet-50-deploy.prototxt \
--weights_file model/ResNet-50-model.caffemodel \
--caffe_dir caffe-master \
--cpu 
```
入参说明:
* `model_file`: 必填。Caffe模型文件（.prototxt）路径。
* `weights_file`: 必填。Caffe权重文件（.caffemodel）路径。    
* `caffe_dir`: 必填。Caffe源代码路径，支持相对路径和绝对路径。    
* `gpu`/`cpu`: 可选。指定推理时使用CPU或则使用的GPU设备ID。

也可以在该示例根目录下使用 bash 脚本指定 `caffe_master` 与 `gpu_id` 执行：
```
bash scripts/run_resnet50_with_arq.sh -c your_caffe_dir -g gpu_id
```


若出现如下信息则说明量化成功（如下推理精度只是样例，请以实际环境量化结果为准）：

```
******final top1:0.86875
******final top5:0.95    //量化后的fake_quant模型在Caffe环境中top1、top5的推理精度
[AMCT][INFO]Run ResNet-50 with quantize success!
```

### 2.3 执行convert_model接口量化示例

通过convert_model接口用户可以基于自己计算得到的量化因子以及Caffe模型，生成可以在昇腾AI处理器上做在线推理的部署模型和可以在Caffe环境下运行精度仿真的Fakequant模型。需提供自己得到的量化参数记录文件, 请参见 `model/record.txt`进行配置。

在该示例根目录执行如下命令转换ResNet-50网络模型:

```bash
python3.7.5 ./src/convert_model.py \
--model_file model/ResNet-50-deploy.prototxt \
--weights_file model/ResNet-50-model.caffemodel \
--record_file model/record.txt \
--gpu 0 \
--caffe_dir caffe-master
```

入参说明:
* `model_file`: 必填。Caffe模型文件（.prototxt）路径。
* `weights_file`: 必填。Caffe权重文件（.caffemodel）路径。
* `record_file`: 必填，量化因子记录文件(.txt)路径。
* `caffe_dir`: 必填。Caffe源代码路径，支持相对路径和绝对路径。
* `gpu`: 可选。指定推理时使用GPU设备ID; 若没有可以使用的gpu, 可忽略此项。


若出现如下信息则说明模型量化成功（如下top1，top5的推理精度只是样例，请以实际环境量化结果为准）：
```
******final top1:0.86875
******final top5:0.95625    //量化后的fake_quant模型在Caffe环境中top1、top5的推理精度
[AMCT][INFO]Run ResNet-50 with quantize success!
```

### 2.4 执行静态非均匀量化

#### 2.4.1 配置文件
执行静态非均匀量化需要配置建议配置文件的 `nuq_config `选项，本示例提供了已经配置好的简易配置文件 [quant.cfg](./src/nuq_files/quant.cfg)， 该配置文件供本示例提供的ResNet-50模型使用，若只执行本示例则无需进行额外修改。

若要使用其他模型，请参见《ATC工具指南》，《CANN开发辅助工具指南》中的‘ATC工具使用指南’。将3.2章节中介绍的均匀量化后的部署模型（.om）转换成.json文件，获取该文件，并修改[quant.cfg](./src/nuq_files/quant.cfg)文件引用该文件路径：
```
nuq_config {
    mapping_file : "../src/nuq_files/resnet50_om_model.json"
    ...
    }
}
```

#### 2.4.2 执行
首先建议确保2.1章节中的原始网络推理验证成功。

如果使用支持GPU的设备，执行：
```bash
python3.7.5 ./src/ResNet50_sample.py \
--model_file model/ResNet-50-deploy.prototxt \
--weights_file model/ResNet-50-model.caffemodel \
--caffe_dir caffe-master \
--gpu 0 \
--cfg_define src/nuq_files/quant.cfg
```

或者仅使用CPU，执行：
```bash
python3.7.5 ./src/ResNet50_sample.py \
--model_file model/ResNet-50-deploy.prototxt \
--weights_file model/ResNet-50-model.caffemodel \
--caffe_dir caffe-master \
--cpu \
--cfg_define src/nuq_files/quant.cfg
```

入参说明:
* `model_file`: 必填。Caffe模型文件（.prototxt）路径。
* `weights_file`: 必填。Caffe权重文件（.caffemodel）路径。
* `caffe_dir`: 必填。Caffe源代码路径，支持相对路径和绝对路径。
* `gpu`/`cpu`: 可选。指定推理时使用CPU或则使用的GPU设备ID。
* `cfg_define`: 必填。非均匀量化简易配置文件路径。

### 2.5 执行自动非均匀量化
自动非均匀量化是AMCT_CAFFE在满足给出的精度损失要求的前提下自动搜索出一个量化配置，使量化后的模型压缩率更高的功能特性。用户需要基于给出的自动非均匀量化基类（`AutoNuqEvaluatorBase`）实现一个子类，需要实现`eval_model(self, model_file, weight_file, batch_num)`和`is_satisfied(self, original_metric, new_metric)`两个方法：

    1）eval_model：根据输入的模型和batch数量（batch_num）进行数据前处理、模型推理、数据后处理，得到模型的评估结果，要求返回的评估结果唯一，如分类网络的top1，检测网络的mAP等，同时也可以是指标加权的结果。

    2）is_satisfied:用于判断量化后的模型是否达到了精度损失的要求，如果达到了则需返回True，否则返回False。

详细请参照`./src/auto_nuq_resnet50_sample.py`中的`AutoNuqEvaluator`类实现方式。
#### 2.5.1 配置文件
配置文件与静态非均匀量化一致，请参考2.3.1章节进行相关配置。

#### 2.5.2 执行
同样，自动非均匀量化有使用`.src/auto_nuq_resnet50_sample.py`量化脚本执行，与使bash脚本`.scripts/run_resnet50_auto_nuq.sh`执行两种方式。后者基于前者封装而成，配置参数较少，用户请根据实际情况选择一种方式执行。

通过`.src/auto_nuq_resnet50_sample.py`执行：
```bash
python3.7.5 ./src/auto_nuq_resnet50_sample.py \
--model_file model/ResNet-50-deploy.prototxt \
--weights_file model/ResNet-50-model.caffemodel \
--caffe_dir caffe-master \
--gpu 0 \
--cfg_define src/nuq_files/quant.cfg \
--dataset caffe-master/examples/imagenet/ilscrc12_val_lmdb
```
入参说明:
* `model_file`: 必填。Caffe模型文件（.prototxt）路径。
* `weights_file`: 必填。Caffe权重文件（.caffemodel）路径。
* `caffe_dir`: 必填。Caffe源代码路径，支持相对路径和绝对路径。
* `gpu`/`cpu`: 可选。指定推理时使用CPU或则使用的GPU设备ID。
* `cfg_define`: 必填。非均匀量化简易配置文件路径。
* `dataset`: 必填。用于测试目标精度的验证集路径。

或则通过`.scripts/run_resnet50_auto_nuq.sh`执行：
```bash
bash scripts/run_resnet50_auto_nuq.sh \
-c /path/to/caffe_master
-g 0 \
-d /path/to/validation/data/set
```
入参说明:
* `-c`: 必填，caffe_master路径
* `-g`: 选填，GPU的设备ID。如果不指定该参数，则默认在CPU上运行
* `-d`: 必填，验证数据集路径

### 2.6 执行量化感知训练示例
在该示例根目录下执行：
```bash
python3.7.5 ./src/ResNet50_retrain.py \
--model_file model/ResNet-50_retrain.prototxt \
--weights_file model/ResNet-50-model.caffemodel \
--gpu 0 \
--caffe_dir caffe-master \
--train_data  caffe-master/examples/imagenet/ilscrc12_val_lmdb \
--test_data caffe-master/examples/imagenet/ilscrc12_val_lmdb
```
入参说明:
* `model_file`: 必填。Caffe模型文件（.prototxt）路径。
* `weights_file`: 必填。Caffe权重文件（.caffemodel）路径。
* `caffe_dir`: 必填。Caffe源代码路径，支持相对路径和绝对路径。
* `gpu`: 可选。指定使用的GPU设备ID。
* `train_data`: 必填。训练数据集路径
* `dataset`: 必填。验证数据集路径

若出现如下信息则说明重训练成功：
```
Network initialization done.
...
Top 1 accuracy = 0.688
Top 5 accuracy = 0.934
```


重训练成功后，在该示例根目录下生成重训练日志文件夹`amct_log`，重训练中间结果文件夹`tmp`，重训练结果文件所在文件夹results（对该模型重新进行重训练时，如下结果文件将会被覆盖。）：

`amct_log`：记录了工具的日志信息，包括重训练过程的日志信息amct_caffe.log。
`results/retrain_results`：重训练结果文件，包括重训练后的模型文件、权重文件，如下所示：
`retrain_atc_model.prototxt`：重训练后的可在昇腾AI处理器部署的模型文件。
`retrain_deploy_model.prototxt`：重训练后的部署模型文件。
`retrain_deploy_weights.caffemodel`：重训练后的可在昇腾AI处理器部署的权重文件。
`retrain_fake_quant_model.prototxt`：重训练后的可在Caffe环境进行精度仿真模型文件。
`retrain_fake_quant_weights.caffemodel`：重训练后的可在Caffe环境进行精度仿真权重文件。
