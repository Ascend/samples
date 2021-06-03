# AMCT_Caffe 张量分解样例

## 1， 准备工作

### 1.1 AMCT_Caffe 环境

执行本用例需要配置 AMCT_Caffe python 环境，详细环境准备流程可以参照[昇腾社区开发者文档](https://ascend.huawei.com/zh/#/document?tag=developer)页面下“全流程开发工具链” 下的 “模型压缩 (Caffe)” 文档进行配置。

### 1.2 原始模型 (Pre-trained model)

本样例依赖指定的原始模型定义文件 (.prototxt) 与预训练权重文件 (.caffemodel)。通过执行以下命令下载 (.prototxt) 模型文件与预训练权重文件(.caffemodel):

```bash
cd model
wegt https://raw.githubusercontent.com/KaimingHe/deep-residual-networks/master/prototxt/ResNet-50-deploy.prototxt --no-check-certifacate
wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet_50/ResNet-50-model.caffemodel
```

若执行成功，将会下载 `ResNet-50-deploy.prototxt` 与 `ResNet-50-model.caffemodel` 到该路径下。

若用户环境无法连接网络。则请先在可连通网络的服务器，下载相应文件上传到 model 路径下。

## 2， 执行样例

切换到该样例根目录下，执行如下命令分解 ResNet-50 网络模型。

```bash
python ./src/tensor_decomposition_sample.py \
    --model_file model/ResNet-50-deploy.prototxt \
    --weights_file model/ResNet-50-model.caffemodel \
    --new_model_file model/ResNet-50-deploy_td.prototxt \
    --new_weights_file model/ResNet-50-model_td.caffemodel \
    --caffe_dir caffe-master
```

入参说明：

* `model_file`: 必填。Caffe 模型文件 (.prototxt) 路径。
* `weights_file`: 必填。Caffe 权重文件 (.caffemodel) 路径。
* `model_file`: 必填。分解后 Caffe 模型文件 (.prototxt) 保存路径。
* `weights_file`: 必填。分解后 Caffe 权重文件 (.caffemodel) 保存路径。
* `caffe_dir`: 必填。Caffe 源代码路径，支持相对路径和绝对路径。

若出现如下信息则说明量化成功（如下算子信息只是样例，请以实际环境分解结果为准）：

```none
INFO - [AMCT]:[AMCT]: Decomposition res3a_branch2b ->['res3a_branch2b_0', 'res3a_branch2b_1', 'res3a_branch2b_2']
INFO - [AMCT]:[AMCT]: Decomposition res3b_branch2b ->['res3b_branch2b_0', 'res3b_branch2b_1', 'res3b_branch2b_2']
INFO - [AMCT]:[AMCT]: Decomposition res3c_branch2b ->['res3c_branch2b_0', 'res3c_branch2b_1', 'res3c_branch2b_2']
```

分解成功后，在 `--new_model_file` 参数和 `--new_weights_file` 参数所指定路径下，生成分解后的模型
