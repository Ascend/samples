# AMCT_Caffe LSTM LeNet-5 量化样例

## 1， 准备工作

### 1.1 AMCT_Caffe 环境

执行本用例需要配置 AMCT_Caffe python 环境，详细环境准备流程可以参照[昇腾社区开发者文档](https://ascend.huawei.com/zh/#/document?tag=developer)页面下“全流程开发工具链” 下的 “模型压缩 (Caffe)” 文档进行配置。

### 1.2 原始模型 (Pre-trained model)

本样例依赖指定的原始模型定义文件 (.prototxt) 与预训练权重文件 (.caffemodel)。
若安装昇腾模型压缩工具的服务器能连接网络, 可执行如下命令获取模型文件:

```bash
cd model
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mnist/mnist-deploy.prototxt
wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mnist/mnist-model.caffemodel
```

若用户环境无法连接网络。则请在可连通网络的服务器，分别访问如下链接下载相应文件:

[链接 A](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mnist/mnist-deploy.prototxt)

[链接 B](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/mnist/mnist-model.caffemodel)

然后上传到刚创建的 model 路径下。

### 1.3 数据集

切换到该样例根目录下执行以下命令创建数据集路径

```bash
cd data
mkdir mnist_data && cd mnist_data
```

若安装昇腾模型压缩工具的服务器能连接网络, 可执行如下命令获取数据以及标签文件:

```none
wget http://yann.lecun.com/exdb/mnist/t10k-images-idx3-ubyte.gz
wget http://yann.lecun.com/exdb/mnist/t10k-labels-idx1-ubyte.gz
```

若用户环境无法连接网络。则请在可连通网络的服务器，分别访问如下链接下载相应软件包:

[MNIST数据](http://yann.lecun.com/exdb/mnist/t10k-imagesidx3-ubyte.gz)（环境初始化后将在 `data/mnist_data/` 目录生成 MNSIT test 数据集：t10k-images-idx3-ubyte）

[MNIST标签](http://yann.lecun.com/exdb/mnist/t10klabels-idx1-ubyte.gz)（环境初始化后将在 `data/mnist_data/` 目录生成 MNSIT test 标签：t10k-labels-idx1-ubyte）

然后上传到刚创建的 mnist_data 路径下。

由于该网络模型比较简单，环境初始化与执行量化动作合并；在执行样例时会根据上述文件在 `mnist/mnist_test_lmdb` 目录自动生成 MNSIT lmdb 格式数据集。

## 2， 执行样例

切换到该样例根目录下，执行如下命令量化 mnist 网络模型。

```none
python3.7.5 ./src/mnist_sample.py \
    --model_file model/mnist-deploy.prototxt \
    --weights_file model/mnist-model.caffemodel \
    --gpu 0 \
    --caffe_dir caffe-master
```

入参说明:

* `model_file`: 必填。Caffe 模型文件 (.prototxt) 路径。
* `weights_file`: 必填。Caffe 权重文件(.caffemodel) 路径。
* `caffe_dir`: 必填。Caffe 源代码路径，支持相对路径和绝对路径。
* `gpu`/`cpu`: 可选。指定推理时使用 CPU 或则使用的 GPU 设备 ID。

若出现如下信息则说明量化成功（如下推理精度只是样例，请以实际环境量化结果为准）：

```none
******final top1:0.9853125
[AMCT][INFO] mnist top1 before quantize is 0.98515625, after quantize is 0.9853125
[AMCT][INFO]Run mnist sample with quantize success!
```
