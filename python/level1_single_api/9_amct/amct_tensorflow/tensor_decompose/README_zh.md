# AMCT_Tensorflow 张量分解样例

## 1， 准备工作

### 1.1 AMCT_Tensorflow环境
执行本用例需要配置AMCT_Tensorflow python环境，详细环境准备流程可以参照[昇腾社区开发者文档](https://ascend.huawei.com/zh/#/document?tag=developer)页面下“全流程开发工具链” 下的 “模型压缩 (Tensorflow)” 文档进行配置。

### 1.2 准备训练数据

若用户环境能连接网络，训练脚本会自动下载数据，无需额外准备；
若用户环境无法连接网络。则请先在可连通网络的服务器，下载相应文件上传到 `data/mnist` 路径下。

    a.MNIST训练数据集获取路径：http://yann.lecun.com/exdb/mnist/train-images-idx3-ubyte.gz
    b.MNIST训练数据集标签文件获取路径：http://yann.lecun.com/exdb/mnist/train-labels-idx1-ubyte.gz
    c.MNIST测试数据集获取路径：http://yann.lecun.com/exdb/mnist/t10k-images-idx3-ubyte.gz
    d.MNIST测试数据集标签文件获取路径：http://yann.lecun.com/exdb/mnist/t10k-labels-idx1-ubyte.gz

### 1.3 通过训练脚本获取模型和权重文件
src目录提供两份TensorFlow训练代码：一份使用Session接口训练，一份使用Estimator接口训练，用户根据实际情况进行选择。执行如下命令生成模型文件和权重文件。
使用样例如下：
```bash
python ./src/train_sample_session.py --data_path data/mnist
```
或
```bash
python ./src/train_sample_estimator.py --data_path data/mnist
```
入参说明:
* `data_path`: 必填。mnist数据集的路径，支持绝对路径和相对路径。  

若提示如下信息，则说明执行成功（如下精度结果只是样例，请以实际环境为准）：
```
Valid Accuracy: 0.9803     //基于mnist数据集的训练精度
```
执行成功后，在model/checkpoints目录，生成模型文件以及权重文件，示例如下：
```
-rw-r--r-- 1 amct amct       85 Jul 28 04:37 checkpoint                            //TensorFlow保存模型时保存的检查点
-rw-r--r-- 1 amct amct 10633856 Jul 28 04:37 model.ckpt-200.data-00000-of-00001    //TensorFlow权重文件 
-rw-r--r-- 1 amct amct      771 Jul 28 04:37 model.ckpt-200.index                  //TensorFlow权重文件索引  
-rw-r--r-- 1 amct amct    60937 Jul 28 04:37 model.ckpt-200.meta                   //TensorFlow模型文件
```

## 2， 执行样例

### 2.1 分解样例
切换到该样例根目录下，执行如下命令分解resnet50网络模型。
```bash
python ./src/decompose_sample.py \
    --meta_path model/checkpoints/model.ckpt-200.meta \
    --ckpt_path model/checkpoints/model.ckpt-200 \
    --save_path model/decomposition/model_td
```
入参说明:
* `meta_path`: 必填。TensorFlow模型文件（.meta）路径。
* `ckpt_path`: 必填。TensorFlow权重文件路径：.data-0000X-of-0000X文件与.index文件所在路径，路径中不必包含文件后缀。
* `save_path`: 必填。张量分解后所得文件的保存路径，支持相对路径和绝对路径，如果没有创建，指定脚本时会自动创建。

分解过程中，日志会打印支持分解的算子名称，以及分解后的算子名称，如下所示（如下示例中的算子信息只是样例，请以实际分解的模型为准）：
```
[AMCT]:[AMCT]: Processing conv2d_1/Conv2D
[AMCT]:[AMCT]: Decompose conv2d_1/Conv2D -> ['conv2d_1/Conv2D/decom_first/decom_first', 'conv2d_1/Conv2D/decom_core/decom_core', 'conv2d_1/Conv2D/decom_last/decom_last']
```
若提示如下信息，则说明分解成功：
```
auto_decomposition complete!
```

### 2.2 微调（finetune)样例
正常情况下，分解后模型的精度会比原始模型有所下降，所以在分解之后，会用一个finetune（微调）过程来提高分解模型的精度。
finetune的方法和普通模型finetune的方式一致，将原始的学习率调小，一般可以从原始学习率的0.1倍开始降低。 
finetune的epoch（1个epoch表示过了1遍训练集中的所有样本）数各个模型不尽相同，分解的卷积层越多，所需要的epoch一般就越多。 
finetune之后的模型，其精度可能与原模型持平，也可能有少许的下降或提升。
src目录提供两份TensorFlow训练代码：一份使用Session接口训练，一份使用Estimator接口训练，用户根据实际情况进行选择。
使用样例如下：
```bash
python ./src/finetune_sample_session.py --data_path data/mnist --save_path model/decomposition/model_td
```
或
```bash
python ./src/finetune_sample_estimator.py --data_path data/mnist --save_path model/decomposition/model_td
```
入参说明:
* `data_path`: 必填。mnist数据集的路径，支持绝对路径和相对路径。
* `save_path`: 必填。分解样例中，分解所得文件的保存路径。

若提示如下信息，则说明finetune成功（如下精度结果只是样例，请以实际环境为准）：
```
Valid Accuracy: 0.9838      //基于mnist数据集的精度
```
finetune完成后，在model目录自动生成finetuned_ckpt目录，用于保存分解后模型的模型文件以及权重文件。示例如下：
```
-rw-r--r-- 1 amct amct        128 Aug  1 04:14 checkpoint
-rw-r--r-- 1 amct amct   10093192 Aug  1 04:14 model.ckpt-100.data-00000-of-00001   //finetune后的权重文件索引
-rw-r--r-- 1 amct amct       1144 Aug  1 04:14 model.ckpt-100.index                 //finetune后的权重文件
-rw-r--r-- 1 amct amct     122783 Aug  1 04:14 model.ckpt-100.meta                  //finetune后的模型文件
-rw-r--r-- 1 amct amct    3171658 Aug  1 04:14 model.pb                             //finetune后可用于量化的pb格式的模型文件
```
