# AMCT_Tensorflow 张量分解样例

## 1 准备工作

### 1.1 AMCT_Tensorflow环境
执行本用例需要配置AMCT_Tensorflow的python环境，详细流程可以参照[昇腾社区开发者文档](https://ascend.huawei.com/zh/#/document?tag=developer)页面下“全流程开发工具链 命令行”下的“命令行工具 (训练)”的“模型压缩”文档进行配置。

### 1.2 准备训练数据

若用户环境能连接网络，训练脚本会自动下载数据，无需额外准备。  
若用户环境无法连接网络，则请先在可连通网络的环境中，运行如下脚本下载数据文件mnist.npz到data目录下，之后将其放置到用户环境中样例的data目录下。
```bash
python scripts/download_mnist.py --data_path=data
```
参数说明：
* `data_path`: \[必选\] MNIST数据存储路径，脚本将下载数据到该路径下。  

## 2 执行样例

张量分解的一般流程为：用户准备训练好的模型文件，使用张量分解工具进行分解，然后修改训练代码加载分解后的权重进行finetune（微调）。本样例中，先通过执行训练脚本获得模型文件，再执行分解脚本对模型进行分解，最后执行finetune脚本加载分解后的权重进行finetune。  

* 关于finetune：通常情况下，分解后模型的精度会比原始模型有所下降，因此在分解之后，通过finetune来提高精度。finetune的超参和普通模型finetune基本一致，将原始的学习率调小，通常可将初始学习率调整为原来的0.1倍；而迭代次数（或epoch数）则因模型而异，分解的卷积层越多，finetune所需的迭代次数一般就越多，保守的做法是采用与从头训练相同的迭代次数；其他超参通常无需调整。

本样例提供两种使用方式：Session方式和Estimator方式。用户可根据实际需求进行参考。  

### 2.1 Session方式

#### 2.1.1 通过训练获得模型文件
运行如下脚本：  
```bash
python ./src/session_train.py --data_path=data --ckpt_path=model/session/baseline/model
```
参数说明：
* `data_path`: \[必选\] MNIST数据存储路径。  
* `ckpt_path`: \[必选\] 模型文件保存路径。  

脚本将启动训练。如见下列信息，则说明执行成功（此处精度结果仅为样例，请以实际运行为准）：
```
Validation Accuracy: 0.9827
```
所得模型文件将保存在model/session/baseline目录：
```
checkpoint                                      # checkpoint列表文件（张量分解不需要）
model-200.data-00000-of-00001                   # 模型权重文件
model-200.index                                 # 模型权重索引文件
model-200.meta                                  # 模型图结构文件
model.pb                                        # pb格式的模型文件（张量分解不需要）
```

#### 2.1.2 分解模型文件
运行如下脚本：  
```bash
python ./src/decompose_ckpt.py --meta_path=model/session/baseline/model-200.meta --ckpt_path=model/session/baseline/model-200 --save_path=model/session/decompose/model
```
参数说明：
* `meta_path`: \[必选\] 待分解模型的定义文件(.meta)路径。  
* `ckpt_path`: \[必选\] 待分解模型的权重文件路径，为.data-XXXXX-of-XXXXX文件与.index文件的共同前缀路径。  
* `save_path`: \[必选\] 张量分解后的模型文件保存路径。  

脚本将执行张量分解。运行时，日志会打印被分解的卷积名称，及其分解后的卷积名称，如下所示（此处分解情况仅为样例，请以实际运行为准）：
```
[AMCT]:[AMCT]: Decompose conv2d_1/Conv2D -> ['conv2d_1/Conv2D/decom_first/decom_first', 'conv2d_1/Conv2D/decom_last/decom_last']
```
如见下列信息，则说明分解成功：
```
[AMCT]:[AMCT]: auto_decomposition complete.
```
分解得到的模型文件将保存在model/session/decompose目录：
```
checkpoint                                      # checkpoint列表文件
model.data-00000-of-00001                       # 模型权重文件
model.index                                     # 模型权重索引文件
model.meta                                      # 模型图结构文件
model.pkl                                       # 图结构改动信息文件，用于finetune时修改原训练代码中的图
```

#### 2.1.3 finetune分解后的模型
运行如下脚本：  
```bash
python ./src/session_finetune.py --data_path=data --save_path=model/session/decompose/model --ckpt_path=model/session/decompose_finetune/model
```
参数说明：
* `data_path`: \[必选\] MNIST数据存储路径。  
* `save_path`: \[必选\] 张量分解后的模型文件保存路径。  
* `ckpt_path`: \[必选\] finetune后的模型文件保存路径。  

脚本将启动finetune训练。如见下列信息，则说明执行成功（此处精度结果仅为样例，请以实际运行为准）：
```
Validation Accuracy: 0.9841
```
所得模型文件将保存在model/session/decompose_finetune目录：
```
checkpoint                                      # checkpoint列表文件
model-100.data-00000-of-00001                   # 模型权重文件
model-100.index                                 # 模型权重索引文件
model-100.meta                                  # 模型图结构文件
model.pb                                        # pb格式的模型文件
```

### 2.2 Estimator方式

#### 2.2.1 通过训练获得模型文件
运行如下脚本：  
```bash
python ./src/estimator_train.py --data_path=data --ckpt_path=model/estimator/baseline/model
```
参数说明：
* `data_path`: \[必选\] MNIST数据存储路径。  
* `ckpt_path`: \[必选\] 模型文件保存路径。  

脚本将启动训练。如见下列信息，则说明执行成功（此处精度结果仅为样例，请以实际运行为准）：
```
Validation Accuracy: 0.9777
```
所得模型文件将保存在model/estimator/baseline目录：
```
checkpoint                                      # checkpoint列表文件（张量分解不需要）
events.out.tfevents.XXXXXX                      # 训练信息文件（张量分解不需要）
graph.pbtxt                                     # 文本形式的图描述文件（张量分解不需要）
model-0.data-00000-of-00001                     # 模型权重文件（本样例不分解该模型）
model-0.index                                   # 模型权重索引文件（本样例不分解该模型）
model-0.meta                                    # 模型图结构文件（本样例不分解该模型）
model-200.data-00000-of-00001                   # 模型权重文件
model-200.index                                 # 模型权重索引文件
model-200.meta                                  # 模型图结构文件
model-200.pb                                    # pb格式的模型文件（张量分解不需要）
```

#### 2.2.2 分解模型文件
运行如下脚本：  
```bash
python ./src/decompose_ckpt.py --meta_path=model/estimator/baseline/model-200.meta --ckpt_path=model/estimator/baseline/model-200 --save_path=model/estimator/decompose/model
```
参数说明：
* `meta_path`: \[必选\] 待分解模型的定义文件(.meta)路径。  
* `ckpt_path`: \[必选\] 待分解模型的权重文件路径，为.data-XXXXX-of-XXXXX文件与.index文件的共同前缀路径。  
* `save_path`: \[必选\] 张量分解后的模型文件保存路径。  

脚本将执行张量分解。运行时，日志会打印被分解的卷积名称，及其分解后的卷积名称，如下所示（此处分解情况仅为样例，请以实际运行为准）：
```
[AMCT]:[AMCT]: Decompose conv2d_1/Conv2D -> ['conv2d_1/Conv2D/decom_first/decom_first', 'conv2d_1/Conv2D/decom_last/decom_last']
```
如见下列信息，则说明分解成功：
```
[AMCT]:[AMCT]: auto_decomposition complete.
```
分解得到的模型文件将保存在model/estimator/decompose目录：
```
checkpoint                                      # checkpoint列表文件
model.data-00000-of-00001                       # 模型权重文件
model.index                                     # 模型权重索引文件
model.meta                                      # 模型图结构文件
model.pkl                                       # 图结构改动信息文件，用于finetune时修改原训练代码中的图
```

#### 2.2.3 finetune分解后的模型
运行如下脚本：  
```bash
python ./src/estimator_finetune.py --data_path=data --save_path=model/estimator/decompose/model --ckpt_path=model/estimator/decompose_finetune/model
```
参数说明：
* `data_path`: \[必选\] MNIST数据存储路径。  
* `save_path`: \[必选\] 张量分解后的模型文件保存路径。  
* `ckpt_path`: \[必选\] finetune后的模型文件保存路径。  

脚本将启动finetune训练。如见下列信息，则说明执行成功（此处精度结果仅为样例，请以实际运行为准）：
```
Validation Accuracy: 0.9828
```
所得模型文件将保存在model/estimator/decompose_finetune目录：
```
checkpoint                                      # checkpoint列表文件
events.out.tfevents.XXXXXX                      # 训练信息文件
graph.pbtxt                                     # 文本形式的图描述文件
model-0.data-00000-of-00001                     # 模型权重文件
model-0.index                                   # 模型权重索引文件
model-0.meta                                    # 模型图结构文件
model-100.data-00000-of-00001                   # 模型权重文件
model-100.index                                 # 模型权重索引文件
model-100.meta                                  # 模型图结构文件
model-100.pb                                    # pb格式的模型文件
```
