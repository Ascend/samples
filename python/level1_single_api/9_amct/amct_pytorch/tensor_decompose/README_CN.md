# AMCT_PyTorch 张量分解样例

## 1 准备工作

### 1.1 AMCT_PyTorch环境
执行本用例需要配置AMCT_PyTorch的python环境，详细流程可以参照[昇腾社区开发者文档](https://ascend.huawei.com/zh/#/document)页面下“开发者文档（社区版/商用版） -- 全流程开发工具链 -- 命令行工具 (推理) -- 模型压缩”文档进行配置。

### 1.2 准备训练数据

若用户环境能连接网络，训练脚本会自动下载数据，无需额外准备。  
若用户环境无法连接网络，则请先在可连通网络的环境中，运行如下脚本下载数据文件到data目录下，之后将其放置到用户环境中样例的data目录下。
```bash
python scripts/download_mnist.py --data_path=data
```
参数说明：
* `data_path`: \[必选\] MNIST数据存储路径，脚本将下载数据到该路径下。  

下载后，将得到如下文件：  
```
data
└── MNIST
    ├── processed
    │   ├── test.pt
    │   └── training.pt
    └── raw
        ├── t10k-images-idx3-ubyte
        ├── t10k-images-idx3-ubyte.gz
        ├── t10k-labels-idx1-ubyte
        ├── t10k-labels-idx1-ubyte.gz
        ├── train-images-idx3-ubyte
        ├── train-images-idx3-ubyte.gz
        ├── train-labels-idx1-ubyte
        └── train-labels-idx1-ubyte.gz
```
注意：运行下载脚本环境中的torch和torchvision版本最好与用户环境相同，否则在训练时可能会导致读取数据失败。若失败，可将data/MNIST/processed目录删除，再尝试训练。

## 2 执行样例

张量分解的一般流程有如下两种：  
1. 在线张量分解：即直接分解并finetune*（微调）。在训练代码中加载训好的模型权重，引入张量分解工具，执行张量分解并进行finetune。  
该流程在张量分解时直接修改模型结构并更新权重，分解后的模型可直接使用，其优点是使用方便，仅需一步操作；而缺点是每次启动都要重新进行分解计算，需要耗费一定时间。  

2. 离线张量分解：即先分解，再finetune。第一步，通过单独的脚本将训好的模型权重加载给模型，引入张量分解工具对模型进行分解，保存所得分解信息文件*和分解后的权重。第二步，在训练代码中引入张量分解工具，读入分解信息文件分解模型结构，加载分解后的权重，进行finetune。  
该流程先将分解信息和分解后的权重保存下来，然后在finetune时直接加载使用，其优点是可以一次分解，多次使用，加载使用分解信息文件和权重几乎不耗费时间；而缺点是需要两步操作，且需保存分解后的权重并在使用时加载。  

* 关于finetune：通常情况下，分解后模型的精度会比原始模型有所下降，因此在分解之后，通过finetune来提高精度。finetune的超参和普通模型finetune基本一致，将原始的学习率调小，通常可将初始学习率调整为原来的0.1倍；而迭代次数（或epoch数）则因模型而异，分解的卷积层越多，finetune所需的迭代次数一般就越多，保守的做法是采用与从头训练相同的迭代次数；其他超参通常无需调整。  

* 关于分解信息文件：该文件通过调用auto_decomposition产生，其存储了网络结构的分解信息，可通过decompose_network加载该文件直接分解网络结构，但不会更新分解后的权重（即保持torch.nn.Conv2d默认值）。通常情况下，使用该文件时，用户需在调用auto_decomposition后自行保存分解后的权重，在decompose_network分解网络结构后，再加载该权重使用。

本样例提供两种训练方式：单进程训练和分布式训练，每种方式都支持在线张量分解和离线张量分解。用户可根据实际需求进行参考。  

### 2.1 公共脚本
单进程训练和分布式训练使用了如下公共脚本，用于在离线张量分解时分解训好的模型，生成分解信息文件，并保存分解后的模型权重（即离线张量分解第一步）。  
运行方式：
```bash
python src/decompose_model.py \
--pretrained-path xxx/baseline.pth \
--decompose-info-path xxx/decompose_info.json \
--decomposed-weights-path xxx/decomposed_weights.pth
```
参数说明：  
* `pretrained-path`: \[必选\] 预训练模型权重的路径。  
* `decompose-info-path`: \[必选\] 分解信息文件的保存路径。  
* `decomposed-weights-path`: \[必选\] 分解后模型权重的保存路径。  

脚本将加载模型权重xxx/baseline.pth（脚本内含模型定义），进行张量分解，将分解信息文件保存到xxx/decompose_info.json，将分解后的模型权重保存到xxx/decomposed_weights.pth。  
注：此处路径仅为示例，详细使用样例请见下文。  

### 2.2 单进程训练

单进程训练脚本为src/simple_train.py，对脚本中各项参数的组合可实现不同功能。  
参数说明：  
* `data-path`: \[可选\] MNIST数据存放路径（默认：data）。  
* `train-batch-size`: \[可选\] 训练的batch size（默认：256）。  
* `test-batch-size`: \[可选\] 测试的batch size（默认：1000）。  
* `lr`: \[可选\] 学习率（默认：0.01）。  
* `steps`: \[可选\] 训练迭代次数（默认：200）。  
* `use-gpu`: \[可选\] 是否使用GPU训练。若不设置则使用CPU训练。  
* `seed`: \[可选\] 随机种子（默认：1）。  
* `log-steps`: \[可选\] 打印日志的迭代次数间隔（默认：20）。  
* `save-path`: \[可选\] 训练后的模型权重保存路径（默认：None，表示不保存）。  
* `pretrained-path`: \[可选\] 预训练模型权重的路径。张量分解在线模式时必需（默认：None，表示不加载预训练模型权重）。  
* `tensor-decompose`: \[可选\] 是否进行张量分解。若不设置则不进行张量分解。  
* `decompose-info-path`: \[可选\] 分解信息文件的路径。离线模式时必需；在线模式时如设置则保存，否则不保存。仅当设置--tensor-decompose时有效（默认：None，表示不保存）。  
* `decomposed-weights-path`: \[可选\] 分解后模型权重的路径。离线模式时必需；在线模式时如设置则保存，否则不保存。仅当设置--tensor-decompose时有效（默认：None，表示不保存）。  
* `run-mode`: \[可选\] 张量分解运行模式，在线模式则设为online，离线模式则设为offline。必须与--tensor-decompose一同使用（默认：online）。

下面通过列举一些典型用法说明该脚本的使用方式。  
以下使用本脚本（src/simple_train.py）的示例，均为CPU运行，如要在GPU上运行，请加--use-gpu参数。  

#### 2.2.1 通过训练获得模型文件
本步骤进行普通训练，从头训练模型获得模型权重文件。  
运行脚本：  
```bash
python src/simple_train.py \
--save-path model/simple/baseline.pth
```
脚本将启动训练。如见下列信息，则说明执行成功（此处精度结果仅为样例，请以实际运行为准）：
```
[Test] Loss: 0.062657, Accuracy: 98.05% (9805/10000)
```
所得模型权重文件将保存在model/simple/baseline.pth。  
本步骤为下列2.2.2和2.2.3步骤的前提。  

#### 2.2.2 在线张量分解
本步骤基于训好的模型权重进行在线张量分解。  

运行脚本：  
```bash
python src/simple_train.py \
--lr 0.001 \
--save-path model/simple/online/decomposed_finetuned.pth \
--pretrained-path model/simple/baseline.pth \
--tensor-decompose \
--decompose-info-path model/simple/online/decompose_info.json \
--run-mode online
```

脚本将加载2.2.1训好的模型权重，执行张量分解，并保存分解信息文件和分解后的权重，然后以0.001的学习率进行finetune。  
运行时，日志会打印被分解的卷积名称及其分解后的卷积名称，以及分解信息文件的保存情况，如下所示（此处分解情况仅为样例，请以实际运行为准）：
```
[AMCT]:[AMCT]: auto_decomposition start.
[AMCT]:[AMCT]: Processing: 'conv1'
[AMCT]:[AMCT]: Processing: 'conv2'
[AMCT]:[AMCT]: Decomposition information file is saved to: xxx/model/simple/online/decompose_info.json
[AMCT]:[AMCT]: Decompose: 'conv2' -> ['conv2.0', 'conv2.1']
[AMCT]:[AMCT]: auto_decomposition complete.
```
运行结束时，如见下列信息，则说明执行成功（此处精度结果仅为样例，请以实际运行为准）：
```
[Test] Loss: 0.047416, Accuracy: 98.39% (9839/10000)
```
所得分解信息文件将保存在model/simple/online/decompose_info.json。  
训练所得模型权重文件将保存在model/simple/online/decomposed_finetuned.pth。如后续需使用该权重，则通过decompose_network加载分解信息文件修改原模型结构，再加载该权重即可。  

#### 2.2.3 离线张量分解
本步骤基于训好的模型权重进行离线张量分解。  
* 第一步  
    使用src/decompose_model.py加载2.2.1训好的模型权重对模型进行分解。  
    运行脚本：  
    ```bash
    python src/decompose_model.py \
    --pretrained-path model/simple/baseline.pth \
    --decompose-info-path model/simple/offline/decompose_info.json \
    --decomposed-weights-path model/simple/offline/decomposed_weights.pth
    ```
    运行时，日志会打印被分解的卷积名称及其分解后的卷积名称，以及分解信息文件的保存情况，如下所示（此处分解情况仅为样例，请以实际运行为准）：
    ```
    [AMCT]:[AMCT]: auto_decomposition start.
    [AMCT]:[AMCT]: Processing: 'conv1'
    [AMCT]:[AMCT]: Processing: 'conv2'
    [AMCT]:[AMCT]: Decomposition information file is saved to: xxx/model/simple/offline/decompose_info.json
    [AMCT]:[AMCT]: Decompose: 'conv2' -> ['conv2.0', 'conv2.1']
    [AMCT]:[AMCT]: auto_decomposition complete.
    ```
    所得分解信息文件将保存在model/simple/offline/decompose_info.json。  
    所得分解后的权重将保存在model/simple/offline/decomposed_weights.pth。  

* 第二步  
    加载第一步所得文件，进行finetune。  
    运行脚本：  
    ```bash
    python src/simple_train.py \
    --lr 0.001 \
    --save-path model/simple/offline/decomposed_finetuned.pth \
    --tensor-decompose \
    --decompose-info-path model/simple/offline/decompose_info.json \
    --decomposed-weights-path model/simple/offline/decomposed_weights.pth \
    --run-mode offline
    ```
    脚本将加载第一步保存的分解信息文件对模型结构进行分解，并加载第一步保存的分解后的权重，然后以0.001的学习率进行finetune。  
    运行时，日志会打印被分解的卷积名称及其分解后的卷积名称，如下所示（此处分解情况仅为样例，请以实际运行为准）：
    ```
    [AMCT]:[AMCT]: decompose_network start.
    [AMCT]:[AMCT]: Decompose: 'conv2' -> ['conv2.0', 'conv2.1']
    [AMCT]:[AMCT]: decompose_network complete.
    ```
    运行结束时，如见下列信息，则说明执行成功（此处精度结果仅为样例，请以实际运行为准）：
    ```
    [Test] Loss: 0.047416, Accuracy: 98.39% (9839/10000)
    ```
    训练所得模型权重文件将保存在model/simple/offline/decomposed_finetuned.pth。如后续需使用该权重，则通过decompose_network加载第一步保存的分解信息文件修改原模型结构，再加载该权重即可。  

### 2.3 分布式训练

分布式训练脚本为src/ddp_train.py，对脚本中各项参数的组合可实现不同功能。  
参数说明：  
* `backend`: \[可选\] 分布式训练的后端。CPU使用gloo，GPU使用nccl（默认：gloo）。  
* `init_method`: \[可选\] 分布式训练的初始化方法（默认：env://）。  
* `local_rank`: \[可选\] 分布式训练的local rank（默认：0）。  
* `data-path`: \[可选\] MNIST数据存放路径（默认：data）。  
* `train-batch-size`: \[可选\] 训练的单个设备的batch size（默认：32）。  
* `test-batch-size`: \[可选\] 在首个设备上测试的batch size（默认：1000）。  
* `lr`: \[可选\] 学习率（默认：0.01）。  
* `steps`: \[可选\] 训练迭代次数（默认：200）。  
* `use-gpu`: \[可选\] 是否使用GPU训练。若不设置则使用CPU训练。  
* `seed`: \[可选\] 随机种子（默认：1）。  
* `log-steps`: \[可选\] 打印日志的迭代次数间隔（默认：20）。  
* `save-path`: \[可选\] 训练后的模型权重保存路径（默认：None，表示不保存）。  
* `pretrained-path`: \[可选\] 预训练模型权重的路径。张量分解在线模式时必需（默认：None，表示不加载预训练模型权重）。  
* `tensor-decompose`: \[可选\] 是否进行张量分解。若不设置则不进行张量分解。  
* `decompose-info-path`: \[可选\] 分解信息文件的路径，必须与--tensor-decompose一同使用。在线模式时会保存文件到该路径，离线模式时会读取该路径的文件（默认：None，表示不涉及该文件）。  
* `decomposed-weights-path`: \[可选\] 分解后模型权重的路径，必须与--tensor-decompose一同使用。在线模式时会保存文件到该路径，离线模式时会读取该路径的文件（默认：None，表示不涉及该文件）。  
* `run-mode`: \[可选\] 张量分解运行模式，在线模式则设为online，离线模式则设为offline。必须与--tensor-decompose一同使用（默认：online）。

下面通过列举一些典型用法说明该脚本的使用方式。  
以下使用本脚本（src/ddp_train.py）的示例，均为CPU运行，如要在GPU上运行，请加--use-gpu参数，并设置--backend为nccl。  

#### 2.3.1 通过训练获得模型文件
本步骤进行普通训练，从头训练模型获得模型权重文件。  
运行脚本：  
```bash
python -m torch.distributed.launch --nproc_per_node=8 \
src/ddp_train.py \
--save-path model/ddp/baseline.pth
```
补充参数说明（下同）：  
* `nproc_per_node`: \[必选\] 分布式训练的每个节点开启的进程数（如使用GPU则不应超过每个节点的GPU数量）。  

脚本将启动训练。如见下列信息，则说明执行成功（此处精度结果仅为样例，请以实际运行为准）：
```
[Test] Loss: 0.061084, Accuracy: 98.02% (9802/10000)
```
所得模型权重文件将保存在model/ddp/baseline.pth。  
本步骤为下列2.3.2和2.3.3步骤的前提。  

#### 2.3.2 在线张量分解
本步骤基于训好的模型权重进行在线张量分解。  

运行脚本：  
```bash
python -m torch.distributed.launch --nproc_per_node=8 \
src/ddp_train.py \
--lr 0.001 \
--save-path model/ddp/online/decomposed_finetuned.pth \
--pretrained-path model/ddp/baseline.pth \
--tensor-decompose \
--decompose-info-path model/ddp/online/decompose_info.json \
--decomposed-weights-path model/ddp/online/decomposed_weights.pth \
--run-mode online
```

脚本将加载2.3.1训好的模型权重，在local_rank为0的进程上先执行张量分解并保存分解信息文件和分解后的权重，之后在其他local_rank的进程上加载所得分解信息文件和分解后的权重，然后以0.001的学习率进行finetune。  
运行时，相关日志如下（此处分解情况仅为样例，请以实际运行为准）。  
local_rank为0的进程日志会打印被分解的卷积名称及其分解后的卷积名称，以及分解信息文件的保存情况：
```
[AMCT]:[AMCT]: auto_decomposition start.
[AMCT]:[AMCT]: Processing: 'conv1'
[AMCT]:[AMCT]: Processing: 'conv2'
[AMCT]:[AMCT]: Decomposition information file is saved to: xxx/model/ddp/online/decompose_info.json
[AMCT]:[AMCT]: Decompose: 'conv2' -> ['conv2.0', 'conv2.1']
[AMCT]:[AMCT]: auto_decomposition complete.
```
其他local_rank的进程日志会打印被分解的卷积名称及其分解后的卷积名称（会打印若干次，取决于进程数）：  
```
[AMCT]:[AMCT]: decompose_network start.
[AMCT]:[AMCT]: Decompose: 'conv2' -> ['conv2.0', 'conv2.1']
[AMCT]:[AMCT]: decompose_network complete.
```
运行结束时，如见下列信息，则说明执行成功（此处精度结果仅为样例，请以实际运行为准）：
```
[Test] Loss: 0.045164, Accuracy: 98.66% (9866/10000)
```
所得分解信息文件将保存在model/ddp/online/decompose_info.json。  
所得分解后的权重将保存在model/ddp/online/decomposed_weights.pth。  
训练所得模型权重文件将保存在model/ddp/online/decomposed_finetuned.pth。如后续需使用该权重，则通过decompose_network加载分解信息文件修改原模型结构，再加载该权重即可。  

#### 2.3.3 离线张量分解
本步骤基于训好的模型权重进行离线张量分解。  
* 第一步  
    使用src/decompose_model.py加载2.3.1训好的模型权重对模型进行分解。  
    运行脚本：  
    ```bash
    python src/decompose_model.py \
    --pretrained-path model/ddp/baseline.pth \
    --decompose-info-path model/ddp/offline/decompose_info.json \
    --decomposed-weights-path model/ddp/offline/decomposed_weights.pth
    ```
    运行时，日志会打印被分解的卷积名称及其分解后的卷积名称，以及分解信息文件的保存情况，如下所示（此处分解情况仅为样例，请以实际运行为准）：
    ```
    [AMCT]:[AMCT]: auto_decomposition start.
    [AMCT]:[AMCT]: Processing: 'conv1'
    [AMCT]:[AMCT]: Processing: 'conv2'
    [AMCT]:[AMCT]: Decomposition information file is saved to: xxx/model/ddp/offline/decompose_info.json
    [AMCT]:[AMCT]: Decompose: 'conv2' -> ['conv2.0', 'conv2.1']
    [AMCT]:[AMCT]: auto_decomposition complete.
    ```
    所得分解信息文件将保存在model/ddp/offline/decompose_info.json。  
    所得分解后的权重将保存在model/ddp/offline/decomposed_weights.pth。  

* 第二步  
    加载第一步所得文件，进行finetune。  
    运行脚本：  
    ```bash
    python -m torch.distributed.launch --nproc_per_node=8 \
    src/ddp_train.py \
    --lr 0.001 \
    --save-path model/ddp/offline/decomposed_finetuned.pth \
    --tensor-decompose \
    --decompose-info-path model/ddp/offline/decompose_info.json \
    --decomposed-weights-path model/ddp/offline/decomposed_weights.pth \
    --run-mode offline
    ```
    脚本会对所有local_rank的进程加载第一步保存的分解信息文件对模型结构进行分解，并加载第一步保存的分解后的权重，然后以0.001的学习率进行finetune。  
    运行时，日志会打印被分解的卷积名称及其分解后的卷积名称，如下所示（会打印若干次，取决于进程数；此处分解情况仅为样例，请以实际运行为准）：
    ```
    [AMCT]:[AMCT]: decompose_network start.
    [AMCT]:[AMCT]: Decompose: 'conv2' -> ['conv2.0', 'conv2.1']
    [AMCT]:[AMCT]: decompose_network complete.
    ```
    运行结束时，如见下列信息，则说明执行成功（此处精度结果仅为样例，请以实际运行为准）：
    ```
    [Test] Loss: 0.045164, Accuracy: 98.66% (9866/10000)
    ```
    训练所得模型权重文件将保存在model/ddp/offline/decomposed_finetuned.pth。如后续需使用该权重，则通过decompose_network加载第一步保存的分解信息文件修改原模型结构，再加载该权重即可。  
