<h2 id="基本信息.md">基本信息</h2>

**发布者（Publisher）：Huawei**

**应用领域（Application Domain）：** Image Classification 

**版本（Version）：1.0**

**修改时间（Modified） ：2022.10.18**

**大小（Size）：1.84MB**

**框架（Framework）：TensorFlow 1.15.0**

**模型格式（Model Format）：**

**精度（Precision）：**

**处理器（Processor）：昇腾910**

**应用级别（Categories）：Research**

**描述（Description）：NAS-Bench-101数据集源代码** 

<h2 id="概述.md">概述</h2>

NAS-Bench-101是谷歌在论文《NAS-Bench-101: Towards Reproducible Neural Architecture Search》中提出的针对神经架构搜索（neural architecture search）领域的架构数据集。该数据集构建了一个包含423k个架构的搜索空间，提供了该搜索空间内的所有架构的性能，从而帮助提高了神经架构搜索领域算法的可复现性。

- 参考论文：

    [Ying C, Klein A, Christiansen E, et al. NAS-bench-101: Towards reproducible neural architecture search[C]//International Conference on Machine Learning. PMLR, 2019: 7105-7114.](https://proceedings.mlr.press/v97/ying19a.html) 

- 参考实现：

    https://github.com/google-research/nasbench/tree/master/nasbench

- 适配昇腾 AI 处理器的实现：
  
  ​      


- 通过Git获取对应commit\_id的代码方法如下：
  
    ```
    git clone {repository_url}    # 克隆仓库的代码
    cd {repository_name}    # 切换到模型的代码仓目录
    git checkout  {branch}    # 切换到对应分支
    git reset --hard ｛commit_id｝     # 代码设置到对应的commit_id
    cd ｛code_path｝    # 切换到模型代码所在路径，若仓库下只有该模型，则无需切换
    ```

- 精度

|    精度差       |  2%以内       | 大于2%|
|----------------|-------------- |---------|
| 架构数目        |    2374      | 126     |

备注：总共评估了2500个架构。

- 速度

平均每个架构训练40分钟。


## 默认配置

- 数据集预处理（以CIFAR-10训练集为例，仅作为用户参考示例）：

  - 图像的输入尺寸为32*32
  - 图像输入格式：TFRecord
  - 随机裁剪图像尺寸
  - 随机水平翻转图像


- 数据集设置
  - train_data_files: ['train_1.tfrecords','train_2.tfrecords','train_3.tfrecords','train_4.tfrecords']
  - valid_data_file: 'validation.tfrecords'
  - test_data_file: 'test.tfrecords'
  - sample_data_file: 'sample.tfrecords'
  - data_format: 'channels_last'
  - num_labels: 10

- 搜索空间设置

  - module_vertices: 7
  - max_edges: 9
  - available_ops: ['conv3x3-bn-relu', 'conv1x1-bn-relu', 'maxpool3x3']

- 模型超参
  - stem_filter_size: 128
  - num_stacks: 3
  - num_modules_per_stack: 3
  - batch_size: 256
  - train_epochs: 108
  - train_seconds: 4.0 * 60 * 60
  - learning_rate: 0.1
  - lr_decay_method: COSINE_BY_STEP
  - momentum: 0.9
  - weight_decay: 1e-4
  - max_attempts: 5
  - intermediate_evaluations: ['0.5']
  - num_repeats: 3

## 支持特性

| 特性列表  | 是否支持 |
|-------|------|
| 分布式训练 |  否   |
| 混合精度  |  否  |
| 并行数据  |  否  |

<h2 id="训练环境准备.md">训练环境准备</h2>

1.  硬件环境准备请参见各硬件产品文档"[驱动和固件安装升级指南]( https://support.huawei.com/enterprise/zh/category/ai-computing-platform-pid-1557196528909)"。需要在硬件设备上安装与CANN版本配套的固件与驱动。
2.  宿主机上需要安装Docker并登录[Ascend Hub中心](https://ascendhub.huawei.com/#/detail?name=ascend-tensorflow-arm)获取镜像。

    当前模型支持的镜像列表如[表1](#zh-cn_topic_0000001074498056_table1519011227314)所示。

    **表 1** 镜像列表

    <a name="zh-cn_topic_0000001074498056_table1519011227314"></a>
    <table><thead align="left"><tr id="zh-cn_topic_0000001074498056_row0190152218319"><th class="cellrowborder" valign="top" width="47.32%" id="mcps1.2.4.1.1"><p id="zh-cn_topic_0000001074498056_p1419132211315"><a name="zh-cn_topic_0000001074498056_p1419132211315"></a><a name="zh-cn_topic_0000001074498056_p1419132211315"></a><em id="i1522884921219"><a name="i1522884921219"></a><a name="i1522884921219"></a>镜像名称</em></p>
    </th>
    <th class="cellrowborder" valign="top" width="25.52%" id="mcps1.2.4.1.2"><p id="zh-cn_topic_0000001074498056_p75071327115313"><a name="zh-cn_topic_0000001074498056_p75071327115313"></a><a name="zh-cn_topic_0000001074498056_p75071327115313"></a><em id="i1522994919122"><a name="i1522994919122"></a><a name="i1522994919122"></a>镜像版本</em></p>
    </th>
    <th class="cellrowborder" valign="top" width="27.16%" id="mcps1.2.4.1.3"><p id="zh-cn_topic_0000001074498056_p1024411406234"><a name="zh-cn_topic_0000001074498056_p1024411406234"></a><a name="zh-cn_topic_0000001074498056_p1024411406234"></a><em id="i723012493123"><a name="i723012493123"></a><a name="i723012493123"></a>配套CANN版本</em></p>
    </th>
    </tr>
    </thead>
    <tbody><tr id="zh-cn_topic_0000001074498056_row71915221134"><td class="cellrowborder" valign="top" width="47.32%" headers="mcps1.2.4.1.1 "><a name="zh-cn_topic_0000001074498056_ul81691515131910"></a><a name="zh-cn_topic_0000001074498056_ul81691515131910"></a><ul id="zh-cn_topic_0000001074498056_ul81691515131910"><li><em id="i82326495129"><a name="i82326495129"></a><a name="i82326495129"></a>ARM架构：<a href="https://ascend.huawei.com/ascendhub/#/detail?name=ascend-tensorflow-arm" target="_blank" rel="noopener noreferrer">ascend-tensorflow-arm</a></em></li><li><em id="i18233184918125"><a name="i18233184918125"></a><a name="i18233184918125"></a>x86架构：<a href="https://ascend.huawei.com/ascendhub/#/detail?name=ascend-tensorflow-x86" target="_blank" rel="noopener noreferrer">ascend-tensorflow-x86</a></em></li></ul>
    </td>
    <td class="cellrowborder" valign="top" width="25.52%" headers="mcps1.2.4.1.2 "><p id="zh-cn_topic_0000001074498056_p1450714271532"><a name="zh-cn_topic_0000001074498056_p1450714271532"></a><a name="zh-cn_topic_0000001074498056_p1450714271532"></a><em id="i72359495125"><a name="i72359495125"></a><a name="i72359495125"></a>20.2.0</em></p>
    </td>
    <td class="cellrowborder" valign="top" width="27.16%" headers="mcps1.2.4.1.3 "><p id="zh-cn_topic_0000001074498056_p18244640152312"><a name="zh-cn_topic_0000001074498056_p18244640152312"></a><a name="zh-cn_topic_0000001074498056_p18244640152312"></a><em id="i162363492129"><a name="i162363492129"></a><a name="i162363492129"></a><a href="https://support.huawei.com/enterprise/zh/ascend-computing/cann-pid-251168373/software" target="_blank" rel="noopener noreferrer">20.2</a></em></p>
    </td>
    </tr>
    </tbody>
    </table>


<h2 id="快速上手.md">快速上手</h2>

- 数据集准备
1. 模型训练使用CIFAR10数据集，数据集请用户自行获取。

2. 数据集训练前需要做预处理操作，请用户参考[nasbench](https://github.com/google-research/nasbench/blob/master/nasbench/scripts/generate_cifar10_tfrecords.py),将数据集封装为tfrecord格式。

3. 数据集处理后，放入data目录下，可正常使用。


## 模型训练

- 单击“立即下载”，并选择合适的下载方式下载源码包。

- 启动训练之前，首先要配置程序运行相关环境变量。

  环境变量配置信息参见：

     [Ascend 910训练平台环境变量设置](https://github.com/Ascend/ModelZoo-TensorFlow/wikis/01.%E8%AE%AD%E7%BB%83%E8%84%9A%E6%9C%AC%E8%BF%81%E7%A7%BB%E6%A1%88%E4%BE%8B/Ascend%20910%E8%AE%AD%E7%BB%83%E5%B9%B3%E5%8F%B0%E7%8E%AF%E5%A2%83%E5%8F%98%E9%87%8F%E8%AE%BE%E7%BD%AE)

- 单卡训练 

  训练有如下两种模式：
  1. 训练指定数量的架构。

     首先在脚本train_mutiple_archs.sh中，配置训练的架构数目以及要训练的架构保存的文件名，请用户根据实际配置，参数如下所示：

     ```
      num=5     #生成五个架构
      arch_file="train_arch"    #生成的5个架构的信息存储在data/train_arch.json中
     ```
     启动单卡训练（脚本为train_mutiple_archs.sh）
     ```
     bash train_mutiple_archs.sh
     ```


  2. 训练单个架构

     启动单卡训练 （脚本为train_single_arch.sh） 

     ```
     bash train_single_arch.sh
     ```

<h2 id="高级参考.md">高级参考</h2>

## 脚本和示例代码

```
NAS-Bench-101
└─├─README.md
  ├─data用于存放数据集
  	├─train_tf
    ├─test_tf
  	├─valid_tf
    └─...
  ├─output 用于存放训练结果 
  	├─all_data.json
  	└─...
  ├─checkpoint
  	└─...
  ├─lib
    ├─evaluate.py
    ├─training_time.py
    ├─test_single.py
    ├─model_spec.py
    ├─model_metrics_pb2.py
    ├─model_builder.py
  	└─...
  ├─train_single_arch.sh 训练单个模型的启动脚本，
  ├─train_mutiple_archs.sh 训练多个模型的启动脚本
  └─...
```


## 训练过程

1.  通过“模型训练”中的训练指令启动单卡训练。

2.  训练多个模型得到的架构性能存储路径为./output。

## 结果数据说明

最终结果输出到output/all_data.json中

以
```
"048eda1b87d274d0a3edd77f6974f7c4": {"trainable_params": 21547914, "total_time": 2770.563542842865, "evaluation_results": [{"epochs": 0, "training_steps": 0, "train_accuracy": 0.09845753014087677, "validation_accuracy": 0.1028645858168602, "test_accuracy": 0.10056089609861374, "predict_time": 32.889620780944824}, {"epochs": 54, "training_steps": 8437, "train_accuracy": 0.35596954822540283, "validation_accuracy": 0.3452523946762085, "test_accuracy": 0.34515222907066345, "predict_time": 34.821516036987305}, {"epochs": 108, "training_steps": 16875, "train_accuracy": 0.9992988705635071, "validation_accuracy": 0.8845152258872986, "test_accuracy": 0.8683894276618958, "predict_time": 34.703524351119995}]}
```
为例

"048eda1b87d274d0a3edd77f6974f7c4"表示经过MD5哈希算法得到的架构字符串Key，trainable_params表示可训练参数数量，total_time表示训练总耗时(s)。evaluation_results中存储第0，54，108个迭代后模型的性能指标，以迭代0为例，epoch表示当前迭代次数，training_steps表示当前训练的步数，train_accuracy表示当前迭代下模型在训练集上的准确率，validation_accuracy表示当前迭代下模型在验证集上的准确率，test_accuracy表示当前迭代下模型在测试集上的准确率，predict_time表示模型在测试数据集上的验证时间。
