<h2 id="基本信息.md">基本信息</h2>

**发布者（Publisher）：Huawei**

**应用领域（Application Domain）：** Image Classification 

**版本（Version）：1.0**

**修改时间（Modified） ：2022.10.18**

**大小（Size）：88kb**

**框架（Framework）：TensorFlow 1.15.0**

**模型格式（Model Format）：**

**精度（Precision）：**

**处理器（Processor）：昇腾910**

**应用级别（Categories）：Research**

**描述（Description）：NAS-Bench-201数据集源代码** 

<h2 id="概述.md">概述</h2>

NAS-Bench-201是百度在论文《NAS-Bench-201: Extending the Scope of Reproducible Neural Architecture Search》中提出的针对神经架构搜索（neural architecture search）领域的架构数据集。该NAS-Bench-201中定义的搜索空间包括由4个节点和5个相关操作选项生成的所有可能的cell结构，总共产生 15625 个cell候选对象。为三个数据集（cifar，cifar100和imagenet ）提供了使用相同设置的训练日志以及每种结构候选的性能。

- 参考论文：

    [Dong X, Yang Y. NAS-Bench-201: Extending the Scope of Reproducible Neural Architecture Search[C]//International Conference on Learning Representations. 2019.](https://openreview.net/forum?id=HJxyZkBKDr) 

- 参考实现：

    https://github.com/D-X-Y/AutoDL-Projects/tree/main/xautodl/models/cell_infers

- 适配昇腾 AI 处理器的实现：
  
  


- 通过Git获取对应commit\_id的代码方法如下：
  
    ```
    git clone {repository_url}    # 克隆仓库的代码
    cd {repository_name}    # 切换到模型的代码仓目录
    git checkout  {branch}    # 切换到对应分支
    git reset --hard ｛commit_id｝     # 代码设置到对应的commit_id
    cd ｛code_path｝    # 切换到模型代码所在路径，若仓库下只有该模型，则无需切换
    ```

- 精度

|    精度差       |  5%以内      | 大于5% |
|----------------|-------------- |---------|
| 架构数目        | 1669 | 831 |

备注：总共评估了2500个架构。

- 速度

  单架构平均耗时约20分钟

## 默认配置

- 数据集预处理（以CIFAR-10训练集为例，仅作为用户参考示例）：

  - 图像的输入尺寸为32*32
  - 图像输入格式：TFRecord
  - 随机裁剪图像尺寸
  - 随机水平翻转图像

- 模型超参
  - epoch: 200
  - batch_size: 256
  - momentum: 0.9
  - weight decay: 0.0005
  - V: 4
  - random flip: p=0.5
  - initial LR: 0.1
  - end LR: 0
  - LR schedule: consine
  - initial channel: 16
  - N: 5

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

3. 数据集处理后，放入dataset目录下，可正常使用。
   

## 模型训练

- 单击“立即下载”，并选择合适的下载方式下载源码包。

- 启动训练之前，首先要配置程序运行相关环境变量。

  环境变量配置信息参见：

     [Ascend 910训练平台环境变量设置](https://github.com/Ascend/ModelZoo-TensorFlow/wikis/01.%E8%AE%AD%E7%BB%83%E8%84%9A%E6%9C%AC%E8%BF%81%E7%A7%BB%E6%A1%88%E4%BE%8B/Ascend%20910%E8%AE%AD%E7%BB%83%E5%B9%B3%E5%8F%B0%E7%8E%AF%E5%A2%83%E5%8F%98%E9%87%8F%E8%AE%BE%E7%BD%AE)

- 单卡训练 
  训练training_arch文件夹下的架构。以使用卡0训练training_arch/arch0.txt文件中架构为例
  
  ```
  bash train_arch.sh
  ```

<h2 id="高级参考.md">高级参考</h2>

## 脚本和示例代码

```
NAS-Bench-201
└─├─README.md
  ├─dataset用于存放数据集
  	├─train_tf
    ├─test_tf
  	├─valid_tf
    └─...
  ├─result 用于存放训练结果 
  	├─result0.txt
  	└─...
  ├─checkpoint
  	└─model.ckpt
  ├─cell.py
  ├─cifar.py
  ├─config.py
  ├─generate_cifar10_tfrecords.py
  ├─genotypes.py
  ├─infer.py
  ├─operations.py
  ├─train_arch.py
  └─...
```


## 训练过程

1.  通过“模型训练”中的训练指令启动单卡训练。

2.  训练多个模型得到的架构性能存储路径为./result。

## 结果数据说明

最终结果存储在result/result.txt中
以

```
id:0,code:[4, 4, 4, 0, 2, 0],time:745.347, acc:80.0
```
为例
id表示当前架构编号，code表示当前架构的对应编码格式，time表示训练总耗时(s)，acc表示当前架构在验证集上的准确率。
