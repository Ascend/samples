中文|[English](README.md)

**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本样例适配3.0.0及以上版本，支持产品为Atlas200DK、Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))。**

**本README只提供命令行方式运行样例的指导**

## 手写数字分类样例

功能：使用lenet模型对输入图片进行分类推理。

样例输入：待推理的jpg图片。

样例输出：图片中的数字。

训练过程参考 [lennet mindspore训练](https://github.com/Ascend/modelzoo/tree/master/built-in/MindSpore/Official/cv/image_classification/LeNet_for_MindSpore)；
 在Ascend910环境， 使用scripts/convert.py 脚本将训练好的checkpoint_lenet-1_1875.ckpt 转换为mnist.air模型文件


### 前提条件

部署此Sample前，需要准备好以下环境：

- 请确认已按照[环境准备和依赖安装](https://github.com/Ascend/samples/tree/master/python/environment)准备好环境。

- 已完成对应产品的开发环境和运行环境安装。

### 软件准备

1. 获取源码包。

   可以使用以下两种方式下载，请选择其中一种进行源码准备。

    - 命令行方式下载（下载时间较长，但步骤简单）。

        开发环境，非root用户命令行中执行以下命令下载源码仓。

       **cd $HOME**

       **git clone https://github.com/Ascend/samples.git**

    - 压缩包方式下载（下载时间较短，但步骤稍微复杂）。

        1. samples仓右上角选择 **克隆/下载** 下拉框并选择 **下载ZIP**。

        2. 将ZIP包上传到开发环境中的普通用户家目录中，例如 **$HOME/ascend-samples-master.zip**。

        3. 开发环境中，执行以下命令，解压zip包。

            **cd $HOME**

            **unzip ascend-samples-master.zip**

2. 获取此应用中所需要的原始网络模型。

    参考下表获取此应用中所用到的原始网络模型及其对应的权重文件，并将其存放到开发环境普通用户下的任意目录，例如：$HOME/models/lenet_mindspore。

    |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
    |---|---|---|
    | mnist | 图片分类推理模型。是基于mindspore的lenet模型。 | https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/lenet/mnist.air |

3. 将原始模型转换为Davinci模型。
   
    **注：请确认环境变量已经在[环境准备和依赖安装](https://github.com/Ascend/samples/tree/dev/python/environment)中配置完成**

    1. 设置LD_LIBRARY_PATH环境变量。

        由于LD_LIBRARY_PATH环境变量在转使用atc工具和运行样例时会产生冲突，所以需要在命令行单独设置此环境变量，方便修改。

        **export install_path=$HOME/Ascend/ascend-toolkit/latest**

        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

    2. 执行以下atc命令进行模型转换。

        **cd $HOME/models/lenet_mindspore**

        **atc --framework=1 --model=mnist.air  --output=mnist --soc_version=Ascend310**

    3. 执行以下命令将转换好的模型复制到样例中model文件夹中。

        **cp ./mnist.om $HOME/samples/python/level2_simple_inference/1_classification/lenet_mindspore_picture/model/**

4. 获取样例需要的测试图片。

    执行以下命令，进入样例的data文件夹中，下载对应的测试图片。

    **cd $HOME/samples/python/level2_simple_inference/1_classification/lenet_mindspore_picture/data**

    **https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/lenet_mindspore/test_image/test1.png**

    **https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/lenet_mindspore/test_image/test2.png** 
    
    **https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/lenet_mindspore/test_image/test3.png**    



### 样例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 执行以下命令,将开发环境的 samples** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。

    **scp -r $HOME/samples/  HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

    **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  

    > - **xxx.xxx.xxx.xxx**为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。

2. <a name="step_2"></a>运行可执行文件。

      **export LD_LIBRARY_PATH=**
      **source ~/.bashrc**
      **cd $HOME/samples/python/level2_simple_inference/1_classification/lenet_mindspore_picture/**     

    切换目录后，执行以下命令运行样例。

    **python3.6 src/classify.py ./data/**
### 查看结果

运行时，会打印推理结果
```
init resource stage:
Init resource success
Init model resource start...
[Model] create model output dataset:
malloc output 0, size 40
Create model output dataset success
Init model resource success
(32, 32)
post process
images:test2.png
======== top5 inference results: =============
label:9  confidence: 0.991472, class: 9
label:7  confidence: 0.003693, class: 7
label:8  confidence: 0.001775, class: 8
label:3  confidence: 0.001515, class: 3
label:4  confidence: 0.000880, class: 4
(32, 32)
post process
images:test3.png
======== top5 inference results: =============
label:7  confidence: 0.958997, class: 7
label:9  confidence: 0.022686, class: 9
label:8  confidence: 0.006465, class: 8
label:3  confidence: 0.005904, class: 3
label:1  confidence: 0.002834, class: 1
(32, 32)
post process
images:test1.png
======== top5 inference results: =============
label:1  confidence: 0.993403, class: 1
label:9  confidence: 0.001830, class: 9
label:8  confidence: 0.001219, class: 8
label:4  confidence: 0.001122, class: 4
label:7  confidence: 0.000977, class: 7
acl resource release all resource
Model release source success
acl resource release stream
acl resource release context
Reset acl device  0
Release acl resource success
run success
```
