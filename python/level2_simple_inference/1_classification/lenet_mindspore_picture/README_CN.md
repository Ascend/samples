中文|[English](README.md)

**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本样例支持产品为Atlas200DK、Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))。**

**本README只提供命令行方式运行样例的指导**

## 手写数字分类样例

功能：使用lenet模型对输入图片进行分类推理。

样例输入：待推理的jpg图片。

样例输出：存放图片中数字的txt文本。

训练过程参考 [lennet mindspore训练](https://github.com/Ascend/modelzoo/tree/master/built-in/MindSpore/Official/cv/image_classification/LeNet_for_MindSpore)；
 在Ascend910环境， 使用scripts/convert.py 脚本将训练好的checkpoint_lenet-1_1875.ckpt 转换为mnist.air模型文件


### 适配要求

本产品的适配要求如下表，如不符合适配要求，样例可能运行失败。
| 适配项 | 适配条件 | 备注 |
|---|---|---|
| 适配版本 | >=5.0.4 | 已完成版本安装，版本信息请参考[版本说明](https://ascend.huawei.com/zh/#/software/cann/notice) |
| 适配硬件 | Atlas200DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))  | 当前已在Atlas200DK和Atlas300测试通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) |
| 第三方依赖 | opencv, numpy | 请参考[第三方依赖安装指导（python样例）](../../../environment)完成对应安装 |

### 软件准备

1. 获取源码包。

   可以使用以下两种方式下载，请选择其中一种进行源码准备。

    - 命令行方式下载（下载时间较长，但步骤简单）。

        开发环境，非root用户命令行中执行以下命令下载源码仓。
        
        ```
        cd ${HOME}
        git clone https://github.com/Ascend/samples.git
        ```
    - 压缩包方式下载（下载时间较短，但步骤稍微复杂）。

        1. samples仓右上角选择 **克隆/下载** 下拉框并选择 **下载ZIP**。
        2. 将zip包上传到开发环境中的普通用户家目录中，例如 **${HOME}/ascend-samples-master.zip**。
        3. 开发环境中，执行以下命令，解压zip包。
        ```
        cd ${HOME}
        unzip ascend-samples-master.zip
        ```
2. 获取此应用中所需要的原始网络模型。
    |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
    |---|---|---|
    | mnist | 图片分类推理模型。是基于mindspore的lenet模型。 | https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/lenet/mnist.air |
    ```
    # 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。
    cd ${HOME}/samples/python/level2_simple_inference/1_classification/lenet_mindspore_picture/model    
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/lenet/mnist.air
    atc --framework=1 --model=mnist.air  --output=mnist --soc_version=Ascend310
    ```

3. 获取样例需要的测试图片。
    ```
    # 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。
    cd ${HOME}/samples/python/level2_simple_inference/1_classification/lenet_mindspore_picture/data    
    wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/lenet_mindspore/test_image/test1.png
    ```

### 样例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 执行以下命令,将开发环境的 **lenet_mindspore_picture** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。
    ```
    # 【xxx.xxx.xxx.xxx】为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。
    scp -r ${HOME}/samples/python/level2_simple_inference/1_classification/lenet_mindspore_picture HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
    ssh HwHiAiUser@xxx.xxx.xxx.xxx
    cd ${HOME}/lenet_mindspore_picture/src    
    ```

2. <a name="step_2"></a>运行样例。
   ```
   cd ${HOME}/samples/python/level2_simple_inference/1_classification/lenet_mindspore_picture/src
   python3.6 classify.py ./data/
   ```

### 查看结果

运行时，会在命令行打印推理结果，显示如下
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
并在outputs文件夹下生成txt文件，存放置信度最高的数字。
