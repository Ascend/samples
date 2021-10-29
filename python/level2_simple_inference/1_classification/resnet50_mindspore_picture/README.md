**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本样例适配CANN3.2.0及以上版本，支持产品为Atlas200DK、Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))。**

**本README只提供命令行方式运行样例的指导**

## resnet50图片分类样例


功能：使用resnet50模型对输入图片进行分类推理。

样例输入：待推理的jpg图片。

样例输出：推理后的jpg图片。

训练过程参考 [resnet50 mindspore训练](https://github.com/Ascend/modelzoo/tree/master/built-in/MindSpore/Official/cv/image_classification/ResNet50_for_MindSpore)； 在Ascend910环境， 使用本仓库中scripts/convert.py 脚本将训练好的resnet-90_1875.ckpt 转换为resnet-90_1875.air模型文件

### 前提条件

部署此Sample前，需要准备好以下环境：

- 请确认已按照[环境准备和依赖安装](../../../environment)准备好环境。

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

    参考下表获取此应用中所用到的原始网络模型及其对应的权重文件，并将其存放到开发环境普通用户下的任意目录，例如：$HOME/models/resnet50_mindspore_picture。
    
    |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
    |---|---|---|
    |  resnet50 | 图片分类推理模型。是基于MindSpore的resnet50模型。  |  [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/resnet50_mindspore/resnet-90_1875.air](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/resnet50_mindspore/resnet-90_1875.air) |


3. 将原始模型转换为Davinci模型。
    
    **注：请确认环境变量已经在[环境准备和依赖安装](../../../environment)中配置完成**

    1. 设置LD_LIBRARY_PATH环境变量。

        由于LD_LIBRARY_PATH环境变量在转使用atc工具和运行样例时会产生冲突，所以需要在命令行单独设置此环境变量，方便修改。    

        **export install_path=$HOME/Ascend/ascend-toolkit/latest**     
        **export LD_LIBRARY_PATH=\\${install_path}/atc/lib64**  

    2. 执行以下命令下载aipp配置文件并使用atc命令进行模型转换。

        **cd $HOME/models/resnet50_mindspore_picture**  

        **atc --model=./resnet-90_1875.air --framework=1 --output=./resnet50 --soc_version=Ascend310**

    3. 执行以下命令将转换好的模型复制到样例中model文件夹中。

        **cp ./resnet50.om $HOME/samples/python/level2_simple_inference/1_classification/resnet50_mindspore_picture/model/**

4. 获取样例需要的测试图片。

    执行以下命令，进入样例的data文件夹中，下载对应的测试图片。

    **cd $HOME/samples/python/level2_simple_inference/1_classification/resnet50_mindspore_picture/data**

    **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/data/airplane.jpg** 

    **wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/data/car.jpg**


### 样例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 执行以下命令,由于样例使用了公共库中的接口，所以这里需要把公共库的文件也拷贝到开发板，这里将开发环境的 **samples** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。

    **scp -r $HOME/samples HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser**

    **ssh HwHiAiUser@xxx.xxx.xxx.xxx**    

    ![](https://images.gitee.com/uploads/images/2020/1106/160652_6146f6a4_5395865.gif "icon-note.gif") **说明：**  
    > - **xxx.xxx.xxx.xxx**为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。

2. <a name="step_2"></a>运行可执行文件。

    - 如果是开发环境与运行环境合一部署，执行以下命令，设置运行环境变量，并切换目录。

      **export LD_LIBRARY_PATH=**

      **source ~/.bashrc**
        
      **cd $HOME/samples/python/level2_simple_inference/1_classification/resnet50_mindspore_picture/src**

    - 如果是开发环境与运行环境分离部署，执行以下命令切换目录。
    
      **cd $HOME/samples/python/level2_simple_inference/1_classification/resnet50_mindspore_picture/src**      

    切换目录后，执行以下命令运行样例。

    **python3.6 classify.py ../data/**
### 查看结果

运行完成后，会在outputs目录下生成带推理结果的jpg图片。
