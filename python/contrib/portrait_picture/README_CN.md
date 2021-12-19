中文|[English](README.md)

**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本README只提供命令行方式运行样例的指导。**

**本样例为清华大学贡献**

## portrait_picture样例

功能：使用Portrait模型对输入图片中人像进行分割，然后与背景图像融合，实现背景替换。

样例输入：带有人像的jpg图片和一张背景图像

样例输出：背景替换后的图像。

### 适配要求

本产品的适配要求如下表，如不符合适配要求，样例可能运行失败。
| 适配项 | 适配条件 | 备注 |
|---|---|---|
| 适配版本 | >=5.0.4 | 已完成版本安装，版本信息请参考[版本说明](https://ascend.huawei.com/zh/#/software/cann/notice) |
| 适配硬件 | Atlas200DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))  | 当前已在Atlas200DK和Atlas300测试通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) |
| 第三方依赖 | opencv,python-acllite | 请参考[第三方依赖安装指导（python样例）](../../../environment)完成对应安装 |

### 软件准备

1. 获取源码包。    
   可以使用以下两种方式下载，请选择其中一种进行源码准备。   
    - 命令行方式下载（下载时间较长，但步骤简单）。
       ```    
       # 开发环境，非root用户命令行中执行以下命令下载源码仓。    
       cd ${HOME}     
       git clone https://github.com/Ascend/samples.git
       ```   
    - 压缩包方式下载（下载时间较短，但步骤稍微复杂）。   
       ``` 
        # 1. samples仓右上角选择 【克隆/下载】 下拉框并选择 【下载ZIP】。    
        # 2. 将ZIP包上传到开发环境中的普通用户家目录中，【例如：${HOME}/ascend-samples-master.zip】。     
        # 3. 开发环境中，执行以下命令，解压zip包。     
        cd ${HOME}    
        unzip ascend-samples-master.zip
       ```

2. 获取此应用中所需要的模型

| **模型名称** | **模型说明**                   | **模型下载路径**                                             |
| ------------ | ------------------------------ | ------------------------------------------------------------ |
| Portrait     | 基于TensorFlow的人像分割模型。 | 请参考https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/portraitnet/ATC_PortraitNet_tf_AE 中README.md原始模型章节，下载**原始模型**及**对应的cfg文件**。 |
    
    
    # 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。也可以参照上表在modelzoo中下载并手工转换，以了解更多细节。     
    cd $HOME/samples/python/contrib/portrait_picture/model
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/PortraitNet%20/portrait.pb   
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/PortraitNet%20/insert_op.cfg
    atc --model=./portrait.pb  --insert_op_conf=./insert_op.cfg  --output="./portrait" --output_type=FP32 --input_shape="Inputs/x_input:1,224,224,3" --framework=3 --soc_version=Ascend310



3. 获取样例需要的测试图片
    ```
    # 执行以下命令，进入样例的data文件夹中，下载对应的测试图片。 
    cd $HOME/samples/python/contrib/portrait_picture/model
    wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/Portrait/background.jpg
    wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/Portrait/ori.jpg
    ```

### 样例运行
**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 执行以下命令,将开发环境的 **portrait_picture** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。
    ```
    # 【xxx.xxx.xxx.xxx】为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。
    scp -r $HOME/samples/python/contrib/portrait_pictureHwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
    ssh HwHiAiUser@xxx.xxx.xxx.xxx
    cd ${HOME}/portrait_picture/src    
    ```
2. <a name="step_2"></a>运行可执行文件。
    ```
    python3.6 main.py
    ```

### 查看结果

运行完成后，会在out/result目录下生成带推理结果的jpg图片。