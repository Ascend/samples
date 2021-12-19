**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行图片样例wiki](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874)。**

## style_transfer_picture样例

功能：使用风格迁移模型对输入图片进行推理，并生成风格化后的图片。

样例输入：原始图片jpg文件。

样例输出：带推理结果的jpg文件。

### 适配要求

本产品的适配要求如下表，如不符合适配要求，样例可能运行失败。

| 适配项     | 适配条件                                                     | 备注                                                         |
| ---------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 适配版本   | >=5.0.4                                                    | 已完成版本安装，版本信息请参考[版本说明](https://ascend.huawei.com/zh/#/software/cann/notice) |
| 适配硬件   | Atlas200DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366)) | 当前已在Atlas200DK和Atlas300测试通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) |
| 第三方依赖 | opencv, python-acllite                                     | 请参考[第三方依赖安装指导（python样例）](../../../environment)完成对应安装 |

### 软件准备

1. 获取源码包。

   可以使用以下两种方式下载，请选择其中一种进行源码准备。   

    - 命令行方式下载（下载时间较长，但步骤简单）。
      ```    
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

2. 获取此应用中所需要的原始网络模型。
    |  **模型说明**  |  **模型下载路径**  |
    |---|---|
    | 基于GAN网络的模型。  |  请参考[https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/style_transfer](https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/style_transfer)目录中README.md下载原始模型章节下载模型和权重文件。 |

    ```
    # 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。也可以参照上表在modelzoo中下载并手工转换，以了解更多细节。 
    #本样例只展示“星空”风格的推理效果，请根据实际情况修改。
    cd $HOME/samples/python/level2_simple_inference/2_object_detection/style_transfer_picture/model    
    wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/style_transfer_picture/xingkong1.pb
    wget https://c7xcode.obs.myhuaweicloud.com/models/style_transfer_picture/tangguo.pb
    wget https://c7xcode.obs.myhuaweicloud.com/models/style_transfer_picture/bijiasuo.pb
    wget https://c7xcode.obs.myhuaweicloud.com/models/style_transfer_picture/work_soldiers.pb    
    atc --model=./xingkong1.pb --framework=3 --output=xingkong1_fp32_nchw_no_aipp --soc_version=Ascend310
    atc --model=./bijiasuo.pb --framework=3 --output=bijiasuo_fp32_nchw_no_aipp --soc_version=Ascend310
    atc --model=./tangguo.pb --framework=3 --output=tangguo_fp32_nchw_no_aipp --soc_version=Ascend310
    atc --model=./work_soldiers.pb --framework=3 --output=work_soldiers_fp32_nchw_no_aipp --soc_version=Ascend310
    ```

3. 获取样例需要的测试图片。
    ```
    # 执行以下命令，进入样例的data文件夹中，下载对应的测试图片。
    cd $HOME/samples/python/level2_simple_inference/2_object_detection/style_transfer_picture/data
    wget https://c7xcode.obs.myhuaweicloud.com/models/style_transfer_picture/data/test.jpg
    cd ../src
    ```


### 样例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 执行以下命令,将开发环境的 **samples** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。
    ```
    # 【xxx.xxx.xxx.xxx】为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。
    scp -r $HOME/samples/python/level2_simple_inference/2_object_detection/style_transfer HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
    ssh HwHiAiUser@xxx.xxx.xxx.xxx
    cd $HOME/samples/python/level2_simple_inference/2_object_detection/style_transfer/src
    ```

2. <a name="step_2"></a>运行可执行文件。
    ```
    （1）星空风格
    python3.6 main.py ../data xingkong
    （2）糖果风格
    python3.6 main.py ../data tangguo
    （3）毕加索风格
    python3.6 main.py ../data bijiasuo
    （4）工农兵风格
    python3.6 main.py ../data worksoldiers
    ```

### 查看结果

运行完成后，会在运行环境的工程目录的outputs文件夹下保存推理后的图片，如下图所示。

![输入图片说明](https://images.gitee.com/uploads/images/2021/1103/094011_426f5b30_7647177.png "image-20211102165506515.png")

