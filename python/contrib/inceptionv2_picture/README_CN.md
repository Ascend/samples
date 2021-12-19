中文|[English](README.md)

**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行图片样例wiki](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874)。**

**本样例由[尚艺良品](https://www.sunywonders.com/index.html#/AboutUs)贡献**

## InceptionV2踢脚线分类样例    
功能：使用InceptionV2模型对输入的踢脚线图片进行分类推理。
样例输入：待推理的踢脚线图片。    
样例输出：推理后的踢脚线图片。   

### 适配要求

本产品的适配要求如下表，如不符合适配要求，样例可能运行失败。
| 适配项 | 适配条件 | 备注 |
|---|---|---|
| 适配版本 | >=5.0.4 | 已完成版本安装，版本信息请参考[版本说明](https://ascend.huawei.com/zh/#/software/cann/notice) |
| 适配硬件 | Atlas200DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))  | 当前已在Atlas200DK和Atlas300测试通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) |
| 第三方依赖 | opencv, python-acllite | 请参考[第三方依赖安装指导（python样例）](../../../environment)完成对应安装 |

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

2. 获取此应用中所需要的原始网络模型。
    |  **模型名称**  |  **模型说明**  |  **模型下载路径**  |
    |---|---|---|
    |  InceptionV2| 图片分类推理模型。是tensorflow框架模型。  |  请参考https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/inceptionv2/ATC_inceptionV2_tf_AE 中README.md原始模型章节，下载**原始模型网络**及**模型权重文件**。 |
    ```
    # 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。也可以参照上表在modelzoo中下载并手工转换，以了解更多细节。     
    cd ${HOME}/samples/python/contrib/inceptionv2_picture/model    
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/InceptionV2/frozen_graph.pb    
    atc --input_shape="input:1,299,299,3" --input_format=NHWC --output="frozen_graph-inception-resnet-test1" --soc_version=Ascend310 --framework=3 --model="./frozen_graph.pb"
    ```

3. 获取样例需要的测试图片。
    ```
    # 执行以下命令，进入样例的data文件夹中，下载对应的测试图片。
    cd $HOME/samples/python/contrib/inceptionv2_picture/data**
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/InceptionV2/pic/1101.jpg
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/InceptionV2/pic/1108.jpg
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/InceptionV2/pic/1309.jpg
    cd ../src
    ```


### 样例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行[步骤2](#step_2)即可。**   

1. 执行以下命令,将开发环境的 **inceptionv2_picture** 目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。
    ```
    # 【xxx.xxx.xxx.xxx】为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。
    scp -r $HOME/samples/python/contrib/inceptionv2_picture HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
    ssh HwHiAiUser@xxx.xxx.xxx.xxx    
    cd ${HOME}/inceptionv2_picture/src 
    ```
2. <a name="step_2"></a>运行可执行文件。
    ```
    python3.6 classify.py ../data/
    ```

### 查看结果
运行完成后，会在outputs目录下生成带推理结果的jpg图片。    
![输入图片说明](https://images.gitee.com/uploads/images/2021/1109/151749_4735843e_5400693.png "屏幕截图.png")