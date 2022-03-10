中文|[English](README.md)

**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行图片样例wiki](https://github.com/Ascend/samples/wikis/Mindstudio运行图片样例?sort_id=3164874)。**

**本案例由上海交通大学提供**

## hpa_classification样例
功能：对蛋白质图像进行自动化分类评估。   
样例输入：未标注的蛋白质荧光显微图片    
样例输出：已经标注分类的蛋白质图谱

### 前置条件
请检查以下条件要求是否满足，如不满足请按照备注进行相应处理。如果CANN版本升级，请同步检查第三方依赖是否需要重新安装（5.0.4及以上版本第三方依赖和5.0.4以下版本有差异，需要重新安装）。
| 条件 | 要求 | 备注 |
|---|---|---|
| CANN版本 | >=5.0.4 | 请参考CANN样例仓介绍中的[安装步骤](https://github.com/Ascend/samples#%E5%AE%89%E8%A3%85)完成CANN安装，如果CANN低于要求版本请根据[版本说明](https://github.com/Ascend/samples/blob/master/README_CN.md#%E7%89%88%E6%9C%AC%E8%AF%B4%E6%98%8E)切换samples仓到对应CANN版本 |
| 硬件要求 | Atlas200DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))  | 当前已在Atlas200DK和Atlas300测试通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) ，其他产品可能需要另做适配|
| 第三方依赖 | opencv, python-acllite | 请参考[第三方依赖安装指导（python样例）](../../environment)选择需要的依赖完成安装 |

### 样例准备

1. 获取源码包。

   可以使用以下两种方式下载，请选择其中一种进行源码准备。   
    - 命令行方式下载（下载时间较长，但步骤简单）。
       ```    
       # 开发环境，非root用户命令行中执行以下命令下载源码仓。    
       cd ${HOME}     
       git clone https://github.com/Ascend/samples.git
       ```
       **注：如果需要切换到其它tag版本，以v0.5.0为例，可执行以下命令。**
       ```
       git checkout v0.5.0
       ```   
    - 压缩包方式下载（下载时间较短，但步骤稍微复杂）。   
       **注：如果需要下载其它版本代码，请先请根据前置条件说明进行samples仓分支切换。**   
       ``` 
        # 1. samples仓右上角选择 【克隆/下载】 下拉框并选择 【下载ZIP】。    
        # 2. 将ZIP包上传到开发环境中的普通用户家目录中，【例如：${HOME}/ascend-samples-master.zip】。     
        # 3. 开发环境中，执行以下命令，解压zip包。     
        cd ${HOME}    
        unzip ascend-samples-master.zip
        ```

2. 获取此应用中所需要的模型     
    | **模型名称** | **模型说明**          | **模型下载路径**                                             |
    | ------------ | --------------------- | ------------------------------------------------------------ |
    | deploy_vel          | 基于Caffe的蛋白质亚细胞定位预测 | 请参考https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/hpa/ATC_hpa_caffe_AE 中README.md原始模型章节，下载**原始模型网络**及**模型权重文件**。 |
    ```
    # 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。也可以参照上表在modelzoo中下载并手工转换，以了解更多细节。     
    cd ${HOME}/samples/python/level2_simple_inference/6_other/colorization_picture/model    
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/hpa/hpa.prototxt    
    wget https://modelzoo-train-atc.obs.cn-north-4.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/hpa/hpa.caffemodel
    atc --model=./hpa.prototxt --weight=./hpa.caffemodel --framework=0 --output=./deploy_vel  --soc_version=Ascend310 --input_format=NCHW --input_fp16_nodes=data --output_type=FP32 --out_nodes="score:0"  
    ```
3. 获取样例需要的测试图片并切换文件夹。   
    ```
    # 执行以下命令，进入样例的data文件夹中，下载对应的测试图片。
    cd ${HOME}/samples/python/contrib/human_protein_map_classification/data
    wget https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/hpa_classification/test_image/test.jpeg
    cd ../src
    ```
### 样例运行

**注：开发环境与运行环境合一部署，请跳过步骤1，直接执行步骤2即可。**

1. 执行以下命令,将开发环境的**human_protein_map_classification**目录上传到运行环境中，例如 **/home/HwHiAiUser**，并以HwHiAiUser（运行用户）登录运行环境（Host）。
    ```
    # 【xxx.xxx.xxx.xxx】为运行环境ip，200DK在USB连接时一般为192.168.1.2，300（ai1s）为对应的公网ip。
    scp -r $HOME/samples/python/contrib/human_protein_map_classification  HwHiAiUser@xxx.xxx.xxx.xxx:/home/HwHiAiUser
    ssh HwHiAiUser@xxx.xxx.xxx.xxx
    cd ${HOME}/human_protein_map_classification/src   
    ```
2. 运行可执行文件。
    ```
    python3.6 main.py ../data/
    ```
### 查看结果

运行完成后，会在运行环境的命令行中打印出推理结果。   
![输入图片说明](https://images.gitee.com/uploads/images/2021/1109/145642_b0586422_5400693.png "屏幕截图.png")

### 常见错误
请参考[常见问题定位](https://github.com/Ascend/samples/wikis/%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E5%AE%9A%E4%BD%8D/%E4%BB%8B%E7%BB%8D)对遇到的错误进行排查。如果wiki中不包含，请在samples仓提issue反馈。
