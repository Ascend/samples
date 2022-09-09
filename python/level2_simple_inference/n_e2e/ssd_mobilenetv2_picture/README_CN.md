中文|[English](README.md)

**本样例为大家学习昇腾软件栈提供参考，非商业目的！**

**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行图片样例wiki](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874)。**

功能：基于CANN ACL，将您训练完成的ssd mobilenetv2模型开发成一个AI应用。   
样例输入：待推理的jpg图片。   
样例输出：检测出的boxes和labels，并将boxes和labels输出到原始图片上。  
 **该案例推理效果取决于你训练的模型效果，本案例也只训练了很短时间，所以效果也是一般的，主要是给大家参考开发。** 

### 前置条件
请检查以下条件要求是否满足，如不满足请按照备注进行相应处理。如果CANN版本升级，请同步检查第三方依赖是否需要重新安装（5.0.4及以上版本第三方依赖和5.0.4以下版本有差异，需要重新安装）。
| 条件 | 要求 | 备注 |
|---|---|---|
| CANN版本 | >=5.0.4 | 请参考CANN样例仓介绍中的[安装步骤](https://github.com/Ascend/samples#%E5%AE%89%E8%A3%85)完成CANN安装，如果CANN低于要求版本请根据[版本说明](https://github.com/Ascend/samples/blob/master/README_CN.md#%E7%89%88%E6%9C%AC%E8%AF%B4%E6%98%8E)切换samples仓到对应CANN版本 |
| 硬件要求 | Atlas200DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))  | 当前已在Atlas200DK和Atlas300测试通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) ，其他产品可能需要另做适配|
| 第三方依赖 | opencv,python-acllite | 请参考[第三方依赖安装指导（python样例）](../../../environment)选择需要的依赖完成安装 |

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
2. 获取此应用中所需要的原始网络模型。本案例中，原始模型是你自己训练出来的air模型，你可以通过如下命令示例完成模型转换(--model后面请替换为你自己训练的模型，建议放到当前案例下模型存放目录）：
    ```
    cd ${HOME}/samples/python/level2_simple_inference/n_e2e/ssd_mobilenetv2_picture/model
    atc --input_format=NCHW --framework=1 --model="ssd_500.air" --output="ssd_500" --soc_version=Ascend310
    ```

3. 获取样例需要的测试图片并切换文件夹(可以用你自己的图片，可以选择任意coco数据集中覆盖的81类物体图片，这里的图片是方便你运行放的一个例子而已)
    ```
    # 执行以下命令，进入样例的data文件夹中，下载对应的测试图片。
    cd ${HOME}/samples/python/level2_simple_inference/n_e2e/ssd_mobilenetv2_picture/data
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/YOLOV3_coco_detection_picture/dog1_1024_683.jpg
    cd ../src
    ```
### 样例运行

**注：开发环境与运行环境不在一起请自行将测试图片拷贝到运行环境。**   

1. <a name="step_2"></a>运行样例。
   ```
    python3 main.py
   ```

### 查看结果

运行完成后，会在样例工程的out/目录下生成推理后的图片。


### 常见错误
请参考[常见问题定位](https://github.com/Ascend/samples/wikis/%E5%B8%B8%E8%A7%81%E9%97%AE%E9%A2%98%E5%AE%9A%E4%BD%8D/%E4%BB%8B%E7%BB%8D)对遇到的错误进行排查。如果wiki中不包含，请在samples仓提issue反馈。