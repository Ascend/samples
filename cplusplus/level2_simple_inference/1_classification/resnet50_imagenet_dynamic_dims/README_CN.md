**本样例为大家学习昇腾软件栈提供参考，非商业目的！**
**本README只提供命令行方式运行样例的指导，如需在Mindstudio下运行样例，请参考[Mindstudio运行图片样例wiki](https://github.com/Ascend/samples/wikis/Mindstudio%E8%BF%90%E8%A1%8C%E5%9B%BE%E7%89%87%E6%A0%B7%E4%BE%8B?sort_id=3164874)。**

# 基于TensorFlow ResNet-50网络实现图片分类<a name="ZH-CN_TOPIC_0302603648"></a>

## 功能描述<a name="section340311417417"></a>

1. 该样例主要是基于TensorFlow ResNet-50网络（动态dims）实现图片分类的功能。

2. 样例会执行两次推理，两次分别设置不同的动态dims档位。第一次将动态dims设置为2,3,224,224档位，第二次设置为2,3,200,200档位。

## 环境要求<a name="section3833348101215"></a>

请检查以下条件要求是否满足，如不满足请按照备注进行相应处理。如果CANN版本升级，请同步检查第三方依赖是否需要重新安装（5.0.4及以上版本第三方依赖和5.0.4以下版本有差异，需要重新安装）。
| 条件 | 要求 | 备注 |
|---|---|---|
| CANN版本 | >=5.0.4 | 请参考CANN样例仓介绍中的[安装步骤](https://github.com/Ascend/samples#%E5%AE%89%E8%A3%85)完成CANN安装，如果CANN低于要求版本请根据[版本说明](https://github.com/Ascend/samples/blob/master/README_CN.md#%E7%89%88%E6%9C%AC%E8%AF%B4%E6%98%8E)切换samples仓到对应CANN版本 |
| 硬件要求 | Atlas200DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))  | 当前已在Atlas200DK和Atlas300测试通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) ，其他产品可能需要另做适配|
| 第三方依赖 | opencv | 请参考[第三方依赖安装指导(C++样例)](../../../environment)完成对应安装 |


## 准备模型和图片<a name="section1593012514400"></a>

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

2. 模型转换。

    将ResNet-50原始模型转换为适配昇腾310处理器的离线模型（\*.om文件）。

    ```
    # 为了方便下载，在这里直接给出原始模型下载及模型转换命令,可以直接拷贝执行。
    cd $HOME/samples/cplusplus/level2_simple_inference/1_classification/resnet50_imagenet_dynamic_dims/model/
    wget https://share123.obs.myhuaweicloud.com/AclBot/issue/resnet50_tensorflow.pb
    atc --model=resnet50_tensorflow.pb --framework=3 --output=resnet50 --soc_version=Ascend310 --input_format=ND --input_shape="Placeholder:-1,-1,-1,3" --dynamic_dims="1,200,200;2,200,200;1,224,224;2,224,224"
    ```

3.  准备测试图片。
    请从以下链接获取该样例的输入图片，放在“/data“目录下。如果目录不存在，需自行创建。
        
    ```    
    cd $HOME/samples/cplusplus/level2_simple_inference/1_classification/resnet50_imagenet_dynamic_dims/data/
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dog1_1024_683.jpg
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dog2_1024_683.jpg
    ```


## 编译运行<a name="section1593012514493"></a>

1.  编译代码。

    ```
    cd $HOME/samples/cplusplus/level2_simple_inference/1_classification/resnet50_imagenet_dynamic_dims/scripts/
    bash build.sh
    ```

2.  运行应用。

    ```
    bash run.sh
    ```

    执行成功后，在屏幕上的关键提示信息示例如下，提示信息中的index表示类别标识、value表示该分类的最大置信度，这些值可能会根据版本、环境有所不同，请以实际情况为准：

    ```
    [INFO] The sample starts to run
    [INFO]  acl init success
    [INFO]  set device 0 success
    [INFO]  create context success
    [INFO]  create stream success
    [INFO]  get run mode success
    [INFO]  load model ../model/resnet50.om success
    [INFO]  create model description success
    [INFO]  create model output success
    [INFO]  start to process file:../data/data_224_224
    [INFO]  ModelSetDynamicInfo g_batchSize:2, g_channels:3, modelHeight:224, modelWidth:224
    [INFO]  read pic file , file name is ../data/dataPic/dog2_1024_683.jpg
    [INFO]  read pic file , file name is ../data/dataPic/dog1_1024_683.jpg
    [INFO]  Read Pic File batchFileSize = 1204224
    [INFO]  create model input success
    [INFO]  model execute success
    [INFO]  destroy model input success
    [INFO]  seq = 0---------------------
    [INFO]  top 1: index[268] value[0.754271] cnt= 1
    [INFO]  top 2: index[267] value[0.082666] cnt= 2
    [INFO]  top 3: index[266] value[0.029132] cnt= 3
    [INFO]  top 4: index[569] value[0.003168] cnt= 4
    [INFO]  top 5: index[130] value[0.002907] cnt= 5
    [INFO]  seq = 1---------------------
    [INFO]  top 1: index[163] value[0.801407] cnt= 1
    [INFO]  top 2: index[162] value[0.029883] cnt= 2
    [INFO]  top 3: index[168] value[0.016311] cnt= 3
    [INFO]  top 4: index[167] value[0.015026] cnt= 4
    [INFO]  top 5: index[165] value[0.003327] cnt= 5
    [INFO]  output data success
    [INFO]  start to process file:../data/data_200_200
    [INFO]  ModelSetDynamicInfo g_batchSize:2, g_channels:3, modelHeight:200, modelWidth:200
    [INFO]  read pic file , file name is ../data/dataPic/dog2_1024_683.jpg
    [INFO]  read pic file , file name is ../data/dataPic/dog1_1024_683.jpg
    [INFO]  Read Pic File batchFileSize = 960000
    [INFO]  create model input success
    [INFO]  model execute success
    [INFO]  destroy model input success
    [INFO]  seq = 0---------------------
    [INFO]  top 1: index[268] value[0.695164] cnt= 1
    [INFO]  top 2: index[267] value[0.156329] cnt= 2
    [INFO]  top 3: index[266] value[0.040463] cnt= 3
    [INFO]  top 4: index[260] value[0.002435] cnt= 4
    [INFO]  top 5: index[152] value[0.001809] cnt= 5
    [INFO]  seq = 1---------------------
    [INFO]  top 1: index[163] value[0.884404] cnt= 1
    [INFO]  top 2: index[168] value[0.007416] cnt= 2
    [INFO]  top 3: index[162] value[0.007132] cnt= 3
    [INFO]  top 4: index[167] value[0.006570] cnt= 4
    [INFO]  top 5: index[165] value[0.002431] cnt= 5
    [INFO]  output data success
    [INFO]  destroy model output success
    [INFO]  execute sample success
    [INFO]  end to destroy stream
    [INFO]  end to destroy context
    [INFO]  end to reset device 0
    [INFO]  end to finalize acl

    ```



