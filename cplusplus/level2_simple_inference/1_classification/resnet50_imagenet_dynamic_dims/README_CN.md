# 基于Caffe ResNet-50网络实现图片分类<a name="ZH-CN_TOPIC_0302603648"></a>

## 功能描述<a name="section340311417417"></a>

该样例主要是基于Caffe ResNet-50网络（动态dims）实现图片分类的功能。

在该样例中：

1.  先使用样例提供的脚本transfer_pic.py，将2张\*.jpg图片都转换为\*.bin格式，同时将图片从1024\*683的分辨率缩放为224\*224,200\*200。

2.  加载离线模型om文件，对2张图片进行同步推理，分别得到推理结果，再对推理结果进行处理，输出top5置信度的类别标识。

    在加载离线模型前，提前将Caffe ResNet-50网络的模型文件转换为适配昇腾AI处理器的离线模型。


## 原理介绍<a name="section3558105154116"></a>

在该Sample中，涉及的关键功能点，如下所示：

-   **初始化**
    -   调用aclInit接口初始化AscendCL配置。
    -   调用aclFinalize接口实现AscendCL去初始化。

-   **Device管理**
    -   调用aclrtSetDevice接口指定用于运算的Device。
    -   调用aclrtGetRunMode接口获取昇腾AI软件栈的运行模式，根据运行模式的不同，内部处理流程不同。
    -   调用aclrtResetDevice接口复位当前运算的Device，回收Device上的资源。

-   **Context管理**
    -   调用aclrtCreateContext接口创建Context。
    -   调用aclrtDestroyContext接口销毁Context。

-   **Stream管理**
    -   调用aclrtCreateStream接口创建Stream。
    -   调用aclrtDestroyStream接口销毁Stream。

-   **内存管理**
    -   调用aclrtMalloc接口申请Device上的内存。
    -   调用aclrtFree接口释放Device上的内存。

-   **数据传输**

    调用aclrtMemcpy接口通过内存复制的方式实现数据传输。

-   **模型推理**
    -   调用aclmdlLoadFromFileWithMem接口从\*.om文件加载模型。
    -   调用aclmdlExecute接口执行模型推理，同步接口。
    -   调用aclmdlUnload接口卸载模型。

-   **数据后处理**

    提供样例代码，处理模型推理的结果，直接在终端上显示top5置信度的类别编号。

    另外，样例中提供了自定义接口DumpModelOutputResult，用于将模型推理的结果写入文件（运行可执行文件后，推理结果文件在运行环境上的应用可执行文件的同级目录下），默认未调用该接口，用户可在sample\_process.cpp中，在调用OutputModelResult接口前，增加如下代码调用DumpModelOutputResult接口：

    ```
    modelProcess.DumpModelOutputResult();
    modelProcess.OutputModelResult();
    ```


## 目录结构<a name="section14723181815424"></a>

样例代码结构如下所示。

```
├── data
│   ├── dog1_1024_683.jpg            //测试数据,需要按指导获取测试图片，放到data目录下
│   ├── dog2_1024_683.jpg            //测试数据,需要按指导获取测试图片，放到data目录下
│   ├── data_200_200
│   |   ├── dog1_200_200.bin
│   |   ├── dog2_200_200.bin
│   ├── data_224_224
│   |   ├── dog1_224_224.bin
│   |   ├── dog2_224_224.bin

├── inc
│   ├── model_process.h               //声明模型处理相关函数的头文件
│   ├── sample_process.h              //声明资源初始化/销毁相关函数的头文件                   
│   ├── utils.h                       //声明公共函数（例如：文件读取函数）的头文件

├── scripts
│   ├── transfer_pic.py               //将*.jpg转换为*.bin，同时将图片从1024*683的分辨率缩放为224*224和200*200
|   ├── build.sh                     //编译脚本
|   ├── run.sh                       //运行脚本

├── src
│   ├── acl.json         //系统初始化的配置文件
│   ├── CMakeLists.txt         //编译脚本
│   ├── main.cpp               //主函数，图片分类功能的实现文件
│   ├── model_process.cpp      //模型处理相关函数的实现文件
│   ├── sample_process.cpp     //资源初始化/销毁相关函数的实现文件                                          
│   ├── utils.cpp              //公共函数（例如：文件读取函数）的实现文件

├── .project     //工程信息文件，包含工程类型、工程描述、运行目标设备类型等
├── CMakeLists.txt    //编译脚本，调用src目录下的CMakeLists文件
```

## 环境要求<a name="section3833348101215"></a>

请检查以下条件要求是否满足，如不满足请按照备注进行相应处理。如果CANN版本升级，请同步检查第三方依赖是否需要重新安装（5.0.4及以上版本第三方依赖和5.0.4以下版本有差异，需要重新安装）。
| 条件 | 要求 | 备注 |
|---|---|---|
| CANN版本 | >=5.0.4 | 请参考CANN样例仓介绍中的[安装步骤](https://github.com/Ascend/samples#%E5%AE%89%E8%A3%85)完成CANN安装，如果CANN低于要求版本请根据[版本说明](https://github.com/Ascend/samples/blob/master/README_CN.md#%E7%89%88%E6%9C%AC%E8%AF%B4%E6%98%8E)切换samples仓到对应CANN版本 |
| 硬件要求 | Atlas200DK/Atlas300([ai1s](https://support.huaweicloud.com/productdesc-ecs/ecs_01_0047.html#ecs_01_0047__section78423209366))  | 当前已在Atlas200DK和Atlas300测试通过，产品说明请参考[硬件平台](https://ascend.huawei.com/zh/#/hardware/product) ，其他产品可能需要另做适配|
| 第三方依赖 | opencv | 请参考[第三方依赖安装指导(C++样例)](../../../environment)完成对应安装 |


## 准备模型和图片<a name="section1593012514400"></a>

1.  下载sample仓代码并上传至环境后，请先进入“cplusplus/level2_simple_inference/1_classification/resnet50_imagenet_dynamic_dims”样例目录。
     
    请注意，下文中的样例目录均指“cplusplus/level2_simple_inference/1_classification/resnet50_imagenet_dynamic_dims”目录。

2.  准备ResNet-50模型。

    将ResNet-50原始模型转换为适配昇腾AI处理器的离线模型（\*.om文件）。

    切换到样例目录/model，执行如下命令(以昇腾310 AI处理器为例)：

    ```
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet50/resnet50.prototxt
    wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet50/resnet50.caffemodel
    atc --model=resnet50.prototxt --weight=resnet50.caffemodel --framework=0 --output=resnet50 --soc_version=Ascend310 --input_format=ND --input_shape="data:-1,3,-1,-1" --dynamic_dims="1,200,200;2,200,200;1,224,224;2,224,224" --input_fp16_nodes=data --output_type=FP32 --out_nodes=prob:0
    ```

3.  准备测试图片。
    1.  请从以下链接获取该样例的输入图片，并以运行用户将获取的文件上传至开发环境的“样例目录/data“目录下。如果目录不存在，需自行创建。
        
        ```    
        wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dog1_1024_683.jpg
        wget https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dog2_1024_683.jpg
        ```

    2.  切换到“样例目录/data“目录下，执行transfer_pic.py脚本，将\*.jpg转换为\*.bin，同时将图片从1024\*683的分辨率缩放为224\*224和200\*200。在“样例目录/data/data_224_224和样例目录/data/data_200_200“目录下分别生成2个\*.bin文件。

        ```
        python3.6 ../scripts/transfer_pic.py
        ```

        如果执行脚本报错“ModuleNotFoundError: No module named 'PIL'”，则表示缺少Pillow库，请使用**pip3 install Pillow --user**命令安装Pillow库。


## 编译运行<a name="section1593012514493"></a>

1.  编译代码。
    1.  以运行用户登录运行环境。

    2.  请先进入“cplusplus/level2_simple_inference/1_classification/resnet50_imagenet_dynamic_dims”样例目录。

        请注意，下文中的样例目录均指“cplusplus/level2_simple_inference/1_classification/resnet50_imagenet_dynamic_dims”目录。

    3.  编译：切换到样例目录“/scripts”目录下,执行编译。

        ```
        bash build.sh
        ```

2.  运行应用。

    1.  运行：切换到样例目录“/scripts”目录下,运行。

        ```
        bash run.sh
        ```

        执行成功后，在屏幕上的关键提示信息示例如下，提示信息中的index表示类别标识、value表示该分类的最大置信度，这些值可能会根据版本、环境有所不同，请以实际情况为准：

        ```
        [INFO]  acl init success
        [INFO]  set device 0 success
        [INFO]  create context success
        [INFO]  create stream success
        [INFO]  get run mode success
        [INFO]  load model ../model/resnet50.om success
        [INFO]  create model description success
        [INFO]  create model output success
        [INFO]  start to process file:../data/data_224_224
        [INFO]  ModelSetDynamicInfo batchSize:2, channels:3, modelHeight:224, modelWidth:224
        [INFO]  read bin file Num =2.
        [INFO]  ReadBinFile batchFileSize =602112
        [INFO]  create model input success
        [INFO]  model execute success
        [INFO]  destroy model input success
        [INFO]  seq = 0---------------------
        [INFO]  top 1: index[267] value[0.936035] cnt= 1
        [INFO]  top 2: index[266] value[0.041138] cnt= 2
        [INFO]  top 3: index[265] value[0.018845] cnt= 3
        [INFO]  top 4: index[219] value[0.002609] cnt= 4
        [INFO]  top 5: index[160] value[0.000295] cnt= 5
        [INFO]  seq = 1---------------------
        [INFO]  top 1: index[161] value[0.767578] cnt= 1
        [INFO]  top 2: index[162] value[0.154785] cnt= 2
        [INFO]  top 3: index[167] value[0.038513] cnt= 3
        [INFO]  top 4: index[163] value[0.021606] cnt= 4
        [INFO]  top 5: index[166] value[0.011658] cnt= 5
        [INFO]  output data success
        [INFO]  start to process file:../data/data_200_200
        [INFO]  ModelSetDynamicInfo batchSize:2, channels:3, modelHeight:200, modelWidth:200
        [INFO]  read bin file Num =2.
        [INFO]  ReadBinFile batchFileSize =480000
        [INFO]  create model input success
        [INFO]  model execute success
        [INFO]  destroy model input success
        [INFO]  seq = 0---------------------
        [INFO]  top 1: index[267] value[0.853027] cnt= 1
        [INFO]  top 2: index[266] value[0.103516] cnt= 2
        [INFO]  top 3: index[265] value[0.036926] cnt= 3
        [INFO]  top 4: index[219] value[0.001499] cnt= 4
        [INFO]  top 5: index[160] value[0.000793] cnt= 5
        [INFO]  seq = 1---------------------
        [INFO]  top 1: index[161] value[0.591797] cnt= 1
        [INFO]  top 2: index[162] value[0.286133] cnt= 2
        [INFO]  top 3: index[167] value[0.063354] cnt= 3
        [INFO]  top 4: index[166] value[0.047455] cnt= 4
        [INFO]  top 5: index[163] value[0.008438] cnt= 5
        [INFO]  output data success
        [INFO]  destroy model output success
        [INFO]  unload model success, modelId is 1
        [INFO]  destroy model description success
        [INFO]  execute sample success
        [INFO]  end to destroy stream
        [INFO]  end to destroy context
        [INFO]  end to reset device 0
        [INFO]  end to finalize acl
        ```



