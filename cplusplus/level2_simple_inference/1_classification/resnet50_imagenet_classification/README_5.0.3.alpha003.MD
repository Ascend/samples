# 基于Caffe ResNet-50网络实现图片分类（同步推理）<a name="ZH-CN_TOPIC_0302603648"></a>

## 功能描述<a name="section340311417417"></a>

该样例主要是基于Caffe ResNet-50网络（单输入、单Batch）实现图片分类的功能。

在该样例中：

1.  先使用样例提供的脚本transferPic.py，将2张\*.jpg图片都转换为\*.bin格式，同时将图片从1024\*683的分辨率缩放为224\*224。
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
    // print the top 5 confidence values with indexes.use function DumpModelOutputResult
    // if want to dump output result to file in the current directory
    processModel.DumpModelOutputResult();
    processModel.OutputModelResult();
    ```


## 目录结构<a name="section14723181815424"></a>

样例代码结构如下所示。

```
├── data
│   ├── dog1_1024_683.jpg            //测试数据,需要按指导获取测试图片，放到data目录下
│   ├── dog2_1024_683.jpg            //测试数据,需要按指导获取测试图片，放到data目录下

├── inc
│   ├── model_process.h               //声明模型处理相关函数的头文件
│   ├── sample_process.h              //声明资源初始化/销毁相关函数的头文件                   
│   ├── utils.h                       //声明公共函数（例如：文件读取函数）的头文件

├── script
│   ├── transferPic.py               //将*.jpg转换为*.bin，同时将图片从1024*683的分辨率缩放为224*224

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

-   操作系统及架构：CentOS 7.6 x86\_64、CentOS aarch64、Ubuntu 18.04 x86\_64、EulerOS x86、EulerOS aarch64
-   编译器：
    -   Ascend 310 EP/Ascend 710/Ascend 910形态编译器：g++ 或 aarch64-linux-gnu-g++

    -   Atlas 200 DK编译器：aarch64-linux-gnu-g++

-   芯片：Ascend 310、Ascend 710、Ascend 910

-   python及依赖的库：python3.7.5、Pillow库
-   已在环境上部署昇腾AI软件栈。


## 获取样例<a name="section3833348101216"></a>

单击[Link](https://github.com/Ascend/samples/tree/master/cplusplus/level2_simple_inference/1_classification/resnet50_imagenet_classification)下载样例包后，解压缩样例包，从“cplusplus/level2_simple_inference/1_classification/resnet50_imagenet_classification”目录下获取本样例的代码。


## 配置环境变量<a name="section1313218510108"></a>

1.  安装CANN软件后，使用CANN运行用户编译、运行应用前，需以运行用户登录环境，设置公共环境变量。
    如下为设置环境变量的命令，其中，$\{install\_path\}表示CANN软件的安装目录，以root用户安装CANN软件时，默认安装目录为“/usr/local/Ascend”;以普通用户安装CANN软件时，默认安装目录为“普通用户家目录/Ascend”。
    ```
    source ${install_path}/bin/setenv.bash
    ```

2.  开发环境上，以运行用户登录环境，设置模型转换依赖的环境变量。
    如下为设置环境变量的命令，其中，Python3.7.5安装路径请根据实际情况替换。
     ```
     export LD_LIBRARY_PATH=/usr/local/python3.7.5/lib:$LD_LIBRARY_PATH
     export PATH=/usr/local/python3.7.5/bin:$PATH
     ```


3.  开发环境上，以运行用户登录环境，设置编译依赖的环境变量，编译脚本src/CMakeLists.txt通过环境变量所设置的头文件、库文件的路径来编译代码。
    如下为设置环境变量的命令，其中，$\{install\_path\}表示CANN软件的安装目录。
     ```
     export DDK_PATH=${install_path}
     export NPU_HOST_LIB=${install_path}/acllib/lib64/stub
     ```

    使用“lib64/stub”目录下的\*.so库，是为了编译基于AscendCL接口的代码逻辑时，不依赖其它组件（例如Driver）的任何\*.so库。编译通过后，运行应用时，会根据环境变量LD\_LIBRARY\_PATH链接到“acllib/lib64“目录下的\*.so库，并自动链接到依赖其它组件的\*.so库。
    


## 编译运行<a name="section1593012514493"></a>

1.  模型转换。
    1.  以运行用户登录开发环境。

    2.  准备数据。

        您可以从以下链接中获取ResNet-50网络的模型文件（\*.prototxt）、预训练模型文件（\*.caffemodel），并以运行用户将获取的文件上传至开发环境的“样例目录/caffe\_model“目录下。如果目录不存在，需要自行创建。

        -   从gitee上获取：单击[Link](https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/resnet50/ATC_resnet50_caffe_AE)，查看README.md，查找获取原始模型的链接。
        -   从GitHub上获取：单击[Link](https://github.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/resnet50/ATC_resnet50_caffe_AE)，查看README.md，查找获取原始模型的链接。 

    3.  将ResNet-50网络转换为适配昇腾AI处理器的离线模型（\*.om文件）。

        切换到样例目录，执行如下命令（以Ascend310芯片为例）：

        ```
        atc --model=caffe_model/resnet50.prototxt --weight=caffe_model/resnet50.caffemodel --framework=0 --output=model/resnet50 --soc_version=Ascend310 --input_format=NCHW --input_fp16_nodes=data -output_type=FP32 --out_nodes=prob:0
        ```

        -   --model：原始模型文件路径。
        -   --weight：权重文件路径。
        -   --framework：原始框架类型。0：表示Caffe；1：表示MindSpore；3：表示TensorFlow；5：表示ONNX。
        -   --soc\_version：
            -   Ascend 310芯片，此处配置为Ascend310。
            -   Ascend 710芯片，此处配置为Ascend710。
            -   Ascend 910芯片，此处配置为Ascend910A或Ascend910B或Ascend910ProA或Ascend910ProB或Ascend910PremiumA，其中，Pro或Premium表示芯片性能提升等级、A或B表示PartialGood等级，请根据实际情况选择。

        -   --input\_format：输入数据的Format。
        -   --input\_fp16\_nodes：指定输入数据类型为FP16的输入节点名称。
        -   --output\_type和--out\_nodes：这2个参数配合使用，指定prob节点的第一个输出的数据类型为float32。
        -   --output：生成的resnet50.om文件存放在“样例目录/model“目录下。建议使用命令中的默认设置，否则在编译代码前，您还需要修改sample\_process.cpp中的omModelPath参数值。

            ```
            const char* omModelPath = "../model/resnet50.om";
            ```



2.  编译代码。
    1.  切换到样例目录，创建目录用于存放编译文件，例如，本文中，创建的目录为“build/intermediates/host“。

        ```
        mkdir -p build/intermediates/host
        ```

    2.  切换到“build/intermediates/host“目录，执行**cmake**生成编译文件。

        “../../../src“表示CMakeLists.txt文件所在的目录，请根据实际目录层级修改。

        -   当开发环境与运行环境操作系统架构相同时，执行如下命令编译。

            ```
            cd build/intermediates/host
            cmake ../../../src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE
            ```

        -   当开发环境与运行环境操作系统架构不同时，执行以下命令进行交叉编译。

            例如，当开发环境为X86架构，运行环境为AArch64架构时，执行以下命令进行交叉编译。
            ```
            cd build/intermediates/host
            cmake ../../../src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE
            ```


    3.  执行**make**命令，生成的可执行文件main在“样例目录/out“目录下。

        ```
        make
        ```


3.  准备输入图片。
    1.  请从以下链接获取该样例的输入图片，并以运行用户将获取的文件上传至开发环境的“样例目录/data“目录下。如果目录不存在，需自行创建。

        [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dog1\_1024\_683.jpg](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dog1_1024_683.jpg)

        [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dog2\_1024\_683.jpg](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dog2_1024_683.jpg)

    2.  以运行用户登录开发环境。
    3.  切换到“样例目录/data“目录下，执行transferPic.py脚本，将\*.jpg转换为\*.bin，同时将图片从1024\*683的分辨率缩放为224\*224。在“样例目录/data“目录下生成2个\*.bin文件。

        ```
        python3.7.5 ../script/transferPic.py
        ```

        如果执行脚本报错“ModuleNotFoundError: No module named 'PIL'”，则表示缺少Pillow库，请使用**pip3.7.5 install Pillow --user**命令安装Pillow库。


4.  运行应用。
    1.  以运行用户将开发环境的样例目录及目录下的文件上传到运行环境（Host），例如“$HOME/acl\_resnet50”。
    2.  以运行用户登录运行环境（Host）。
    3.  切换到可执行文件main所在的目录，例如“$HOME/acl\_resnet50/out”，给该目录下的main文件加执行权限。

        ```
        chmod +x main
        ```

    4.  切换到可执行文件main所在的目录，例如“$HOME/acl\_resnet50/out”，运行可执行文件。

        ```
        ./main
        ```

        执行成功后，在屏幕上的关键提示信息示例如下，提示信息中的index表示类别标识、value表示该分类的最大置信度，这些值可能会根据版本、环境有所不同，请以实际情况为准：

        ```
        [INFO] acl init success
        [INFO] open device 0 success
        [INFO] create context success
        [INFO] create stream success
        [INFO] load model ../model/resnet50.om success
        [INFO] create model description success
        [INFO] create model output success
        [INFO] start to process file:../data/dog1_1024_683.bin
        [INFO] model execute success
        [INFO] top 1: index[161] value[xxxxxx]
        [INFO] top 2: index[xxx] value[xxxxxx]
        [INFO] top 3: index[xxx] value[xxxxxx]
        [INFO] top 4: index[xxx] value[xxxxxx]
        [INFO] top 5: index[xxx] value[xxxxxx]
        [INFO] output data success
        [INFO] start to process file:../data/dog2_1024_683.bin
        [INFO] model execute success
        [INFO] top 1: index[267] value[xxxxxx]
        [INFO] top 2: index[xxx] value[xxxxxx]
        [INFO] top 3: index[xxx] value[xxxxxx]
        [INFO] top 4: index[xxx] value[xxxxxx]
        [INFO] top 5: index[xxx] value[xxxxxx]
        [INFO] output data success
        [INFO] Unload model success, modelId is 1
        [INFO] execute sample success
        [INFO] end to destroy stream 
        [INFO] end to destroy context
        [INFO] end to reset device is 0
        ```

