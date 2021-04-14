# 基于Caffe ResNet-50网络实现图片分类（异步推理）<a name="ZH-CN_TOPIC_0302603656"></a>

## 功能描述<a name="section340311417417"></a>

该样例主要是基于Caffe ResNet-50网络（单输入、单Batch）实现图片分类的功能。

在该样例中：

1.  先使用样例提供的脚本transferPic.py，将2张\*.jpg图片都转换为\*.bin格式，同时将图片从1024\*683的分辨率缩放为224\*224。
2.  加载离线模型om文件，对2张\*.jpg图片进行n次异步推理（n作为运行应用的参数，由用户配置），分别得到n次推理结果后，再对推理结果进行处理，输出top1置信度的类别标识。

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
    -   调用aclrtSetCurrentContext接口设置线程的Context。
    -   调用aclrtDestroyContext接口销毁Context。

-   **Stream管理**
    -   调用aclrtCreateStream接口创建Stream。
    -   调用aclrtDestroyStream接口销毁Stream。

-   **内存管理**
    -   调用aclrtMallocHost接口申请Host上内存。
    -   调用aclrtFreeHost释放Host上的内存。
    -   调用aclrtMalloc接口申请Device上的内存。
    -   调用aclrtFree接口释放Device上的内存。

-   **数据传输**

    调用aclrtMemcpy接口通过内存复制的方式实现数据传输。

-   **模型推理**
    -   调用aclmdlLoadFromFileWithMem接口从\*.om文件加载模型。
    -   创建新线程（例如t1），在线程函数内调用aclrtProcessReport接口，等待指定时间后，触发回调函数（例如CallBackFunc，用于处理模型推理结果）。
    -   调用aclrtSubscribeReport接口，指定处理Stream上回调函数（CallBackFunc）的线程（t1）。
    -   调用aclmdlExecuteAsync接口执行模型推理，异步接口。
    -   调用aclrtLaunchCallback接口，在Stream的任务队列中增加一个需要在Host/Device上执行的回调函数（CallBackFunc）。
    -   调用aclrtSynchronizeStream接口，阻塞应用程序运行，直到指定Stream中的所有任务都完成。
    -   调用aclrtUnSubscribeReport接口，取消线程注册，Stream上的回调函数（CallBackFunc）不再由指定线程（t1）处理。
    -   模型推理结束后，调用aclmdlUnload接口卸载模型。

-   **数据后处理**

    提供样例代码，处理模型推理的结果，直接在终端上显示top1置信度的类别编号。

    另外，样例中提供了自定义接口DumpModelOutputResult，用于将模型推理的结果写入文件（运行可执行文件后，推理结果文件在运行环境上的应用可执行文件的同级目录下），默认未调用该接口，用户可在sample\_process.cpp中，在调用OutputModelResult接口前，增加如下代码调用DumpModelOutputResult接口：

    ```
    // print the top 1 confidence values with indexes.use function DumpModelOutputResult
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
│   ├── memory_pool.h                 //声明内存池处理相关函数的头文件
│   ├── model_process.h               //声明模型处理相关函数的头文件
│   ├── sample_process.h              //声明资源初始化/销毁相关函数的头文件                   
│   ├── utils.h                       //声明公共函数（例如：文件读取函数）的头文件

├── script
│   ├── transferPic.py               //将*.jpg转换为*.bin，同时将图片从1024*683的分辨率缩放为224*224

├── src
│   ├── acl.json         //系统初始化的配置文件
│   ├── CMakeLists.txt         //编译脚本
│   ├── main.cpp               //主函数，图片分类功能的实现文件
│   ├── memory_pool.cpp        //内存池处理相关函数的实现文件
│   ├── model_process.cpp      //模型处理相关函数的实现文件
│   ├── sample_process.cpp     //资源初始化/销毁相关函数的实现文件                                          
│   ├── utils.cpp              //公共函数（例如：文件读取函数）的实现文件

├── .project     //工程信息文件，包含工程类型、工程描述、运行目标设备类型等
├── CMakeLists.txt    //编译脚本，调用src目录下的CMakeLists文件
```

## 环境要求<a name="section3833348101215"></a>

-   操作系统及架构：CentOS 7.6 x86\_64、CentOS aarch64、Ubuntu 18.04 x86\_64、EulerOS x86、EulerOS aarch64
-   版本：3.3.0
-   编译器：
    -   Ascend 310 EP/Ascend 710/Ascend 910形态编译器：
        -   运行环境操作系统架构为x86时，编译器为g++
        -   运行环境操作系统架构为Arm时，编译器为aarch64-linux-gnu-g++

    -   Atlas 200 DK编译器：aarch64-linux-gnu-g++

-   芯片：Ascend 310、Ascend 710、Ascend 910

-   python及依赖的库：python3.7.5、Pillow库
-   已在环境上部署昇腾AI软件栈。

## 配置环境变量<a name="section2576153161110"></a>

-   **Ascend 310 EP/Ascend 910：**
    1.  开发环境上，设置模型转换依赖的环境变量。

        $\{install\_path\}表示开发套件包Ascend-cann-toolkit所在的路径。

        ```
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        ```

    2.  开发环境上，设置环境变量，编译脚本src/CMakeLists.txt通过环境变量所设置的头文件、库文件的路径来编译代码。

        如下为设置环境变量的示例，请将$HOME/Ascend/ascend-toolkit/latest/_\{os\_arch\}_替换为开发套件包Ascend-cann-toolkit下对应架构的FwkACLlib的路径。

        -   当运行环境操作系统架构为x86时，执行以下命令：

            ```
            export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux
            export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux/fwkacllib/lib64/stub
            ```

        -   当运行环境操作系统架构为Arm时，执行以下命令：

            ```
            export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
            export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/arm64-linux/fwkacllib/lib64/stub
            ```


        使用“$HOME/Ascend/ascend-toolkit/latest/_\{os\_arch\}_/fwkacllib/lib64/stub”目录下的\*.so库，是为了编译基于AscendCL接口的代码逻辑时，不依赖其它组件（例如Driver）的任何\*.so库。编译通过后，在Host上运行应用时，会根据环境变量LD\_LIBRARY\_PATH链接到“fwkacllib/lib64“或“acllib/lib64“目录下的\*.so库，并自动链接到依赖其它组件的\*.so库。

        设置环境变量后，还需修改src/CMakeLists.txt文件中的如下配置段，将“**acllib**”修改为“**fwkacllib**”。

        ```
        # Header path
        include_directories(
            ${INC_PATH}/acllib/include/
            ../inc/
        )
        ```

    3.  运行环境上，设置环境变量，运行应用时需要根据环境变量找到对应的库文件。
        -   若运行环境上安装的是开发套件包Ascend-cann-toolkit，环境变量设置如下：

            如下为设置环境变量的示例，请将$HOME/Ascend/ascend-toolkit/latest替换为FwkACLlib的路径。

            ```
            export LD_LIBRARY_PATH=$HOME/Ascend/ascend-toolkit/latest/fwkacllib/lib64
            ```

        -   若运行环境上安装的是Ascend-cann-nnrt包，环境变量设置如下：

            如下为设置环境变量的示例，请将$HOME/Ascend/nnrt/latest替换为ACLlib的路径。

            ```
            export LD_LIBRARY_PATH=$HOME/Ascend/nnrt/latest/acllib/lib64
            ```



-   **Ascend 710：**
    1.  开发环境上，设置模型转换依赖的环境变量。

        $\{install\_path\}表示开发套件包Ascend-cann-toolkit所在的路径。

        ```
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        ```

    2.  开发环境上，设置环境变量，编译脚本src/CMakeLists.txt通过环境变量所设置的头文件、库文件的路径来编译代码。

        如下为设置环境变量的示例，请将$HOME/Ascend/ascend-toolkit/latest/_\{os\_arch\}_替换为开发套件包Ascend-cann-toolkit下对应架构的ACLlib的路径。

        -   当运行环境操作系统架构为x86时，执行以下命令：

            ```
            export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux
            export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux/acllib/lib64/stub
            ```

        -   当运行环境操作系统架构为Arm时，执行以下命令：

            ```
            export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
            export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/arm64-linux/acllib/lib64/stub
            ```


        使用“$HOME/Ascend/ascend-toolkit/latest/_\{os\_arch\}_/acllib/lib64/stub”目录下的\*.so库，是为了编译基于AscendCL接口的代码逻辑时，不依赖其它组件（例如Driver）的任何\*.so库。编译通过后，在Host上运行应用时，会根据环境变量LD\_LIBRARY\_PATH链接到“$HOME/Ascend/nnrt/latest/acllib/lib64”目录下的\*.so库，并自动链接到依赖其它组件的\*.so库。

    3.  运行环境上，设置环境变量，运行应用时需要根据环境变量找到对应的库文件。

        如下为设置环境变量的示例，请将$HOME/Ascend/nnrt/latest替换为ACLlib的路径。

        ```
        export LD_LIBRARY_PATH=$HOME/Ascend/nnrt/latest/acllib/lib64
        ```


-   **Atlas 200 DK：**

    仅需在开发环境上设置环境变量，运行环境上的环境变量在制卡时已配置，此处无需单独配置。

    1.  开发环境上，设置模型转换依赖的环境变量。

        $\{install\_path\}表示开发套件包Ascend-cann-toolkit所在的路径。

        ```
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        ```

    2.  开发环境上，设置环境变量，编译脚本src/CMakeLists.txt通过环境变量所设置的头文件、库文件的路径来编译代码。

        如下为设置环境变量的示例，请将$HOME/Ascend/ascend-toolkit/latest/arm64-linux替换为开发套件包Ascend-cann-toolkit下Arm架构的ACLlib的路径。

        ```
        export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
        export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/arm64-linux/acllib/lib64/stub
        ```

        使用“$HOME/Ascend/ascend-toolkit/latest/arm64-linux/acllib/lib64/stub”目录下的\*.so库，是为了编译基于AscendCL接口的代码逻辑时，不依赖其它组件（例如Driver）的任何\*.so库。编译通过后，在板端环境上运行应用时，会根据环境变量LD\_LIBRARY\_PATH链接到“$HOME/Ascend/acllib/lib64”目录下的\*.so库，并自动链接到依赖其它组件的\*.so库。



## 编译运行（Ascend 310 EP/Ascend 710/Ascend 910）<a name="section7572134019439"></a>

1.  模型转换。
    1.  以运行用户登录开发环境。
    2.  设置环境变量。

        $\{install\_path\}表示开发套件包Ascend-cann-toolkit的安装路径。

        ```
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        ```

    3.  准备数据。

        您可以从以下链接中获取ResNet-50网络的模型文件（\*.prototxt）、预训练模型文件（\*.caffemodel），并以运行用户将获取的文件上传至开发环境的“样例目录/caffe\_model“目录下。如果目录不存在，需要自行创建。

        -   从gitee上获取：单击[Link](https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/resnet50/ATC_resnet50_caffe_AE)，查看README.md，查找获取原始模型的链接。
        -   从GitHub上获取：单击[Link](https://github.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/resnet50/ATC_resnet50_caffe_AE)，查看README.md，查找获取原始模型的链接。 

    4.  将ResNet-50网络转换为适配昇腾AI处理器的离线模型（\*.om文件）。

        切换到样例目录，执行如下命令：

        ```
        atc --model=caffe_model/resnet50.prototxt --weight=caffe_model/resnet50.caffemodel --framework=0 --output=model/resnet50 --soc_version=${soc_version} --input_format=NCHW --input_fp16_nodes=data -output_type=FP32 --out_nodes=prob:0
        ```

        -   --model：原始模型文件路径。
        -   --weight：权重文件路径。
        -   --framework：原始框架类型。0：表示Caffe；1：表示MindSpore；3：表示TensorFlow；5：表示ONNX。
        -   --soc\_version：
            -   Ascend 310芯片，此处配置为Ascend310。
            -   Ascend 710芯片，此处配置为Ascend710。
            -   Ascend 910芯片，此处配置为Ascend910A或Ascend910B或Ascend910ProA或Ascend910ProB或Ascend910PremiumA，其中，Pro或Premium表示芯片性能提升等级、A或B表示PartialGood等级，请根据实际情况选择。

        -   --input\_format：模型输入数据的Format。
        -   --input\_fp16\_nodes：指定输入数据类型为FP16的输入节点名称。
        -   --output\_type和--out\_nodes：这2个参数配合使用，指定prob节点的第一个输出的数据类型为float32。
        -   --output：生成的resnet50.om文件存放在“样例目录/model“目录下。建议使用命令中的默认设置，否则在编译代码前，您还需要修改sample\_process.cpp中的omModelPath参数值。

            ```
            const char* omModelPath = "../model/resnet50.om";
            ```



2.  编译代码。
    1.  以运行用户登录开发环境。
    2.  切换到样例目录，创建目录用于存放编译文件，例如，本文中，创建的目录为“build/intermediates/host“。

        ```
        mkdir -p build/intermediates/host
        ```

    3.  切换到“build/intermediates/host“目录，执行**cmake**生成编译文件。

        “../../../src“表示CMakeLists.txt文件所在的目录，请根据实际目录层级修改。

        -   当运行环境操作系统架构为x86时，执行如下命令编译。

            ```
            cd build/intermediates/host
            cmake ../../../src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE
            ```

        -   当运行环境操作系统架构为Arm时，执行以下命令进行交叉编译。

            ```
            cd build/intermediates/host
            cmake ../../../src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE
            ```


    4.  执行**make**命令，生成的可执行文件main在“样例目录/out“目录下。

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
    1.  以运行用户将开发环境的样例目录及目录下的文件上传到运行环境（Host），例如“$HOME/acl\_resnet50\_async”。
    2.  以运行用户登录运行环境（Host）。
    3.  切换到可执行文件main所在的目录，例如“$HOME/acl\_resnet50\_async/out”，给该目录下的main文件加执行权限。

        ```
        chmod +x main
        ```

    4.  切换到可执行文件main所在的目录，例如“$HOME/acl\_resnet50\_async/out”，运行可执行文件。

        -   运行可执行文件，不带参数时：
            -   执行模型异步推理的次数默认为100次；
            -   callback间隔默认为1，表示1次异步推理后，下发一次callback任务；
            -   内存池中的内存块的个数默认为100个。

        -   运行可执行文件，带参数时：
            -   第一个参数表示执行模型异步推理的次数；
            -   第二个参数表示下发callback间隔，参数值为0时表示不下发callback任务，参数值为非0值（例如m）时表示m次异步推理后下发一次callback任务；
            -   第三个参数表示内存池中内存块的个数，内存块个数需大于等于模型异步推理的次数。


        ```
        ./main
        ```

        执行成功后，在屏幕上的关键提示信息示例如下：

        ```
        [INFO]  ./main param1 param2 param3, param1 is execute model times(default 100), param2 is callback interval(default 1), param3 is memory pool size(default 100)
        [INFO]  execute times = 100
        [INFO]  callback interval = 1
        [INFO]  memory pool size = 100
        [INFO]  acl init success
        [INFO]  open device 0 success
        [INFO]  create context success
        [INFO]  create stream success
        [INFO]  get run mode success
        [INFO]  load model ../model/resnet50.om success
        [INFO]  create model description success
        [INFO]  init memory pool success
        [INFO]  subscribe report success
        [INFO]  top 1: index[267] value[0.889648]
        [INFO]  top 1: index[161] value[0.836914]
        [INFO]  top 1: index[267] value[0.889648]
        [INFO]  top 1: index[161] value[0.836914]
        [INFO]  top 1: index[161] value[0.836914]
        [INFO]  top 1: index[267] value[0.889648]
        [INFO]  top 1: index[161] value[0.836914]
        [INFO]  top 1: index[267] value[0.889648]
        [INFO]  top 1: index[161] value[0.836914]
        [INFO]  top 1: index[267] value[0.889648]
        ......
        [INFO]  top 1: index[161] value[0.836914]
        [INFO]  top 1: index[267] value[0.889648]
        [INFO]  model execute success
        [INFO]  unsubscribe report success
        [INFO]  unload model success, modelId is 1
        [INFO]  execute sample success
        [INFO]  end to destroy stream
        [INFO]  end to destroy context
        [INFO]  end to reset device is 0
        [INFO]  end to finalize acl
        ```



## 编译运行（Atlas 200 DK）<a name="section112572102521"></a>

1.  模型转换。
    1.  以运行用户登录开发环境。
    2.  设置环境变量。

        $\{install\_path\}表示开发套件包Ascend-cann-toolkit的安装路径。

        ```
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        ```

    3.  准备数据。

        您可以从以下链接中获取ResNet-50网络的模型文件（\*.prototxt）、预训练模型文件（\*.caffemodel），并以运行用户将获取的文件上传至开发环境的“样例目录/caffe\_model“目录下。如果目录不存在，需要自行创建。

        -   从gitee上获取：单击[Link](https://github.com/Ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/resnet50/ATC_resnet50_caffe_AE)，查看README.md，查找获取原始模型的链接。
        -   从GitHub上获取：单击[Link](https://github.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/resnet50/ATC_resnet50_caffe_AE)，查看README.md，查找获取原始模型的链接。

    4.  将ResNet-50网络转换为适配昇腾AI处理器的离线模型（\*.om文件）。

        切换到样例目录，执行如下命令：

        ```
        atc --model=caffe_model/resnet50.prototxt --weight=caffe_model/resnet50.caffemodel --framework=0 --output=model/resnet50 --soc_version=${soc_version} --input_format=NCHW --input_fp16_nodes=data -output_type=FP32 --out_nodes=prob:0
        ```

        -   --model：原始模型文件路径。
        -   --weight：权重文件路径。
        -   --framework：原始框架类型。0：表示Caffe；1：表示MindSpore；3：表示TensorFlow；5：表示ONNX。
        -   --soc\_version：
            -   Ascend 310芯片，此处配置为Ascend310。
            -   Ascend 710芯片，此处配置为Ascend710。
            -   Ascend 910芯片，此处配置为Ascend910A或Ascend910B或Ascend910ProA或Ascend910ProB或Ascend910PremiumA，其中，Pro或Premium表示芯片性能提升等级、A或B表示PartialGood等级，请根据实际情况选择。

        -   --input\_format：模型输入数据的Format。
        -   --input\_fp16\_nodes：指定输入数据类型为FP16的输入节点名称。
        -   --output\_type和--out\_nodes：这2个参数配合使用，指定prob节点的第一个输出的数据类型为float32。
        -   --output：生成的resnet50.om文件存放在“样例目录/model“目录下。建议使用命令中的默认设置，否则在编译代码前，您还需要修改sample\_process.cpp中的omModelPath参数值。

            ```
            const char* omModelPath = "../model/resnet50.om";
            ```



2.  编译代码。
    1.  以运行用户登录开发环境。
    2.  切换到“acl\_resnet50\_async/data“目录下，执行transferPic.py脚本，将\*.jpg转换为\*.bin，同时将图片从1024\*683的分辨率缩放为224\*224。在“样例目录/data“目录下生成2个\*.bin文件。

        ```
        python3.7.5 ../script/transferPic.py
        ```

        如果执行脚本报错“ModuleNotFoundError: No module named 'PIL'”，则表示缺少Pillow库，请使用**pip3.7.5 install Pillow --user**命令安装Pillow库。

    3.  切换到样例目录，创建目录用于存放编译文件，例如，本文中，创建的目录为“build/intermediates/minirc“。

        ```
        mkdir -p build/intermediates/minirc
        ```

    4.  切换到“build/intermediates/minirc“目录，执行**cmake**生成编译文件。

        “../../../src“表示CMakeLists.txt文件所在的目录，请根据实际目录层级修改。

        ```
        cd build/intermediates/minirc
        cmake ../../../src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE
        ```

    5.  执行**make**命令，生成的可执行文件main在“样例目录/out“目录下。

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
    1.  以运行用户将开发环境的样例目录及目录下的文件上传到板端环境，例如“$HOME/acl\_resnet50\_async”。
    2.  以运行用户登录板端环境。
    3.  切换到可执行文件main所在的目录，例如“$HOME/acl\_resnet50\_async/out”，给该目录下的main文件加执行权限。

        ```
        chmod +x main
        ```

    4.  切换到可执行文件main所在的目录，例如“$HOME/acl\_resnet50\_async/out”，运行可执行文件。

        -   运行可执行文件，不带参数时：
            -   执行模型异步推理的次数默认为100次；
            -   callback间隔默认为1，表示1次异步推理后，下发一次callback任务；
            -   内存池中的内存块的个数默认为100个。

        -   运行可执行文件，带参数时：
            -   第一个参数表示执行模型异步推理的次数；
            -   第二个参数表示下发callback间隔，参数值为0时表示不下发callback任务，参数值为非0值（例如m）时表示m次异步推理后下发一次callback任务；
            -   第三个参数表示内存池中内存块的个数，内存块个数需大于等于模型异步推理的次数。


        ```
        ./main
        ```

        执行成功后，在屏幕上的关键提示信息示例如下：

        ```
        [INFO]  ./main param1 param2 param3, param1 is execute model times(default 100), param2 is callback interval(default 1), param3 is memory pool size(default 100)
        [INFO]  execute times = 100
        [INFO]  callback interval = 1
        [INFO]  memory pool size = 100
        [INFO]  acl init success
        [INFO]  open device 0 success
        [INFO]  create context success
        [INFO]  create stream success
        [INFO]  get run mode success
        [INFO]  load model ../model/resnet50.om success
        [INFO]  create model description success
        [INFO]  init memory pool success
        [INFO]  subscribe report success
        [INFO]  top 1: index[267] value[0.889648]
        [INFO]  top 1: index[161] value[0.836914]
        [INFO]  top 1: index[267] value[0.889648]
        [INFO]  top 1: index[161] value[0.836914]
        [INFO]  top 1: index[161] value[0.836914]
        [INFO]  top 1: index[267] value[0.889648]
        [INFO]  top 1: index[161] value[0.836914]
        [INFO]  top 1: index[267] value[0.889648]
        [INFO]  top 1: index[161] value[0.836914]
        [INFO]  top 1: index[267] value[0.889648]
        ......
        [INFO]  top 1: index[161] value[0.836914]
        [INFO]  top 1: index[267] value[0.889648]
        [INFO]  model execute success
        [INFO]  unsubscribe report success
        [INFO]  unload model success, modelId is 1
        [INFO]  execute sample success
        [INFO]  end to destroy stream
        [INFO]  end to destroy context
        [INFO]  end to reset device is 0
        [INFO]  end to finalize acl
        ```



