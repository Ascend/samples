# 基于Caffe ResNet-50网络实现图片分类（视频解码+同步推理）<a name="ZH-CN_TOPIC_0302603678"></a>

## 功能描述<a name="section7940919203810"></a>

该样例主要是基于Caffe ResNet-50网络（单输入、单Batch）实现图片分类的功能。

将Caffe ResNet-50网络的模型文件转换为适配昇腾AI处理器的离线模型（\*.om文件），在样例中，加载该om文件，将1个\*.h265格式的视频码流（仅包含一帧）循环10次解码出10张YUV420SP NV12格式的图片，对该10张图片做缩放，再对10张YUV420SP NV12格式的图片进行推理，分别得到推理结果后，再对推理结果进行处理，输出最大置信度的类别标识以及top5置信度的总和。

转换模型时，需配置色域转换参数，用于将YUV420SP格式的图片转换为RGB格式的图片，才能符合模型的输入要求。

## 原理介绍<a name="section6271153719394"></a>

在该样例中，涉及的关键功能点，如下所示：

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
    -   调用aclrtSynchronizeStream接口阻塞程序运行，直到指定stream中的所有任务都完成。

-   **内存管理**
    -   调用aclrtMallocHost接口申请Host上内存。
    -   调用aclrtFreeHost释放Host上的内存。
    -   调用aclrtMalloc接口申请Device上的内存。
    -   调用aclrtFree接口释放Device上的内存。
    -   执行数据预处理时，若需要申请Device上的内存存放输入或输出数据，需调用acldvppMalloc申请内存、调用acldvppFree接口释放内存。

-   **数据传输**

    调用aclrtMemcpy接口通过内存复制的方式实现数据传输。

-   **数据预处理**
    -   视频解码

        调用aclvdecSendFrame接口将视频码流解码成YUV420SP格式图片。

    -   缩放

        调用acldvppVpcResizeAsync接口将YUV420SP NV12格式图片缩小成分辨率为224\*224的图片。


-   **模型推理**
    -   调用aclmdlLoadFromFileWithMem接口从\*.om文件加载模型。
    -   调用aclmdlExecute接口执行模型推理。

        推理前，通过\*.om文件中的色域转换参数将YUV420SP格式的图片转换为RGB格式的图片。

    -   调用aclmdlUnload接口卸载模型。


## 目录结构<a name="section1394162513386"></a>

样例代码结构如下所示。

```
├── caffe_model
│   ├── aipp.cfg        //带色域转换参数的配置文件，模型转换时使用

├── data
│   ├── vdec_h265_1frame_rabbit_1280x720.h265            //测试数据,需要按指导获取测试图片，放到data目录下

├── inc
│   ├── dvpp_process.h               //声明数据预处理相关函数的头文件
│   ├── model_process.h              //声明模型处理相关函数的头文件
│   ├── sample_process.h               //声明资源初始化/销毁相关函数的头文件                  
│   ├── utils.h                       //声明公共函数（例如：文件读取函数）的头文件
│   ├── vdec_process.h                 //声明视频处理函数的头文件

├── src
│   ├── acl.json         //系统初始化的配置文件
│   ├── CMakeLists.txt         //编译脚本
│   ├── dvpp_process.cpp       //数据预处理相关函数的实现文件
│   ├── main.cpp               //主函数，图片分类功能的实现文件
│   ├── model_process.cpp      //模型处理相关函数的实现文件
│   ├── sample_process.cpp     //资源初始化/销毁相关函数的实现文件                                       
│   ├── utils.cpp              //公共函数（例如：文件读取函数）的实现文件
│   ├── vdec_process.cpp                 //声明视频处理函数的实现文件

├── .project     //工程信息文件，包含工程类型、工程描述、运行目标设备类型等
├── CMakeLists.txt    //编译脚本，调用src目录下的CMakeLists文件
```

## 环境要求<a name="section3833348101215"></a>

-   操作系统及架构：CentOS 7.6 x86\_64、CentOS aarch64、Ubuntu 18.04 x86\_64
-   版本：20.0、20.1、20.2
-   编译器：
    -   Ascend310 EP/Ascend710形态编译器：g++
    -   Atlas 200 DK编译器：aarch64-linux-gnu-g++

-   芯片：Ascend310、Ascend710
-   python及依赖的库：python3.7.5
-   已完成昇腾AI软件栈在开发环境、运行环境上的部署。

## 配置环境变量<a name="section19360144319911"></a>

-   **Ascend310 EP/Ascend710：**
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


        使用“$HOME/Ascend/ascend-toolkit/latest/_\{os\_arch\}_/acllib/lib64/stub”目录下的\*.so库，是为了编译基于AscendCL接口的代码逻辑时，不依赖其它组件（例如Driver）的任何\*.so库。编译通过后，在Host上运行应用时，通过配置环境变量，应用会链接到Host上“$HOME/Ascend/nnrt/latest/acllib/lib64”目录下的\*.so库，运行时会自动链接到依赖其它组件的\*.so库。

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

        使用“$HOME/Ascend/ascend-toolkit/latest/arm64-linux/acllib/lib64/stub”目录下的\*.so库，是为了编译基于AscendCL接口的代码逻辑时，不依赖其它组件（例如Driver）的任何\*.so库。编译通过后，在板端环境上运行应用时，通过配置环境变量，应用会链接到板端环境上“$HOME/Ascend/acllib/lib64”目录下的\*.so库，运行时会自动链接到依赖其它组件的\*.so库。



## 编译运行（Ascend310 EP/Ascend710）<a name="section6441858122516"></a>

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

        -   [https://gitee.com/HuaweiAscend/models/tree/master/computer\_vision/classification/resnet50](https://gitee.com/HuaweiAscend/models/tree/master/computer_vision/classification/resnet50)
        -   [https://github.com/Ascend-Huawei/models/tree/master/computer\_vision/classification/resnet50](https://github.com/Ascend-Huawei/models/tree/master/computer_vision/classification/resnet50)，查看README.\*.md，查找获取文件的链接

    4.  将ResNet-50网络转换为适配昇腾AI处理器的离线模型（\*.om文件），转换模型时，需配置色域转换参数，用于将YUV420SP格式的图片转换为RGB格式的图片。

        切换到样例目录，执行如下命令：

        ```
        atc --model=caffe_model/resnet50.prototxt --weight=caffe_model/resnet50.caffemodel --framework=0 --soc_version=${soc_version} --insert_op_conf=caffe_model/aipp.cfg --output=model/resnet50_aipp 
        ```

        -   --model：原始模型文件路径。
        -   --weight：权重文件路径。
        -   --framework：原始框架类型。0：表示Caffe；1：表示MindSpore；3：表示TensorFlow；5：表示ONNX。
        -   --soc\_version：Ascend310芯片，此处配置为Ascend310；Ascend710芯片，此处配置为Ascend710。
        -   --insert\_op\_conf：插入AIPP（AI Preprocessing）算子的配置文件路径，用于在AI Core上完成图像预处理，包括改变图像尺寸、色域转换（转换图像格式）、减均值/乘系数（改变图像像素），数据处理之后再进行真正的模型推理。
        -   --output：生成的resnet50\_aipp.om文件存放在“样例目录/model“目录下。建议使用命令中的默认设置，否则在编译代码前，您还需要修改sample\_process.cpp中的omModelPath参数值。

            ```
            const char* omModelPath = "../model/resnet50_aipp.om";
            ```



2.  编译代码。
    1.  以运行用户登录开发环境。
    2.  切换到样例目录，创建目录用于存放编译文件，例如，本文中，创建的目录为“build/intermediates/host“。

        ```
        mkdir -p build/intermediates/host
        ```

    3.  切换到“build/intermediates/host“目录，执行**cmake**生成编译文件。

        “../../../src“表示CMakeLists.txt文件所在的目录，请根据实际目录层级修改。

        -   当开发环境与运行环境操作系统架构相同时，执行如下命令编译。

            ```
            cd build/intermediates/host
            cmake ../../../src -DCMAKE_CXX_COMPILER=g++ -DCMAKE_SKIP_RPATH=TRUE
            ```

        -   当开发环境与运行环境操作系统架构不同时，执行以下命令进行交叉编译。

            ```
            cd build/intermediates/host
            cmake ../../../src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE
            ```


    4.  执行**make**命令，生成的可执行文件main在“样例目录/out“目录下。

        ```
        make
        ```


3.  准备输入视频码流。

    从以下链接获取输入视频码流文件，并以运行用户上传至开发环境的“样例目录/data“目录下。如果目录不存在，需自行创建。

    [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/vdec_h265_1frame_rabbit_1280x720.h265](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/vdec_h265_1frame_rabbit_1280x720.h265)

4.  运行应用。
    1.  以运行用户将开发环境的样例目录及目录下的文件上传到运行环境（Host），例如“$HOME/acl\_vdec\_resnet50”。
    2.  以运行用户登录运行环境（Host）。
    3.  切换到可执行文件main所在的目录，例如“$HOME/acl\_vdec\_resnet50/out”，给该目录下的main文件加执行权限。

        ```
        chmod +x main
        ```

    4.  切换到可执行文件main所在的目录，例如“$HOME/acl\_vdec\_resnet50/out”，运行可执行文件。

        ```
        ./main
        ```

        可执行文件执行成功后，在屏幕上的关键提示信息示例如下：

        ```
        ......
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_0, size=4000
        [INFO]  start check result file:./result/model_output_0
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_0.txt]
        [INFO]  result:classType[331],top1[0.908203],top5[1.000015]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_1, size=4000
        [INFO]  start check result file:./result/model_output_1
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_1.txt]
        [INFO]  result:classType[688],top1[0.596680],top5[0.901611]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_2, size=4000
        [INFO]  start check result file:./result/model_output_2
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_2.txt]
        [INFO]  result:classType[331],top1[0.908203],top5[1.000015]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_3, size=4000
        [INFO]  start check result file:./result/model_output_3
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_3.txt]
        [INFO]  result:classType[331],top1[0.908203],top5[1.000015]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_4, size=4000
        [INFO]  start check result file:./result/model_output_4
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_4.txt]
        [INFO]  result:classType[331],top1[0.908203],top5[1.000015]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_5, size=4000
        [INFO]  start check result file:./result/model_output_5
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_5.txt]
        [INFO]  result:classType[331],top1[0.908203],top5[1.000015]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_6, size=4000
        [INFO]  start check result file:./result/model_output_6
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_6.txt]
        [INFO]  result:classType[331],top1[0.908203],top5[1.000015]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_7, size=4000
        [INFO]  start check result file:./result/model_output_7
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_7.txt]
        [INFO]  result:classType[331],top1[0.908203],top5[1.000015]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_8, size=4000
        [INFO]  start check result file:./result/model_output_8
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_8.txt]
        [INFO]  result:classType[331],top1[0.908203],top5[1.000015]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_9, size=4000
        [INFO]  start check result file:./result/model_output_9
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_9.txt]
        [INFO]  result:classType[331],top1[0.908203],top5[1.000015]
        [INFO]  unload model success, modelId is 1
        [INFO]  execute sample success
        ......
        ```

        -   可执行文件运行过程中，VDEC解码成功后，模型推理前，会在main文件同级生成outdir目录，用于保存解码后的YUV420SP NV12格式的图片，推理结束后，系统会自动清理outdir目录。
        -   可执行文件执行成功后，同时会在main文件同级的result目录下生成结果文件，便于后期查看。\*.h265格式的视频码流（仅包含一帧）被解码出一张YUV420SP NV12格式的图片，对该张图片做十次循环处理，包括缩放图片和模型推理，分别得到十张图片的推理结果：
            -   model\_output\_\*：模型推理结果的二进制文件。
            -   model\_output\_\*.txt：模型推理结果的txt文件。




## 编译运行（Atlas 200 DK）<a name="section967981311262"></a>

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

        -   [https://gitee.com/HuaweiAscend/models/tree/master/computer\_vision/classification/resnet50](https://gitee.com/HuaweiAscend/models/tree/master/computer_vision/classification/resnet50)
        -   [https://github.com/Ascend-Huawei/models/tree/master/computer\_vision/classification/resnet50](https://github.com/Ascend-Huawei/models/tree/master/computer_vision/classification/resnet50)，查看README.\*.md，查找获取文件的链接

    4.  将ResNet-50网络转换为适配昇腾AI处理器的离线模型（\*.om文件），转换模型时，需配置色域转换参数，用于将YUV420SP格式的图片转换为RGB格式的图片。

        切换到样例目录，执行如下命令：

        ```
        atc --model=caffe_model/resnet50.prototxt --weight=caffe_model/resnet50.caffemodel --framework=0 --soc_version=${soc_version} --insert_op_conf=caffe_model/aipp.cfg --output=model/resnet50_aipp 
        ```

        -   --model：原始模型文件路径。
        -   --weight：权重文件路径。
        -   --framework：原始框架类型。0：表示Caffe；1：表示MindSpore；3：表示TensorFlow；5：表示ONNX。
        -   --soc\_version：Ascend310芯片，此处配置为Ascend310；Ascend710芯片，此处配置为Ascend710。
        -   --insert\_op\_conf：插入AIPP（AI Preprocessing）算子的配置文件路径，用于在AI Core上完成图像预处理，包括改变图像尺寸、色域转换（转换图像格式）、减均值/乘系数（改变图像像素），数据处理之后再进行真正的模型推理。
        -   --output：生成的resnet50\_aipp.om文件存放在“样例目录/model“目录下。建议使用命令中的默认设置，否则在编译代码前，您还需要修改sample\_process.cpp中的omModelPath参数值。

            ```
            const char* omModelPath = "../model/resnet50_aipp.om";
            ```



2.  编译代码。
    1.  以运行用户登录开发环境。
    2.  切换到样例目录，创建目录用于存放编译文件，例如，本文中，创建的目录为“build/intermediates/minirc“。

        ```
        mkdir -p build/intermediates/minirc
        ```

    3.  切换到“build/intermediates/minirc“目录，执行**cmake**生成编译文件。

        “../../../src“表示CMakeLists.txt文件所在的目录，请根据实际目录层级修改。

        ```
        cd build/intermediates/minirc
        cmake ../../../src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE
        ```

    4.  执行**make**命令，生成的可执行文件main在“样例目录/out“目录下。

        ```
        make
        ```


3.  准备输入视频码流。

    从以下链接获取输入视频码流文件，并以运行用户上传至开发环境的“样例目录/data“目录下。如果目录不存在，需自行创建。

    [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/vdec_h265_1frame_rabbit_1280x720.h265](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/vdec_h265_1frame_rabbit_1280x720.h265)

4.  运行应用。
    1.  以运行用户将开发环境的样例目录及目录下的文件上传到板端环境，例如“$HOME/acl\_vdec\_resnet50”。
    2.  以运行用户登录板端环境。
    3.  切换到可执行文件main所在的目录，例如“$HOME/acl\_vdec\_resnet50/out”，给该目录下的main文件加执行权限。

        ```
        chmod +x main
        ```

    4.  切换到可执行文件main所在的目录，例如“$HOME/acl\_vdec\_resnet50/out”，运行可执行文件。

        ```
        ./main
        ```

        可执行文件执行成功后，在屏幕上的关键提示信息示例如下：

        ```
        ......
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_0, size=4000
        [INFO]  start check result file:./result/model_output_0
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_0.txt]
        [INFO]  result:classType[331],top1[0.908203],top5[1.000015]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_1, size=4000
        [INFO]  start check result file:./result/model_output_1
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_1.txt]
        [INFO]  result:classType[688],top1[0.596680],top5[0.901611]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_2, size=4000
        [INFO]  start check result file:./result/model_output_2
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_2.txt]
        [INFO]  result:classType[331],top1[0.908203],top5[1.000015]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_3, size=4000
        [INFO]  start check result file:./result/model_output_3
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_3.txt]
        [INFO]  result:classType[331],top1[0.908203],top5[1.000015]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_4, size=4000
        [INFO]  start check result file:./result/model_output_4
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_4.txt]
        [INFO]  result:classType[331],top1[0.908203],top5[1.000015]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_5, size=4000
        [INFO]  start check result file:./result/model_output_5
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_5.txt]
        [INFO]  result:classType[331],top1[0.908203],top5[1.000015]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_6, size=4000
        [INFO]  start check result file:./result/model_output_6
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_6.txt]
        [INFO]  result:classType[331],top1[0.908203],top5[1.000015]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_7, size=4000
        [INFO]  start check result file:./result/model_output_7
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_7.txt]
        [INFO]  result:classType[331],top1[0.908203],top5[1.000015]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_8, size=4000
        [INFO]  start check result file:./result/model_output_8
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_8.txt]
        [INFO]  result:classType[331],top1[0.908203],top5[1.000015]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_9, size=4000
        [INFO]  start check result file:./result/model_output_9
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_9.txt]
        [INFO]  result:classType[331],top1[0.908203],top5[1.000015]
        [INFO]  unload model success, modelId is 1
        [INFO]  execute sample success
        ......
        ```

        -   可执行文件运行过程中，VDEC解码成功后，模型推理前，会在main文件同级生成outdir目录，用于保存解码后的YUV420SP NV12格式的图片，推理结束后，系统会自动清理outdir目录。
        -   可执行文件执行成功后，同时会在main文件同级的result目录下生成结果文件，便于后期查看。\*.h265格式的视频码流（仅包含一帧）被解码出一张YUV420SP NV12格式的图片，对该张图片做十次循环处理，包括缩放图片和模型推理，分别得到十张图片的推理结果：
            -   model\_output\_\*：模型推理结果的二进制文件。
            -   model\_output\_\*.txt：模型推理结果的txt文件。




