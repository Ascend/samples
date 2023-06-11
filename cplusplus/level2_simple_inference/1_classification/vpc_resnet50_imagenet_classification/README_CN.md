# 基于Caffe ResNet-50网络实现图片分类（图片解码+缩放+同步推理）<a name="ZH-CN_TOPIC_0302603667"></a>

## 功能描述<a name="section7940919203810"></a>

该样例主要是基于Caffe ResNet-50网络（单输入、单Batch）实现图片分类的功能。

在该样例中：

1.  先将2张\*.jpg图片分别解码，得到2张YUV420SP格式的图片，再将YUV420SP格式的图片都缩放成分辨率为224\*224的图片。
2.  加载离线模型om文件，对2张图片进行推理，分别得到推理结果，再对推理结果进行处理，输出最大置信度的类别标识。

    在加载离线模型前，提前将Caffe ResNet-50网络的模型文件转换为适配昇腾AI处理器的离线模型，转换模型时，需配置色域转换参数，用于将YUV420SP格式的图片转换为RGB格式的图片，才能符合模型的输入要求。





## 目录结构<a name="section1394162513386"></a>

样例代码结构如下所示。

```
├── caffe_model
│   ├── aipp.cfg        //带色域转换参数的配置文件，模型转换时使用

├── data
│   ├── dog1_1024_683.jpg            //测试数据,需要按指导获取测试图片，放到data目录下
│   ├── dog2_1024_683.jpg            //测试数据,需要按指导获取测试图片，放到data目录下

├── inc
│   ├── dvpp_process.h               //声明数据预处理相关函数的头文件
│   ├── model_process.h              //声明模型处理相关函数的头文件
│   ├── sample_process.h               //声明资源初始化/销毁相关函数的头文件
│   ├── singleOp_process.h              //声明单算子执行相关函数的头文件                   
│   ├── utils.h                       //声明公共函数（例如：文件读取函数）的头文件

├── out
│   ├── op_models
│   ├──├──op_list.json              //Cast算子和ArgMaxV2算子的算子描述信息

├── src
│   ├── acl.json         //系统初始化的配置文件
│   ├── CMakeLists.txt         //编译脚本
│   ├── dvpp_process.cpp       //数据预处理相关函数的实现文件
│   ├── main.cpp               //主函数，图片分类功能的实现文件
│   ├── model_process.cpp      //模型处理相关函数的实现文件
│   ├── sample_process.cpp     //资源初始化/销毁相关函数的实现文件
│   ├── singleOp_process.cpp   //单算子执行相关函数的实现文件                                          
│   ├── utils.cpp              //公共函数（例如：文件读取函数）的实现文件

├── .project     //工程信息文件，包含工程类型、工程描述、运行目标设备类型等
├── CMakeLists.txt    //编译脚本，调用src目录下的CMakeLists文件
```

## 环境要求<a name="section3833348101215"></a>

-   操作系统及架构：CentOS 7.6 x86\_64、CentOS aarch64、Ubuntu 18.04 x86\_64、EulerOS x86、EulerOS aarch64
-   编译器：g++ 或 aarch64-linux-gnu-g++
-   芯片：Ascend 310、Ascend 310P、Ascend 910
-   python及依赖的库：python3.7.5
-   已在环境上部署昇腾AI软件栈，并配置对应的的环境变量，请参见[Link](https://www.hiascend.com/document)中对应版本的CANN安装指南。

    以下步骤中，开发环境指编译开发代码的环境，运行环境指运行算子、推理或训练等程序的环境，运行环境上必须带昇腾AI处理器。开发环境和运行环境可以合设在同一台服务器上，也可以分设在不同的服务器上，分设场景下，开发环境下编译出来的可执行文件，在运行环境下执行时，若两者服务器上的操作系统架构不同，则需要在开发环境中执行交叉编译。


## 准备模型和图片<a name="section13133171616100"></a>

1.  配置CANN基础环境变量和Python环境变量，请参见[Link](../../../environment/environment_variable_configuration_CN.md)。

2.  以运行用户登录开发环境。

3.  下载sample仓代码并上传至环境后，请先进入“cplusplus/level2_simple_inference/1_classification/vpc_resnet50_imagenet_classification”样例目录。

    请注意，下文中的样例目录均指“cplusplus/level2_simple_inference/1_classification/vpc_resnet50_imagenet_classification”目录。

4.  准备ResNet-50模型。
    1.  获取ResNet-50原始模型。

        您可以从以下链接中获取ResNet-50网络的模型文件（\*.prototxt）、权重文件（\*.caffemodel），并以运行用户将获取的文件上传至开发环境的“样例目录/caffe\_model“目录下。如果目录不存在，需要自行创建。

        -   ResNet-50网络的模型文件（\*.prototxt）：单击[Link](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet50/resnet50.prototxt)下载该文件。
        -   ResNet-50网络的权重文件（\*.caffemodel）：单击[Link](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet50/resnet50.caffemodel)下载该文件。 

    2.  将ResNet-50原始模型转换为适配昇腾AI处理器的离线模型（\*.om文件），转换模型时，需配置色域转换参数，用于将YUV420SP格式的图片转换为RGB格式的图片。

        切换到样例目录，执行如下命令(以昇腾310 AI处理器为例)：

        ```
        atc --model=caffe_model/resnet50.prototxt --weight=caffe_model/resnet50.caffemodel --framework=0 --soc_version=Ascend310 --insert_op_conf=caffe_model/aipp.cfg --output=model/resnet50_aipp 
        ```

        -   --model：原始模型文件路径。
        -   --weight：权重文件路径。
        -   --framework：原始框架类型。0：表示Caffe；1：表示MindSpore；3：表示TensorFlow；5：表示ONNX。
        -   --soc\_version：昇腾AI处理器的版本。进入“CANN软件安装目录/compiler/data/platform_config”目录，".ini"文件的文件名即为昇腾AI处理器的版本，请根据实际情况选择。
        -   --insert\_op\_conf：插入AIPP（AI Preprocessing）算子的配置文件路径，用于在AI Core上完成图像预处理，包括改变图像尺寸、色域转换（转换图像格式）、减均值/乘系数（改变图像像素），数据处理之后再进行真正的模型推理。
        -   --output：生成的resnet50\_aipp.om文件存放在“样例目录/model“目录下。建议使用命令中的默认设置，否则在编译代码前，您还需要修改sample\_process.cpp中的omModelPath参数值。

            ```
            const char* omModelPath = "../model/resnet50_aipp.om";
            ```

5.  准备单算子模型文件。

    将Cast和ArgMaxV2两个算子的算子描述信息（\*.json文件）编译成适配昇腾AI处理器的离线模型（\*.om文件），用于运行算子时使用。

    切换到样例目录，执行如下命令(以昇腾310 AI处理器为例)：

    ```
    atc --singleop=out/op_models/op_list.json --soc_version=Ascend310 --output=out/op_models
    ```

    -   --singleop：单算子定义文件（\*.json文件）。
    -   --soc\_version：昇腾AI处理器的版本。进入“CANN软件安装目录/compiler/data/platform_config”目录，".ini"文件的文件名即为昇腾AI处理器的版本，请根据实际情况选择。

    -   --output参数：生成的om文件必须放在“out/op\_models“目录下。

6.  准备测试图片。

    请从以下链接获取该样例的输入图片，并以运行用户将获取的文件上传至开发环境的“样例目录/data“目录下。如果目录不存在，需自行创建。

    [https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dog1\_1024\_683.jpg](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dog1_1024_683.jpg)

    [https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dog2\_1024\_683.jpg](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dog2_1024_683.jpg)


## 编译运行<a name="section13133171616172"></a>

1.  配置CANN基础环境变量和Python环境变量，请参见[Link](../../../environment/environment_variable_configuration_CN.md)。

2.  编译代码。
    1.  以运行用户登录开发环境。

    2.  请先进入“cplusplus/level2_simple_inference/1_classification/vpc_resnet50_imagenet_classification”样例目录。

        请注意，下文中的样例目录均指“cplusplus/level2_simple_inference/1_classification/vpc_resnet50_imagenet_classification”目录。

    3. 设置环境变量，配置程序编译依赖的头文件与库文件路径。
  
       设置以下环境变量后，编译脚本会根据“{DDK_PATH}环境变量值/runtime/include/acl”目录查找编译依赖的头文件，根据{NPU_HOST_LIB}环境变量指向的目录查找编译依赖的库文件。“$HOME/Ascend”请替换“Ascend-cann-toolkit”包的实际安装路径。

       **注意**，在配置{NPU_HOST_LIB}环境变量时，需使用的“runtime/lib64/stub”目录下*.so库，确保在编译基于AscendCL接口的应用程序时，不依赖其它组件（例如Driver）的*.so库，编译成功后，运行应用程序时，系统会根据LD_LIBRARY_PATH环境变量查找“Ascend-cann-toolkit安装目录/runtime/lib64”目录下的*.so库，同时会自动链接到所依赖的其它组件的*.so库。
  
       -   当开发环境与运行环境的操作系统架构相同时，配置示例如下所示：
  
           ```
           export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest
           export NPU_HOST_LIB=$DDK_PATH/runtime/lib64/stub
           ```
  
       -   当开发环境与运行环境的操作系统架构不同时，配置示例如下所示：
           
           例如，当开发环境为X86架构、运行环境为AArch64架构时，则涉及交叉编译，需在开发环境上安装AArch64架构的软件包，将{DDK_PATH}环境变量的路径指向AArch64架构的软件包安装目录（如下所示），便于使用与运行环境架构相同的软件包中的头文件和库文件来编译代码。
  
           ```
           export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
           export NPU_HOST_LIB=$DDK_PATH/runtime/lib64/stub
           ```
       
       您可以登录对应的环境，执行“uname -a”命令查询其操作系统的架构。

    4.  切换到样例目录，创建目录用于存放编译文件，例如，本文中，创建的目录为“build/intermediates/host“。

        ```
        mkdir -p build/intermediates/host
        ```

    5.  切换到“build/intermediates/host“目录，执行以下命令生成编译文件。

        “../../../src“表示CMakeLists.txt文件所在的目录，请根据实际目录层级修改。

        将DCMAKE\_SKIP\_RPATH设置为TRUE，代表不会将rpath信息（即NPU_HOST_LIB配置的路径）添加到编译生成的可执行文件中去，可执行文件运行时会自动搜索实际设置的LD_LIBRARY_PATH中的动态链接库。

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
        您可以登录对应的环境，执行“uname -a”命令查询其操作系统的架构。

    6.  执行以下命令，生成的可执行文件main在“样例目录/out“目录下。

        ```
        make
        ```

3.  运行应用。

    当开发环境和运行环境可以合设在同一台服务器上时，无需执行以下第1~3步，直接执行以下第4步。

    1.  以运行用户将开发环境的样例目录及目录下的文件上传到运行环境，例如“$HOME/cplusplus/level2_simple_inference/1_classification/vpc_resnet50_imagenet_classification”。
    2.  以运行用户登录运行环境。
    3.  切换到可执行文件main所在的目录，例如“$HOME/cplusplus/level2_simple_inference/1_classification/vpc_resnet50_imagenet_classification/out”，给该目录下的main文件加执行权限。

        ```
        chmod +x main
        ```

    4.  切换到可执行文件main所在的目录，例如“$HOME/cplusplus/level2_simple_inference/1_classification/vpc_resnet50_imagenet_classification/out”，运行可执行文件。

        ```
        ./main
        ```

        执行成功后，在屏幕上显示最大置信度的类别标识，示例如下。

        ```
        [INFO] acl init success
        [INFO] open device 0 success
        [INFO] create context success
        [INFO] create stream success
        [INFO] dvpp init resource success
        [INFO] load model ../model/resnet50_aipp.om success
        [INFO] create model description success
        [INFO] create model output success
        [INFO] start to process picture:../data/dog1_1024_683.jpg
        [INFO] Process dvpp success
        [INFO] model execute success
        [INFO] execute singleOp Cast success
        [INFO] execute ArgMaxV2 success
        [INFO] singleOp process success
        [INFO] ---> index of classification result is 161
        [INFO] start to process picture:../data/dog2_1024_683.jpg
        [INFO] Process dvpp success
        [INFO] model execute success
        [INFO] execute singleOp Cast success
        [INFO] execute ArgMaxV2 success
        [INFO] singleOp process success
        [INFO] ---> index of classification result is 267 
        [INFO] Unload model success, modelId is 1
        [INFO] execute sample success
        [INFO] end to destroy stream 
        [INFO] end to destroy context
        [INFO] end to reset device is 0
        ```


        >**说明：** 
        >类别标签和类别的对应关系与训练模型时使用的数据集有关，本样例使用的模型是基于imagenet数据集进行训练的，您可以在互联网上查阅imagenet数据集的标签及类别的对应关系，例如，可单击[Link](https://blog.csdn.net/weixin_44676081/article/details/106755135)查看。
        >当前屏显信息中的类别标识与类别的对应关系如下：
        >"161": \["basset", "basset hound"\]、
        >"267": \["standard poodle"\]。



## 关键接口介绍<a name="section6271153719394"></a>

在该样例中，涉及的关键功能点及其接口，如下所示：

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
    -   调用acldvppJpegDecodeAsync接口将\*.jpg图片解码成YUV420SP格式图片。
    -   调用acldvppVpcResizeAsync接口将YUV420SP格式图片缩小成分辨率为224\*224的图片。

-   **模型推理**
    -   调用aclmdlLoadFromFileWithMem接口从\*.om文件加载模型。
    -   调用aclmdlExecute接口执行模型推理。

        推理前，通过\*.om文件中的色域转换参数将YUV420SP格式的图片转换为RGB格式的图片。

    -   调用aclmdlUnload接口卸载模型。

-   **数据后处理（单算子调用）**

    处理模型推理的结果，通过调用算子Cast将推理结果的数据类型从float32转成float16，再调用ArgMaxV2算子从推理结果中查找最大置信度的类别标识。

    通过aclopExecuteV2接口加载与执行算子。