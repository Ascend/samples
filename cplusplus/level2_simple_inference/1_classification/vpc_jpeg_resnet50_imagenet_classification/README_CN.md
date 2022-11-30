# 基于Caffe ResNet-50网络实现图片分类（图片解码+抠图缩放+图片编码+同步推理）<a name="ZH-CN_TOPIC_0302603658"></a>

## 功能描述<a name="section7940919203810"></a>

该样例主要是基于Caffe ResNet-50网络（单输入、单Batch）实现图片分类的功能。

根据运行应用的入参，该样例可实现以下功能：

-   将一张YUV420SP格式的图片编码为\*.jpg格式的图片。
-   将两张\*.jpg格式的解码成两张YUV420SP NV12格式的图片，缩放，再进行模型推理，分别得到两张图片的推理结果后，处理推理结果，输出最大置信度的类别标识以及top5置信度的总和。
-   将两张\*.jpg格式的解码成两张YUV420SP NV12格式的图片，抠图，再进行模型推理，分别得到两张图片的推理结果后，处理推理结果，输出最大置信度的类别标识以及top5置信度的总和。
-   将两张\*.jpg格式的解码成两张YUV420SP NV12格式的图片，抠图贴图，再进行模型推理，分别得到两张图片的推理结果后，处理推理结果，输出最大置信度的类别标识以及top5置信度的总和。
-   将YUV420SP NV12格式的图片（分辨率8192\*8192）缩放，得到4000\*4000。

该样例中用于推理的模型文件是\*.om文件（适配昇腾AI处理器的离线模型），转换模型时，需配置色域转换参数，用于将YUV420SP格式的图片转换为RGB格式的图片，才能符合模型的输入要求。


## 目录结构<a name="section1394162513386"></a>

样例代码结构如下所示。

```
├── caffe_model
│   ├── aipp.cfg        //带色域转换参数的配置文件，模型转换时使用

├── data
│   ├── persian_cat_1024_1536_283.jpg            //测试数据,需要按指导获取测试图片，放到data目录下
│   ├── wood_rabbit_1024_1061_330.jpg            //测试数据,需要按指导获取测试图片，放到data目录下
│   ├── wood_rabbit_1024_1068_nv12.yuv            //测试数据,需要按指导获取测试图片，放到data目录下
│   ├── dvpp_vpc_8192x8192_nv12.yuv               //测试数据,需要按指导获取测试图片，放到data目录下

├── inc
│   ├── dvpp_process.h               //声明数据预处理相关函数的头文件
│   ├── model_process.h              //声明模型处理相关函数的头文件
│   ├── sample_process.h               //声明资源初始化/销毁相关函数的头文件                  
│   ├── utils.h                       //声明公共函数（例如：文件读取函数）的头文件

├── src
│   ├── acl.json         //系统初始化的配置文件
│   ├── CMakeLists.txt         //编译脚本
│   ├── dvpp_process.cpp       //数据预处理相关函数的实现文件
│   ├── main.cpp               //主函数，图片分类功能的实现文件
│   ├── model_process.cpp      //模型处理相关函数的实现文件
│   ├── sample_process.cpp     //资源初始化/销毁相关函数的实现文件                                       
│   ├── utils.cpp              //公共函数（例如：文件读取函数）的实现文件

├── .project     //工程信息文件，包含工程类型、工程描述、运行目标设备类型等
├── CMakeLists.txt    //编译脚本，调用src目录下的CMakeLists文件
```

## 环境要求<a name="section3833348101215"></a>

-   操作系统及架构：CentOS 7.6 x86\_64、CentOS aarch64、Ubuntu 18.04 x86\_64、EulerOS x86、EulerOS aarch64
-   编译器：g++或aarch64-linux-gnu-g++
-   芯片：Ascend 310、Ascend 310P、Ascend 910
-   python及依赖的库：python3.7.5
-   已在环境上部署昇腾AI软件栈，并配置对应的的环境变量，请参见[Link](https://www.hiascend.com/document)中对应版本的CANN安装指南。
    
    以下步骤中，开发环境指编译开发代码的环境，运行环境指运行算子、推理或训练等程序的环境，运行环境上必须带昇腾AI处理器。开发环境和运行环境可以合设在同一台服务器上，也可以分设，分设场景下，开发环境下编译出来的可执行文件，在运行环境下执行时，若开发环境和运行环境上的操作系统架构不同，则需要在开发环境中执行交叉编译。

  
## 准备模型和测试数据<a name="section183454368179"></a>

1.  配置CANN基础环境变量和Python环境变量，请参见[Link](../../../environment/environment_variable_configuration_CN.md)。

2.  以运行用户登录开发环境。

3.  下载sample仓代码并上传至环境后，请先进入“cplusplus/level2_simple_inference/1_classification/vpc_jpeg_resnet50_imagenet_classification”样例目录。
     
    请注意，下文中的样例目录均指“cplusplus/level2_simple_inference/1_classification/vpc_jpeg_resnet50_imagenet_classification”目录。

4.  准备ResNet-50模型。

    1.  获取ResNet-50原始模型。

        您可以从以下链接中获取ResNet-50网络的模型文件（\*.prototxt）、权重文件（\*.caffemodel），并以运行用户将获取的文件上传至开发环境的“样例目录/caffe\_model“目录下。如果目录不存在，需要自行创建。

        -   ResNet-50网络的模型文件（\*.prototxt）：单击[Link](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet50/resnet50.prototxt)下载该文件。
        -   ResNet-50网络的权重文件（\*.caffemodel）：单击[Link](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/resnet50/resnet50.caffemodel)下载该文件。 

    2.  将ResNet-50网络转换为适配昇腾AI处理器的离线模型（\*.om文件），转换模型时，需配置色域转换参数，用于将YUV420SP格式的图片转换为RGB格式的图片。

        切换到样例目录，执行如下命令：

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
5.  准备测试图片。

    请从以下链接获取该样例的测试图片，并以运行用户将获取的文件上传至开发环境的“样例目录/data“目录下。如果目录不存在，需自行创建。

    [https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dvpp\_vpc\_8192x8192\_nv12.yuv](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dvpp_vpc_8192x8192_nv12.yuv)

    [https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/persian\_cat\_1024\_1536\_283.jpg](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/persian_cat_1024_1536_283.jpg)

    [https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/wood\_rabbit\_1024\_1061\_330.jpg](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/wood_rabbit_1024_1061_330.jpg)

    [https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/wood\_rabbit\_1024\_1068\_nv12.yuv](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/wood_rabbit_1024_1068_nv12.yuv)


## 编译运行<a name="section183454368119"></a>

1.  配置CANN基础环境变量和Python环境变量，请参见[Link](../../../environment/environment_variable_configuration_CN.md)。

2.  编译代码。
    1.  以运行用户登录开发环境。

    2.  下载sample仓代码并上传至环境后，请先进入“cplusplus/level2_simple_inference/1_classification/vpc_jpeg_resnet50_imagenet_classification”样例目录。
     
        请注意，下文中的样例目录均指“cplusplus/level2_simple_inference/1_classification/vpc_jpeg_resnet50_imagenet_classification”目录。

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

    5.  切换到“build/intermediates/host“目录，执行**cmake**生成编译文件。

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


    6.  执行**make**命令，生成的可执行文件main在“样例目录/out“目录下。

        ```
        make
        ```

3.  运行应用。
    1.  以运行用户将开发环境的样例目录及目录下的文件上传到运行环境（Host），例如“$HOME/acl\_vpc\_jpege\_resnet50”。
    2.  以运行用户登录运行环境（Host）。
    3.  切换到可执行文件main所在的目录，例如“$HOME/acl\_vpc\_jpege\_resnet50/out”，给该目录下的main文件加执行权限。

        ```
        chmod +x main
        ```

    4.  切换到可执行文件main所在的目录，例如“$HOME/acl\_vpc\_jpege\_resnet50/out”，运行可执行文件。

         >**说明：** 
         >类别标签和类别的对应关系与训练模型时使用的数据集有关，本样例使用的模型是基于imagenet数据集进行训练的，您可以在互联网上查阅imagenet数据集的标签及类别的对应关系，例如，可单击[Link](https://blog.csdn.net/weixin_44676081/article/details/106755135)查看。

        1.  将两张\*.jpg格式的解码成两张YUV420SP NV12格式的图片，缩放，再进行模型推理，分别得到两张图片的推理结果。

            ```
            ./main 0
            ```

            执行结果示例如下，提示信息中的classType表示类别标签、top1表示该分类的最大置信度、top5表示最大的5个置信度之和，这些值可能会根据版本、环境有所不同，请以实际情况为准：

            ```
            [INFO] acl init success
            [INFO] open device 0 success
            [INFO] create context success
            [INFO] create stream success
            [INFO] dvpp init resource success
            [INFO] load model ../model/resnet50_aipp.om success
            [INFO] create model description success
            [INFO] create model output success
            [INFO]---------------------------------------------
            [INFO] start to process picture:../data/persian_cat_1024_1536_283.jpg
            [INFO] call JpegD
            [INFO] call vpcResize
            [INFO] Process dvpp success
            [INFO] model execute success
            [INFO] result : classType[283], top1[ xxxxxx], top5[xxxxxx]
            [INFO]---------------------------------------------
            [INFO] start to process picture:../data/wood_rabbit_1024_1061_330.jpg
            [INFO] call JpegD
            [INFO] call vpcResize
            [INFO] Process dvpp success
            [INFO] model execute success
            [INFO] result : classType[330], top1[ xxxxxx], top5[xxxxxx]
            [INFO]---------------------------------------------
            [INFO] Unload model success, modelId is 1
            [INFO] execute sample success
            [INFO] end to destroy stream 
            [INFO] end to destroy context
            [INFO] end to reset device is 0
            ```

        2.  将两张\*.jpg格式的解码成两张YUV420SP NV12格式的图片，抠图，再进行模型推理，分别得到两张图片的推理结果。

            ```
            ./main 1
            ```

            执行结果示例如下，提示信息中的classType表示类别标签、top1表示该分类的最大置信度、top5表示最大的5个置信度之和，这些值可能会根据版本、环境有所不同，请以实际情况为准：

            ```
            [INFO] acl init success
            [INFO] open device 0 success
            [INFO] create context success
            [INFO] create stream success
            [INFO] dvpp init resource success
            [INFO] load model ../model/resnet50_aipp.om success
            [INFO] create model description success
            [INFO] create model output success
            [INFO]---------------------------------------------
            [INFO] start to process picture:../data/persian_cat_1024_1536_283.jpg
            [INFO] call JpegD
            [INFO] call vpcCrop
            [INFO] Process dvpp success
            [INFO] model execute success
            [INFO] result : classType[284], top1[xxxxxx], top5[xxxxxx]
            [INFO]---------------------------------------------
            [INFO] start to process picture:../data/wood_rabbit_1024_1061_330.jpg
            [INFO] call JpegD
            [INFO] call vpcCrop
            [INFO] Process dvpp success
            [INFO] model execute success
            [INFO] result : classType[330], top1[xxxxxx], top5[xxxxxx]
            [INFO]---------------------------------------------
            [INFO] Unload model success, modelId is 1
            [INFO] execute sample success
            [INFO] end to destroy stream 
            [INFO] end to destroy context
            [INFO] end to reset device is 0
            ```

        3.  将两张\*.jpg格式的解码成两张YUV420SP NV12格式的图片，抠图贴图，再进行模型推理，分别得到两张图片的推理结果。

            ```
            ./main 2
            ```

            执行结果示例如下，提示信息中的classType表示类别标签、top1表示该分类的最大置信度、top5表示最大的5个置信度之和，这些值可能会根据版本、环境有所不同，请以实际情况为准：

            ```
            [INFO] acl init success
            [INFO] open device 0 success
            [INFO] create context success
            [INFO] create stream success
            [INFO] dvpp init resource success
            [INFO] load model ../model/resnet50_aipp.om success
            [INFO] create model description success
            [INFO] create model output success
            [INFO]---------------------------------------------
            [INFO] start to process picture:../data/persian_cat_1024_1536_283.jpg
            [INFO] call JpegD
            [INFO] call vpcCropAndPaste
            [INFO] Process dvpp success
            [INFO] model execute success
            [INFO] result : classType[284], top1[xxxxxx], top5[xxxxxx]
            [INFO]---------------------------------------------
            [INFO] start to process picture:../data/wood_rabbit_1024_1061_330.jpg
            [INFO] call JpegD
            [INFO] call vpcCropAndPaste
            [INFO] Process dvpp success
            [INFO] model execute success
            [INFO] result : classType[331], top1[xxxxxx], top5[xxxxxx]
            [INFO]---------------------------------------------
            [INFO] Unload model success, modelId is 1
            [INFO] execute sample success
            [INFO] end to destroy stream 
            [INFO] end to destroy context
            [INFO] end to reset device is 0
            ```

        4.  将一张YUV420SP格式的图片编码为\*.jpg格式的图片。

            ```
            ./main 3
            ```

            执行结果示例如下：

            ```
            [INFO] acl init success
            [INFO] open device 0 success
            [INFO] create context success
            [INFO] create stream success
            [INFO] dvpp init resource success
            [INFO] start to process picture:../data/wood_rabbit_1024_1068_nv12.yuv
            [INFO] call JpegE
            [INFO] end to destroy stream 
            [INFO] end to destroy context
            [INFO] end to reset device is 0
            ```

        5.  将一张分辨率为8192\*8192的YUV420SP格式的图片缩放至4000\*4000。

            ```
            ./main 4
            ```

            执行结果示例如下：

            ```
            [INFO] acl init success
            [INFO] open device 0 success
            [INFO] create context success
            [INFO] create stream success
            [INFO] get run mode success
            [INFO] dvpp process 8k resize begin
            [INFO] dvpp init resource success
            [INFO] dvpp process 8k resize success
            [INFO] end to destroy stream 
            [INFO] end to destroy context
            [INFO] end to reset device is 0
            [INFO] end to finalize acl
            ```


        执行可执行文件成功后，同时会在main文件同级的result目录下生成结果文件，便于后期查看。结果文件如下：

        -   dvpp\_output\_0：persian\_cat\_1024\_1536\_283.jpg图片经过缩放或抠图或抠图贴图之后的结果图片。
        -   dvpp\_output\_1：wood\_rabbit\_1024\_1061\_330.jpg图片经过缩放或抠图或抠图贴图之后的结果图片。
        -   model\_output\_0：persian\_cat\_1024\_1536\_283.jpg图片的模型推理结果，二进制文件。
        -   model\_output\_0.txt：persian\_cat\_1024\_1536\_283.jpg图片的模型推理结果，txt文件。
        -   model\_output\_1：wood\_rabbit\_1024\_1061\_330.jpg图片的模型推理结果，二进制文件。
        -   model\_output\_1.txt：wood\_rabbit\_1024\_1061\_330.jpg图片的模型推理结果，txt文件。
        -   jpege\_output\_0.jpg：wood\_rabbit\_1024\_1068\_nv12.yuv图片结果编码后的结果图片。
        -   dvpp\_vpc\_4000x4000\_nv12.yuv：dvpp\_vpc\_8192x8192\_nv12.yuv图片缩放后的结果图片。



## 关键接口介绍<a name="section6271153719394"></a>

在该样例中，涉及的关键功能点及其接口介绍，如下所示：

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
    -   图片编码

        调用acldvppJpegEncodeAsync接口将YUV420SP格式的图片编码为\*.jpg格式的图片。

    -   图片解码

        调用acldvppJpegDecodeAsync接口将\*.jpg图片解码成YUV420SP格式图片。

    -   缩放

        调用acldvppVpcResizeAsync接口对YUV420SP格式的输入图片进行缩放。

    -   抠图

        调用acldvppVpcCropAsync接口按指定区域从输入图片中抠图，再将抠的图片存放到输出内存中，作为输出图片。

    -   抠图贴图

        调用acldvppVpcCropAndPasteAsync接口按指定区域从输入图片中抠图，再将抠的图片贴到目标图片的指定位置，作为输出图片。


-   **模型推理**
    -   调用aclmdlLoadFromFileWithMem接口从\*.om文件加载模型。
    -   调用aclmdlExecute接口执行模型推理。

        推理前，通过\*.om文件中的色域转换参数将YUV420SP格式的图片转换为RGB格式的图片。

    -   调用aclmdlUnload接口卸载模型。

