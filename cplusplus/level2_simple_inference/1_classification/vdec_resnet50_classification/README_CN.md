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

-   操作系统及架构：CentOS 7.6 x86\_64、CentOS aarch64、Ubuntu 18.04 x86\_64、EulerOS x86、EulerOS aarch64
-   编译器：g++或aarch64-linux-gnu-g++
-   芯片：Ascend 310、Ascend 310P、Ascend 910
-   python及依赖的库：python3.7.5
-   已在环境上部署昇腾AI软件栈

## 配置环境变量<a name="section19360144319911"></a>

- 开发环境上环境变量配置

  1. CANN-Toolkit包提供进程级环境变量配置脚本，供用户在进程中引用，以自动完成CANN基础环境变量的配置，配置示例如下所示

     ```
     . ${HOME}/Ascend/ascend-toolkit/set_env.sh
     ```

     “$HOME/Ascend”请替换“Ascend-cann-toolkit”包的实际安装路径。

  2. 算子编译依赖Python，以Python3.7.5为例，请以运行用户执行如下命令设置Python3.7.5的相关环境变量。

     ```
     #用于设置python3.7.5库文件路径
     export LD_LIBRARY_PATH=/usr/local/python3.7.5/lib:$LD_LIBRARY_PATH
     #如果用户环境存在多个python3版本，则指定使用python3.7.5版本
     export PATH=/usr/local/python3.7.5/bin:$PATH
     ```

     Python3.7.5安装路径请根据实际情况进行替换，您也可以将以上命令写入~/.bashrc文件中，然后执行source ~/.bashrc命令使其立即生效。
     
  3. 开发环境上，设置环境变量，配置AscendCL单算子验证程序编译依赖的头文件与库文件路径。
  
     编译脚本会按环境变量指向的路径查找编译依赖的头文件和库文件，“$HOME/Ascend”请替换“Ascend-cann-toolkit”包的实际安装路径。
  
     -   当运行环境操作系统架构是x86时，配置示例如下所示：
  
         ```
         export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux
         export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub
         ```
  
     -   当运行环境操作系统架构时AArch64时，配置示例如下所示：
  
         ```
         export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
         export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub
         ```
- 运行环境上环境变量配置
  
  -   若运行环境上安装的“Ascend-cann-toolkit”包，环境变量设置如下：
  
      ```
      . ${HOME}/Ascend/ascend-toolkit/set_env.sh
      ```
  
  -   若运行环境上安装的“Ascend-cann-nnrt”包，环境变量设置如下：
  
      ```
      . ${HOME}/Ascend/nnrt/set_env.sh
      ```
  
  -   若运行环境上安装的“Ascend-cann-nnae”包，环境变量设置如下：
  
      ```
      . ${HOME}/Ascend/nnae/set_env.sh
      ```
  
    “$HOME/Ascend”请替换相关软件包的实际安装路径。



## 编译运行<a name="section6441858122516"></a>

1.  模型转换。
    1.  以运行用户登录开发环境。

    2.  准备数据。

        您可以从以下链接中获取ResNet-50网络的模型文件（\*.prototxt）、预训练模型文件（\*.caffemodel），并以运行用户将获取的文件上传至开发环境的“样例目录/caffe\_model“目录下。如果目录不存在，需要自行创建。

        -   从gitee上获取：单击[Link](https://github.com/Ascend/ModelZoo-TensorFlow/tree/master/TensorFlow/contrib/cv/resnet50/ATC_resnet50_caffe_AE)，查看README.md，查找获取原始模型的链接。
        -   从GitHub上获取：单击[Link](https://github.com/ascend/modelzoo/tree/master/contrib/TensorFlow/Research/cv/resnet50/ATC_resnet50_caffe_AE)，查看README.md，查找获取原始模型的链接。 

    3.  将ResNet-50网络转换为适配昇腾AI处理器的离线模型（\*.om文件），转换模型时，需配置色域转换参数，用于将YUV420SP格式的图片转换为RGB格式的图片。

        切换到样例目录，执行如下命令(以昇腾310 AI处理器为例)：

        ```
        atc --model=caffe_model/resnet50.prototxt --weight=caffe_model/resnet50.caffemodel --framework=0 --soc_version=Ascend310 --insert_op_conf=caffe_model/aipp.cfg --output=model/resnet50_aipp 
        ```

        -   --model：原始模型文件路径。
        -   --weight：权重文件路径。
        -   --framework：原始框架类型。0：表示Caffe；1：表示MindSpore；3：表示TensorFlow；5：表示ONNX。
        -   --soc\_version：
            -   昇腾310 AI处理器，此处配置为Ascend310。
            -   昇腾310P AI处理器，此处配置为Ascend310P。
            -   昇腾910 AI处理器，此处配置为Ascend910A或Ascend910B或Ascend910ProA或Ascend910ProB或Ascend910PremiumA，其中，Pro或Premium表示芯片性能提升等级、A或B表示PartialGood等级，请根据实际情况选择。

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


    4.  执行**make**命令，生成的可执行文件main在“样例目录/out“目录下。

        ```
        make
        ```


3.  准备输入视频码流。

    从以下链接获取输入视频码流文件，并以运行用户上传至开发环境的“样例目录/data“目录下。如果目录不存在，需自行创建。

    [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/vdec\_h265\_1frame\_rabbit\_1280x720.h265](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/vdec_h265_1frame_rabbit_1280x720.h265)

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

        可执行文件执行成功后，在屏幕上的关键提示信息示例如下，提示信息中的classType表示类别标识、top1表示该分类的最大置信度、top5表示最大的5个置信度之和，这些值可能会根据版本、环境有所不同，请以实际情况为准：

        ```
        ......
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_0, size=4000
        [INFO]  start check result file:./result/model_output_0
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_0.txt]
        [INFO]  result:classType[331],top1[xxxxxx],top5[xxxxxx]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_1, size=4000
        [INFO]  start check result file:./result/model_output_1
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_1.txt]
        [INFO]  result:classType[688],top1[xxxxxx],top5[xxxxxx]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_2, size=4000
        [INFO]  start check result file:./result/model_output_2
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_2.txt]
        [INFO]  result:classType[331],top1[xxxxxx],top5[xxxxxx]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_3, size=4000
        [INFO]  start check result file:./result/model_output_3
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_3.txt]
        [INFO]  result:classType[331],top1[xxxxxx],top5[xxxxxx]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_4, size=4000
        [INFO]  start check result file:./result/model_output_4
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_4.txt]
        [INFO]  result:classType[331],top1[xxxxxx],top5[xxxxxx]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_5, size=4000
        [INFO]  start check result file:./result/model_output_5
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_5.txt]
        [INFO]  result:classType[331],top1[xxxxxx],top5[xxxxxx]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_6, size=4000
        [INFO]  start check result file:./result/model_output_6
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_6.txt]
        [INFO]  result:classType[331],top1[xxxxxx],top5[xxxxxx]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_7, size=4000
        [INFO]  start check result file:./result/model_output_7
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_7.txt]
        [INFO]  result:classType[331],top1[xxxxxx],top5[xxxxxx]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_8, size=4000
        [INFO]  start check result file:./result/model_output_8
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_8.txt]
        [INFO]  result:classType[331],top1[xxxxxx],top5[xxxxxx]
        [INFO]  Process dvpp success
        [INFO]  model execute success
        [INFO]  output[0] DataBuffer, buffer addr = 0x10100007c000, buffer size = 4000
        [INFO]  memcopy output data from device to host buffer success.
        [INFO]  create output file success, filename=./result/model_output_9, size=4000
        [INFO]  start check result file:./result/model_output_9
        [INFO]  check result success, file exist
        [INFO]  reselut file: [./result/model_output_9.txt]
        [INFO]  result:classType[331],top1[xxxxxx],top5[xxxxxx]
        [INFO]  unload model success, modelId is 1
        [INFO]  execute sample success
        ......
        ```

        -   可执行文件运行过程中，VDEC解码成功后，模型推理前，会在main文件同级生成outdir目录，用于保存解码后的YUV420SP NV12格式的图片，推理结束后，系统会自动清理outdir目录。
        -   可执行文件执行成功后，同时会在main文件同级的result目录下生成结果文件，便于后期查看。\*.h265格式的视频码流（仅包含一帧）被解码出一张YUV420SP NV12格式的图片，对该张图片做十次循环处理，包括缩放图片和模型推理，分别得到十张图片的推理结果：
            -   model\_output\_\*：模型推理结果的二进制文件。
            -   model\_output\_\*.txt：模型推理结果的txt文件。


