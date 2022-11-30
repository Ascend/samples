中文|[English](README.md)
# 基于Caffe YOLOv3网络实现目标检测（动态Batch/动态分辨率）<a name="ZH-CN_TOPIC_0302603644"></a>

## 功能描述<a name="section340311417417"></a>

该样例主要是基于Caffe YOLOv3网络、在动态Batch或动态多分辨率场景下实现目标检测的功能。

将Caffe  YOLOv3网络的模型文件转换为适配昇腾AI处理器的离线模型（\*.om文件），转换命令中需要设置不同档位的Batch数（样例中batch档位分为1，2，4，8）或不同档位的分辨率（样例中分辨率档位分为416, 416；832，832；1248，1248），在应用中加载该om文件，通过传参设置选择不同档位的Batch数或者分辨率进行推理，并将推理结果保存到文件中。


## 目录结构<a name="section14723181815424"></a>

样例代码结构如下所示。

```
├── data
│   ├── tools_generate_data.py            //测试数据生成脚本

├── inc
│   ├── model_process.h               //声明模型处理相关函数的头文件
│   ├── sample_process.h              //声明资源初始化/销毁相关函数的头文件                   
│   ├── utils.h                       //声明公共函数（例如：文件读取函数）的头文件

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
-   编译器：g++或aarch64-linux-gnu-g++
-   芯片：Ascend 310、Ascend 310P、Ascend 910
-   python及依赖的库：python3.7.5
-   已在环境上部署昇腾AI软件栈，并配置对应的的环境变量，请参见[Link](https://www.hiascend.com/document)中对应版本的CANN安装指南。
    
    以下步骤中，开发环境指编译开发代码的环境，运行环境指运行算子、推理或训练等程序的环境，运行环境上必须带昇腾AI处理器。开发环境和运行环境可以合设在同一台服务器上，也可以分设，分设场景下，开发环境下编译出来的可执行文件，在运行环境下执行时，若开发环境和运行环境上的操作系统架构不同，则需要在开发环境中执行交叉编译。

## 准备模型和测试数据<a name="section9202202861529"></a>

1.  配置CANN基础环境变量和Python环境变量，请参见[Link](../../../environment/environment_variable_configuration_CN.md)。

2.  以运行用户登录开发环境。

3.  下载sample仓代码并上传至环境后，请先进入“cplusplus/level2_simple_inference/2_object_detection/YOLOV3_dynamic_batch_detection_picture”样例目录。
     
    请注意，下文中的样例目录均指“cplusplus/level2_simple_inference/2_object_detection/YOLOV3_dynamic_batch_detection_picture”目录。

4.  准备yolov3模型。

    1.  获取yolov3原始模型。

        您可以从以下链接中获取yolov3网络的模型文件（\*.prototxt）、权重文件（\*.caffemodel），并以运行用户将获取的文件上传至开发环境的“样例目录/caffe\_model“目录下。如果目录不存在，需要自行创建。

        -   yolov3网络的模型文件（\*.prototxt）：单击[Link](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/yolov3.prototxt)下载该文件。
        -   yolov3网络的权重文件（\*.caffemodel）：单击[Link](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/003_Atc_Models/AE/ATC%20Model/Yolov3/yolov3.caffemodel)下载该文件。 


    2.  切换到样例目录，将yolov3网络转换为适配昇腾AI处理器的离线模型（\*.om文件）。

        如果模型推理的输入数据是动态Batch的，执行如下命令转换模型(以昇腾310 AI处理器为例)：

        ```
        atc --model=caffe_model/yolov3.prototxt --weight=caffe_model/yolov3.caffemodel --framework=0 --input_shape="data:-1,3,416,416;img_info:-1,4" --input_format=NCHW --dynamic_batch_size="1,2,4,8" --soc_version=Ascend310 --output=model/yolov3_dynamic_batch
        ```

        如果模型推理的输入数据是动态分辨率的，执行如下命令转换模型(以昇腾310 AI处理器为例)：

        ```
        atc --model=caffe_model/yolov3.prototxt --weight=caffe_model/yolov3.caffemodel --framework=0 --input_shape="data:1,3,-1,-1" --input_format=NCHW --dynamic_image_size="416,416;832,832;1248,1248" --soc_version=Ascend310  --output=model/yolov3_dynamic_hw 
        ```

        -   --model：原始模型文件路径。
        -   --weight：权重文件路径。
        -   --framework：原始框架类型。0：表示Caffe；1：表示MindSpore；3：表示TensorFlow；5：表示ONNX。
        -   --input\_shape：模型输入数据的Shape。
        -   --input\_format：模型输入数据的Format。
        -   --dynamic\_batch\_size：设置动态Batch档位参数，适用于执行推理时，每次处理图片数量不固定的场景。
        -   --dynamic\_image\_size：设置输入图片的动态分辨率参数，适用于执行推理时，每次处理图片宽和高不固定的场景。
        -   --soc\_version：昇腾AI处理器的版本。进入“CANN软件安装目录/compiler/data/platform_config”目录，".ini"文件的文件名即为昇腾AI处理器的版本，请根据实际情况选择。
        -   --output：生成的yolov3\_dynamic\_batch.om或者yolov3\_dynamic\_hw.om文件存放在“样例目录/model“目录下。建议使用命令中的默认设置，否则在编译代码前，您还需要修改sample\_process.cpp中的omModelPath参数值。

            ```
            string omModelPath = "../model/yolov3_dynamic_batch.om";
            ......
            string omModelPath = "../model/yolov3_dynamic_hw.om";
            ```
5.  准备测试数据。

    切换到“样例目录/data“目录下，执行tools\_generate\_data.py脚本，可以在“样例目录/data“目录下生成各种不同Batch或者不同分辨率场景下的的测试bin文件。

    示例命令如下，可以生成Batch数为1，通道数为3，宽、高都为416像素，数据类型是float32的bin文件input\_float32\_1x3x416x416.bin.in。

    ```
    python3.7 tools_generate_data.py input -s [1,3,416,416] -r [2,3] -d float32
    ```

    tools\_generate\_data.py脚本参数说明如下：

    -   input：bin文件前缀名。
    -   s：输入数据的Shape信息。
    -   r：每个通道像素取值范围，该取值范围中的最小值、最大值必须在\[0,255\]范围。执行tools\_generate\_data.py脚本，会在-r参数指定的取值范围内随机生成每个通道的像素值。
    -   d：数据格式，支持int8、uint8、float16、float32、int32、uint32。


## 编译运行<a name="section17151737173112"></a>

1.  配置CANN基础环境变量和Python环境变量，请参见[Link](../../../environment/environment_variable_configuration_CN.md)。

2.  编译代码。
    1.  以运行用户登录开发环境。

    2.  下载sample仓代码并上传至环境后，请先进入“cplusplus/level2_simple_inference/2_object_detection/YOLOV3_dynamic_batch_detection_picture”样例目录。
     
        请注意，下文中的样例目录均指“cplusplus/level2_simple_inference/2_object_detection/YOLOV3_dynamic_batch_detection_picture”目录。

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
    1.  以运行用户将开发环境的样例目录及目录下的文件上传到运行环境（Host），例如“$HOME/acl\_yolov3\_dynamic\_batch”。
    2.  以运行用户登录运行环境（Host）。
    3.  切换到可执行文件main所在的目录，例如“$HOME/acl\_yolov3\_dynamic\_batch/out”，给该目录下的main文件加执行权限。

        ```
        chmod +x main
        ```

    4.  切换到可执行文件main所在的目录，例如“$HOME/acl\_yolov3\_dynamic\_batch/out”，运行可执行文件。
        -   动态Batch场景下，执行如下命令，可执行程序的入参需替换为实际的Batch数，必须为模型转换时通过--dynamic\_batch\_size参数指定的其中一档Batch数：

            ```
            ./main 1
            ```

            执行成功后，在屏幕上的关键提示信息示例如下：

            ```
            [INFO]  1: ./main [param], [param] is dynamic batch. It should be 1,2,4 or 8. For example: ./main 8;
            [INFO]  2: ./main [param1] [param2], [param1] is dynamic height. [param1] is dynamic width. It should be 416, 416; 832, 832; 1248, 1248. For example: ./main 416 416
            [INFO]  acl init success.
            [INFO]  set device 0 success.
            [INFO]  create context success.
            [INFO]  create stream success.
            [INFO]  get run mode success.
            [INFO]  load model ../model/yolov3_dynamic_batch.om success.
            [INFO]  create model description success.
            [INFO]  start to process file: ../data/input_float32_1x3x416x416.bin.in
            [INFO]  create model output success.
            [INFO]  set dynamic batch size[1] success.
            [INFO]  model input num[3], output num[2].
            [INFO]  start to print input tensor desc:
            [INFO]  index[0]: name[data], inputSize[16613376], fotmat[0], dataType[0]
            [INFO]  dimcount:[4],dims:[-1][3][416][416]
            [INFO]  index[1]: name[img_info], inputSize[16], fotmat[0], dataType[0]
            [INFO]  dimcount:[2],dims:[1][4]
            [INFO]  index[2]: name[ascend_mbatch_shape_data], inputSize[4], fotmat[2], dataType[3]
            [INFO]  dimcount:[1],dims:[1]
            [INFO]  start to print output tensor desc:
            [INFO]  index[0]: name[detection_out3:0:box_out], outputSize[196608], fotmat[0], dataType[0]
            [INFO]  dimcount:[2],dims:[8][6144]
            [INFO]  index[1]: name[detection_out3:1:box_out_num], outputSize[256], fotmat[0], dataType[3]
            [INFO]  dimcount:[2],dims:[8][8]
            [INFO]  start to print model dynamic batch info:
            [INFO]  dynamic batch count:[4],dims:{[1][2][4][8]}
            [INFO]  start to print model current output shape info:
            [INFO]  index:0,dims:[1][6144]
            [INFO]  index:1,dims:[1][8]
            [INFO]  model execute success.
            [INFO]  dump data success.
            [INFO]  unload model success, modelId is 1.
            [INFO]  execute sample success.
            [INFO]  end to destroy stream.
            [INFO]  end to destroy context.
            [INFO]  end to reset device: 0.
            [INFO]  end to finalize acl.
            ```


        -   动态分辨率场景下，可执行程序的第一个、第二个入参需要分别替换为实际的高、宽，必须为模型转换时通过--dynamic\_image\_size参数指定的其中一档分辨率：

            ```
            ./main 416 416
            ```

            执行成功后，在屏幕上的关键提示信息示例如下：

            ```
            [INFO]  1: ./main [param], [param] is dynamic batch. It should be 1,2,4 or 8. For example: ./main 8;
            [INFO]  2: ./main [param1] [param2], [param1] is dynamic height. [param1] is dynamic width. It should be 416, 416; 832, 832; 1248, 1248. For example: ./main 416 416
            [INFO]  acl init success.
            [INFO]  set device 0 success.
            [INFO]  create context success.
            [INFO]  create stream success.
            [INFO]  get run mode success.
            [INFO]  load model ../model/yolov3_dynamic_hw.om success.
            [INFO]  create model description success.
            [INFO]  start to process file: ../data/input_float32_1x3x416x416.bin.in
            [INFO]  create model input success.
            [INFO]  create model output success.
            [INFO]  set dynamic hw[416, 416] success.
            [INFO]  model input num[3], output num[2].
            [INFO]  start to print input tensor desc:
            [INFO]  index[0]: name[data], inputSize[18690048], fotmat[0], dataType[0]
            [INFO]  dimcount:[4],dims:[1][3][-1][-1]
            [INFO]  index[1]: name[img_info], inputSize[16], fotmat[0], dataType[0]
            [INFO]  dimcount:[2],dims:[1][4]
            [INFO]  index[2]: name[ascend_mbatch_shape_data], inputSize[8], fotmat[2], dataType[3]
            [INFO]  dimcount:[1],dims:[2]
            [INFO]  start to print output tensor desc:
            [INFO]  index[0]: name[detection_out3:0:box_out], outputSize[24576], fotmat[0], dataType[0]
            [INFO]  dimcount:[2],dims:[1][6144]
            [INFO]  index[1]: name[detection_out3:1:box_out_num], outputSize[32], fotmat[0], dataType[3]
            [INFO]  dimcount:[2],dims:[1][8]
            [INFO]  start to print model dynamic hw info:
            [INFO]  dynamic hw count:[3],dims:{[416, 416][832, 832][1248, 1248]}
            [INFO]  start to print model current output shape info:
            [INFO]  index:0,dims:[1][6144]
            [INFO]  index:1,dims:[1][8]
            [INFO]  model execute success.
            [INFO]  dump data success.
            [INFO]  unload model success, modelId is 1.
            [INFO]  destroy model description success.
            [INFO]  destroy model input success.
            [INFO]  destroy model output success.
            [INFO]  execute sample success.
            [INFO]  end to destroy stream.
            [INFO]  end to destroy context.
            [INFO]  end to reset device: 0.
            ```


## 关键接口介绍<a name="section3558105154116"></a>

在该Sample中，涉及的关键功能点及其接口，如下所示：

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
    -   调用aclmdlSetDynamicBatchSize设置Batch数或者调用aclmdlSetDynamicHWSize设置分辨率。
    -   调用aclmdlExecute接口执行模型推理，同步接口。
    -   调用aclmdlUnload接口卸载模型。

-   **数据后处理**

    样例中提供了自定义接口DumpModelOutputResult，用于将模型推理的结果写入文件（运行可执行文件后，推理结果文件在运行环境上的应用可执行文件的同级目录下）：

    ```
    processModel.DumpModelOutputResult();
    ```

