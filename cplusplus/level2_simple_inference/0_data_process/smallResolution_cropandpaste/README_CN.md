# 数据预处理（抠图贴图）<a name="ZH-CN_TOPIC_0302603622"></a>

## 功能描述<a name="section7940919203810"></a>

对指定输入图片进行抠图（包括抠图区域小于10\*6），再贴图到输出图片中。


## 目录结构<a name="section1394162513386"></a>

样例代码结构如下所示。

```
├── data
│   ├── dvpp_vpc_1920x1080_nv12.yuv            //测试数据,需要按指导获取测试图片，放到data目录下

├── inc
│   ├── dvpp_process.h               //声明数据预处理相关函数的头文件
│   ├── sample_process.h               //声明模型处理相关函数的头文件                  
│   ├── utils.h                       //声明公共函数（例如：文件读取函数）的头文件

├── src
│   ├── acl.json               //系统初始化的配置文件
│   ├── CMakeLists.txt         //编译脚本
│   ├── dvpp_process.cpp       //数据预处理相关函数的实现文件
│   ├── main.cpp               //主函数，抠图贴图功能的实现文件
│   ├── sample_process.cpp     //资源初始化/销毁相关函数的实现文件                                       
│   ├── utils.cpp              //公共函数（例如：文件读取函数）的实现文件

├── CMakeLists.txt    //编译脚本，调用src目录下的CMakeLists文件
```

## 环境要求<a name="section3833348101215"></a>

-   操作系统及架构：CentOS 7.6 x86\_64、CentOS aarch64、Ubuntu 18.04 x86\_64、EulerOS x86、EulerOS aarch64
-   编译器：g++或aarch64-linux-gnu-g++
-   芯片：Ascend 310、Ascend 310P、Ascend 910
-   python及依赖的库：python3.7.5
-   已在环境上部署昇腾AI软件栈，并配置对应的的环境变量，请参见[Link](https://www.hiascend.com/document)中对应版本的CANN安装指南。
    
    以下步骤中，开发环境指编译开发代码的环境，运行环境指运行算子、推理或训练等程序的环境，运行环境上必须带昇腾AI处理器。开发环境和运行环境可以合设在同一台服务器上，也可以分设，分设场景下，开发环境下编译出来的可执行文件，在运行环境下执行时，若开发环境和运行环境上的操作系统架构不同，则需要在开发环境中执行交叉编译。

## 准备测试图片<a name="section1931223812148"></a>

1.  下载sample仓代码并上传至环境后，请先进入“cplusplus/level2_simple_inference/0_data_process/smallResolution_cropandpaste”样例目录。
     
    请注意，下文中的样例目录均指“cplusplus/level2_simple_inference/0_data_process/smallResolution_cropandpaste”目录。

2.  准备测试图片。

    请从以下链接获取该样例的输入图片，并以运行用户将获取的文件上传至开发环境的“样例目录/data“目录下。如果目录不存在，需自行创建。

    [https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dvpp\_vpc\_1920x1080\_nv12.yuv](https://obs-9be7.obs.cn-east-2.myhuaweicloud.com/models/aclsample/dvpp_vpc_1920x1080_nv12.yuv)
  

## 编译运行<a name="section16011204259"></a>

1.  配置CANN基础环境变量和Python环境变量，请参见[Link](../../../environment/environment_variable_configuration_CN.md)。

2.  编译代码。
    1.  以运行用户登录开发环境。

    2.  下载sample仓代码并上传至环境后，请先进入“cplusplus/level2_simple_inference/0_data_process/smallResolution_cropandpaste”样例目录。
     
    请注意，下文中的样例目录均指“cplusplus/level2_simple_inference/0_data_process/smallResolution_cropandpaste”目录。

    3. 设置环境变量，配置程序编译依赖的头文件与库文件路径。
  
       设置以下环境变量后，编译脚本会根据“{DDK_PATH}环境变量值/runtime/include/acl”目录查找编译依赖的头文件，根据{NPU_HOST_LIB}环境变量指向的目录查找编译依赖的库文件。“$HOME/Ascend”请替换“Ascend-cann-toolkit”包的实际安装路径。
  
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
    1.  以运行用户将开发环境的样例目录及目录下的文件上传到运行环境（Host），例如“$HOME/acl\_vpc\_smallResolution\_crop”。
    2.  以运行用户登录运行环境（Host）。
    3.  切换到可执行文件main所在的目录，例如“$HOME/acl\_vpc\_smallResolution\_crop/out”，给该目录下的main文件加执行权限。

        ```
        chmod +x main
        ```

    4.  切换到可执行文件main所在的目录，例如“$HOME/acl\_vpc\_smallResolution\_crop/out”，如下示例命令可实现：2次调用VPC进行抠图、缩放，将输入图片中指定区域分辨率为6\*6的子图抠出，并放大到分辨率为224\*224的图片。

        ```
        ./main --inImgName ../data/dvpp_vpc_1920x1080_nv12.yuv --inFormat 1 --inWidth 1920 --inHeight 1080 --cLeftOffset 0 --cRightOffset 5 --cTopOffset 0 --cBottomOffset 5 --outImgName output_224_224.yuv --outFormat 1 --outWidth 224 --outHeight 224 --pLeftOffset 0 --pRightOffset 223 --pTopOffset 0 --pBottomOffset 223
        ```

        参数说明如下，您可以根据实际需求修改：

        -   inImgName：输入图像文件的路径，包含文件名。
        -   inFormat：输入图片的格式。
        -   inWidth：输入图片的宽。
        -   inHeight：输入图片的高。
        -   cLeftOffset：抠图左偏移，必须为偶数。
        -   cRightOffset：抠图右偏移，必须为奇数。
        -   cTopOffset：抠图上偏移，必须为偶数。
        -   cBottomOffset：抠图下偏移，必须为奇数。
        -   outImgName：输出图像文件的路径，包含文件名。
        -   outFormat：输出图片的格式。
        -   outWidth：输出图片的宽。
        -   outHeight：输出图片的高。
        -   pLeftOffset：贴图左偏移，必须为偶数，需要16对齐。
        -   pRightOffset：贴图右偏移，必须为奇数。
        -   pTopOffset：贴图上偏移，必须为偶数。
        -   pBottomOffset：贴图下偏移，必须为奇数。

        执行成功后，在屏幕上的关键提示信息示例如下。

        ```
        [INFO]  acl init success
        [INFO]  set device 0 success
        [INFO]  create context success
        [INFO]  create stream success
        [INFO]  get run mode success
        [INFO]  dvpp init resource success
        [INFO]  call SplitProcessCropAndPaste
        [INFO]  vpc crop and paste success
        [INFO]  execute sample success
        [INFO]  end to destroy stream
        [INFO]  end to destroy context
        [INFO]  end to reset device 0
        ```


## 关键接口介绍<a name="section6271153719394"></a>

在该样例中，涉及的关键功能点及其接口，如下所示：

-   初始化
    -   调用aclInit接口初始化AscendCL配置。
    -   调用aclFinalize接口实现AscendCL去初始化。

-   Device管理
    -   调用aclrtSetDevice接口指定用于运算的Device。
    -   调用aclrtGetRunMode接口获取昇腾AI软件栈的运行模式，根据运行模式的不同，内部处理流程不同。
    -   调用aclrtResetDevice接口复位当前运算的Device，回收Device上的资源。

-   Context管理
    -   调用aclrtCreateContext接口创建Context。
    -   调用aclrtDestroyContext接口销毁Context。

-   Stream管理
    -   调用aclrtCreateStream接口创建Stream。
    -   调用aclrtDestroyStream接口销毁Stream。
    -   调用aclrtSynchronizeStream接口阻塞程序运行，直到指定stream中的所有任务都完成。

-   内存管理
    -   调用aclrtMallocHost接口申请Host上内存。
    -   调用aclrtFreeHost释放Host上的内存。
    -   调用aclrtMalloc接口申请Device上的内存。
    -   调用aclrtFree接口释放Device上的内存。
    -   执行数据预处理时，若需要申请Device上的内存存放输入或输出数据，需调用acldvppMalloc申请内存、调用acldvppFree接口释放内存。

-   数据传输

    调用aclrtMemcpy接口通过内存复制的方式实现数据传输。

-   数据预处理

    抠图贴图：调用acldvppVpcCropAndPasteAsync接口按指定区域从输入图片中抠图，再将抠的图片贴到目标图片的指定位置，作为输出图片。



