# 媒体数据处理（抠图，一图多框）<a name="ZH-CN_TOPIC_0302603665"></a>

## 功能描述<a name="section7940919203810"></a>

该样例从一张YUV420SP NV12格式的输入图片中按指定区域分别抠出八张224\*224子图（YUV420SP NV12）。

## 原理介绍<a name="section6271153719394"></a>

在该样例中，涉及的关键功能点，如下所示：

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

    调用acldvppVpcBatchCropAsync接口按指定区域从输入图片中抠图，再将抠的图片存放到输出内存中，作为输出图片。


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
│   ├── main.cpp               //主函数，一图多框抠图功能的实现文件
│   ├── sample_process.cpp     //资源初始化/销毁相关函数的实现文件                                       
│   ├── utils.cpp              //公共函数（例如：文件读取函数）的实现文件

├── CMakeLists.txt    //编译脚本，调用src目录下的CMakeLists文件
```

## 环境要求<a name="section3833348101215"></a>

-   操作系统及架构：CentOS 7.6 x86\_64、CentOS aarch64、Ubuntu 18.04 x86\_64、EulerOS x86、EulerOS aarch64
-   编译器：g++或aarch64-linux-gnu-g++
-   芯片：Ascend 310、Ascend 310P、Ascend 910
-   python及依赖的库：python3.7.5
-   已在环境上部署昇腾AI软件栈

## 配置环境变量<a name="section137281211130"></a>

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
     
    3. 设置环境变量，配置程序编译依赖的头文件与库文件路径。
  
       设置以下环境变量后，编译脚本会根据“{DDK_PATH}环境变量值/acllib/include/acl”目录查找编译依赖的头文件，根据{NPU_HOST_LIB}环境变量指向的目录查找编译依赖的库文件。“$HOME/Ascend”请替换“Ascend-cann-toolkit”包的实际安装路径。
  
       -   当开发环境与运行环境的操作系统架构相同时，配置示例如下所示：
  
           ```
           export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest
           export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub
           ```
  
       -   当开发环境与运行环境的操作系统架构不同时，配置示例如下所示：
           
           例如，当开发环境为X86架构、运行环境为AArch64架构时，则涉及交叉编译，需在开发环境上安装AArch64架构的软件包，将{DDK_PATH}环境变量的路径指向AArch64架构的软件包安装目录（如下所示），便于使用与运行环境架构相同的软件包中的头文件和库文件来编译代码。
  
           ```
           export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
           export NPU_HOST_LIB=$DDK_PATH/acllib/lib64/stub
           ```
       
       您可以登录对应的环境，执行“uname -a”命令查询其操作系统的架构。

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
  

## 编译运行<a name="section19471849121012"></a>

1.  编译代码。
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


2.  准备输入图片。

    请从以下链接获取该样例的输入图片，并以运行用户将获取的文件上传至开发环境的“样例目录/data“目录下。如果目录不存在，需自行创建。

    [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dvpp\_vpc\_1920x1080\_nv12.yuv](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dvpp_vpc_1920x1080_nv12.yuv)

3.  运行应用。
    1.  以运行用户将开发环境的样例目录及目录下的文件上传到运行环境（Host），例如“$HOME/acl\_vpc\_batchcrop”。
    2.  以运行用户登录运行环境（Host）。
    3.  切换到可执行文件main所在的目录，例如“$HOME/acl\_vpc\_batchcrop/out”，给该目录下的main文件加执行权限。

        ```
        chmod +x main
        ```

    4.  切换到可执行文件main所在的目录，例如“$HOME/acl\_vpc\_batchcrop/out”，运行可执行文件。

        ```
        ./main
        ```

        执行成功后，在屏幕上的关键提示信息示例如下。

        ```
        [INFO]  aclInit success, ret = 0.
        [INFO]  open device 0 success
        [INFO]  create context success
        [INFO]  create stream success
        [INFO]  dvpp init resource success
        [INFO]  open file = ./dvpp_vpc_1920x1080_nv12.yuv success.
        [INFO]  start set inputDesc success.
        [INFO]  write out to file ./cropName0 success.
        [INFO]  write out to file ./cropName1 success.
        [INFO]  write out to file ./cropName2 success.
        [INFO]  write out to file ./cropName3 success.
        [INFO]  write out to file ./cropName4 success.
        [INFO]  write out to file ./cropName5 success.
        [INFO]  write out to file ./cropName6 success.
        [INFO]  write out to file ./cropName7 success.
        [INFO]  ProcessBatchCrop success.
        [INFO]  ProcessBatchCrop success.
        [INFO]  DestroyBatchCropResource start
        [INFO]  DestroyBatchCropResource end
        [INFO]  SampleProcess DestroyResource start.
        [INFO]  end to destroy stream
        [INFO]  end to destroy context
        [INFO]  0 deviceID
        [INFO]  end to reset device is 0
        [INFO]  SampleProcess DestroyResource success.
        [INFO]  end to finalize acl
        ......
        ```

        执行可执行文件成功后，同时会在main文件同级的目录下生成结果文件，便于后期查看。

        cropName0、cropName1、cropName2、cropName3、cropName4、cropName5、cropName6、cropName7这八张图片是从输入图片dvpp\_vpc\_1920×1980\_nv12.yuv中抠出的子图。

