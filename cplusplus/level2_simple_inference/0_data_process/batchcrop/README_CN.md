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
-   版本：20.3
-   编译器：
    -   Ascend 310 EP/Ascend 710/Ascend 910形态编译器：
        -   运行环境操作系统架构为x86时，编译器为g++
        -   运行环境操作系统架构为Arm时，编译器为aarch64-linux-gnu-g++

    -   Atlas 200 DK编译器：aarch64-linux-gnu-g++

-   芯片：Ascend 310、Ascend 710、Ascend 910

-   已在环境上部署昇腾AI软件栈。

## 配置环境变量<a name="section137281211130"></a>

-   **Ascend 310 EP/Ascend 910：**
    1.  开发环境上，设置环境变量，编译脚本src/CMakeLists.txt通过环境变量所设置的头文件、库文件的路径来编译代码。

        如下为设置环境变量的示例，请将$HOME/Ascend/ascend-toolkit/latest/_\{os\_arch\}_替换为开发套件包Ascend-cann-toolkit下对应架构的ACLlib的路径。

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

    2.  运行环境上，设置环境变量，运行应用时需要根据环境变量找到对应的库文件。
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
    1.  开发环境上，设置环境变量，编译脚本src/CMakeLists.txt通过环境变量所设置的头文件、库文件的路径来编译代码。

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

    2.  运行环境上，设置环境变量，运行应用时需要根据环境变量找到对应的库文件。

        如下为设置环境变量的示例，请将$HOME/Ascend/nnrt/latest替换为ACLlib的路径。

        ```
        export LD_LIBRARY_PATH=$HOME/Ascend/nnrt/latest/acllib/lib64
        ```


-   **Atlas 200 DK：**

    仅需在开发环境上设置环境变量，运行环境上的环境变量在制卡时已配置，此处无需单独配置。

    如下为设置环境变量的示例，请将$HOME/Ascend/ascend-toolkit/latest/arm64-linux替换为开发套件包Ascend-cann-toolkit下Arm架构的ACLlib的路径。

    ```
    export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
    export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/arm64-linux/acllib/lib64/stub
    ```

    使用“$HOME/Ascend/ascend-toolkit/latest/arm64-linux/acllib/lib64/stub”目录下的\*.so库，是为了编译基于AscendCL接口的代码逻辑时，不依赖其它组件（例如Driver）的任何\*.so库。编译通过后，在板端环境上运行应用时，会根据环境变量LD\_LIBRARY\_PATH链接到“$HOME/Ascend/acllib/lib64”目录下的\*.so库，并自动链接到依赖其它组件的\*.so库。


## 编译运行（Ascend 310 EP/Ascend 710/Ascend 910）<a name="section19471849121012"></a>

1.  编译代码。
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



## 编译运行（Atlas 200 DK）<a name="section112152503113"></a>

1.  编译代码。
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


2.  准备输入图片。

    请从以下链接获取该样例的输入图片，并以运行用户将获取的文件上传至开发环境的“样例目录/data“目录下。如果目录不存在，需自行创建。

    [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dvpp\_vpc\_1920x1080\_nv12.yuv](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dvpp_vpc_1920x1080_nv12.yuv)

3.  运行应用。
    1.  以运行用户将开发环境的“acl\_vpc\_batchcrop“目录及目录下的文件上传到板端环境，例如“$HOME/acl\_vpc\_batchcrop”。
    2.  以运行用户登录板端环境。
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



