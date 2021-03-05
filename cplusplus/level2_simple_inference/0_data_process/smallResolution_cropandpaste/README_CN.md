# 数据预处理（抠图贴图）<a name="ZH-CN_TOPIC_0302603622"></a>

## 功能描述<a name="section7940919203810"></a>

对指定输入图片进行抠图（包括抠图区域小于10\*6），再贴图到输出图片中。

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

    抠图贴图：调用acldvppVpcCropAndPasteAsync接口按指定区域从输入图片中抠图，再将抠的图片贴到目标图片的指定位置，作为输出图片。


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
-   版本：20.3
-   编译器：
    -   Ascend 310 EP/Ascend 710/Ascend 910形态编译器：
        -   运行环境操作系统架构为x86时，编译器为g++
        -   运行环境操作系统架构为Arm时，编译器为aarch64-linux-gnu-g++

    -   Atlas 200 DK编译器：aarch64-linux-gnu-g++

-   芯片：Ascend 310、Ascend 710、Ascend 910

-   已在环境上部署昇腾AI软件栈。

## 配置环境变量<a name="section1931223812141"></a>

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


## 编译运行（Ascend 310 EP/Ascend 710/Ascend 910）<a name="section16011204259"></a>

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
    1.  以运行用户将开发环境的样例目录及目录下的文件上传到运行环境（Host），例如“$HOME/acl\_vpc\_smallResolution\_crop”。
    2.  以运行用户登录运行环境（Host）。
    3.  切换到可执行文件main所在的目录，例如“$HOME/acl\_vpc\_smallResolution\_crop/out”，给该目录下的main文件加执行权限。

        ```
        chmod +x main
        ```

    4.  切换到可执行文件main所在的目录，例如“$HOME/acl\_vpc\_smallResolution\_crop/out”，如下示例命令可实现：2次调用VPC进行抠图、缩放，将输入图片中指定区域分辨率为6\*6的子图抠出，并放大到分辨率为224\*224的图片。

        ```
        ./main --inImgName dvpp_vpc_1920x1080_nv12.yuv --inFormat 1 --inWidth 1920 --inHeight 1080 --cLeftOffset 0 --cRightOffset 5 --cTopOffset 0 --cBottomOffset 5 --outImgName output_224_224.yuv --outFormat 1 --outWidth 224 --outHeight 224 --pLeftOffset 0 --pRightOffset 223 --pTopOffset 0 --pBottomOffset 223
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



## 编译运行（Atlas 200 DK）<a name="section18557246182520"></a>

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
    1.  以运行用户将开发环境的样例目录及目录下的文件上传到板端环境，例如“$HOME/acl\_vpc\_smallResolution\_crop”。
    2.  以运行用户登录板端环境。
    3.  切换到可执行文件main所在的目录，例如“$HOME/acl\_vpc\_smallResolution\_crop/out”，给该目录下的main文件加执行权限。

        ```
        chmod +x main
        ```

    4.  切换到可执行文件main所在的目录，例如“$HOME/acl\_vpc\_smallResolution\_crop/out”，如下示例命令可实现：2次调用VPC进行抠图、缩放，将输入图片中指定区域分辨率为6\*6的子图抠出，并放大到分辨率为224\*224的图片。

        ```
        ./main --inImgName dvpp_vpc_1920x1080_nv12.yuv --inFormat 1 --inWidth 1920 --inHeight 1080 --cLeftOffset 0 --cRightOffset 5 --cTopOffset 0 --cBottomOffset 5 --outImgName output_224_224.yuv --outFormat 1 --outWidth 224 --outHeight 224 --pLeftOffset 0 --pRightOffset 223 --pTopOffset 0 --pBottomOffset 223
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



