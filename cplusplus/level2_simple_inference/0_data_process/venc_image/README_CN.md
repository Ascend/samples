# 媒体数据处理（视频编码）<a name="ZH-CN_TOPIC_0302602980"></a>

## 功能描述<a name="section7940919203810"></a>

该样例将一张YUV420SP NV12格式的图片连续编码n次，生成一个H265格式的视频码流（n可配，通过运行应用时设置入参来配置，默认为16次）。

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

-   内存管理
    -   调用aclrtMallocHost接口申请Host上内存。
    -   调用aclrtFreeHost释放Host上的内存。
    -   调用aclrtMalloc接口申请Device上的内存。
    -   调用aclrtFree接口释放Device上的内存。
    -   执行数据预处理时，若需要申请Device上的内存存放输入或输出数据，需调用acldvppMalloc申请内存、调用acldvppFree接口释放内存。

-   数据传输

    调用aclrtMemcpy接口通过内存复制的方式实现数据传输。

-   数据预处理

    视频编码，调用aclvencSendFrame接口将将待编码的图片传到编码器进行编码。


## 目录结构<a name="section1394162513386"></a>

样例代码结构如下所示。

```
├── data
│   ├── dvpp_venc_128x128_nv12.yuv            //测试数据,需要按指导获取测试图片，放到data目录下

├── inc
│   ├── venc_process.h               //声明数据预处理相关函数的头文件
│   ├── sample_process.h               //声明模型处理相关函数的头文件                  
│   ├── utils.h                       //声明公共函数（例如：文件读取函数）的头文件

├── src
│   ├── acl.json               //系统初始化的配置文件
│   ├── CMakeLists.txt         //编译脚本
│   ├── main.cpp               //主函数，多图多框抠图功能的实现文件
│   ├── sample_process.cpp     //资源初始化/销毁相关函数的实现文件                                       
│   ├── utils.cpp              //公共函数（例如：文件读取函数）的实现文件
│   ├── venc_process.cpp       //数据预处理相关函数的实现文件

├── CMakeLists.txt    //编译脚本，调用src目录下的CMakeLists文件
```

## 环境要求<a name="section3833348101215"></a>

-   操作系统及架构：CentOS 7.6 x86\_64、CentOS aarch64、Ubuntu 18.04 x86\_64
-   版本：20.2
-   编译器：
    -   Ascend310 EP/Ascend710 EP形态编译器：g++
    -   Ascend310开发者板编译器：aarch64-linux-gnu-g++

-   芯片：Ascend310、Ascend710
-   已完成昇腾AI软件栈在开发环境、运行环境上的部署。

## 配置环境变量<a name="section145174543134"></a>

-   **Ascend310 EP/Ascend710 EP：**
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


        使用“$HOME/Ascend/ascend-toolkit/latest/_\{os\_arch\}_/acllib/lib64/stub”目录下的\*.so库，是为了编译基于AscendCL接口的代码逻辑时，不依赖其它组件（例如Driver）的任何\*.so库。编译通过后，在Host上运行应用时，通过配置环境变量，应用会链接到Host上“$HOME/Ascend/nnrt/latest/acllib/lib64”目录下的\*.so库，运行时会自动链接到依赖其它组件的\*.so库。

    2.  运行环境上，设置环境变量，运行应用时需要根据环境变量找到对应的库文件。

        如下为设置环境变量的示例，请将$HOME/Ascend/nnrt/latest替换为ACLlib的路径。

        ```
        export LD_LIBRARY_PATH=$HOME/Ascend/nnrt/latest/acllib/lib64
        ```


-   **Ascend310开发者板：**

    仅需在开发环境上设置环境变量，运行环境上的环境变量在制卡时已配置，此处无需单独配置。

    如下为设置环境变量的示例，请将$HOME/Ascend/ascend-toolkit/latest/arm64-linux替换为开发套件包Ascend-cann-toolkit下Arm架构的ACLlib的路径。

    ```
    export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
    export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/arm64-linux/acllib/lib64/stub
    ```

    使用“$HOME/Ascend/ascend-toolkit/latest/arm64-linux/acllib/lib64/stub”目录下的\*.so库，是为了编译基于AscendCL接口的代码逻辑时，不依赖其它组件（例如Driver）的任何\*.so库。编译通过后，在板端环境上运行应用时，通过配置环境变量，应用会链接到板端环境上“$HOME/Ascend/acllib/lib64”目录下的\*.so库，运行时会自动链接到依赖其它组件的\*.so库。


## 编译运行（Ascend310 EP/Ascend710 EP）<a name="section492582041817"></a>

1.  编译代码。
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


2.  准备输入图片。

    请从以下链接获取该样例的输入图片，并以运行用户将获取的文件上传至开发环境的“样例目录/data“目录下。如果目录不存在，需自行创建。

    [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dvpp_venc_128x128_nv12.yuv](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dvpp_venc_128x128_nv12.yuv)

3.  运行应用。
    1.  以运行用户将开发环境的样例目录及目录下的文件上传到运行环境（Host），例如“$HOME/acl\_venc”。
    2.  以运行用户登录运行环境（Host）。
    3.  切换到可执行文件main所在的目录，例如“$HOME/acl\_venc/out”，给该目录下的main文件加执行权限。

        ```
        chmod +x main
        ```

    4.  切换到可执行文件main所在的目录，例如“$HOME/acl\_venc/out”，运行可执行文件。

        执行可执行文件成功后，同时会在main文件同级output目录下生成编码结果文件dvpp\_venc\_128x128.h265，便于后期查看。

        执行可执行文件时，可以增加入参来配置编码的次数，例如“./main 20”，如果不增加入参，则默认编码次数为16次。

        ```
        ./main
        ```

        执行成功后，在屏幕上的关键提示信息示例如下。

        ```
        [INFO]  ./main param, param is execute venc times(default 16)
        [INFO]  acl init success
        [INFO]  open device 0 success
        [INFO]  create context success
        [INFO]  get run mode success
        [INFO]  create process callback thread successfully, threadId = 139926343980800
        [INFO]  start check result fold:output/
        [INFO]  make directory successfully.
        [INFO]  check result success, fold exist
        [INFO]  process callback thread start
        [INFO]  venc init resource success
        [INFO]  success to callback, stream size:5431
        [INFO]  success to callback, stream size:704
        [INFO]  success to callback, stream size:693
        [INFO]  success to callback, stream size:922
        [INFO]  success to callback, stream size:1153
        [INFO]  success to callback, stream size:689
        [INFO]  success to callback, stream size:857
        [INFO]  success to callback, stream size:1273
        [INFO]  success to callback, stream size:785
        [INFO]  success to callback, stream size:976
        [INFO]  success to callback, stream size:869
        [INFO]  success to callback, stream size:1445
        [INFO]  success to callback, stream size:845
        [INFO]  success to callback, stream size:1003
        [INFO]  success to callback, stream size:922
        [INFO]  success to callback, stream size:905
        [INFO]  venc process success
        [INFO]  execute sample success
        [INFO]  destory process callback thread success
        [INFO]  end to destroy context
        [INFO]  end to reset device is 0
        [INFO]  end to finalize acl
        ```



## 编译运行（Ascend310开发者板）<a name="section313154671820"></a>

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

    [https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dvpp_venc_128x128_nv12.yuv](https://c7xcode.obs.cn-north-4.myhuaweicloud.com/models/aclsample/dvpp_venc_128x128_nv12.yuv)

3.  运行应用。
    1.  以运行用户将开发环境的样例目录及目录下的文件上传到板端环境，例如“$HOME/acl\_venc”。
    2.  以运行用户登录板端环境。
    3.  切换到可执行文件main所在的目录，例如“$HOME/acl\_venc/out”，给该目录下的main文件加执行权限。

        ```
        chmod +x main
        ```

    4.  切换到可执行文件main所在的目录，例如“$HOME/acl\_venc/out”，运行可执行文件。

        执行可执行文件成功后，同时会在main文件同级output目录下生成编码结果文件dvpp\_venc\_128x128.h265，便于后期查看。

        执行可执行文件时，可以增加入参来配置编码的次数，例如“./main 20”，如果不增加入参，则默认编码次数为16次。

        ```
        ./main
        ```

        执行成功后，在屏幕上的关键提示信息示例如下。

        ```
        [INFO]  ./main param, param is execute venc times(default 16)
        [INFO]  acl init success
        [INFO]  open device 0 success
        [INFO]  create context success
        [INFO]  get run mode success
        [INFO]  create process callback thread successfully, threadId = 139926343980800
        [INFO]  start check result fold:output/
        [INFO]  make directory successfully.
        [INFO]  check result success, fold exist
        [INFO]  process callback thread start
        [INFO]  venc init resource success
        [INFO]  success to callback, stream size:5431
        [INFO]  success to callback, stream size:704
        [INFO]  success to callback, stream size:693
        [INFO]  success to callback, stream size:922
        [INFO]  success to callback, stream size:1153
        [INFO]  success to callback, stream size:689
        [INFO]  success to callback, stream size:857
        [INFO]  success to callback, stream size:1273
        [INFO]  success to callback, stream size:785
        [INFO]  success to callback, stream size:976
        [INFO]  success to callback, stream size:869
        [INFO]  success to callback, stream size:1445
        [INFO]  success to callback, stream size:845
        [INFO]  success to callback, stream size:1003
        [INFO]  success to callback, stream size:922
        [INFO]  success to callback, stream size:905
        [INFO]  venc process success
        [INFO]  execute sample success
        [INFO]  destory process callback thread success
        [INFO]  end to destroy context
        [INFO]  end to reset device is 0
        [INFO]  end to finalize acl
        ```



