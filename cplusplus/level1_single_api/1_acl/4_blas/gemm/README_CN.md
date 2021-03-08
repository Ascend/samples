# 实现矩阵-矩阵乘运算<a name="ZH-CN_TOPIC_0302603657"></a>

## 功能描述<a name="section14302794244"></a>

该样例主要实现矩阵-矩阵相乘的运算：C = αAB + βC，A、B、C都是16\*16的矩阵，矩阵乘的结果是一个16\*16的矩阵。

## 原理介绍<a name="section1934715933412"></a>

在该样例中，涉及的关键功能点，如下所示：

-   **初始化**
    -   调用aclInit接口初始化AscendCL配置。
    -   调用aclFinalize接口实现AscendCL去初始化。

-   **Device管理**
    -   调用aclrtSetDevice接口指定用于运算的Device。
    -   调用aclrtGetRunMode接口获取昇腾AI软件栈的运行模式，根据运行模式的不同，内部处理流程不同。
    -   调用aclrtResetDevice接口复位当前运算的Device，回收Device上的资源。

-   **Stream管理**
    -   调用aclrtCreateStream接口创建Stream。
    -   调用aclrtDestroyStream接口销毁Stream。
    -   调用aclrtSynchronizeStream接口阻塞程序运行，直到指定stream中的所有任务都完成。

-   **内存管理**
    -   调用aclrtMallocHost接口申请Host上内存。
    -   调用aclrtFreeHost释放Host上的内存。
    -   调用aclrtMalloc接口申请Device上的内存。
    -   调用aclrtFree接口释放Device上的内存。

-   **数据传输**
    -   调用aclrtMemcpy接口通过内存复制的方式实现数据传输。

-   **单算子调用**
    -   调用aclblasGemmEx接口实现矩阵-矩阵相乘的运算，由用户指定矩阵中元素的数据类型。在aclblasGemmEx接口内部封装了系统内置的矩阵乘算子GEMM。
    -   使用ATC（Ascend Tensor Compiler）工具将内置的矩阵乘算子GEMM的算子描述信息（包括输入输出Tensor描述、算子属性等）编译成适配昇腾AI处理器的离线模型（\*.om文件），用于验证矩阵乘算子GEMM的运行结果。


## 目录结构<a name="section1338083213243"></a>

样例代码结构如下所示。

```
├── inc                                 
│   ├── common.h                    //定义公共函数（例如：文件读取函数）的头文件
│   ├── gemm_runner.h               //定义矩阵乘运算相关函数的头文件
                 
├── run
│   ├── out  
│   │   ├──test_data
│   │   │   ├── config                           
│   │   │   │     ├── acl.json           //系统初始化的配置文件
│   │   │   │     ├── gemm.json          //矩阵乘算子的算子描述信息
│   │   │   ├── data                           
│   │   │   │     ├── generate_data.py   //用于生成矩阵A、矩阵B的数据

├── src
│   ├── CMakeLists.txt             //编译脚本
│   ├── common.cpp                 //公共函数（例如：文件读取函数）的实现文件
│   ├── gemm_main.cpp              //主函数的实现文件                         
│   ├── gemm_runner.cpp            //执行矩阵乘运算相关函数的实现文件
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
-   python及依赖的库：python3.7.5
-   已在环境上部署昇腾AI软件栈。

## 配置环境变量<a name="section4721588588"></a>

-   **Ascend 310 EP/Ascend 910：**
    1.  开发环境上，设置模型转换依赖的环境变量。

        $\{install\_path\}表示开发套件包Ascend-cann-toolkit所在的路径。

        ```
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        ```

    2.  开发环境上，设置环境变量，编译脚本src/CMakeLists.txt通过环境变量所设置的头文件、库文件的路径来编译代码。

        如下为设置环境变量的示例，请将$HOME/Ascend/ascend-toolkit/latest/_\{os\_arch\}_替换为开发套件包Ascend-cann-toolkit下对应架构的FwkACLlib的路径。

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

    3.  运行环境上，设置环境变量，运行应用时需要根据环境变量找到对应的库文件。
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


        使用“$HOME/Ascend/ascend-toolkit/latest/_\{os\_arch\}_/acllib/lib64/stub”目录下的\*.so库，是为了编译基于AscendCL接口的代码逻辑时，不依赖其它组件（例如Driver）的任何\*.so库。编译通过后，在Host上运行应用时，会根据环境变量LD\_LIBRARY\_PATH链接到“$HOME/Ascend/nnrt/latest/acllib/lib64”目录下的\*.so库，并自动链接到依赖其它组件的\*.so库。

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

        使用“$HOME/Ascend/ascend-toolkit/latest/arm64-linux/acllib/lib64/stub”目录下的\*.so库，是为了编译基于AscendCL接口的代码逻辑时，不依赖其它组件（例如Driver）的任何\*.so库。编译通过后，在板端环境上运行应用时，会根据环境变量LD\_LIBRARY\_PATH链接到“$HOME/Ascend/acllib/lib64”目录下的\*.so库，并自动链接到依赖其它组件的\*.so库。



## 编译运行（Ascend 310 EP/Ascend 710/Ascend 910）<a name="section105241721131111"></a>

1.  模型转换。
    1.  以运行用户登录开发环境。
    2.  设置环境变量。

        $\{install\_path\}表示开发套件包Ascend-cann-toolkit所在的路径。

        ```
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        ```

    3.  将矩阵乘算子的算子描述信息（\*.json文件）编译成适配昇腾AI处理器的离线模型（\*.om文件），运行矩阵乘算子时使用。

        切换到样例目录，执行如下命令：

        ```
        atc --singleop=run/out/test_data/config/gemm.json --soc_version=${soc_version} --output=run/out/op_models
        ```

        -   --singleop：单算子定义文件（\*.json文件）。
        -   --soc\_version：
            -   Ascend 310芯片，此处配置为Ascend310。
            -   Ascend 710芯片，此处配置为Ascend710。
            -   Ascend 910芯片，此处配置为Ascend910A或Ascend910B或Ascend910ProA或Ascend910ProB或Ascend910PremiumA，其中，Pro或Premium表示芯片性能提升等级、A或B表示PartialGood等级，请根据实际情况选择。

        -   --output：生成的om文件必须放在“run/out/op\_models“目录下。


2.  编译代码。
    1.  以运行用户登录开发环境。
    2.  切换到“样例目录/run/out/test\_data/data“目录下，执行generate\_data.py脚本，在“run/out/test\_data/data“目录下生成矩阵A的数据文件matrix\_a.bin、矩阵B的数据文件matrix\_b.bin、矩阵C的数据文件matrix\_c.bin。

        ```
        python3.7.5 generate_data.py
        ```

        “run/out/test\_data/data“目录下生成的output.bin文件，其中的数据是generate\_data.py脚本中直接通过公式计算出来的矩阵乘结果，不作为该样例的输入数据。

    3.  切换到样例目录下，创建目录用于存放编译文件，例如，本文中，创建的目录为“build/intermediates/host“。

        ```
        mkdir -p build/intermediates/host
        ```

    4.  切换到“build/intermediates/host“目录，执行**cmake**生成编译文件。

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


    5.  执行**make**命令，生成的可执行文件execute\_gemm\_op在“样例目录/run/out“目录下。

        ```
        make
        ```


3.  运行应用。
    1.  以运行用户将开发环境的样例目录及目录下的文件上传到运行环境（Host），例如“$HOME/acl\_execute\_gemm”。
    2.  以运行用户登录运行环境（Host）。
    3.  切换到可执行文件execute\_gemm\_op所在的目录，例如“$HOME/acl\_execute\_gemm/run/out”，给该目录下的execute\_gemm\_op文件加执行权限。

        ```
        chmod +x execute_gemm_op
        ```

    4.  切换到可执行文件execute\_gemm\_op所在的目录，例如“$HOME/acl\_execute\_gemm/run/out”，运行可执行文件。

        ```
        ./execute_gemm_op
        ```

        执行成功后，会直接在终端窗口显示矩阵A的数据、矩阵B的数据以及矩阵乘的结果，同时在“result\_files“目录下，生成存放矩阵乘结果的matrix\_c.bin文件。



## 编译运行（Atlas 200 DK）<a name="section7393133582419"></a>

1.  模型转换。
    1.  以运行用户登录开发环境。
    2.  设置环境变量。

        $\{install\_path\}表示开发套件包Ascend-cann-toolkit所在的路径。

        ```
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        ```

    3.  将矩阵乘算子的算子描述信息（\*.json文件）编译成适配昇腾AI处理器的离线模型（\*.om文件），运行矩阵乘算子时使用。

        切换到样例目录，执行如下命令：

        ```
        atc --singleop=run/out/test_data/config/gemm.json --soc_version=${soc_version} --output=run/out/op_models
        ```

        -   --singleop：单算子定义文件（\*.json文件）。
        -   --soc\_version：
            -   Ascend 310芯片，此处配置为Ascend310。
            -   Ascend 710芯片，此处配置为Ascend710。
            -   Ascend 910芯片，此处配置为Ascend910A或Ascend910B或Ascend910ProA或Ascend910ProB或Ascend910PremiumA，其中，Pro或Premium表示芯片性能提升等级、A或B表示PartialGood等级，请根据实际情况选择。

        -   --output：生成的om文件必须放在“run/out/op\_models“目录下。


2.  编译代码。
    1.  以运行用户登录开发环境。
    2.  切换到“样例目录/run/out/test\_data/data“目录下，执行generate\_data.py脚本，在“run/out/test\_data/data“目录下生成矩阵A的数据文件matrix\_a.bin、矩阵B的数据文件matrix\_b.bin、矩阵C的数据文件matrix\_c.bin。

        ```
        python3.7.5 generate_data.py
        ```

        “run/out/test\_data/data“目录下生成的output.bin文件，其中的数据是generate\_data.py脚本中直接通过公式计算出来的矩阵乘结果，不作为该样例的输入数据。

    3.  切换到样例目录，创建目录用于存放编译文件，例如，本文中，创建的目录为“build/intermediates/minirc“。

        ```
        mkdir -p build/intermediates/minirc
        ```

    4.  切换到“build/intermediates/minirc“目录，执行**cmake**生成编译文件。

        “../../../src“表示CMakeLists.txt文件所在的目录，请根据实际目录层级修改。

        ```
        cd build/intermediates/minirc
        cmake ../../../src -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ -DCMAKE_SKIP_RPATH=TRUE
        ```

    5.  执行**make**命令，生成的可执行文件execute\_gemm\_op在“样例目录/run/out“目录下。

        ```
        make
        ```


3.  运行应用。
    1.  以运行用户将开发环境的样例目录及目录下的文件上传到板端环境，例如“$HOME/acl\_execute\_gemm”。
    2.  以运行用户登录板端环境。
    3.  切换到可执行文件execute\_gemm\_op所在的目录，例如“$HOME/acl\_execute\_gemm/run/out”，给该目录下的execute\_gemm\_op文件加执行权限。

        ```
        chmod +x execute_gemm_op
        ```

    4.  切换到可执行文件execute\_gemm\_op所在的目录，例如“$HOME/acl\_execute\_gemm/run/out”，运行可执行文件。

        ```
        ./execute_gemm_op
        ```

        执行成功后，会直接在终端窗口显示矩阵A的数据、矩阵B的数据以及矩阵乘的结果，同时在“result\_files“目录下，生成存放矩阵乘结果的matrix\_c.bin文件。



