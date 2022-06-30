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
-   编译器：g++或aarch64-linux-gnu-g++
-   芯片：Ascend 310、Ascend 310P、Ascend 910
-   python及依赖的库：python3.7.5
-   已在环境上部署昇腾AI软件栈。


## 配置环境变量<a name="section4721588588"></a>

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
  


## 编译运行<a name="section105241721131111"></a>

1.  模型转换。
    1.  以运行用户登录开发环境。

    2.  将矩阵乘算子的算子描述信息（\*.json文件）编译成适配昇腾AI处理器的离线模型（\*.om文件），运行矩阵乘算子时使用。

        切换到样例目录，执行如下命令(以昇腾310 AI处理器为例)：

        ```
        atc --singleop=run/out/test_data/config/gemm.json --soc_version=Ascend310 --output=run/out/op_models
        ```

        -   --singleop：单算子定义文件（\*.json文件）。
        -   --soc\_version：昇腾AI处理器的版本。进入“CANN软件安装目录/compiler/data/platform_config”目录，".ini"文件的文件名即为昇腾AI处理器的版本，请根据实际情况选择。

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


