# AddMatMatElements算子运行验证

## 功能描述

该样例实现了对[自定义算子AddMatMatElements](../../1_custom_op/doc/Add_CN.md)的功能验证，通过将自定义算子转换为单算子离线模型文件，然后通过AscendCL加载单算子模型文件进行运行。

说明：单算子模型文件的生成只依赖算子代码实现文件、算子原型定义、算子信息库，不依赖算子适配插件。

## 目录结构

```
├── inc                           //头文件目录
│   ├── common.h                  // 声明公共方法类，用于读取二进制文件
│   ├── operator_desc.h          //算子描述声明文件，包含算子输入/输出，算子类型以及输入描述与输出描述
│   ├── op_runner.h              //算子运行相关信息声明文件，包含算子输入/输出个数，输入/输出大小等
├── run                           // 单算子执行需要的文件存放目录
│   ├── out    // 单算子执行需要的可执行文件存放目录
│        └── test_data         // 测试数据存放目录
│           ├── config
│               └── acl.json       //用于进行~acl初始化，请勿修改此文件
│               └── add_op.json    // 算子描述文件，用于构造单算子模型文件
│           ├── data
│               └── generate_data.py    // 生成测试数据的脚本
├── src
│   ├── CMakeLists.txt    // 编译规则文件
│   ├── common.cpp         // 公共函数，读取二进制文件函数的实现文件
│   ├── main.cpp    // 将单算子编译为om文件并加载om文件执行，此文件中包含算子的相关配置，若验证其他单算子，基于此文件修改
│   ├── operator_desc.cpp     // 构造算子的输入与输出描述
│   ├── op_runner.cpp   // 单算子编译与运行函数实现文件
```

## 环境要求

-   操作系统及架构：CentOS x86\_64、CentOS aarch64、Ubuntu 18.04 x86\_64、EulerOS x86、EulerOS aarch64
-   芯片：Ascend 310、Ascend 310P、Ascend 910
-   python及依赖的库：python3.7.5
-   已完成昇腾AI软件栈的部署。
-   已参考[custom\_op](../../1_custom_op)完成自定义算子的编译部署。

## 配置环境变量

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
     
     设置以下环境变量后，编译脚本会根据“{DDK_PATH}环境变量值/runtime/include/acl”目录查找编译依赖的头文件，根据{NPU_HOST_LIB}环境变量指向的目录查找编译依赖的库文件。“$HOME/Ascend”请替换“Ascend-cann-toolkit”包的实际安装路径。

     - 当开发环境与运行环境的操作系统架构相同时，配置示例如下所示：

       ```
       export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest
       export NPU_HOST_LIB=$DDK_PATH/runtime/lib64/stub
       ```

     - 当开发环境与运行环境的操作系统架构不同时，配置示例如下所示：
       
       例如，当开发环境为X86架构、运行环境为AArch64架构时，则涉及交叉编译，需在开发环境上安装AArch64架构的软件包，将{DDK_PATH}环境变量的路径指向AArch64架构的软件包安装目录（如下所示，注意arm64-linux仅作为示例说明，请以实际架构目录名称为准），便于使用与运行环境架构相同的软件包中的头文件和库文件来编译代码。

       ```
       export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
       export NPU_HOST_LIB=$DDK_PATH/runtime/lib64/stub
       ```

- 运行环境上环境变量配置

  - 若运行环境上安装的“Ascend-cann-toolkit”包，环境变量设置如下：

    ```
    . ${HOME}/Ascend/ascend-toolkit/set_env.sh
    ```

  - 若运行环境上安装的“Ascend-cann-nnrt”包，环境变量设置如下：

    ```
    . ${HOME}/Ascend/nnrt/set_env.sh
    ```

  - 若运行环境上安装的“Ascend-cann-nnae”包，环境变量设置如下：

    ```
    . ${HOME}/Ascend/nnae/set_env.sh
    ```

    “$HOME/Ascend”请替换相关软件包的实际安装路径。

## 编译运行

1.  生成AddMatMatElements算子的单算子离线模型文件。
    1.  以运行用户（例如HwHiAiUser）登录开发环境，并进入样例工程的“acl\_execute\_add/run/out“目录。
    2.  在out目录下执行如下命令，生成单算子模型文件。

        **atc --singleop=test\_data/config/add\_op.json  --soc\_version=_$\{soc\_version\}_  --output=op\_models**

        其中：

        -   singleop：算子描述的json文件。
        -   soc\_version：昇腾AI处理器的型号，请根据实际情况替换。

        -   --output=op\_models：代表生成的模型文件存储在当前目录下的op\_models文件夹下。

        模型转换成功后，会生成如下文件：

        在当前目录的op\_models目录下生成单算子的模型文件**0\_AddMatMatElements\_3\_32\_3\_32\_3\_32\_1\_1\_3\_32.om**，命名规范为：序号+opType + 输入的描述\(dateType\_format\_shape\)+输出的描述。

        dataType以及format对应枚举值请从CANN软件所在目录下的“include/graph/types.h”文件中查看，枚举值从0开始依次递增。

        **说明：**模型转换时，会优先去查找自定义算子库去匹配模型文件中的算子。


2.  生成测试数据。

    进入样例工程目录的run/out/test\_data/data目录下，执行如下命令：

    **python3.7.5 generate\_data.py**

    会在当前目录下生成三个shape为\(3, 32\)，两个shape为\(1, 1\)，数据类型为float32的数据文件input\_0.bin、input\_1.bin和input\_2.bin，数据类型为int32的数据文件input\_3.bin、input\_4.bin，用于进行AddMatMatElements算子的验证。

3. 编译样例工程，生成单算子验证可执行文件。
    1.  切换到样例工程根目录acl\_execute\_add，然后在样例工程根目录下执行如下命令创建目录用于存放编译文件，例如，创建的目录为“build/intermediates/host“。

       **mkdir -p build/intermediates/host**

    2. 切换到“build/intermediates/host”目录，执行cmake命令生成编译文件。

       “../../../src“表示CMakeLists.txt文件所在的目录，请根据实际目录层级修改。

       DCMAKE_SKIP_RPATH需设置为TRUE，代表不会将rpath信息（即NPU_HOST_LIB配置的路径）添加到编译生成的可执行文件中去，可执行文件运行时会自动搜索实际设置的LD_LIBRARY_PATH中的动态链接库。

       -   当开发环境与运行环境操作系统架构相同时，执行如下命令编译。

           **cd build/intermediates/host**

           **cmake ../../../src -DCMAKE\_CXX\_COMPILER=g++ -DCMAKE\_SKIP\_RPATH=TRUE**

       -   当开发环境与运行环境操作系统架构不同时，需要使用交叉编译。

           例如，当开发环境为X86架构，运行环境为AArch64架构时，执行以下命令进行交叉编译。

           **cd build/intermediates/host**

           **cmake ../../../src -DCMAKE\_CXX\_COMPILER=aarch64-linux-gnu-g++ -DCMAKE\_SKIP\_RPATH=TRUE**

    3. 执行如下命令，生成可执行文件。

       **make**

       会在工程目录的“run/out“目录下生成可执行文件   **execute\_AddMatMatElements\_op**。


4.  在硬件设备的Host侧执行单算子验证文件。
    1.  以运行用户（例如HwHiAiUser）拷贝开发环境中样例工程acl\_execute\_AddMatMatElements/run/目录下的out文件夹到运行环境（硬件设备Host侧）任一目录，例如上传到/home/HwHiAiUser/HIAI\_PROJECTS/run\_AddMatMatElements/目录下。

        **说明：**若您的开发环境即为硬件设备的Host侧，此拷贝操作可跳过。

    2.  在运行环境中执行execute\_AddMatMatElements\_op文件，验证单算子模型文件。

        在/home/HwHiAiUser/HIAI\_PROJECTS/run\_AddMatMatElements/out目录下执行如下命令：

        **chmod +x execute\_AddMatMatElements\_op**

        **./execute\_AddMatMatElements\_op**
