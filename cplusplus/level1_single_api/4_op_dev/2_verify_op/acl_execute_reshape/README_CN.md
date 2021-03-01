# ReShape算子运行验证<a name="ZH-CN_TOPIC_0304173128"></a>

## 功能描述<a name="section1421916179418"></a>

该样例实现了对[自定义算子ReshapeCust](../../1_custom_op/doc/Reshape_CN.md)的功能验证，通过将自定义算子转换为单算子离线模型文件，然后通过AscendCL加载单算子模型文件进行运行。

说明：单算子模型文件的生成只依赖算子代码实现文件、算子原型定义、算子信息库，不依赖算子适配插件。

## 目录结构<a name="section8733528154320"></a>

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
│               └── reshape_op.json    // 算子描述文件，用于构造单算子模型文件
│           ├── data
│               └── generate_data.py    // 生成测试数据的脚本
├── src
│   ├── CMakeLists.txt    // 编译规则文件
│   ├── common.cpp         // 公共函数，读取二进制文件函数的实现文件  
│   ├── main.cpp    // 将单算子编译为om文件并加载om文件执行，此文件中包含算子的相关配置，若验证其他单算子，基于此文件修改
│   ├── operator_desc.cpp     // 构造算子的输入与输出描述
│   ├── op_runner.cpp   // 单算子编译与运行函数实现文件
```

## 环境要求<a name="zh-cn_topic_0230709958_section1256019267915"></a>

-   操作系统及架构：CentOS x86\_64、CentOS aarch64、EulerOS x86、EulerOS aarch64
-   版本：20.3
-   编译器：

    Ascend 310 EP/Ascend 710/Ascend 910编译器

    -   运行环境操作系统架构为x86时，编译器为g++
    -   运行环境操作系统架构为arm64时，编译器为aarch64-linux-gnu-g++

-   芯片：Ascend 310，Ascend 710，Ascend 910
-   python及依赖的库：python3.7.5
-   已完成昇腾AI软件栈在开发环境、运行环境上的部署。
-   已参考[custom\_op](../../1_custom_op)完成自定义算子的编译部署。

## 配置环境变量<a name="section5357113119427"></a>

-   Ascend 310 EP/Ascend 910
    1.  开发环境上，设置生成单算子离线模型的环境变量。

        环境变量配置示例如下：

        ```
        export install_path=$HOME/Ascend/ascend-toolkit/latest
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        export ASCEND_AICPU_PATH=${install_path}
        ```

        “$HOME/Ascend”请替换“Ascend-cann-toolkit”包的实际安装路径。

    2.  开发环境上，设置环境变量，配置AscendCL单算子验证程序编译依赖的头文件与库文件路径。

        编译脚本会按环境变量指向的路径查找编译依赖的头文件和库文件，“$HOME/Ascend”请替换“Ascend-cann-toolkit”包的实际安装路径。

        -   当运行环境操作系统架构是x86时，配置示例如下所示：

            ```
            export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux
            export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux/fwkacllib/lib64/stub
            ```

        -   当运行环境操作系统架构时arm64时，配置示例如下所示：

            ```
            export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
            export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/arm64-linux/fwkacllib/lib64/stub
            ```


        ```
        说明：
        使用FwkACLlib组件安装路径下“lib64/stub”目录下的*.so库，是为了编译基于AscendCL接口的代码逻辑时，不依赖其它组件的任何*.so库。编译通过后，在Host上运行应用时，会根据“LD_LIBRARY_PATH”环境变量链接到“fwkacllib/lib64”或者“acllib/lib64”目录下的*.so库，并自动链接到其他组件依赖的*.so库。
        ```
    
    3.  运行环境上，设置运行应用时依赖AscendCL库文件的环境变量。
    
        -   若运行环境上安装的“Ascend-cann-toolkit”包，环境变量设置如下：
    
            ```
            export LD_LIBRARY_PATH=$HOME/Ascend/ascend-toolkit/latest/fwkacllib/lib64
            ```
    
        -   若运行环境上安装的“Ascend-cann-nnrt”包，环境变量设置如下：
    
            ```
            export LD_LIBRARY_PATH=$HOME/Ascend/nnrt/latest/acllib/lib64
            ```
    
        -   若运行环境上安装的“Ascend-cann-nnae”包，环境变量设置如下：
    
            ```
            export LD_LIBRARY_PATH=$HOME/Ascend/nnae/latest/fwkacllib/lib64
            ```


        “$HOME/Ascend”请替换相关软件包的实际安装路径。


-   Ascend 710
    1.  开发环境上，设置生成单算子离线模型的环境变量。

        环境变量配置示例如下：

        ```
        export install_path=$HOME/Ascend/ascend-toolkit/latest
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        export ASCEND_AICPU_PATH=${install_path}
        ```

        install\_path为开发套件包Ascend-cann-toolkit的安装路径。

    2.  开发环境上，设置环境变量，配置AscendCL单算子验证程序编译依赖的头文件与库文件路径。

        编译脚本会按环境变量指向的路径查找编译依赖的头文件和库文件，“$HOME/Ascend”请替换“Ascend-cann-toolkit”包的实际安装路径。

        -   当运行环境操作系统架构是x86时，配置示例如下所示：

            ```
            export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux
            export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux/acllib/lib64/stub
            ```

        -   当运行环境操作系统架构时arm64时，配置示例如下所示：

            ```
            export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
            export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/arm64-linux/acllib/lib64/stub
            ```


        ```
        说明：
        使用ACLlib组件安装路径下“lib64/stub”目录下的*.so库，是为了编译基于AscendCL接口的代码逻辑时，不依赖其它组件的任何*.so库。编译通过后，在Host上运行应用时，会根据“LD_LIBRARY_PATH”环境变量链接到“acllib/lib64”目录下的*.so库，并自动链接到其他组件依赖的*.so库。
        ```
    
    3.  运行环境上，设置运行应用时依赖AscendCL库文件的环境变量。
    
        如下为设置环境变量的示例，请将$HOME/Ascend/nnrt/latest替换为Ascend-cann-nnrt包的实际安装路径。
    
        ```
        export LD_LIBRARY_PATH=$HOME/Ascend/nnrt/latest/acllib/lib64
        ```



## 编译运行（Ascend 310 EP/Ascend 710/Ascend 910）<a name="section1042793885117"></a>

1.  生成ReshapeCust算子的单算子离线模型文件。
    1.  以运行用户（例如HwHiAiUser）登录开发环境，并进入样例工程的“acl\_execute\_reshape/run/out“目录。
    2.  在out目录下执行如下命令，生成单算子模型文件。

        **atc --singleop=test\_data/config/reshape\_op.json  --soc\_version=_$\{soc\_version\}_  --output=op\_models**

        其中：

        -   singleop：算子描述的json文件。
        -   soc\_version：昇腾AI处理器的型号。
        -   --output=op\_models：代表生成的模型文件存储在当前目录下的op\_models文件夹下。

        模型转换成功后，会生成如下文件：

        在当前目录的op\_models目录下生成单算子的模型文件**0\_ReshapeCust\_3\_2\_9\_3\_2\_2\_3\_2\_3\_3.om**，命名规范为：序号+opType + 输入的描述\(dateType\_format\_shape\)+输出的描述。

        dataType以及format对应枚举值请从ATC组件所在目录下的“atc/include/graph/types.h”文件中查看，枚举值从0开始依次递增。

        **说明：**模型转换时，会优先去查找自定义算子库去匹配模型文件中的算子。


2.  生成测试数据。

    进入样例工程目录的run/out/test\_data/data目录下，执行如下命令：

    **python3.7.5 generate\_data.py**

    会在当前目录下生成第一个shape为\(9\)，第二个shape为\(2\)，value为\[3,3\]，数据类型均为int32的数据文件input\_0.bin与input\_1.bin，用于进行ReshapeCust算子的验证。

3.  编译样例工程，生成单算子验证可执行文件。
    1.  针对Ascend 310与Ascend 910，修改src/CMakeLists.txt文件中的如下配置段，将“**acllib**”修改为“**fwkacllib**”，Ascend 710无需执行此步骤。

        ```
        # Header path
        include_directories(
            ${INC_PATH}/acllib/include/
            ../inc/
        )
        ```

    2.  切换到样例工程根目录acl\_execute\_reshape，然后在样例工程根目录下执行如下命令创建目录用于存放编译文件，例如，创建的目录为“build/intermediates/host“。

        **mkdir -p build/intermediates/host**

    3.  切换到“build/intermediates/host”目录，执行cmake命令生成编译文件。

        -   当运行环境操作系统架构为x86时，执行如下命令编译。

            **cd build/intermediates/host**

            **cmake ../../../src -DCMAKE\_CXX\_COMPILER=g++ -DCMAKE\_SKIP\_RPATH=TRUE**

        -   当运行环境操作系统架构为arm64时，执行以下命令进行交叉编译。

            **cd build/intermediates/host**

            **cmake ../../../src -DCMAKE\_CXX\_COMPILER=aarch64-linux-gnu-g++ -DCMAKE\_SKIP\_RPATH=TRUE**


        参数说明如下：
    
        -   “../../../src”表示CMakeLists.txt文件所在的目录，请根据实际目录层级修改。
        -   DCMAKE\_CXX\_COMPILER：编译应用程序所用的编译器。
        -   DCMAKE\_SKIP\_RPATH：**设置为TRUE**，代表不会将rpath信息（即NPU\_HOST\_LIB配置的路径）添加到编译生成的可执行文件中去。可执行文件运行时会自动搜索实际设置的LD\_LIBRARY\_PATH（“xxx/acllib/lib64”或“xxx/fwkacllib/lib64”）中的动态链接库。
    
    4.  执行如下命令，生成可执行文件。
    
        **make**
    
        会在工程目录的“run/out“目录下生成可执行文件**execute\_custom\_reshape\_op**。


4.  在硬件设备的Host侧执行单算子验证文件。
    1.  以运行用户（例如HwHiAiUser）拷贝开发环境中样例工程acl\_execute\_reshape/run/目录下的out文件夹到运行环境（硬件设备Host侧）任一目录，例如上传到/home/HwHiAiUser/HIAI\_PROJECTS/run\_reshape/目录下。

        **说明：**若您的开发环境即为硬件设备的Host侧，此拷贝操作可跳过。

    2.  在运行环境中执行execute\_custom\_reshape\_op文件，验证单算子模型文件。

        在/home/HwHiAiUser/HIAI\_PROJECTS/run\_reshape/out目录下执行如下命令：

        **chmod +x execute\_custom\_reshape\_op**

        **./execute\_custom\_reshape\_op**

        会有如下屏显信息（注意：由于数据生成脚本生成的数据文件是随机的，屏显显示的数据会有不同）：

        ```
        [INFO]  Open device[0] success
        [INFO]  Set input[0] from test_data/data/input_0.bin success.
        [INFO]  Input[0]:
                38        44        76        87        65         8        21         6        70
        [INFO]  Input[0] shape is:9
        [INFO]  Set input[1] from test_data/data/input_1.bin success.
        [INFO]  Input[1]:
                 3         3
        [INFO]  Input[1] shape is:2
        [INFO]  Copy input[0] success
        [INFO]  Copy input[1] success
        [INFO]  Create stream success
        [INFO]  Execute ReshapeCust success
        [INFO]  Synchronize stream success
        [INFO]  Copy output[0] success
        [INFO]  Output[0]:
                38        44        76        87        65         8        21         6        70
        [INFO]  Output[0] shape is:3 3
        [INFO]  Write output[0] success. output file = result_files/output_0.bin
        [INFO]  Run op success
        ```

        可见：输入数据1=输出数据，但是shape不相同，输出的shape和输入数据2的值相同，ReshapeCust算子验证结果正确。

        result\_files/output\_0.bin：输出数据的二进制文件。



