# Conv2d算子运行验证<a name="ZH-CN_TOPIC_0302083387"></a>

## 功能描述<a name="section1421916179418"></a>

该样例实现了对[自定义算子Conv2dTik](../../1_custom_op/doc/Conv2d.md)的功能验证，通过将自定义算子转换为单算子离线模型文件，然后通过AscendCL加载单算子模型文件进行运行。

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
│               └── conv2d_tik_op.json    // 算子描述文件，用于构造单算子模型文件
│           ├── data
│               └── check_out.py    // 生成期望结果并与实际结果进行比对的脚本
│               └── generate_conv2d.py    // 生成测试数据的脚本
├── src
│   ├── CMakeLists.txt    // 编译规则文件
│   ├── common.cpp         // 公共函数，读取二进制文件函数的实现文件  
│   ├── main.cpp    // 将单算子编译为om文件并加载om文件执行，此文件中包含算子的相关配置，若验证其他单算子，基于此文件修改
│   ├── operator_desc.cpp     // 构造算子的输入与输出描述
│   ├── op_runner.cpp   // 单算子编译与运行函数实现文件
```

## 环境要求<a name="zh-cn_topic_0230709958_section1256019267915"></a>

-   操作系统及架构：CentOS x86\_64、CentOS aarch64、Ubuntu 18.04 x86\_64
-   版本：20.2
-   编译器：
    -   Ascend 310 EP/Ascend 710编译器：g++
    -   Atlas 200 DK编译器：aarch64-linux-gnu-g++

-   芯片：Ascend310、Ascend710
-   python及依赖的库：python3.7.5
-   已完成昇腾AI软件栈的部署。
-   已参考[custom\_op](../../1_custom_op)完成自定义算子的编译部署。

## 配置环境变量<a name="section053142383519"></a>

-   Ascend 310 EP/Ascend 710
    1.  开发环境上，设置生成单算子离线模型的环境变量。

        环境变量配置示例如下：

        ```
        export install_path=$HOME/Ascend/ascend-toolkit/latest
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        ```

        install\_path为开发套件包Ascend-cann-toolkit的安装路径。

    2.  开发环境上，设置环境变量，配置AscendCL单算子验证程序编译依赖的头文件与库文件路径。

        编译脚本会按环境变量指向的路径查找编译依赖的头文件和库文件，请将$HOME/Ascend/ascend-toolkit/latest替换为开发套件包Ascend-cann-toolkit的安装路径。

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
        使用ACLlib组件安装路径下“lib64/stub”目录下的*.so库，是为了编译基于AscendCL接口的代码逻辑时，不依赖其它组件的任何*.so库。
        编译通过后，在Host上运行应用时，通过配置环境变量，应用会链接到Host上“$HOME/Ascend/nnrt/latest/acllib/lib64”目录下的*.so库，运行时会自动链接到依赖其它组件的*.so库。
        ```

    3.  运行环境上，设置运行应用时依赖AscendCL库文件的环境变量。

        如下为设置环境变量的示例，请将$HOME/Ascend/nnrt/latest替换为Ascend-cann-nnrt包的实际安装路径。

        ```
        export LD_LIBRARY_PATH=$HOME/Ascend/nnrt/latest/acllib/lib64
        ```


-   Atlas 200 DK
    1.  开发环境上，设置生成单算子离线模型的环境变量。

        环境变量配置示例如下：

        ```
        export install_path=$HOME/Ascend/ascend-toolkit/latest
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        ```

        install\_path为开发套件包Ascend-cann-toolkit的安装路径。

    2.  开发环境上，设置环境变量，配置AscendCL单算子验证程序编译依赖的头文件与库文件路径。

        如下为设置环境变量的示例，请将$HOME/Ascend/ascend-toolkit/latest替换为开发套件包Ascend-cann-toolkit的安装路径。

        ```
        export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
        export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/arm64-linux/acllib/lib64/stub
        ```

        ```
        说明：
        使用ACLlib组件安装路径下“lib64/stub”目录下的*.so库，是为了编译基于AscendCL接口的代码逻辑时，不依赖其它组件的任何*.so库。
        编译通过后，在板端环境上运行应用时，通过配置环境变量，应用会链接到板端环境上“$HOME/Ascend/acllib/lib64”目录下的*.so库，运行时会自动链接到依赖其它组件的*.so库。
        ```

        运行环境上AscendCL运行时依赖的环境变量LD\_LIBRARY\_PATH在制卡时已配置，此处无需单独配置。



## 编译运行（Ascend 310 EP/Ascend 710）<a name="section13498193914019"></a>

1.  生成Conv2dTik算子的单算子离线模型文件。
    1.  以运行用户（例如HwHiAiUser）登录开发环境，并进入样例工程的“acl\_execute\_conv2d/run/out“目录。
    2.  在out目录下执行如下命令，生成单算子模型文件。

        **atc --singleop=test\_data/config/conv2d\_tik\_op.json  --soc\_version=_$\{soc\_version\} _  --output=op\_models**

        其中：

        -   singleop：算子描述的json文件。
        -   soc\_version：昇腾AI处理器的型号，请根据实际情况替换。

            可从ATC安装路径下的“atc/data/platform\_config”目录下查看支持的昇腾AI处理器的类型，对应“\*.ini”文件的名字即为\{soc\_version\}。如果用户根据上述方法仍旧无法确定具体使用的$\{soc\_version\}，则：

            1.  单击如下手册中的链接并进入该手册，[CANN Ascend-DMI工具用户指南](https://support.huawei.com/enterprise/zh/ascend-computing/atlas-data-center-solution-pid-251167910?category=operation-maintenance)。
            2.  完成“使用工具\>使用前准备“，然后进入“使用工具\>设备实时状态查询“章节。
            3.  使用相关命令查看芯片的详细信息，例如使用**ascend-dmi -i -dt**命令查看芯片的详细信息，返回信息中“Chip Name“对应取值即为具体使用的$\{soc\_version\}。

        -   --output=op\_models：代表生成的模型文件存储在当前目录下的op\_models文件夹下。

        模型转换成功后，会生成如下文件：

        在当前目录的op\_models目录下生成单算子的模型文件**0\_Conv2DTik\_1\_0\_8\_512\_7\_7\_1\_0\_512\_512\_3\_3\_1\_0\_8\_512\_7\_7.om**，命名规范为：序号+opType + 输入的描述\(dateType\_format\_shape\)+输出的描述。

        dataType以及format对应枚举值请从ATC组件所在目录下的“atc/include/graph/types.h”文件中查看，枚举值从0开始依次递增。

        **说明：**模型转换时，会优先去查找自定义算子库去匹配模型文件中的算子。


2.  生成测试数据。

    进入样例工程目录的run/out/test\_data/data目录下，执行如下命令：

    **python3.7.5 generate\_conv2d.py**

    会在当前目录下生成数据文件input\_0.bin与input\_1.bin，用于进行Conv2dTik算子的验证。

3.  编译样例工程，生成单算子验证可执行文件。
    1.  切换到样例工程根目录acl\_execute\_conv2d，然后在样例工程根目录下执行如下命令创建目录用于存放编译文件，例如，创建的目录为“build/intermediates/host“。

        **mkdir -p build/intermediates/host**

    2.  切换到“build/intermediates/host”目录，执行cmake命令生成编译文件。

        **cd build/intermediates/host**

        **cmake ../../../src -DCMAKE\_CXX\_COMPILER=g++ -DCMAKE\_SKIP\_RPATH=TRUE**

        -   “../../../src”表示CMakeLists.txt文件所在的目录，请根据实际目录层级修改。
        -   DCMAKE\_CXX\_COMPILER：编译此应用程序所用的编译器。
        -   DCMAKE\_SKIP\_RPATH：**请设置为TRUE**，代表不会将rpath信息（即NPU\_HOST\_LIB配置的路径）添加到编译生成的可执行文件中去。

            可执行文件运行时会自动搜索实际设置的LD\_LIBRARY\_PATH（“xxx/acllib/lib64”或“xxx/fwkacllib/lib64”）中的动态链接库。


    3.  执行如下命令，生成可执行文件。

        **make**

        会在工程目录的“run/out“目录下生成可执行文件**execute\_conv2d\_op**。


4.  在硬件设备的Host侧执行单算子验证文件。
    1.  以运行用户（例如HwHiAiUser）拷贝开发环境中样例工程acl\_execute\_conv2d/run/目录下的out文件夹到运行环境（硬件设备Host侧）任一目录，例如上传到/home/HwHiAiUser/HIAI\_PROJECTS/run\_conv2d/目录下。

        **说明：**若您的开发环境即为硬件设备的Host侧，此拷贝操作可跳过。

    2.  在运行环境中执行execute\_conv2d\_op文件，验证单算子模型文件。

        在/home/HwHiAiUser/HIAI\_PROJECTS/run\_conv2d/out目录下执行如下命令：

        **chmod +x execute\_conv2d\_op**

        **./execute\_conv2d\_op**

        执行完成后，会屏显打印输入数据及输出数据，同时会生成结果二进制文件result\_files/output\_0.bin。

    3.  执行如下命令，将实际结果与预期结果进行比对。

        进入test\_data/data目录下，执行如下命令：

        **python3.7.5 check\_out.py**

        **说明：**执行此脚本需要确保环境中已安装TensorFlow 1.15版本。此脚本调用TensorFlow命令生成预期结果，然后将算子实际运行结果与预期结果进行比对。

        若回显如下，则代表实际运行结果与预期结果比对成功。

        ```
        Compared with the tf conv2d method, the result is correct.
        ```

        若回显如下，则代表实际运行结果与预期结果比对失败。

        ```
        Compared with the tf conv2d method, the result is wrong.
        ```



## 编译运行（Atlas 200 DK）<a name="section9789152720117"></a>

1.  生成Conv2dTik算子的单算子离线模型文件。
    1.  以运行用户（例如HwHiAiUser）登录开发环境，并进入样例工程的“acl\_execute\_conv2d/run/out“目录。
    2.  在out目录下执行如下命令，生成单算子模型文件。

        **atc --singleop=test\_data/config/conv2d\_tik\_op.json  --soc\_version=_$\{soc\_version\} _  --output=op\_models**

        其中：

        -   singleop：算子描述的json文件。
        -   soc\_version：昇腾AI处理器的型号，请根据实际情况替换。

            可从ATC安装路径下的“atc/data/platform\_config”目录下查看支持的昇腾AI处理器的类型，对应“\*.ini”文件的名字即为\{soc\_version\}。如果用户根据上述方法仍旧无法确定具体使用的$\{soc\_version\}，则：

            1.  单击如下手册中的链接并进入该手册，[CANN Ascend-DMI工具用户指南](https://support.huawei.com/enterprise/zh/ascend-computing/atlas-data-center-solution-pid-251167910?category=operation-maintenance)。
            2.  完成“使用工具\>使用前准备“，然后进入“使用工具\>设备实时状态查询“章节。
            3.  使用相关命令查看芯片的详细信息，例如使用**ascend-dmi -i -dt**命令查看芯片的详细信息，返回信息中“Chip Name“对应取值即为具体使用的$\{soc\_version\}。

        -   --output=op\_models：代表生成的模型文件存储在当前目录下的op\_models文件夹下。

        模型转换成功后，会生成如下文件：

        在当前目录的op\_models目录下生成单算子的模型文件**0\_Conv2DTik\_1\_0\_8\_512\_7\_7\_1\_0\_512\_512\_3\_3\_1\_0\_8\_512\_7\_7.om**，命名规范为：序号+opType + 输入的描述\(dateType\_format\_shape\)+输出的描述。

        dataType以及format对应枚举值请从ATC组件所在目录下的“atc/include/graph/types.h”文件中查看，枚举值从0开始依次递增。

        **说明：**模型转换时，会优先去查找自定义算子库去匹配模型文件中的算子。


2.  生成测试数据。

    进入样例工程目录的run/out/test\_data/data目录下，执行如下命令：

    **python3.7.5 generate\_conv2d.py**

    会在当前目录下生成数据文件input\_0.bin与input\_1.bin，用于进行Conv2dTik算子的验证。

3.  编译样例工程，生成单算子验证可执行文件。
    1.  切换到样例工程根目录acl\_execute\_conv2d，然后在样例工程根目录下执行如下命令创建目录用于存放编译文件，例如，创建的目录为“build/intermediates/host“。

        **mkdir -p build/intermediates/host**

    2.  切换到“build/intermediates/host”目录，执行cmake命令生成编译文件。

        **cd build/intermediates/host**

        **cmake ../../../src -DCMAKE\_CXX\_COMPILER=aarch64-linux-gnu-g++ -DCMAKE\_SKIP\_RPATH=TRUE**

        -   “../../../src”表示CMakeLists.txt文件所在的目录，请根据实际目录层级修改。
        -   DCMAKE\_CXX\_COMPILER：编译此应用程序所用的编译器。
        -   DCMAKE\_SKIP\_RPATH：**请设置为TRUE**，代表不会将rpath信息（即NPU\_HOST\_LIB配置的路径）添加到编译生成的可执行文件中去。

            可执行文件运行时会自动搜索实际设置的LD\_LIBRARY\_PATH（“xxx/acllib/lib64”）中的动态链接库。


    3.  执行如下命令，生成可执行文件。

        **make**

        会在工程目录的“run/out“目录下生成可执行文件**execute\_conv2d\_op**。


4.  在运行环境上执行单算子验证文件。
    1.  以运行用户（例如HwHiAiUser用户）拷贝开发环境中样例工程acl\_execute\_conv2d/run/目录下的out文件夹到板端环境任一目录，例如上传到/home/HwHiAiUser/HIAI\_PROJECTS/run\_conv2d/目录下。
    2.  在板端环境中执行execute\_conv2d\_op文件，验证单算子模型文件。

        在/home/HwHiAiUser/HIAI\_PROJECTS/run\_conv2d/out目录下执行如下命令：

        **chmod +x execute\_conv2d\_op**

        **./execute\_conv2d\_op**

        执行完成后，会屏显出输入数据及输出数据，同时会生成结果二进制文件result\_files/output\_0.bin。

    3.  test\_data/data目录中提供了**check\_out.py**脚本用于比对算子实际结果与调用TensorFlow命令生成的预期结果。

        开发者可将生成结果文件后的out文件夹拷贝到开发环境，然后在开发环境上进入test\_data/data目录执行如下操作进行结果校验。

        **python3.7.5 check\_out.py**

        **说明：**执行此脚本需要确保开发环境中已安装TensorFlow 1.15版本。此脚本调用TensorFlow命令生成预期结果，然后将算子实际运行结果与预期结果进行比对。

        若回显如下，则代表实际运行结果与预期结果比对成功。

        ```
        Compared with the tf conv2d method, the result is correct.
        ```

        若回显如下，则代表实际运行结果与预期结果比对失败。

        ```
        Compared with the tf conv2d method, the result is wrong.
        ```



