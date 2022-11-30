# 特征检索功能验证<a name="ZH-CN_TOPIC_0302083215"></a>

## 功能描述<a name="section1421916179418"></a>

该样例实现了对特征检索的功能验证，生成随机底库，然后随机生成特征数据进行特征检索。

## 目录结构<a name="section8733528154320"></a>

```
├── inc                           // 头文件目录
│   ├── common.h                  // 声明公共方法类，用于生成随机特征数据相关接口定义
│   ├── fv_resource.h             // 特征检索相关资源管理胡声明文件，包含资源初始化，去初始化
│   ├── fv_search.h               // 特征检索运行相关信息声明文件，包含生成底库，特征查询，底库删除等
├── run                           // 执行需要的文件存放目录
│   ├── out    // 执行需要的可执行文件存放目录
├── src
│   ├── CMakeLists.txt    // 编译规则文件
│   ├── common.cpp         // 公共函数，用于生成随机特征数据相关接口定义 
│   ├── main.cpp    // 运行主程序代码，负责总体运行流程
│   ├── fv_resource.cpp     // 资源管理函数实现文件
│   ├── fv_search.cpp   // 运行函数实现文件
```

## 环境要求<a name="zh-cn_topic_0230709958_section1256019267915"></a>

-   操作系统及架构：CentOS x86\_64、CentOS aarch64、Ubuntu 18.04 x86\_64、EulerOS x86、EulerOS aarch64
-   芯片：Ascend 310P
-   已完成昇腾AI软件栈的部署。

## 配置环境变量<a name="section053142383519"></a>

-   Ascend 310P
    1.  开发环境上，设置环境变量，配置AscendCL单算子验证程序编译依赖的头文件与库文件路径。

        编译脚本会按环境变量指向的路径查找编译依赖的头文件和库文件，“$HOME/Ascend”请替换“Ascend-cann-toolkit”包的实际安装路径。

        -   当运行环境操作系统架构是x86时，配置示例如下所示：

            ```
            export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux
            export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux/runtime/lib64/stub
            ```

        -   当运行环境操作系统架构时arm64时，配置示例如下所示：

            ```
            export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
            export NPU_HOST_LIB=$HOME/Ascend/ascend-toolkit/latest/arm64-linux/runtime/lib64/stub
            ```


        ```
        说明：
        使用ACLlib组件安装路径下“lib64/stub”目录下的*.so库，是为了编译基于AscendCL接口的代码逻辑时，不依赖其它组件的任何*.so库。编译通过后，在Host上运行应用时，会根据“LD_LIBRARY_PATH”环境变量链接到“runtime/lib64”目录下的*.so库，并自动链接到其他组件依赖的*.so库。
        ```
    
    2.  运行环境上，设置运行应用时依赖AscendCL库文件的环境变量。
    
        如下为设置环境变量的示例，请将$HOME/Ascend/nnrt/latest替换为Ascend-cann-nnrt包的实际安装路径。
    
        ```
        export LD_LIBRARY_PATH=$HOME/Ascend/nnrt/latest/runtime/lib64
        ```

## 编译运行（Ascend 310P）<a name="section170442411445"></a>

1.  进入样例工程的目录。
    1.  以运行用户（例如HwHiAiUser）登录开发环境，并进入样例工程的目录。

2.  编译样例工程，生成验证可执行文件。
    1.  切换到样例工程根目录，然后在样例工程根目录下执行如下命令创建目录用于存放编译文件，例如，创建的目录为“build/intermediates/host“。

        **mkdir -p build/intermediates/host**

    2.  切换到“build/intermediates/host”目录，执行cmake命令生成编译文件。

        -   当开发环境与运行环境操作系统架构相同时，执行如下命令编译。

            **cd build/intermediates/host**

            **cmake ../../../src -DCMAKE\_CXX\_COMPILER=g++ -DCMAKE\_SKIP\_RPATH=TRUE**

        -   当开发环境与运行环境操作系统架构不同时，需要使用交叉编译。

            例如，当开发环境为X86架构，运行环境为AArch64架构时，执行以下命令进行交叉编译。

            **cd build/intermediates/host**
            
            **cmake ../../../src -DCMAKE\_CXX\_COMPILER=aarch64-linux-gnu-g++ -DCMAKE\_SKIP\_RPATH=TRUE**


        参数说明如下：
    
        -   “../../../src”表示CMakeLists.txt文件所在的目录，请根据实际目录层级修改。
        -   DCMAKE\_CXX\_COMPILER：编译应用程序所用的编译器。
        -   DCMAKE\_SKIP\_RPATH：**设置为TRUE**，代表不会将rpath信息（即NPU\_HOST\_LIB配置的路径）添加到编译生成的可执行文件中去。可执行文件运行时会自动搜索实际设置的LD\_LIBRARY\_PATH（“xxx/runtime/lib64”或“xxx/compiler/lib64”）中的动态链接库。
    
    4.  执行如下命令，生成可执行文件。
    
        **make**
    
        会在工程目录的“run/out“目录下生成可执行文件**execute\_fv\_search**。


3.  在硬件设备的Host侧执行验证文件。
    1.  以运行用户（例如HwHiAiUser）拷贝开发环境中样例工程execute\_fv\_search/run/目录下的out文件夹到运行环境（硬件设备Host侧）任一目录，例如上传到/home/HwHiAiUser/HIAI\_PROJECTS/run\_add/目录下。

        **说明：**若您的开发环境即为硬件设备的Host侧，此拷贝操作可跳过。

    2.  在运行环境中执行execute\_fv\_search文件。

        在run/out目录下执行如下命令：

        **chmod +x execute\_fv\_search**

        **./execute\_fv\_search**

        会有如下屏显信息（注意：由于数据生成脚本生成的数据文件是随机的，屏显显示的数据会有不同）：

        ```
        [INFO]  Search Result:
        [INFO]  ============================================================================
        [INFO]  Search Index[0] top[0]: id0 = 1, id1 = 1, offset = 338, distance = 2.08
        [INFO]  Search Index[0] top[1]: id0 = 0, id1 = 0, offset = 338, distance = 2.08
        [INFO]  Search Index[0] top[2]: id0 = 1, id1 = 1, offset = 471, distance = 2.20
        [INFO]  Search Index[0] top[3]: id0 = 0, id1 = 0, offset = 471, distance = 2.20
        [INFO]  Search Index[0] top[4]: id0 = 0, id1 = 0, offset = 1471, distance = 2.20
        [INFO]  ============================================================================
        [INFO]  feature search success!!
        ```

        可见输出结果按找匹配度进行排序。


