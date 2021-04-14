# Readme\_zh<a name="ZH-CN_TOPIC_0295405599"></a>

## 功能描述<a name="section1681015358439"></a>

基于AscendCL提供的C语言接口，该样例可实现Host到Device的内存异步拷贝功能。

## 原理介绍<a name="section11805164454319"></a>

样例代码的主要逻辑如下：

1.  AscendCL初始化与资源申请
    1.  调用aclInit接口初始化AscendCL配置。
    2.  按顺序依次调用aclrtSetDevice、aclrtCreateContext、aclrtSetCurrentContext、aclrtCreateStream接口设置用于计算的Device、Context、Stream。
    3.  调用aclrtMallocHost、aclrtMalloc接口分别申请Host、Device的内存。

2.  执行Host到Device的内存异步拷贝

    调用aclrtMemcpyAsync接口实现内存异步拷贝。

3.  资源释放与AscendCL去初始化。
    1.  调用aclrtFreeHost、aclrtFree接口分别释放Host、Device的内存。
    2.  按顺序依次调用aclrtDestroyStream、aclrtDestroyContext、aclrtResetDevice接口释放Context、Stream、Device资源。
    3.  调用aclFinalize接口实现AscendCL去初始化。


## 目录结构（可选）<a name="section0835133122514"></a>

如果样例只有一个cpp文件，就不需要介绍目录结构，如果有多个cpp文件，则需要介绍目录介绍，举例如下：

```
├acl_resnet50

├── data
│   ├── dog1_1024_683.jpg            //测试数据
│   ├── dog2_1024_683.jpg            //测试数据

├── inc
│   ├── model_process.h               //声明模型处理相关函数的头文件
│   ├── sample_process.h              //声明资源初始化/销毁相关函数的头文件                   
│   ├── utils.h                       //声明公共函数（例如：文件读取函数）的头文件

├── script
│   ├── transferPic.py               //将*.jpg转换为*.bin，同时将图片从1024*683的分辨率缩放为224*224

├── src
│   ├── acl.json         //系统初始化的配置文件
│   ├── CMakeLists.txt         //编译脚本
│   ├── main.cpp               //主函数，图片分类功能的实现文件
│   ├── model_process.cpp      //模型处理相关函数的实现文件
│   ├── sample_process.cpp     //资源初始化/销毁相关函数的实现文件                                          
│   ├── utils.cpp              //公共函数（例如：文件读取函数）的实现文件

├── .project     //工程信息文件，包含工程类型、工程描述、运行目标设备类型等
├── CMakeLists.txt    //编译脚本，调用src目录下的CMakeLists文件
```

## 环境要求<a name="section146273362218"></a>

根据实际sample依赖的环境来修改。

-   操作系统及架构：CentOS x86系统、CentOS aarch64系统
-   编译器：g++
-   芯片：Ascend310
-   python及依赖的库：python3.7.5、Pillow库（如果需要python脚本生成测试数据，就写明对python的要求，没有的话，就删除这一条）

## 样例运行<a name="section19331755174320"></a>

1.  模型转换。（可选，只有整网模型推理才需要这个步骤）

    XXX场景下的模型转换请参见XXXX文档（给超链接，专门有文档讲各场景下如何模型转换）

2.  准备测试数据。（可选，单算子执行需要测试数据或整网推理需要输入图片数据时才需要这个步骤）

    如果需要执行脚本生成测试数据，就在此处补充脚本的用法，例如：

    从XXX路径获取脚本后，执行transferPic.py脚本，将\*.jpg转换为\*.bin，同时将图片从1024\*683的分辨率缩放为224\*224。在“acl\_resnet50/data“目录下生成2个\*.bin文件。

    ```
    python3.7.5 ../script/transferPic.py
    ```

    如果执行脚本报错“ModuleNotFoundError: No module named 'PIL'”，则表示缺少Pillow库，请使用**pip3.7.5 install Pillow --user**命令安装Pillow库。

3.  编译代码。

    示例命令如下，请根据实际环境上\*.cpp文件所在的路径、可执行文件的名称及存放可执行文件的路径、头文件所在路径、库文件所在路径替换。

    ```
    g++ -DENABLE_DVPP_INTERFACE  ./Async_Memcpy_Host2Device.cpp -o ./Async_Memcpy_Host2Device -I/usr/local/Ascend/ascend-toolkit/latest/acllib_linux.x86_64/include -I/usr/include/  -L/usr/local/Ascend/ascend-toolkit/latest/acllib_linux.x86_64/lib64/stub -lascendcl -lstdc++
    ```

4.  切换到可执行文件所在的目录，运行应用。

    命令示例如下，可执行文件的名称请根据实际情况替换。

    ```
    ./Async_Memcpy_Host2Device 
    ```

    运行结果示例如下：

    ```
    XXXXXXXXXXXXXXXXXXXXXX
    XXXXXXXXXXXXXXXXXXXXXX
    XXXXXXXXXXXXXXXXXXXXXX
    ```

    运行应用后，可在XXXXXX路径下查看详细日志，日志关键字如下：（此处的关键字可以跟主要原理处介绍的关键步骤对应上，便于理解）

    -   XXXX：表示开始初始化AscendCL
    -   XXXX：表示开始申请内存
    -   XXXX：表示开始内存拷贝
    -   XXXX：表示拷贝成功
    -   XXXX


