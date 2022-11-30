# Slice算子运行验证

## 功能描述

该样例实现了对自定义算子Slice的功能验证，通过将自定义算子转换为单算子离线模型文件，然后通过AscendCL加载单算子模型文件进行运行。

Slice算子功能：从tensor中提取切片。此操作从tensor“x”(第一个输入)中提取大小为“size”(第三个输入)的切片，从“begin”(第二个输入)指定的位置开始

说明：单算子模型文件的生成只依赖算子代码实现文件、算子原型定义、算子信息库，不依赖算子适配插件。

## 目录结构

```
├── op_models_fp16        // json转换为离线模型文件存放目录
├── src 
│   ├── slice_test.py    // 将单算子运行文件
│   ├── slice.json   // 自定义单算子json文件
```

## 环境要求

-   操作系统及架构：CentOS x86\_64、CentOS aarch64、Ubuntu 18.04 x86\_64、Ubuntu 18.04 aarch64、EulerOS x86、EulerOS aarch64
-   芯片：Ascend 310、Ascend 310P、Ascend 910
-   python及依赖的库：Python3.7.*x*（3.7.0 ~ 3.7.11）、Python3.8.*x*（3.8.0 ~ 3.8.11）
-   已完成昇腾AI软件栈的部署。
-   已完成自定义算子的编译部署。

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

     编译脚本会按环境变量指向的路径查找编译依赖的头文件和库文件，“$HOME/Ascend”请替换“Ascend-cann-toolkit”包的实际安装路径。

     - 当运行环境操作系统架构是x86时，配置示例如下所示：

       ```
       export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/x86_64-linux
       export NPU_HOST_LIB=$DDK_PATH/runtime/lib64/stub
       ```

     - 当运行环境操作系统架构时AArch64时，配置示例如下所示：

       ```
       export DDK_PATH=$HOME/Ascend/ascend-toolkit/latest/arm64-linux
       export NPU_HOST_LIB=$DDK_PATH/runtime/lib64/stub
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
  



## 运行样例

1.  生成Slice算子的单算子离线模型文件。
    1.  以运行用户（例如HwHiAiUser）登录开发环境，并进入样例工程的“src“目录。
    2.  在src目录下执行如下命令，生成单算子模型文件。

        **atc --singleop=./slice.json  --soc\_version=*Ascend310*  --output=../op_models_fp16**

        其中：

        -   singleop：算子描述的json文件。
        -   soc\_version：昇腾AI处理器的型号，此处以Ascend310为例，请根据实际情况替换。

        -   --output=../op_models_fp16：代表生成的模型文件存储在当前目录下的op_models_fp16文件夹下。

        模型转换成功后，会生成如下文件：

        在当前目录的op\_models目录下生成单算子的模型文件**0_Slice_1_2_600_3000_3_2_2_3_2_2_1_2_600_3000.om**，命名规范为：序号+opType + 输入的描述\(dateType\_format\_shape\)+输出的描述。

        dataType以及format对应枚举值请从CANN软件所在目录下的“include/graph/types.h”文件中查看，枚举值从0开始依次递增。

        **说明：**模型转换时，会优先去查找自定义算子库去匹配模型文件中的算子。


2. 运行样例工程，在硬件设备的Host侧执行单算子验证文件。
   1. 切换到样例工程根目录下的src文件夹

   2. 执行python3 slice_test.py

   3. 会有如下屏显信息（注意：由于输入是脚本中提前写好的，可自行修改输入进行验证）：

      ```
      count: [1800000]
      //count代表着输出的结构中有多少个0，来验证是否与输入对应(具体数据参考slice_test.py脚本)
      ```
