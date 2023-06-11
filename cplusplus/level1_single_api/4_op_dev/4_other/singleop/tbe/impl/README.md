# BatchNorm实现样例

## 功能描述

此样例通过TIK方式实现了BatchNorm算子，BatchNorm算子是对输入进行归一化处理，x<sub>norm</sub>=（x−μ）/σ，其中， μ和 σ是计算的均值和方差。

此样例针对不同的shape范围，制定不同的Tiling策略，从而形成多个算子实现文件，再将算子实现文件编译成二进制文件（.o）。后续可参考[BatchNorm](https://github.com/Ascend/samples/tree/master/cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_batchnorm)运行sample执行对应shape的算子。

## 环境要求

-   操作系统及架构：CentOS x86系统、CentOS aarch64系统、Ubuntu 18.04 x86_64
-   python及依赖的库：Python3.7.*x*（3.7.0~3.7.11）、Python3.8.*x*（3.8.0~3.8.11）
-   已完成昇腾AI软件栈在开发环境、运行环境上的部署。

## 算子编译

编译BatchNorm算子，生成算子二进制文件（\*.o）及算子描述文件（\*.json）。

1. 以HwHiAiUser（运行用户）登录开发环境。

2. 设置环境变量。

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
      #设置昇腾AI处理器型号，此处以Ascend310为例，请根据实际情况替换。
      export SOC_VERSION=*Ascend310*
      ```

      Python3.7.5安装路径请根据实际情况进行替换，您也可以将以上命令写入~/.bashrc文件中，然后执行source ~/.bashrc命令使其立即生效。

3. 单算子编译。

   **python3.7.5 batch\_norm\_tilingmode1.py**

   **python3.7.5 batch\_norm\_tilingmode2.py**

   **python3.7.5 batch\_norm\_tilingmode3.py**

4. 编译完成后，在当前目录下生成kernel\_meta文件夹，包含算子二进制文件（\*.o）及算子描述文件（\*.json）。

