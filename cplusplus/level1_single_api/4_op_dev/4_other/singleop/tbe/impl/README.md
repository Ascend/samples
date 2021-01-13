# BatchNorm实现样例<a name="ZH-CN_TOPIC_0303868527"></a>

## 功能描述<a name="section5991635141815"></a>

此样例通过TIK方式实现了BatchNorm算子，BatchNorm算子是对输入进行归一化处理，x<sub>norm</sub>=（x−μ）/σ，其中， μ和 σ是计算的均值和方差。

此样例针对不同的shape范围，制定不同的Tiling策略，从而形成多个算子实现文件，再将算子实现文件编译成二进制文件（.o）。后续可参考[BatchNorm](https://gitee.com/ascend/samples/tree/dev/cplusplus/level1_single_api/4_op_dev/2_verify_op/acl_execute_batchnorm)运行sample执行对应shape的算子。

## 环境要求<a name="section3833348101215"></a>

-   操作系统及架构：CentOS x86系统、CentOS aarch64系统
-   python及依赖的库：python3.7.5
-   已完成昇腾AI软件栈在开发环境、运行环境上的部署。

## 算子编译<a name="section2501928153120"></a>

编译BatchNorm算子，生成算子二进制文件（\*.o）及算子描述文件（\*.json）。

1. 以HwHiAiUser（运行用户）登录开发环境。

2. 设置环境变量。

   利用export命令，在当前终端下声明环境变量，关闭Shell终端失效。

   ```
   export install_path=/home/HwHiAiUser/Ascend/ascend-toolkit/latest  
   export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
   export LD_LIBRARY_PATH=${install_path}/atc/lib64:$LD_LIBRARY_PATH
   export PYTHONPATH=${install_path}/atc/python/site-packages:$PYTHONPATH
   export ASCEND_OPP_PATH=${install_path}/opp
   ```

   请将install\_path替换为ATC的实际安装路径。

3. 单算子编译。

   **python3.7.5 batch\_norm\_tilingmode1.py**

   **python3.7.5 batch\_norm\_tilingmode2.py**

   **python3.7.5 batch\_norm\_tilingmode3.py**

4. 编译完成后，在当前目录下生成kernel\_meta文件夹，包含算子二进制文件（\*.o）及算子描述文件（\*.json）。

