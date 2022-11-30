# BatchNorm算子运行验证<a name="ZH-CN_TOPIC_0303909994"></a>

## 功能描述<a name="section5991635141815"></a>

此样例实现了对自定义动态shape算子BatchNorm的功能验证，可通过AscendCL接口不同的Tiling策略执行对应的算子二进制文件，完成单算子调用。

## 环境要求<a name="section15875915982"></a>

-   操作系统及架构：CentOS x86系统、CentOS aarch64系统、Ubuntu 18.04 x86\_64
-   芯片：Ascend310、Ascend310P
-   python及依赖的库：Python3.7.*x*（3.7.0 ~ 3.7.11）、Python3.8.*x*（3.8.0 ~ 3.8.11）
-   已完成昇腾AI软件栈在开发环境、运行环境上的部署。
-   已参考[batch_norm](https://gitee.com/ascend/samples/tree/master/cplusplus/level1_single_api/4_op_dev/4_other/singleop/tbe/impl)完成自定义算子的编译部署。

## Ascend EP场景下运行验证（Host侧）<a name="section115611114292"></a>

**编译BatchNorm算子运行工程，生成单算子验证可执行文件。**

1. 以HwHiAiUser（运行用户）登录开发环境。

2. 设置环境变量，配置编译依赖的头文件与库文件路径。
   
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

3. 在样例工程根目录“acl\_execute\_batchnorm”下执行如下命令创建目录用于存放编译文件，例如，创建的目录为“build/intermediates/host“。

   **mkdir -p build/intermediates/host**

4. 切换到“build/intermediates/host”目录，执行cmake命令生成编译文件。

   “../../../src“表示CMakeLists.txt文件所在的目录，请根据实际目录层级修改。

   DCMAKE_SKIP_RPATH需设置为TRUE，代表不会将rpath信息（即NPU_HOST_LIB配置的路径）添加到编译生成的可执行文件中去，可执行文件运行时会自动搜索实际设置的LD_LIBRARY_PATH中的动态链接库。

   - 当开发环境与运行环境操作系统架构相同时，执行如下命令编译。

     **cd build/intermediates/host**

     **cmake ../../../src -DCMAKE\_CXX\_COMPILER=g++ -DCMAKE\_SKIP\_RPATH=TRUE**

   - 当开发环境与运行环境操作系统架构不同时，需要使用交叉编译。

     例如，当开发环境为X86架构，运行环境为AArch64架构时，执行以下命令进行交叉编译。

     **cd build/intermediates/host**

     **cmake ../../../src -DCMAKE\_CXX\_COMPILER=aarch64-linux-gnu-g++ -DCMAKE\_SKIP\_RPATH=TRUE**

5. 执行如下命令，生成可执行文件。

   **make**

   会在工程目录的“run/out“目录下生成可执行文件**execute\_batchnorm\_op**。

6. 切换到可执行文件所在目录“acl\_execute\_batchnorm/run/out”。

7. 将BatchNorm实现样例中生成的算子kernel\_meta文件夹拷贝到可执行文件所在目录。

**在硬件设备的Host侧执行单算子验证文件。**

1. 将算子运行需要的文件上传到运行环境。

   以运行用户将开发环境“acl\_execute\_batchnorm/run/out”目录下所有文件上传到运行环境（硬件设备Host侧）任一目录，若后续需要进行单算子profiling操作，建议上传到/home/HwHiAiUser/HIAI\_PROJECTS目录下，例如上传到/home/HwHiAiUser/HIAI\_PROJECTS/run\_batchnorm/目录下。

2. 设置CANN软件基础环境变量。

   ```
    . ${HOME}/Ascend/nnrt/set_env.sh
   ```

3. 在运行环境中执行**execute\_batchnorm\_op**文件。

   在/home/HwHiAiUser/HIAI\_PROJECTS/run\_batchnorm/out目录下执行如下命令：

   **chmod +x execute\_batchnorm\_op**

   **./execute\_batchnorm\_op** **argv\[1\] argv\[2\] argv\[3\] argv\[4\]**

   其中，argv\[1\] argv\[2\] argv\[3\] argv\[4\]分别表示输入的n c h w值，命令示例：

   **./execute\_batchnorm\_op 1 1024 8 8**

   会有如下屏显信息：

   ```
   [INFO] [SelectAclopBatchNorm] input shape is: 1 1024 8 8
   [INFO] [SelectAclopBatchNorm] tiling mode is: 1
   [INFO] [SelectAclopBatchNorm] param is:64,4,64,64,1024,3,0,0,0,0
   The result of BatchNorm is(showing no more than 16 elements):
   5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, Case passed!
   ```

   当前sample中的input为全1，均值为0.5，方差为0.1，计算出的output为（1-0.5）/0.1=5，最多输出16个数据。

