# 样例使用指导<a name="ZH-CN_TOPIC_0302914394"></a>

## 功能描述<a name="section5991635141815"></a>

本样例为IR模型构建样例，支持通过以下几种方式构建Graph，并生成适配昇腾AI处理器的离线模型，用户可任选其一。
               
-   通过算子原型构建Graph
-   将TensorFlow原始模型解析为Graph
-   将Caffe原始模型解析为Graph

## 目录结构<a name="section766832317011"></a>

```
├── src
│   ├──main.cpp           //实现文件 
├── Makefile              //编译脚本 
├── data         
│   ├──data_generate.py   // 通过算子原型构建Graph时，用于生成Graph所需要的数据信息，例如权重、偏置等数据
│   ├──tensorflow_generate.py  // 将TensorFlow原始模型解析为Graph时，用于生成.pb格式的TensorFlow模型
│   ├──caffe_generate.py  // 将Caffe原始模型解析为Graph时，用于生成.pbtxt格式的Caffe模型与.caffemodel格式的权重文件     
```

## 环境要求<a name="section3833348101215"></a>

-   操作系统及架构：CentOS x86系统、CentOS aarch64系统、Euleros x86系统、Euleros aarch64系统
-   编译器：g++
-   芯片：all
-   python及依赖的库：python3.7.5
-   已完成昇腾AI软件栈在开发环境上的部署
-   如果采用模型解析方式构图，请在当前环境安装tensorflow1.15.0和caffe

## 准备构图数据<a name="section48724517295"></a>

-   如果用户需要通过算子原型构建Graph，请执行以下操作准备构图数据：
    1.  在**data**目录执行数据生成脚本，**python3.7.5  data_generate.py**
    2.  执行结束后，在**data**目录下生成.bin格式的数据。后续模型构建时会从该文件读取权重、偏置等数据。

-   如果用户需要将TensorFlow原始模型解析为Graph，请执行以下操作准备构图数据：
    1.  在**data**目录执行tensorflow原始模型生成脚本，**python3.7.5  tensorflow_generate.py**
    2.  执行结束后，在**data**目录下生成.pb格式的模型文件。后续将原始模型解析时会使用该tensorflow模型，名称为tf_test.pb。

-   如果用户需要将Caffe原始模型解析为Graph，请执行以下操作准备构图数据：
    1.  在**data**目录执行caffe原始模型生成脚本，**python3.7.5  caffe_generate.py**
    2.  执行结束后，在**data**目录下生成.pbtxt格式的模型文件与.caffemodel格式的权重文件。后续将caffe原始模型与权重文件解析时会使用两者，名称分别为caffe_test.pbtxt与caffe_test.caffemodel。


## 程序编译<a name="section6697627144813"></a>

1. 根据实际情况修改**Makefile**文件中的如下信息。

   - ASCEND_PATH：指定到ATC或FwkACLlib的安装目录。

   - INCLUDES：需要包含的头文件，对于本示例，无需修改。如果是用户自行开发的代码，当需要添加头文件时，在示例下方直接增加行即可，注意不要删除原有项目。如果网络中有自定义算子，请增加自定义算子的原型定义头文文件。

   - LIBS：需要链接的库，对于本示例，无需修改。如果是用户自行开发的代码，当需要添加链接库时，在示例下方直接增加行即可，注意不要删除原有项目。

     >禁止链接软件包中的其他so，否则后续升级可能会导致兼容性问题。

2. 执行如下命令进行编译。

   如果安装的是ATC组件，依次执行**make clean**和**make ir_build**。

   如果安装的是FwkACLlib组件，依次执行**make clean**和**make fwk_ir_build**。

3. 编译结束后，在**out**目录下生成可执行文件**ir_build**或者**fwk_ir_build。**

## 程序运行<a name="section1843713353512"></a>

1. 配置环境变量。

   如果通过ATC组件进行离线模型编译，需要配置如下环境变量：

   ```
   export install_path=/home/HwHiAiUser/Ascend/ascend-toolkit/latest  # ATC组件安装路径，请根据实际修改
   export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
   export LD_LIBRARY_PATH=${install_path}/atc/lib64:$LD_LIBRARY_PATH
   export PYTHONPATH=${install_path}/atc/python/site-packages:$PYTHONPATH
   export ASCEND_OPP_PATH=${install_path}/opp
   ```

   如果通过FwkACLlib组件进行离线模型编译，需要配置如下环境变量：

   ```
   export install_path=/home/HwHiAiUser/Ascend/ascend-toolkit/latest   # FwkACLlib组件安装路径，请根据实际修改
   export PATH=${install_path}/fwkacllib/ccec_compiler/bin:${install_path}/fwkacllib/bin:$PATH
   export LD_LIBRARY_PATH=${install_path}/fwkacllib/lib64:$LD_LIBRARY_PATH
   export PYTHONPATH=${install_path}/fwkacllib/python/site-packages:$PYTHONPATH
   export ASCEND_OPP_PATH=${install_path}/opp
   ```

2. 在**out**目录下执行可执行文件。

   - 如果用户采用算子原型构图方式，请执行如下命令：

     如果安装了ATC组件，请执行：**./ir_build  _${soc_version}_  gen**

     如果安装了FwkACLlib组件，请执行：**./fwk_ir_build_ ${soc_version}_  gen**

     _${soc_version}_：昇腾AI处理器的版本，可从“${install_path}/atc/data/platform_config”或者“${install_path}/fwkacllib/data/platform_config”查看，".ini"文件的文件名（将大写字母转换为小写）即为对应的_${soc_version}_。如果用户根据上述方法仍旧无法确定具体使用的${soc_version}，则：

     1.  单击如下手册中的链接并进入该手册，[CANN Ascend-DMI工具用户指南](https://support.huawei.com/enterprise/zh/ascend-computing/atlas-data-center-solution-pid-251167910?category=operation-maintenance)。
     2.  完成“使用工具>使用前准备“，然后进入“使用工具>设备实时状态查询“章节。
     3.  使用相关命令查看芯片的详细信息，例如使用**ascend-dmi -i -dt**命令查看芯片的详细信息，返回信息中“Chip Name“对应取值（将大写字母转换为小写）即为具体使用的_${soc_version}_。

     编译成功提示：

     ```
     ========== Generate Graph1 Success!========== 
     Build Model1 SUCCESS!
     Save Offline Model1 SUCCESS!
     ```

   - 如果用户采用将TensorFlow原始模型解析为Graph的构图方式，请执行如下命令：

     如果安装了ATC组件，请执行：**./ir_build  _${soc_version}_  tf**

     如果安装了FwkACLlib组件，请执行：**./fwk_ir_build_ ${soc_version}_  tf**

     编译成功提示：

     ```
     ========== Generate graph from tensorflow origin model success.========== 
     Build Model1 SUCCESS!
     Save Offline Model1 SUCCESS!
     ```

   - 如果用户采用将Caffe原始模型解析为Graph的构图方式，请执行如下命令：

     如果安装了ATC组件，请执行：**./ir_build  _${soc_version}_  caffe**

     如果安装了FwkACLlib组件，请执行：**./fwk_ir_build_ ${soc_version}_  caffe**

     编译成功提示：

     ```
     ========== Generate graph from caffe origin model success.========== 
     Build Model1 SUCCESS!
     Save Offline Model1 SUCCESS!
     ```

3. 检查执行结果。

   在**out**目录下生成离线模型文件。
