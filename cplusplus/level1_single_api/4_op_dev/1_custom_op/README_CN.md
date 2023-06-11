# README

## 概述

本样例包含了TBE自定义算子、AI CPU自定义算子以及TensorFlow Scope融合规则开发的代码样例，同时提供了对应的编译规则文件，开发者可以直接基于本样例追加自己的自定义算子实现代码，然后进行工程的编译即可获得自定义算子安装包。

## 目录结构

Caffe与TensorFlow共存的自定义算子样例工程的目录结构如下所示：

```
├── CMakeLists.txt //算子工程的CMakeList.txt
├── README.md       
├── custom.proto    // 原始框架为Caffe的自定义算子的proto定义文件    
├── build.sh       //  工程编译入口脚本 
├── cpukernel      // AI CPU算子实现文件及信息库文件所在目录
│   ├── CMakeLists.txt
│   ├── impl    //算子实现文件目录
│   │      ├── xx.cc
│   │      ├── xx.h
│   ├── op_info_cfg   //算子信息库文件目录
│   │      ├── aicpu_kernel
│                ├── xx.ini     //算子信息库文件
│   ├── testcase   
│       ├── tf_test  //基于TensorFlow框架的算子测试文件目录，此目录下代码仅支持在昇腾910 AI处理器上运行。
│           ├── {Operator Name}
│              ├── tf_xx.py          //算子测试代码
├── framework      //算子插件实现文件目录
│   ├── CMakeLists.txt
│   ├── caffe_plugin    //Caffe算子适配插件实现代码及CMakeList文件所在目录
│       ├── CMakeLists.txt 
│       ├── xx_plugin.cc 
│   ├── tf_plugin    //TensorFlow算子适配插件实现代码及CMakeList文件所在目录
│       ├── CMakeLists.txt 
│       ├── xx_plugin.cc 
│   ├── onnx_plugin    //ONNX算子适配插件实现代码及CMakeList文件所在目录
│       ├── CMakeLists.txt 
│       ├── xx_plugin.cc 
│   ├── tf_scope_fusion_pass    //Scope融合规则实现代码及CMakeList文件所在目录
│       └── xx_pass.h      //融合规则头文件
│       └── xx_pass.cc    //融合规则实现
│       └── CMakeLists.txt
├── op_proto     //算子原型定义文件及CMakeList文件所在目录   
│   ├── xx.h
│   ├── xx.cc
│   ├── CMakeLists.txt   //算子IR定义文件的CMakeList.txt，会被算子工程的CMakeList.txt调用
├── tbe 
│   ├── CmakeLists.txt   
│   ├── impl    //算子实现文件目录
│   │      ├── xx.py
│   │      ├── __init__.py      //Python中的package标识文件
│   ├── op_info_cfg   //算子信息库文件目录
│       └── ai_core
│           ├── ${Soc Version}           //昇腾AI处理器类型
│               ├── xx.ini
│   ├── testcase   
│       ├── tf_test  //基于TensorFlow的算子测试文件目录，此目录下代码仅支持在昇腾910 AI处理器上运行。
│           ├── op_name                      //单算子网络测试代码
│              ├── tf_xx.py
├── cmake 
│   ├── config.cmake
│   ├── util
│       ├── makeself       //编译相关公共文件存放目录
│       ├── parse_ini_to_json.py       // 将算子信息定义.ini文件转换为信息库json文件的脚本
│       ├── gen_ops_filter.sh          // 用于生成记录支持的TensorFlow的NPU算子文件
├── scripts     //自定义算子工程打包相关脚本
├── tools
```

## 样例介绍

-   TBE自定义算子样例
    -   Add算子，请参见[Add](doc/Add_CN.md)。
    -   LeakyRelu算子，请参见[LeakyRelu](doc/LeakyRelu_CN.md)。
    -   ScatterNdAdd算子，请参见[ScatterNdAdd](doc/ScatterNdAdd_CN.md)。
    -   Conv2d算子，请参见[Conv2d](doc/Conv2d_CN.md)。
    -   Matmul算子，请参见[Matmul](doc/Matmul_CN.md)。
    -   Permute算子，请参见[Permute](doc/Permute_CN.md)。
    -   Upsample算子，请参见[Upsample](doc/Upsample_CN.md)。
    -   Softmax算子，请参见[Softmax](doc/Softmax_CN.md)。
    -   BatchMultiClassNonMaxSuppression算子，请参见[BatchMultiClassNonMaxSuppression](doc/BatchMultiClassNonMaxSuppression_CN.md)。

-   AI CPU自定义算子样例
    -   ReshapeCust算子，请参见[Reshape](doc/Reshape_CN.md)。
    -   UniqueCust算子，请参见[Unique](doc/Unique_CN.md)。
    -   AddBlockCust算子，此算子支持分块并行计算，请参见[AddBlockCust](doc/AddBlockCust_CN.md)。

-   Scope融合规则样例

    Scope融合规则实现样例，请参见[tf_scope_fusion_pass](framework/tf_scope_fusion_pass)：

    - decode_bbox_v2_scope_fusion_pass为多对一场景融合示例，目标是将Decode Scope下的所有小算子融合为DecodeBboxV2算子。Decode Scope内包括2个Exp算子/4个Mul算子/4个Sub算子/2的倍数个RealDiv算子/2个Unpack算子/1个Pack算子/3个Transpose算子/不能包括Softmax算子。
    - decode_bbox_v2_multi_pass为多对多场景融合示例，目标是将Decode Scope下的所有小算子融合为DecodeBboxV2算子和Identity的算子组合。Decode Scope内包括2个Exp算子/4个Mul算子/4个Sub算子/2的倍数个RealDiv算子/2个Unpack算子/1个Pack算子/3个Transpose算子/不能包括Softmax算子。
    - scope_batchmulticlass_nms_pass多对一场景融合示例，目标是将Batchmulticlass_nms Scope下的所有小算子融合为Batchmulticlass_nms算子。Batchmulticlass_nms Scope内包括1个NonMaxSuppressionV2算子/4个Maximum算子/11个Merge算子/不能包括Transpose算子。
    
    Scope融合算子适配插件实现文件，请参见[tf_plugin](framework/tf_plugin)：
    
    - decode_bbox_v2_scope_fussion_plugin对应为decode_bbox_v2_scope_fusion_pass融合算子的适配插件实现文件。
-   将算子映射为子图（一对多映射）样例

    将第三方框架中的一个算子映射为多个CANN算子示例，对应的算子适配插件实现文件，请参见[onnx_plugin](framework/onnx_plugin)。


## 环境要求

-   操作系统及架构：CentOS x86_64、CentOS aarch64、Ubuntu 18.04 x86_64、Ubuntu 18.04 aarch64、EulerOS x86、EulerOS aarch64
-   python及依赖的库：Python3.7.x（3.7.0 ~ 3.7.11）、Python3.8.x（3.8.0 ~ 3.8.11）
-   已完成昇腾AI软件栈的部署。

## 配置CANN软件基础环境变量
-  CANN组合包提供进程级环境变量设置脚本，供用户在进程中引用，以自动完成环境变量设置。执行命令参考如下，以下示例均为非root用户默认安装路径，请以实际安装的CANN包及实际安装路径为准。
   
     ```
      # 安装toolkit包时配置
      . ${HOME}/Ascend/ascend-toolkit/set_env.sh 
        
      # 安装nnrt包时配置
      . ${HOME}/Ascend/nnrt/set_env.sh 
      
      # 安装nnae包时配置
      . ${HOME}/Ascend/nnae/set_env.sh 
      
      # 安装fwkplugin包时配置
      . /${HOME}/Ascend/fwkplugin/set_env.sh
      
     ```
    
-  算子编译依赖Python，以Python3.7.5为例，请以运行用户执行如下命令设置Python3.7.5的相关环境变量。
   
      ```
      #用于设置python3.7.5库文件路径
      export LD_LIBRARY_PATH=/usr/local/python3.7.5/lib:$LD_LIBRARY_PATH
      #如果用户环境存在多个python3版本，则指定使用python3.7.5版本
      export PATH=/usr/local/python3.7.5/bin:$PATH
      ```
    Python3.7.5安装路径请根据实际情况进行替换，您也可以将以上命令写入~/.bashrc文件中，然后执行source ~/.bashrc命令使其立即生效。

## 算子工程编译

1.  在样例工程的“custom.proto“文件中增加原始框架为Caffe的自定义算子的定义。

    若开发其他框架的算子，此步骤跳过，custom.proto文件如下所示：

    ```
    syntax = "proto2";
    package domi.caffe;
    message NetParameter {
      optional string name = 1; 
      // LayerParameter定义，保持默认，用户无需修改
      repeated LayerParameter layer = 100;  // ID 100 so layers are printed last.
    
    }
    message LayerParameter {
      optional string name = 1;  // 模型解析所需要定义，保持默认，用户无需修改。
      optional string type = 2;  // 模型解析所需要定义，保持默认，用户无需修改。
    
      // 在LayerParameter中添加自定义算子层的定义，ID需要保持唯一，取值原则为：不与内置caffe.proto中编号重复，且小于5000。
      // 内置的caffe.proto存储路径为ATC安装路径下的“include/proto/caffe.proto”。
      optional CustomTest1Parameter custom_test1_param = 1000;  
      optional CustomTest2Parameter custom_test2_param = 1001; 
    }
    
    // 增加自定义算子层的定义
    message CustomTest1Parameter {
        optional bool adj_x1 = 1 [default = false];
        optional bool adj_x2 = 2 [default = false];
    }
    // 若自定义算子中无属性需要进行解析映射，则message xxParameter定义保持空，如下所示：
    message CustomTest2Parameter {
    }
    ```

    ```
    须知：
    Parameter的类型（粗斜体部分）建议保持唯一，不与内置caffe.proto（“compiler/include/proto/caffe.proto”）定义重复。
    样例代码的custom.proto文件中已包含样例中样例中的自定义Caffe算子的定义，若有其他自定义算子，请基于此文件追加。
    ```
    
2. 修改build.sh脚本，根据实际开发环境信息修改相关环境变量配置。

   修改buid.sh脚本头部的如下环境变量。

   - ASCEND\_TENSOR\_COMPILER\_INCLUDE：CANN软件头文件所在路径。

     请取消此环境变量的注释，并修改为CANN软件头文件所在路径，例如：

     ```
     export ASCEND_TENSOR_COMPILER_INCLUDE=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/include
     ```

   -   TOOLCHAIN\_DIR：AI CPU算子使用的编译器路径，请取消此环境变量的注释，并按照下述描述修改。
       - 针对Ascend EP场景，请配置为HCC编译器所在路径，例如：

          ```
          export TOOLCHAIN_DIR=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/toolkit/toolchain/hcc
          ```
       - 针对Ascend RC场景（例如:Atlas 200 DK），请配置为g++交叉编译器所在bin文件夹的上级目录，例如，交叉编译器存储路径为“/usr/bin/aarch64-linux-gnu-g++”，则TOOLCHAIN_DIR配置如下：
          ```
          export TOOLCHAIN_DIR=/usr
          ```         

   -   AICPU\_KERNEL\_TARGET：AI CPU算子实现文件编译生成的动态库文件名称。
       -   若不配置此环境变量，使用默认值：cust\_aicpu\_kernels。

           **注意**：AI CPU算子信息库（cpukernel/op\_info\_cfg/aicpu\_kernel/xx.ini）中的“opInfo.kernelSo”字段需要配置为生成的动态库文件的名称，例如若“AICPU\_KERNEL\_TARGET”的值为“cust\_aicpu\_kernels”，则生成的动态库文件的名称为“libcust\_aicpu\_kernels.so”。
           
       -   若用户需要自定义此动态库文件的名字，请取消此环境变量的注释，并自行修改，例如：

           ```
           export AICPU_KERNEL_TARGET=xxx
           ```


   -    AICPU\_SOC\_VERSION：昇腾AI处理器的类型，请配置为AI CPU组件安装路径中“opp/built-in/op_impl/aicpu/aicpu_kernel/lib”路径下的文件夹名称，即“libcpu_kernels_context.a”与“libcpu_kernels_v1.0.1.so”所在文件夹的名称。
   -    vendor_name：标识自定义算子所属厂商的名称，默认值为“customize”。建议开发者自行指定所属厂商名称，避免和其他厂商提供的算子包冲突。当前TBE自定义算子工程中算子实现代码文件所在的目录名为impl，算子包部署后，为避免多厂商的算子实现python包名冲突，所在的目录名会修改为${vendor_name}_impl的格式。


3.  执行算子工程编译。

    - 若您只需要编译TBE算子，请在算子工程目录下执行如下命令。

      **chmod +x build.sh**

      **./build.sh -t**


    - 若您只需要编译AI CPU算子，请在算子工程目录下执行如下命令。

      **chmod +x build.sh**

      **./build.sh -c**

    - 若您既需要编译TBE算子，又需要编译AI CPU算子，请在算子工程目录下执行如下命令。

      **chmod +x build.sh**

      **./build.sh**

    编译成功后，会在当前目录下创建build\_out目录，并在build\_out目录下生成自定义算子安装包**custom\_opp\__<target os\>\_<target architecture\>_.run**。
    
    **说明：**

    -  若重新进行工程编译，请先执行./build.sh clean命令进行编译文件的清理。
    -  若您开发的自定义算子既包含TBE算子，又包含AI CPU算子，请选择同时编译，生成一个自定义算子安装包。因为当前版本，仅支持安装一个自定义算子安装包，后面安装的自定义算子包会覆盖之前安装的算子包。



## 算子部署

1.  训练场景下，您需要将算子工程编译生成的自定义算子安装包**custom\_opp\__<target os\>\_<target architecture\>_.run**以运行用户拷贝到运行环境任一路径，如果您的 开发环境即为运行环境，此操作可跳过；推理场景下无需执行此操作，自定义算子部署到开发环境的OPP算子库即可。

2.  在编译生成的自定义算子安装包所在路径下，执行如下命令，安装自定义算子包。

    **./custom\_opp\__<target os\>\_<target architecture\>_.run --install-path=\<path\>**

    --install-path为可选参数，用于指定自定义算子包的安装目录。支持指定绝对路径和相对路径，运行用户需要对指定的安装路径有可读写权限。下文描述中的<vendor_name>为算子工程编译时build.sh脚本中字段“vendor_name”的取值，默认为“customize”。
    -  默认安装场景，不配置--install-path参数，安装成功后会将编译生成的自定义算子相关文件部署到opp/vendors/<vendor_name>目录。
    -  指定目录安装场景，配置--install-path参数，安装成功后会将编译生成的自定义算子相关文件部署到\<path\>/<vendor_name>目录，并在\<path\>/<vendor_name>/bin目录下新增set_env.bash，写入当前自定义算子包相关的环境变量。

       **须知：**
       如果部署算子包时通过配置--install-path参数指定了算子包的安装目录，则在使用自定义算子前，需要执行source \<path\>/<vendor_name>/bin/set_env.bash命令，set_env.bash脚本中将自定义算子包的安装路径追加到环境变量ASCEND_CUSTOM_OPP_PATH中，使自定义算子在当前环境中生效。

    以默认安装场景为例，部署后目录结构示例如下所示：
    ```
    
    ├── opp    //算子库目录
    │   ├── vendors    //自定义算子所在目录
    │       ├── config.ini     // 自定义算子优先级配置文件
    │       ├── vendor_name1   // 存储对应厂商部署的自定义算子，此名字为编译自定义算子安装包时配置的vendor_name，若未配置，默认值为customize
    │           ├── op_impl
    │               ├── ai_core    // TBE自定义算子实现文件及算子信息库所在目录
    │                   ├── tbe      
    │                       ├── config
    │                           ├── ${soc_version}     //昇腾AI处理器类型
    │                               ├── aic-${soc_version}-ops-info.json     //TBE自定义算子信息库文件
    │                       ├── vendor_name1_impl               //TBE自定义算子实现代码文件
    │                           ├── xx.py
    │               ├── cpu          //AI CPU自定义算子实现库及算子信息库所在目录
    │                   ├── aicpu_kernel
    │                       ├── impl
    │                           ├── libcust_aicpu_kernels.so    //AI CPU自定义算子实现库文件
    │                   ├── config   
    │                       ├── cust_aicpu_kernel.json          //AI CPU自定义算子信息库文件
    │               ├── vector_core   //此目录预留，无需关注
    │           ├── framework
    │               ├── caffe       //存放Caffe框架的自定义算子插件库
    │                   ├── libcust_caffe_parsers.so      //算子插件库文件，包含了自定义算子的插件解析函数
    │                   ├── custom.proto  //自定义算子的原始定义，算子编译过程中会读取此文件自动解析算子原始定义
    │               ├── onnx       //存放ONNX框架的自定义算子插件库
    │                   ├── libcust_onnx_parsers.so      //算子插件库文件，包含了自定义算子的插件解析函数
    │               ├── tensorflow         //存放TensorFlow框架的自定义算子插件库及npu对相关自定义算子支持度的配置文件
    │                   ├── libcust_tf_parsers.so         //算子插件库文件
    │                   ├── libcust_tf_scope_fusion.so    //scope融合规则定义库文件
    │                   ├── npu_supported_ops.json   //Ascend 910场景下使用的文件
    │           ├── op_proto        //自定义算子原型库所在目录
    │               ├── libcust_op_proto.so   
    │       ├── vendor_name2   // 存储厂商vendor_name2部署的自定义算子
    ```
    注：其他目录与文件，开发者无需关注。

3.  配置自定义算子优先级。
    多算子包共存的情况下，若不同的算子包目录下存在相同OpType的自定义算子，则以优先级高的算子包目录下的算子为准。下面介绍如何配置算子包优先级：
    - 默认安装场景
      当“opp/vendors”目录下存在多个厂商的自定义算子时，您可通过配置“opp/vendors”目录下的“config.ini”文件，配置自定义算子包的优先级。

      “config.ini”文件的配置示例如下：
      ```
      load_priority=vendor_name1,vendor_name2,vendor_name3
      ```
      - “load_priority”：优先级配置序列的关键字，不允许修改。
      - “vendor_name1,vendor_name2,vendor_name3”：自定义算子厂商的优先级序列，按照优先级从高到低的顺序进行排列。
    - 指定目录安装场景
      指定目录安装场景下，如果需要多个自定义算子包同时生效，分别执行各算子包安装路径下的set_env.bash脚本即可。每次脚本执行都会将当前算子包的安装路径追加到ASCEND_CUSTOM_OPP_PATH环境变量的最前面。因此可以按照脚本执行顺序确定优先级：脚本执行顺序越靠后，算子包优先级越高。

      比如先执行source \<path\>/vendor_name1/bin/set_env.bash，后执行source \<path\>/vendor_name2/bin/set_env.bash，vendor_name2算子包的优先级高于vendor_name1。ASCEND_CUSTOM_OPP_PATH示例如下：
      
      ```
      ASCEND_CUSTOM_OPP_PATH=\<path\>/vendor_name2:\<path\>/vendor_name1：
      ```
    - 指定目录安装场景下安装的算子包优先级高于默认方式安装的算子包。


## 算子ST验证

请参见[2_verify_op](../2_verify_op)中的样例对自定义算子进行验证。

## 算子网络验证

推理场景下，用户可在使用ATC工具进行模型转换时加载自定义算子生成离线模型文件，然后执行模型推理时调用。

训练场景下，可以执行包含自定义算子的模型训练，也可以通过TensorFlow前端构造只包含自定义算子的单算子网络并运行验证。

本样例中提供了训练场景下如下单算子网络验证样例，以下样例需要在运行环境中执行：

TBE算子：Add、ScatterNdAdd，单算子网络验证文件可参见“tbe/testcases/tf\_test/_<OpType\>_”目录下的xx.py文件。

单算子网络测试文件执行步骤如下所示：

1.  设置环境变量。

    完成CANN软件基础环境变量配置后，还需要利用export命令，在当前终端下声明如下环境变量，关闭Shell终端失效。

    ```
    export ASCEND_DEVICE_ID=0
    ```

​        **ASCEND\_DEVICE\_ID**为昇腾AI处理器的逻辑ID。 取值范围\[0,N-1\]，默认为0。其中N为当前物理机/虚拟机/容器内的设备总数。


2.  运行单算子网络测试脚本。
    1.  进入xx.py文件所在目录。
    2.  执行如下命令执行单算子网络测试代码。

        **python3.7.5  _xx.py_**

        TBE算子：Add与ScatterNdAdd

            执行网络测试脚本后，结果为True，表示在昇腾AI处理器上运行结果与CPU上运行结果一致，运行结果正确。
            
            ```
            2020-03-06 11:03:45.383022: I tf_adapter/kernels/geop_npu.cc:304] [GEOP] GeOp Finalize success, geop_num_:0
            ====================================
            True
            ====================================
            ```



## 融合规则验证

推理场景下，用户可在使用ATC工具进行模型转换时，通过命令行参数**enable\_scope\_fusion\_passes**指定需要生效的融合规则，从而该验证该融合规则是否生效。下面针对xx的融合规则，给出验证方法：

1.  构造模型含有Decode Scope。构造模型前需要安装依赖的第三方软件和库：

    -   tensorflow 1.15.0
    -   object\_detection r2.3.0，下载链接：https://github.com/tensorflow/models/tree/r2.3.0

    生成模型的方法为：

    1.  假设下载的object\_detection库存放路径为 _<third\_party\_dir\>_，工作路径为  _<work\_dir\>_，在工作路径下复制库文件：

        mkdir -p official/vision/detection/utils

        cp -r  _<third\_party\_dir_\>/official/vision/detection/utils/object\_detection  _<work\_dir\>_/official/vision/detection/utils/

    2.  _<work\_dir\>_ 下创建python脚本gen\_decode\_bbox.py， 脚本内容参考：

        ```
        import tensorflow as tf
        import numpy as np
        from official.vision.detection.utils.object_detection import faster_rcnn_box_coder
        from official.vision.detection.utils.object_detection import box_list
        
        def get_operator(input_data, scales, output_data=None):
            input_data_0 = tf.cast(input_data[0], tf.float32)
            input_data_1 = tf.cast(input_data[1], tf.float32)
            anchors = box_list.BoxList(input_data_0)
            coder = faster_rcnn_box_coder.FasterRcnnBoxCoder(scale_factors=scales)
            boxes = coder.decode(input_data_1, anchors).get()
            boxes_fp16 = tf.cast(boxes, tf.float16)
            return boxes_fp16
        
        def main(unused_argv):
            scales = [2.0, 3.0, 4.0, 5.0];
            shape_params = (96, 4)
            dtype_params = np.float16
            x_data = np.random.uniform(-2, 2, size=shape_params).astype(dtype_params)
            y_data = np.random.uniform(-2, 2, size=shape_params).astype(dtype_params)
        
            tf.compat.v1.disable_eager_execution()
            x = tf.compat.v1.placeholder(dtype_params, shape=shape_params)
            y = tf.compat.v1.placeholder(dtype_params, shape=shape_params)
            input_data = [x, y]
            output = get_operator(input_data, scales)
            with tf.compat.v1.Session() as session:
                result = session.run(output, feed_dict={x: x_data, y: y_data})
                # save the model
                tf.io.write_graph(session.graph_def, 'results', 'decode_bbox_v2.pb', as_text=False)
            print('====== End of the generated file ======')
        
        if __name__ == "__main__":
            main(None)
        
        ```

    3.  执行脚本。

        **python3 gen\_decode\_bbox.py**

    4.  生成的pb文件位于  _<work\_dir\>_/results/_目录下。_

2.  下面我们通过ATC工具编译验证scope融合效果。
    1.  设置环境变量。

        完成CANN软件基础环境变量配置后，还需要额外配置如下环境变量。
        
        ```
        export DUMP_GE_GRAPH=3     # 控制dump图的内容多少，配置为3表示仅dump显示节点关系的精简版图文件
        export DUMP_GRAPH_LEVEL=3  # 控制dump图的个数，配置为3表示仅dump最后的生成的build图
        ```
        
    2. 进行模型转换时，此时通过--enable\_scope\_fusion\_passes指定规则名称。

       **atc --model=decode\_bbox\_v2.pb --framework=3 --output=mymodel --soc\_version=$\{soc\_version\} --enable\_scope\_fusion\_passes=DecodeBboxV2ScopeFusionPass --log=info**

       其中，soc\_version：昇腾AI处理器的型号，请根据实际情况替换。

       可从ATC安装路径下的“compiler/data/platform\_config”目录下查看支持的昇腾AI处理器的类型，对应“\*.ini”文件的名字即为{soc\_version\}。

3.  结果验证。
    1.  在INFO日志中可以看到pass的设置情况：
        1.  使能融合规则的日志：

            ```
            SetPassEnableFlag:enable flag of scope fusion pass:DecodeBboxV2ScopeFusionPass is set with true.
            ```

        2.  创建融合规则的日志：

            ```
            CreateScopeFusionPass:Create scope fusion pass, pass name = DecodeBboxV2ScopeFusionPass.
            ```

        3.  匹配融合规则的日志：

            ```
            Run:[scope_fusion] Scope pass DecodeBboxV2ScopeFusionPass's patterns is matched.
            ```


    2.  从dump的图ge\_proto\_xxxxx\_Build.txt中可以看到融合后的算子DecodeBboxV2及相关属性：
    
       ![输入图片说明](https://images.gitee.com/uploads/images/2020/1223/171156_7faf65b4_5474059.png "zh-cn_image_0303338211.png")
    
        说明：
        scope多对多融合示例也可以使用此模型与方法进行验证。
## 将算子映射为子图（一对多映射）验证

用户可使用ATC模型转换工具对算子映射为子图的效果进行验证。下面给出验证方法：

1.  构造包含AddN算子的onnx模型。构造模型前需要安装依赖的第三方软件onnx 1.12.0。

    生成模型的方法为：

    1.  假设用户工作路径为  _<work\_dir\>_，在工作路径下创建python脚本gen\_addn.py， 脚本内容参考：


        ```
        import os
        import numpy as np
        import onnx

        def gen_onnx():
            X = onnx.helper.make_tensor_value_info("X", onnx.TensorProto.FLOAT, [5])
            Y = onnx.helper.make_tensor_value_info("Y", onnx.TensorProto.FLOAT, [5])
            Z = onnx.helper.make_tensor_value_info("Z", onnx.TensorProto.FLOAT, [5])
            output = onnx.helper.make_tensor_value_info("output", onnx.TensorProto.FLOAT, [5])

            node0 = onnx.helper.make_node("AddN", inputs=["X", "Y", "Z"], outputs=["output"])

            inputs = [X, Y, Z]
            outputs = [output]

            graph_def = onnx.helper.make_graph(
                [node0],
                "addn_model",
                inputs,
                outputs
            )

            model_def = onnx.helper.make_model(graph_def)
            model_def.opset_import[0].version = 11
            onnx.save(model_def, "addn_model.onnx")
            print(model_def)
        
        if __name__ == "__main__":
            gen_onnx()
        ```

    3.  执行脚本，生成的onnx模型文件"addn_model.onnx"位于  _<work\_dir\>_目录下。

        **python3 gen\_addn.py**

2.  通过ATC模型转换功能验证算子映射子图效果。
    1.  设置环境变量。

        完成CANN软件基础环境变量配置后，还需要额外配置如下环境变量。
        
        ```
        export DUMP_GE_GRAPH=2     # 控制dump图的内容多少
        export DUMP_GRAPH_LEVEL=2  # 控制dump图的个数
        ```
        
    2. 进行模型转换。

       **atc --model=./addn_model.onnx --framework=5 --output=./addn --input_format=NCHW --soc\_version=$\{soc\_version\}**

       其中，soc\_version：昇腾AI处理器的型号，请根据实际情况替换。可从ATC安装路径下的“compiler/data/platform\_config”目录下查看支持的昇腾AI处理器的类型，对应“\*.ini”文件的名字即为{soc\_version\}。
       模型转换完成后会在执行atc命令的当前目录下生成一系列按"ge_onnx*.pbtxt"命名方式命名的文件。这些文件是基于ONNX的开源模型描述结构，可以使用Netron等可视化软件打开。

    3. 结果验证。

       ge\_onnx\_00000000\_graph\_0\_RunCustomPassBegin.pbtxt是ge获取到的经过parse处理的整张下沉图。使用Netron等可视化软件打开原始模型和 ge\_onnx\_00000000\_graph\_0\_RunCustomPassBegin.pbtxt可以看到算子映射子图的实际效果。


 