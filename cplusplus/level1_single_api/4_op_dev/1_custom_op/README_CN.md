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
│       ├── xx_plugin.cpp 
│   ├── tf_plugin    //TensorFlow算子适配插件实现代码及CMakeList文件所在目录
│       ├── CMakeLists.txt 
│       ├── xx_plugin.cpp 
│   ├── tf_scope_fusion_pass    //Scope融合规则实现代码及CMakeList文件所在目录
│       └── xx_pass.h      //融合规则头文件
│       └── xx_pass.cpp    //融合规则实现
│       └── CMakeLists.txt
├── op_proto     //算子原型定义文件及CMakeList文件所在目录   
│   ├── xx.h
│   ├── xx.cpp
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
    -   LSTM算子，请参见[LSTM](doc/LSTM_CN.md)。
    -   BatchMultiClassNonMaxSuppression算子，请参见[BatchMultiClassNonMaxSuppression](doc/BatchMultiClassNonMaxSuppression_CN.md)。

-   AI CPU自定义算子样例
    -   ReshapeCust算子，请参见[Reshape](doc/Reshape_CN.md)。
    -   UniqueCust算子，请参见[Unique](doc/Unique_CN.md)。

-   Scope融合规则样例

    Scope融合规则实现样例，请参见[tf_scope_fusion_pass](framework/tf_scope_fusion_pass)：

    - decode_bbox_v2_scope_fusion_pass为多对一场景融合示例，目标是将Decode Scope下的所有小算子融合为DecodeBboxV2算子。Decode Scope内包括2个Exp算子/4个Mul算子/4个Sub算子/2的倍数个RealDiv算子/2个Unpack算子/1个Pack算子/3个Transpose算子/不能包括Softmax算子。
    - decode_bbox_v2_multi_pass为多对多场景融合示例，目标是将Decode Scope下的所有小算子融合为DecodeBboxV2算子和Identity的算子组合。Decode Scope内包括2个Exp算子/4个Mul算子/4个Sub算子/2的倍数个RealDiv算子/2个Unpack算子/1个Pack算子/3个Transpose算子/不能包括Softmax算子。
    - scope_batchmulticlass_nms_pass多对一场景融合示例，目标是将Batchmulticlass_nms Scope下的所有小算子融合为Batchmulticlass_nms算子。Batchmulticlass_nms Scope内包括1个NonMaxSuppressionV2算子/4个Maximum算子/11个Merge算子/不能包括Transpose算子。
    
    Scope融合算子适配插件实现文件，请参见[tf_plugin](framework/tf_plugin)：
    
    - decode_bbox_v2_scope_fussion_plugin对应为decode_bbox_v2_scope_fusion_pass融合算子的适配插件实现文件。


## 环境要求

-   操作系统及架构：CentOS x86_64、CentOS aarch64、Ubuntu 18.04 x86_64、EulerOS x86、EulerOS aarch64
-   版本：20.2
-   python及依赖的库：python3.7.5
-   已完成昇腾AI软件栈的部署。

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
      // 内置的caffe.proto存储路径为ATC或FwkACLlib安装路径下的“include/proto/caffe.proto”。
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
    Parameter的类型（粗斜体部分）建议保持唯一，不与内置caffe.proto（“atc/include/proto/caffe.proto”或者“fwkacllib/include/proto/caffe.proto”）定义重复。
    样例代码的custom.proto文件中已包含样例中样例中的自定义Caffe算子的定义，若有其他自定义算子，请基于此文件追加。
    ```

2.  修改build.sh脚本，根据实际开发环境信息修改相关环境变量配置。

    修改buid.sh脚本头部的如下环境变量。

    -   ASCEND\_OPP\_PATH：OPP组件的安装路径。

        请取消此环境变量的注释，并修改为实际的OPP组件的安装路径，例如：

        ```
        export ASCEND_OPP_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/opp

        ```
    -   ASCEND\_AICPU_PATH：AI CPU组件的安装路径。

        请取消此环境变量的注释，并修改为实际AI CPU组件的安装路径，例如：

        ```
        export ASCEND_AICPU_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest

        ```

    -   ASCEND\_TENSOR\_COMPLIER\_INCLUDE：ATC组件或者FwkACLlib组件的头文件所在路径。
        -   若不配置此环境变量，默认使用路径：“/usr/local/Ascend/atc/include”。
        -   若实际ATC或FwkACLlib安装路径不为默认路径，请取消此环境变量的注释，并修改为实际的ATC或FwkACLlib组件的头文件所在路径，例如：

            ```
            export ASCEND_TENSOR_COMPLIER_INCLUDE=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/atc/include
            ```

            或者

            ```
            export ASCEND_TENSOR_COMPLIER_INCLUDE=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/fwkacllib/include
            ```

    -   TOOLCHAIN\_DIR：Toolkit组件中HCC编译器所在路径，无默认值，此编译器用于对样例中的AI CPU算子进行编译。

        请取消此环境变量的注释，并修改为实际的HCC编译器所在，例如：

        ```
        export TOOLCHAIN_DIR=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/toolkit/toolchain/hcc
        ```

    -   AICPU\_KERNEL\_TARGET：AI CPU算子实现文件编译生成的动态库文件名称。
        -   若不配置此环境变量，使用默认值：cust\_aicpu\_kernels。

            **注意**：AI CPU算子信息库（cpukernel/op\_info\_cfg/aicpu\_kernel/xx.ini）中的“opInfo.kernelSo”字段需要配置为生成的动态库文件的名称，例如若“AICPU\_KERNEL\_TARGET”的值为“cust\_aicpu\_kernels”，则生成的动态库文件的名称为“libcust\_aicpu\_kernels.so”。
            
        -   若用户需要自定义此动态库文件的名字，请取消此环境变量的注释，并自行修改，例如：

            ```
            export AICPU_KERNEL_TARGET=xxx
            ```


    -   SYSTEM\_INFO：标志编译生成的算子包的形态的名称，若不设置SYSTEM\_INFO环境变量，则会自动根据操作系统类型及架构获取。
    
        若用户需要自定义生成的算子包形态名称，请取消此环境变量的注释，并自行修改，例如，若操作系统版本为CentOS，架构为aarch64，则可设置为：
    
        ```
        export SYSTEM_INFO=centos_aarch64
        ```
    
        则编译生成的算子包名称为custom\_opp\__centos\_aarch64_.run。


3.  执行算子工程编译。

    在自定义算子样例工程目录下执行如下操作进行自定义算子工程的编译。

    **chmod +x build.sh**

    **./build.sh**

    编译成功后，会在当前目录下创建build\_out目录，并在build\_out目录下生成自定义算子安装包**custom\_opp\__<target os\>\_<target architecture\>_.run**。
    
    **说明：**若重新进行工程编译，请先执行**./build.sh clean**命令进行编译文件的清理。


## 算子部署

1.  设置环境变量。

    以HwHiAiUser用户执行如下命令，在当前终端下声明环境变量，关闭Shell终端失效。

    ```
    export ASCEND_OPP_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/opp
    ```

    /home/HwHiAiUser/Ascend/ascend-toolkit/latest表示OPP组件安装路径，请根据实际路径修改。

2.  在编译生成的自定义算子安装包所在路径下，执行如下命令，安装自定义算子包。

    **./custom\_opp\__<target os\>\_<target architecture\>_.run**

    命令执行成功后，会将编译生成的自定义算子相关文件部署到opp对应目录下的custom路径下，部署后目录结构示例如下所示：

    ```
    ├── opp      //算子库目录
    │   ├── op_impl
    │       ├── built-in
    │       ├── custom
    │           ├── ai_core
    │                ├── tbe
    │                    ├── config
    │                        ├── ${soc_version}     //昇腾AI处理器类型
    │                            ├── aic-${soc_version}-ops-info.json     //TBE自定义算子信息库
    │                    ├── custom_impl               //TBE自定义算子实现代码文件
    │                        ├── xx.py
    │           ├── vector_core   //此目录预留，无需关注
    │           ├── cpu          //AI CPU自定义算子实现库及算子信息库所在目录
    │                ├── aicpu_kernel
    │                    ├── custom_impl
    │                        ├── libcust_aicpu_kernels.so   //AI CPU算子实现库文件
    │                ├── config
    │                    ├── cust_aicpu_kernel.json         //AI CPU算子信息库
    │   ├── framework
    │       ├── built-in
    │       ├── custom
    │           ├── caffe       //存放Caffe框架的自定义算子插件库
    │               ├── libcust_caffe_parsers.so      //算子插件库文件，包含了自定义算子的插件解析函数
    │               ├── custom.proto  //自定义算子的原始定义，算子编译过程中会读取此文件自动解析算子原始定义
    │           ├── tensorflow         //存放TensorFlow框架的自定义算子插件库及npu对相关自定义算子支持度的配置文件
    │               ├── libcust_tf_parsers.so         //算子插件库文件
    │               ├── libcust_tf_scope_fusion.so    //scope融合规则定义库文件
    │               ├── npu_supported_ops.json   //Ascend 910场景下使用的文件
    │   ├── op_proto
    │       ├── built-in
    │       ├── custom
    │           ├── libcust_op_proto.so    //自定义算子原型库文件
    ```

    注：其他目录与文件，自定义算子部署无需关注。


## 算子ST验证

请参见[2_verify_op](../2_verify_op)中的样例对自定义算子进行验证。

## 算子网络验证

推理场景下，用户可在使用ATC工具进行模型转换时加载自定义算子生成离线模型文件，然后执行模型推理时调用。

训练场景下，可以执行包含自定义算子的模型训练，也可以通过TensorFlow前端构造只包含自定义算子的单算子网络并运行验证。

本样例中提供了训练场景下如下单算子网络验证样例：

TBE算子：Add、ScatterNdAdd，单算子网络验证文件可参见“tbe/testcases/tf\_test/_<OpType\>_”目录下的xx.py文件。

单算子网络测试文件执行步骤如下所示：

1.  设置环境变量。

    利用export命令，在当前终端下声明环境变量，关闭Shell终端失效。

    ```
    export install_path=/home/HwHiAiUser/Ascend/nnae/latest
    export ASCEND_DEVICE_ID=0
    # FwkACLlib包依赖
    export PYTHONPATH=${install_path}/fwkacllib/python/site-packages:$PYTHONPATH
    export LD_LIBRARY_PATH=${install_path}/fwkacllib/lib64:$LD_LIBRARY_PATH
    export PATH=${install_path}/fwkacllib/ccec_compiler/bin:${install_path}/fwkacllib/bin:$PATH
    # Driver包依赖
    export LD_LIBRARY_PATH=/usr/local/Ascend/driver/lib64/common/:/usr/local/Ascend/driver/lib64/driver:$LD_LIBRARY_PATH # 仅容器训练场景配置
    # TFPlugin包依赖
    export PYTHONPATH=${install_path}/tfplugin/python/site-packages:$PYTHONPATH
    # OPP包依赖
    export ASCEND_OPP_PATH=${install_path}/opp
    ```


    -   **install\_path**为Fwkacllib组件、Driver组件及OPP组件的安装目录。
    -   **ASCEND\_DEVICE\_ID**为昇腾AI处理器的逻辑ID。
    
        取值范围\[0,N-1\]，默认为0。其中N为当前物理机/虚拟机/容器内的设备总数。


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

        ```
        export install_path=/home/HwHiAiUser/Ascend/ascend-toolkit/latest   # 开发套件包Ascend-cann-toolkit的安装路径  
        # PATH请根据实际安装包选择ATC或者FwkACLlib的安装路径
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        # export PATH=${install_path}/fwkacllib/ccec_compiler/bin:${install_path}/fwkacllib/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        export DUMP_GE_GRAPH=3     # 控制dump图的内容多少，配置为3表示仅dump显示节点关系的精简版图文件
        export DUMP_GRAPH_LEVEL=3  # 控制dump图的个数，配置为3表示仅dump最后的生成的build图
        ```

    2.  进行模型转换时，此时通过--enable\_scope\_fusion\_passes指定规则名称。

        **atc --model=decode\_bbox\_v2.pb --framework=3 --output=mymodel --soc\_version=$\{soc\_version\} --enable\_scope\_fusion\_passes=DecodeBboxV2ScopeFusionPass --log=info**

        其中，soc\_version：昇腾AI处理器的型号，请根据实际情况替换。

        可从ATC安装路径下的“atc/data/platform\_config”目录或FwkACLlib安装路径下的“fwkacllib/data/platform\_config”目录下查看支持的昇腾AI处理器的类型，对应“\*.ini”文件的名字即为{soc\_version\}。如果用户根据上述方法仍旧无法确定具体使用的$\{soc\_version\}，则：

        1.  单击如下手册中的链接并进入该手册，[CANN Ascend-DMI工具用户指南](https://support.huawei.com/enterprise/zh/ascend-computing/atlas-data-center-solution-pid-251167910?category=operation-maintenance)。
        2.  完成“使用工具\>使用前准备“，然后进入“使用工具\>设备实时状态查询“章节。
        3.  使用相关命令查看芯片的详细信息，例如使用**ascend-dmi -i -dt**命令查看芯片的详细信息，返回信息中“Chip Name“对应取值即为具体使用的 _$\{soc\_version\}_。


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



