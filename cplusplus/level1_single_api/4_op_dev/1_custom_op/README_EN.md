# README

## Introduction

This sample contains code samples for TBE custom operator, AI CPU custom operator, and TensorFlow scope fusion pattern development. In addition, the corresponding build scripts are provided. Developers can add their own custom operator implementations based on this sample and then build the project to obtain a custom operator package \(OPP\).

## Directory Structure

The directory of a Caffe or TensorFlow custom operator sample project is organized as follows:

```
├── CMakeLists.txt // CMakeList.txt of the operator project
├── README.md       
├── custom.proto    // Original prototext definition file of a Caffe operator
├── build.sh       //  Entry script for building a project
├── cpukernel      // Directory of AI CPU operator implementation and information library files
│   ├── CMakeLists.txt
│   ├── impl    // Directory of operator implementation files
│   │      ├── xx.cc
│   │      ├── xx.h
│   ├── op_info_cfg   // Directory of operator information library files
│   │      ├── aicpu_kernel
│                ├── xx.ini     // Operator information library file
│   ├── testcase   
│       ├── tf_test  // Directory of the TensorFlow operator test file. The code in this directory can run only on the Ascend 910 AI Processor.
│           ├── {Operator Name}
│              ├── tf_xx.py          // Operator test code
├── framework      // Directory for storing the operator plug-in implementation file
│   ├── CMakeLists.txt
│   ├── caffe_plugin    // Directory of the Caffe operator plug-in implementation code and the CMakeList file
│       ├── CMakeLists.txt 
│       ├── xx_plugin.cpp 
│   ├── tf_plugin    // Directory of the TensorFlow operator plug-in implementation code and the CMakeList file
│       ├── CMakeLists.txt 
│       ├── xx_plugin.cpp 
│   ├── tf_scope_fusion_pass    // Directory of the scope fusion pattern implementation code and the CMakeList file
│       └── xx_pass.h      // Fusion pattern header file
│       └── xx_pass.cpp    // Fusion pattern implementation
│       └── CMakeLists.txt
├── op_proto     // Directory of the operator prototype definition file and the CMakeList file
│   ├── xx.h
│   ├── xx.cpp
│   ├── CMakeLists.txt   // CMakeList.txt of the operator IR definition file, called by CMakeList.txt of the operator project
├── tbe 
│   ├── CMakeLists.txt   
│   ├── impl    // Directory of operator implementation files
│   │      ├── xx.py
│   │      ├── __init__.py      // Python package identification file
│   ├── op_info_cfg   // Directory of operator information library files
│       └── ai_core
│           ├── ${SoC Version}           // Ascend AI Processor model
│               ├── xx.ini
│   ├── testcase   
│       ├── tf_test  // Directory of the TensorFlow-based operator test file. The code in this directory can run only on the Ascend 910 AI processor.
│           ├── op_name                      // Code for verifying a single-operator on network
│              ├── tf_xx.py
├── cmake 
│   ├── config.cmake
│   ├── util
│       ├── makeself       // Directory of build-related common files
│       ├── parse_ini_to_json.py       // Script used to convert the operator information definition file (.ini) to the information library (.json)
│       ├── gen_ops_filter.sh          // File used to generate supported TensorFlow NPU operators
├── scripts     // Scripts used for custom operator project packaging
├── tools
```

## Sample Overview

-   TBE custom operator samples
    -   Add. For details, see  [Add](doc/Add_EN.md).
    -   LeakyRelu. For details, see  [LeakyRelu](doc/LeakyRelu_EN.md).
    -   ScatterNdAdd. For details, see  [ScatterNdAdd](doc/ScatterNdAdd_EN.md).
    -   Conv2d. For details, see  [Conv2d](doc/Conv2d_EN.md).
    -   Matmul. For details, see  [Matmul](doc/Matmul_EN.md).
    -   Permute. For details, see  [Permute](doc/Permute_EN.md).
    -   Upsample. For details, see  [Upsample](doc/Upsample_EN.md).
    -   LSTM. For details, see  [LSTM](doc/LSTM_EN.md).

-   AI CPU custom operator samples
    -   ReshapeCust. For details, see  [Reshape](doc/Reshape_EN.md).
    -   UniqueCust. For details, see  [Unique](doc/Unique_EN.md).

-   Scope fusion pattern samples

    This sample contains the following fusion pattern samples. For details, see  [tf\_scope\_fusion\_pass](framework/tf_scope_fusion_pass).

    -   **decode\_bbox\_v2\_scope\_fusion\_pass**: This fusion pattern is an example of fusion in the many-to-one scenario, which is used to fuse all small operators under Decode Scope into the DecodeBboxV2 operator. Decode Scope includes two Exp operators, four Mul operators, four Sub operators, a multiple of two RealDiv operators, two Unpack operators, one Pack operator, and three Transpose operators. Softmax operators are not included.
    -   **decode\_bbox\_v2\_multi\_pass**: This fusion pattern is an example of fusion in the many-to-many scenario, which is used to fuse all small operators under Decode Scope into a combination of DecodeBboxV2 and Identity operators. Decode Scope includes two Exp operators, four Mul operators, four Sub operators, a multiple of two RealDiv operators, two Unpack operators, one Pack operator, and three Transpose operators. Softmax operators are not included.
    -   **scope\_batchmulticlass\_nms\_pass**: This fusion pattern is an example of fusion in the many-to-one scenario, which is used to fuse all small operators under Batchmulticlass\_nms Scope into the Batchmulticlass\_nms operator. Batchmulticlass\_nms Scope includes one NonMaxSuppressionV2 operator, four Maximum operators, and eleven Merge operators. Transpose operators are not included.

    You can refer to the following operations to compile and deploy the scope fusion pattern with the operator project, and use the command parameter  **enable\_scope\_fusion\_passes**  to specify the fusion pattern during model conversion to verify whether the fusion pattern takes effect.


## Environment Requirements

-   OS and architecture: CentOS x86\_64, CentOS AArch64, Ubuntu 18.04 x86\_64, EulerOS x86, EulerOS AArch64
-   Version: 20.2
-   Python version and dependency library: Python 3.7.5
-   Ascend AI Software Stack deployed

## Operator Project Building

1.  In the  **custom.proto**  file of the sample project, add the definition of the Caffe custom operator.

    If you want to develop operators on other frameworks, skip this step. The  **custom.proto**  file definition is as follows:

    ```
    syntax = "proto2";
    package domi.caffe;
    message NetParameter {
      optional string name = 1; 
      // LayerParameter definition. Retain the default definition.
      repeated LayerParameter layer = 100;  // ID 100 so layers are printed last.
    
    }
    message LayerParameter {
      optional string name = 1;  // Definition for model parsing. Retain the default definition.
      optional string type = 2;  // Definition for model parsing. Retain the default definition.
    
      // Add the definition of the custom layer to LayerParameter. The ID must be unique in the built-in caffe.proto file and must be less than 5000.
      // The built-in caffe.proto file is stored in include/proto/caffe.proto in the ATC or FwkACLlib installation path.
      optional CustomTest1Parameter custom_test1_param = 1000;  
      optional CustomTest2Parameter custom_test2_param = 1001; 
    }
    
    // Add the definition of the custom layer.
    message CustomTest1Parameter {
        optional bool adj_x1 = 1 [default = false];
        optional bool adj_x2 = 2 [default = false];
    }
    // If no attribute in the custom operator needs to be parsed and mapped, leave the definition of message xxParameter empty, as shown in the following:
    message CustomTest2Parameter {
    }
    ```

    ```
    Before You Start
    You are advised to keep the parameter type (in bold and italic) unique and not the same as that defined in the built-in caffe.proto file in the atc/include/proto/caffe.proto or fwkacllib/include/proto/caffe.proto directory.
    The custom.proto file in the sample code contains the definition of the Caffe custom operator. If there are other custom operators, add their definitions to this file.
    ```

2.  Configure the environment variables and build settings in the  **build.sh**  script based on the actual development environment:

    Configure the following environment variables in the header of the  **build.sh**  script:

    -   ASCEND\_OPP\_PATH: specifies the installation path of the OPP component.

        Uncomment this environment variable and change it to the actual OPP installation path. For example:

        ```
        export ASCEND_OPP_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/opp
        ```

    -   ASCEND\_AICPU\_PATH: specifies the installation path of the AI CPU component.

        ```
        export ASCEND_AICPU_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest
        ```

        Uncomment this environment variable and change it to the actual AI CPU installation path.

    -   **ASCEND\_TENSOR\_COMPLIER\_INCLUDE**  specifies the path of the ATC header files.
        -   If it is not set, the default path  **/usr/local/Ascend/atc/include**  is used.
        -   If the actual ATC or FwkACLlib installation path is not the default path, uncomment this environment variable and change it to the actual path of the ATC or FwkACLlib header files. For example:

            ```
            export ASCEND_TENSOR_COMPLIER_INCLUDE=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/atc/include
            ```

            or

            ```
            export ASCEND_TENSOR_COMPLIER_INCLUDE=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/fwkacllib/include
            ```


    -   **TOOLCHAIN\_DIR**  specifies the directory of the HCC cross-compiler in the Toolkit component, without default value specified. This compiler is used to build the AI CPU operator in the sample.
    
        Uncomment this environment variable and change it to the actual path of the HCC compiler. For example:
    
        ```
        export TOOLCHAIN_DIR=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/toolkit/toolchain/hcc
        ```
    
    -   **AICPU\_KERNEL\_TARGET**  specifies the name of the dynamic library file generated after the implementation file of the AI CPU operator is built.
        -   If this environment variable is not configured, the default value  **cust\_aicpu\_kernels**  is used.
    
            **Note**: The  **opInfo.kernelSo**  field in the AI CPU operator information library \(**cpukernel/op\_info\_cfg/aicpu\_kernel/xx.ini**\) must be set to the name of the generated dynamic library file. For example, if the value of  **AICPU\_KERNEL\_TARGET**  is  **cust\_aicpu\_kernels**, the name of the generated dynamic library file is  **libcust\_aicpu\_kernels.so**.
    
        -   If you need to customize the name of the dynamic library file, uncomment the environment variable and modify the name as follows:
    
            ```
            export AICPU_KERNEL_TARGET=xxx
            ```


    -   **SYSTEM\_INFO**  specifies the form name of the built OPP. If the environment variable  **SYSTEM\_INFO**  is not set, the value is automatically obtained based on the OS type and architecture.
    
        If you need to customize the OPP name, uncomment the environment variable and change it as required. For example, if the OS version is CentOS and the architecture is AArch64, set the environment variable as follows:
    
        ```
        export SYSTEM_INFO=centos_aarch64
        ```
    
        The built OPP name is  **custom\_opp\__centos\_aarch64_.run**.


3.  Build the operator project.

    Run the following command in the directory of the custom operator sample project to build a custom operator project:

    **chmod + x build.sh**

    **./build.sh**

    After successful build, an OPP runfile  **custom\_opp\__<target OS\>\_<target architecture\>_.run**  is generated in the  **build\_out**  directory.

    **Note**: If you need to rebuild the project, run the  **./build.sh clean**  command to clear the build outputs.


## Operator Deployment

1.  Set the environment variables.

    Run the following command as the  **HwHiAiUser**  user to declare the environment variable on the current terminal. The variable becomes invalid when the shell terminal is closed.

    ```
    export ASCEND_OPP_PATH=/home/HwHiAiUser/Ascend/ascend-toolkit/latest/opp
    ```

    /home/HwHiAiUser/Ascend/ascend-toolkit/latest  specifies the installation path of the OPP component. Change it to the actual path.

2.  In the path of the built custom OPP, run the following command to install the custom OPP.

    **./custom\_opp\__<target os\>\_<target architecture\>_.run**

    After the command is executed successfully, the custom operator files generated after build are deployed in the custom directory of the OPP directory as follows:

    ```
    ├── opp      // OPP directory
    │   ├── op_impl
    │       ├── built-in
    │       ├── custom
    │           ├── ai_core
    │               ├── tbe
    │                   ├── config
    ${soc_version} // Ascend AI Processor model
    │                           ├── aic-${sos_version}-ops-info.json     // Custom TBE operator info library file
    │                   ├── custom_impl               // Custom TBE operator implementation code
    │                       ├── xx.py
    │           ├── vector_core   // Reserved directory, which can be ignored
    │           ├── cpu          // Directory of AI CPU custom operator implementation file and information library.
    │                ├── aicpu_kernel
    │                    ├── custom_impl
    │                        ├── libcust_aicpu_kernels.so   //Custom AI CPU operator implementation library file
    │                ├── config
    │                    ├── cust_aicpu_kernel.json         //Custom AI CPU operator information library file
    │   ├── framework
    │       ├── built-in
    │       ├── custom
    │           ├── caffe       // Directory of the plug-in library of the Caffe custom operator
    │               ├── libcust_caffe_parsers.so      // Operator plug-in library file, including the parsing functions of custom operator plug-in
    │               ├── custom.proto  // Original definition file of the custom operator. This file is read during the operator building to obtain the operator original definition.
    │           ├── tensorflow         // Directory for storing the plug-in library of the TensorFlow custom operator and the configuration file for configuring the NPU's support for the custom operator
    │               ├── libcust_tf_parsers.so         // Operator plug-in library file
    │               ├── libcust_tf_scope_fusion.so    // Scope fusion pattern definition library file
    │               ├── npu_supported_ops.json   // File applicable to Ascend 910
    │   ├── op_proto
    │       ├── built-in
    │       ├── custom
    │           ├── libcust_op_proto.so    // Prototype library file of the custom operator
    ```

    Note: You do not need to pay attention to other directories and files during the custom operator deployment.


## Operator ST Verification

Verify custom operator by referring to the sample in  [2\_verify\_op](../2_verify_op).

## Operator Network Verification

In the inference scenario, you can load the custom operator to generate an offline model file during model conversion and execute the model for model inference.

In the training scenario, you can train a model that contains the custom operator or construct a single-operator network containing only custom operators at the frontend of TensorFlow and perform verification.

This sample provides the following single-operator network verification samples in the training scenario:

-   TBE operators: Add and ScatterNdAdd. For details about the network verification file of a single-operator, see the  **xx.py**  file in the  **tbe/testcases/tf\_test/<OpType\>**  directory.
-   AI CPU operator: UniqueCust. For details about the network verification file of a single-operator, see the  **cpukernel/testcases/tf\_test/unique/tf\_unique.py**  file.

To execute a single-operator network test file, perform the following operations:

1.  Set the environment variables.

    Run the  **export**  command to declare environment variables on the current terminal. The environment variables become invalid when the shell terminal is closed.

    ```
    export install_path=/home/HwHiAiUser/Ascend/nnae/latest
    export ASCEND_DEVICE_ID=0
    # FwkACLlib Package Dependency
    export PYTHONPATH=${install_path}/fwkacllib/python/site-packages:$PYTHONPATH
    export LD_LIBRARY_PATH=${install_path}/fwkacllib/lib64:$LD_LIBRARY_PATH
    export PATH=${install_path}/fwkacllib/ccec_compiler/bin:${install_path}/fwkacllib/bin:$PATH
    # Driver Package Dependency
    export LD_LIBRARY_PATH=/usr/local/Ascend/driver/lib64/common/:/usr/local/Ascend/driver/lib64/driver:$LD_LIBRARY_PATH # Required only in the container-based training scenario
    # TFPlugin Package Dependency
    export PYTHONPATH=${install_path}/tfplugin/python/site-packages:$PYTHONPATH
    # OPP Package Dependency
    export ASCEND_OPP_PATH=${install_path}/opp
    ```

    -   **install\_path**  specifies the installation path of the FwkACLlib, Driver, and OPP components.
    -   **ASCEND\_DEVICE\_ID**  specifies the logical ID of the Ascend AI Processor.

        The value range is \[0,  _N_  – 1\] and the default value is  **0**.  _N_  specifies the device count in the physical machine, VM, or container.


2.  Run the single-operator network test script.
    1.  Go to the directory where the  **xx.py**  file is located.
    2.  Run the following command to execute the single-operator network test code:

        **python3.7.5  _xx.py_**

        -   TBE operators: Add and ScatterNdAdd

            After the network test script is executed, if the result is  **True**, indicating that the execution result on the Ascend AI Processor and the execution result on the CPU are consistent and correct.

            ```
            2020-03-06 11:03:45.383022: I tf_adapter/kernels/geop_npu.cc:304] [GEOP] GeOp Finalize success, geop_num_:0
            ====================================
            True
            ====================================
            ```

        -   AI CPU operator: UniqueCust

            If the following information is displayed, the operator is successfully executed.

            ```
            2020-11-22 03:15:36.998210: I tf_adapter/optimizers/add_input_pass.cc:96] job is localhost Skip the optimizer : AddInputPass.
            y: [1 2 4 7 8]
            ...
            2020-11-22 03:15:37.001986: I tf_adapter/optimizers/add_input_pass.cc:96] job is localhost Skip the optimizer : AddInputPass.
            idx: [0 0 1 2 2 2 3 4 4]
            ```

            Check the device OS log and determine that the custom AI CPU operator is invoked. The log path is  **/var/log/npu/slog/device-os-0/device-os-0\_xxx.log**.

            Search for the  **OpType**  of the operator:  **UniqueCust**. If information similar to the following is displayed, AI CPU custom operator is successfully executed.

            ```
            [INFO] AICPU(49034,aicpu_custom_scheduler):2020-11-22-08:29:45.301.762 [../../../../../../../../../cann/ops/built-in/aicpu/context/common/cpu_kernel_register.cc:74][AICPU][RunCpuKernel:74][tid:49046]:RunCpuKernel:UniqueCust begin.
            [INFO] CCECPU(49034,aicpu_custom_scheduler):2020-11-22-08:29:45.302.414 [../../../../../../../aicpu/aicpu_device/aicpu_cust_schedule/core/aicpusd_op_executor.cpp:126][ExecuteKernel][tid:49046][AICPU_SCHEDULE] Call custom api RunCpuKernel in /home/HwHiAiUser/cust_aicpu/176041/libcust_cpu_kernels.so success.
            [INFO] AICPU(49034,aicpu_custom_scheduler):2020-11-22-08:29:45.302.436 [../../../../../../../../aicpu/aicpu_device/utils/aicpu_sharder/aicpu_context.cc:70][SetOpname][tid:49046][AICPU_SHARDER] "set op name to null for thread[6]"
            [INFO] CCECPU(49034,aicpu_custom_scheduler):2020-11-22-08:29:45.302.448 [../../../../../../../aicpu/aicpu_device/aicpu_cust_schedule/core/aicpusd_event_process.cpp:91][ExecuteTsKernelTask][tid:49046][AICPU_SCHEDULE] Execute aicpu custom kernel success.
            ```




## Fusion Pattern Verification

In the inference scenario, you can use the command parameter  **enable\_scope\_fusion\_passes**  to specify the fusion pattern to take effect during model conversion to check whether the fusion pattern takes effect. The following provides the verification method based on the  [tf\_scope\_fusion\_pass](https://gitee.com/ascend/samples/tree/dev/cplusplus/level1_single_api/4_op_dev/1_custom_op/framework/tf_scope_fusion_pass)  fusion pattern:

1.  Construct a model containing Decode Scope. Before constructing a model, you need to install the following third-party software and library dependencies:

    -   tensorflow 1.15.0
    -   object\_detection r2.3.0, which can be downloaded from https://github.com/tensorflow/models/tree/r2.3.0

    Generate a model as follows:

    1.  Assume that the downloaded  **object\_detection**  library is stored in  **_<third\_party\_dir\>_**  and the working directory is  **_<work\_dir\>_**. Run the following commands in the working directory:

        mkdir -p official/vision/detection/utils

        cp -r  _<third\_party\_dir_\>/official/vision/detection/utils/object\_detection  _<work\_dir\>_/official/vision/detection/utils/

    2.  Create the  **gen\_decode\_bbox.py**  python script in  **_<work\_dir\>_**  as follows:

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

    3.  Run the script.

        **python3 gen\_decode\_bbox.py**

    4.  Find the generated .pb file in the  **_<work\_dir\>_/results/**  directory.

2.  Build and verify the scope fusion result.
    1.  Set the environment variables.

        ```
        export install_path=/home/HwHiAiUser/Ascend/ascend-toolkit/latest   # Installation path of the Ascend-cann-toolkit toolkit
        # PATH: Select the ATC or FwkACLlib installation path based on the site requirements.
        export PATH=${install_path}/atc/ccec_compiler/bin:${install_path}/atc/bin:$PATH
        # export PATH=${install_path}/fwkacllib/ccec_compiler/bin:${install_path}/fwkacllib/bin:$PATH
        export ASCEND_OPP_PATH=${install_path}/opp
        export DUMP_GE_GRAPH=3     # Set the graph dump mode. In this case, only the node relationships are dumped.
        export DUMP_GRAPH_LEVEL=3  # Set the graph to dump. Only the generated built graph is dumped.
        ```

    2.  During model conversion,  **--enable\_scope\_fusion\_passes**  is used to specify the pattern name.

        **atc --model=decode\_bbox\_v2.pb --framework=3 --output=mymodel --soc\_version=$\{soc\_version\} --enable\_scope\_fusion\_passes=DecodeBboxV2ScopeFusionPass --log=info**

        In the preceding command,  **soc\_version**  specifies the model of the Ascend AI Processor. Replace it with the actual version.

        It is the exact name of the .ini file in  **atc/data/platform\_config**  in the ATC installation path or  **fwkacllib/data/platform\_config**  in the FwkACLlib installation path. If you still cannot determine the  **_$\{sos\_version\}_**  using the preceding method, perform the following operations:

        1.  Click  [here](https://support.huawei.com/enterprise/en/ascend-computing/atlas-data-center-solution-pid-251167910?category=operation-maintenance)  to download  _CANN Ascend-DMI Tool User Guide_.
        2.  Complete the operations described in  **Tool Usage Guide**  \>  **Before You Start**, and then proceed to  **Tool Usage Guide**  \>  **Querying Device Real-Time Status**.
        3.  Run the related command to view the chip details. For example, in the output of the  **ascend-dmi -i -dt**  command,  **Chip Name**  field corresponds to  **_$\{sos\_version\}_**.


3.  Verify the result.
    1.  You can view the pattern setting in the INFO log.
        1.  Log for enabling a fusion pattern:

            ```
            SetPassEnableFlag:enable flag of scope fusion pass:DecodeBboxV2ScopeFusionPass is set with true.
            ```

        2.  Log for creating a fusion pattern:

            ```
            CreateScopeFusionPass:Create scope fusion pass, pass name = DecodeBboxV2ScopeFusionPass.
            ```

        3.  Log for matching a fusion pattern:

            ```
            Run:[scope_fusion] Scope pass DecodeBboxV2ScopeFusionPass's patterns is matched.
            ```


    2.  Find the fused operator DecodeBboxV2 and its attributes in the dumped  **ge\_proto\_xxxxx\_Build.txt**  graph.
    
        ![](https://images.gitee.com/uploads/images/2020/1223/171156_7faf65b4_5474059.png "zh-cn_image_0303338211.png")
    
        **NOTE:** 
        >This method also applies to many-to-many scope fusion verification.



