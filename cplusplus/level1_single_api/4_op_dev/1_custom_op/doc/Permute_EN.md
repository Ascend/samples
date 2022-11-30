# Permute<a name="EN-US_TOPIC_0302083089"></a>

## Overview<a name="section1924110242164"></a>

Implemented by using TIK, the Permute operator is used to permute the dimension order.

**Note: This sample supports only format conversion from NCHW to NHWC.**

## Operator Analysis<a name="section277922514179"></a>

1.  Specify the operator function.

    The Permute operator is used to permute the dimension order.

2.  Specify the input and output.
    -   The Permute operator has one input  **x**, one output  **y**, and one attribute  **order**  \(dimension order\). This sample supports the format conversion from NCHW to NHWC, that is, from \[0, 1, 2, 3\] to \[0, 2, 3, 1\].
    -   Both the operator input and output are of type  **float16**.
    -   The operator input supports all shapes. The output has the same shape as the inputs.
    -   The supported input format is  **NCHW**.

3.  Determine the operator development mode and the compute API.

    The Permute operator needs to compute different elements in different dimensions of a tensor, which is not supported by TBE DSL APIs or  TVM primitive  APIs . Therefore, the operator is implemented by using TIK.

    The core computation process is as follows:

    1.  Read data into the  Unified Buffer.
    2.  Call the  **vec\_trans\_scatter\(\)**  API to convert NCHW to NHWC.
    3.  Move data from the  Unified Buffer  to the  Global Memory.

4.  Specify the operator implementation file name, operator implementation function name, and  _OpType_.

    -   _OpType_  is named in upper camel case.
    -   The implementation file name and implementation function name of the operator are converted based on the following rules:
        -   Replace the first uppercase letter with a lowercase letter.

            Example: Abc -\> abc

        -   Replace each uppercase letter following lowercase letters with an underscore \(\_\) and a lowercase letter.

            Example: AbcDef -\> abc\_def

        -   Uppercase letters following a digit or an uppercase letter are regarded as a character string. If there is a lowercase letter after this string, replace the last uppercase letter in this string with an underscore \(\_\) and a lowercase letter, and replace the other uppercase letters with lowercase letters. If there is no lowercase letter after the string, directly replace the string with lowercase letters.

            Examples: ABCDef -\> abc\_def; Abc2DEf -\> abc2d\_ef; Abc2DEF -\> abc2def; ABC2dEF -\> abc2d\_ef



    To avoid messing up with the built-in Permute operators, define the  _OpType_  as  **PermuteTik**  and the implementation file name and implementation function name as  **permute\_tik**.


## Code Implementation<a name="section657125913571"></a>

-   Operator Implementation

    For details about the implementation code of the PermuteTik operator, see  [permute\_tik.py](../tbe/impl/permute_tik.py). The implementation logic of the calculation function is as follows:

    1.  Define the  **Permute**  class and initialize the parameters used for subsequent computation in the initialization function. The key is to compute the size of each input shape and memory size to be allocated in the  Global Memory. Obtain the actual physical space of the UB by calling  **tbe\_platform.cce\_conf.get\_soc\_spec\(tbe\_platform.cce\_conf.UB\_SIZE\)**. In subsequent steps, the calculated tiling parameters \(including the shape size and size of the UB\) will be used to configure the parameters of APIs such as  **data\_move**  and  **vec\_trans\_scatter**. Separate the tiling logic from the operator compute logic to implement shape generalization of the operator. In this way, data with different shapes can be computed in the same compute logic. You only need to change the tiling parameters to optimize the number of movement and compute process times, thereby achieving generalization and high performance.

        ```
        class Permute:
            """
            Function: store permute parameters  and compute permute
            """
        
            def __init__(self, input_dict):
                ...
            def get_shape_info(self):
                ...
            ...
            ...
            def permute_compute(self):
                ...
        ```

    2.  There are three scenarios based on the shape:
        -   Scenario 1: C=1 or H\*W=1. Conversion is not required.

            Directly compute the tiling parameters based on the input data size. If multiple AI Cores are required, enable double buffering and AI Core parallelism using the  **for\_range**  loop to tile input data, achieving efficient computation. In addition, UB tensors must be defined in AI Core parallelism loop to avoid conflicts during build.

        -   Other scenarios: C ≥ 16 and C < 16.

            The preceding two scenarios have the same tiling strategy: perform tiling along the N axis to process data on multiple AI Cores. The only difference lies on the remainder data processing logic during format conversion.

            **get\_block\_num\_and\_loop\_cycle\(\)**  is used to determine whether to enable AI Core parallelism and double buffering and the number of loop cycles in each AI Core if enabled.

            In the scenario C ≥ 16,  **compute\_c\_ge\_16\(\)**  processes the H and W axes as if they are one axis. The C axis is preferentially tiled to ensure that there is abundant memory for data of the C axis.

            In the scenario C < 16,  **compute\_c\_lt\_16\(\)**  processes the H and W axes as if they are one axis \(H\*W\). When the memory for H\*W data is insufficient, the H\*W data needs to be tiled.



-   Operator Prototype Definition

    For the definition of the PermuteTik prototype, see  [permute\_tik.h](../op_proto/permute_tik.h)  and  [permute\_tik.cc](../op_proto/permute_tik.cc).

-   Operator Information Library

    For details, see the  **permute\_tik.ini**  file in the corresponding chip version directory  [tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core).

-   Operator Plug-in

    Parse and map the operator whose type is Permute in the original Caffe model to the PermuteTik operator that adapts to the Ascend AI Processor. For details about the complete code, see  [caffe\_permute\_tik\_plugin.cc](../framework/caffe_plugin/caffe_permute_tik_plugin.cc).


## Supported SoCs<a name="section13382182116471"></a>

If the operator information library file of the corresponding chip version exists in the  [tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)  directory, the operator supports the corresponding chip version.

