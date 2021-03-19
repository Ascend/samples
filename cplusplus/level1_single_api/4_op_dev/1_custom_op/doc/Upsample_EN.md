# Upsample<a name="EN-US_TOPIC_0302083325"></a>

## Overview<a name="section882342317476"></a>

Implemented by using TIK, the Upsample operator is used to scale up the feature map in a neural network.

## Operator Analysis<a name="section3574144154711"></a>

1.  Specify the operator function.

    The Upsample operator is used to scale up the feature map in a neural network by using an interpolation method.

2.  Specify the input and output.
    -   The Upsample operator has one input  **x**, one output  **y**, and three attributes.
    -   Both the operator input and output data types are  **float16**  and  **float32**.
    -   The operator input supports all shapes.
    -   The supported input format is  **NC1HWC0**.
    -   The three attributes are  **scale**,  **stride\_h**, and  **stride\_w**.

3.  Determine the operator development mode and the compute API.
    1.  The Upsample operator needs to compute different elements in different dimensions of a tensor, which is not supported by TBE DSL APIs or  TVM primitive  APIs . Therefore, the operator is implemented by using TIK.
    2.  The core computation process is as follows:
        1.  Call the  **data\_move\(\)**  API to read data into the  Unified Buffer.
        2.  Call the  **vec\_muls\(\)**  API to multiply the input with a scaling coefficient.
        3.  Call the  **data\_move\(\)**  API to move data from the  Unified Buffer  to the  Global Memory.


4.  Specify the operator implementation file name, operator implementation function name, and  _OpType_.

    -   _OpType_  is named in upper camel case.
    -   The implementation file name and implementation function name of the operator are converted based on the following rules:
        -   Replace the first uppercase letter with a lowercase letter.

            Example: Abc -\> abc

        -   Replace each uppercase letter following lowercase letters with an underscore \(\_\) and a lowercase letter.

            Example: AbcDef -\> abc\_def

        -   Uppercase letters following a digit or an uppercase letter are regarded as a character string. If there is a lowercase letter after this string, replace the last uppercase letter in this string with an underscore \(\_\) and a lowercase letter, and replace the other uppercase letters with lowercase letters. If there is no lowercase letter after the string, directly replace the string with lowercase letters.

            Examples: ABCDef -\> abc\_def; Abc2DEf -\> abc2d\_ef; Abc2DEF -\> abc2def; ABC2dEF -\> abc2d\_ef



    To avoid messing up with the built-in Upsample operators, define the  _OpType_  as  **UpsampleTik**  and the implementation file name and implementation function name as  **upsample\_tik**.


## Code Implementation<a name="section657125913571"></a>

-   Operator Implementation

    For details about the implementation code of the UpsampleTik operator, see  [upsample\_tik.py](../tbe/impl/upsample_tik.py).

-   Operator Prototype Definition

    For details about the prototype definition of UpsampleTik, see  [upsample\_tik.h](../op_proto/upsample_tik.h)  and  [upsample\_tik.cc](../op_proto/upsample_tik.cc).

-   Operator Information Library

    For details, see the  **upsample\_tik.ini**  file in the corresponding chip version directory  [tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core).

-   Operator Plug-in

    Parse and map the operator whose type is UpsampleTik in the original Caffe model to the UpsampleTik operator that adapts to the Ascend AI Processor. For details about the complete code, see  [upsample\_plugin.cc](../framework/caffe_plugin/upsample_plugin.cc).


## Supported SoCs<a name="section13382182116471"></a>

If the operator information library file of the corresponding chip version exists in the  [tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)  directory, the operator supports the corresponding chip version.
