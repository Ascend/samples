# Conv2d<a name="EN-US_TOPIC_0302083411"></a>

## Overview<a name="section1973805311174"></a>

Implemented by using TIK, the Conv2d operator performs the convolution 2-D operation on the input tensor and a weight tensor and outputs the result tensor.

## Operator Analysis<a name="section19621012171817"></a>

1.  Specify the operator function.

    The Conv2d operator performs the 2-D convolution operation on the input tensor and a weight tensor, and outputs the result tensor, as shown in  [Figure 1](#fig2099591662413).

    **Figure  1**  Conv2d operator computation process<a name="fig2099591662413"></a>  
    ![](https://images.gitee.com/uploads/images/2020/1223/174800_6754ce07_5474059.png "conv2d-operator-computation-process")

2.  Specify the inputs and output.
    -   The Conv2d operator has two inputs \(**x**  and  **filter**\), one output \(**y**\), and three attributes.
    -   Both the operator input and output are of type  **float16**.
    -   The operator input supports static shapes. The output shape and input shape must meet the mathematical expression of the operator.
    -   The supported input format is  **NCHW**.
    -   The operator has three attributes:  **strides**,  **pads**, and  **dilations**, which are set to  **\[1, 1, 1, 1\]**.

3.  Determine the operator development mode and the compute API.
    1.  The 2-D convolution operation can be implemented by the  **conv2d\(\)**  API.

        When the  **conv2d\(\)**  API processes the  **strides**  and  **dilations**  attributes, the N and C dimensions must be set to  **1**. In this example, the H and W dimensions are also set to  **1**.

    2.  Data movement from the  Global Memory  to the  L1 Buffer  can be implemented by calling the  **data\_move\(\)**  API.
    3.  The result data movement from the  L1OUT Buffer  to the  Global Memory  can be implemented by calling the  **fixpipe\(\)**  API.

4.  Specify the operator implementation file name, operator implementation function name, and  _OpType_.

    -   _OpType_  is named in upper camel case.
    -   The implementation file name and implementation function name of the operator are converted based on the following rules:
        -   Replace the first uppercase letter with a lowercase letter.

            Example: Abc -\> abc

        -   Replace each uppercase letter following lowercase letters with an underscore \(\_\) and a lowercase letter.

            Example: AbcDef -\> abc\_def

        -   Uppercase letters following a digit or an uppercase letter are regarded as a character string. If there is a lowercase letter after this string, replace the last uppercase letter in this string with an underscore \(\_\) and a lowercase letter, and replace the other uppercase letters with lowercase letters. If there is no lowercase letter after the string, directly replace the string with lowercase letters.

            Examples: ABCDef -\> abc\_def; Abc2DEf -\> abc2d\_ef; Abc2DEF -\> abc2def; ABC2dEF -\> abc2d\_ef



    To avoid messing up with the built-in Conv2D operators, define the  _OpType_  as  **Conv2DTik**  and the implementation file name and implementation function name as  **conv2d\_tik**.


## Code Implementation<a name="section657125913571"></a>

-   Operator Implementation

    In this sample, the input the Conv2DTik operator is of type  **float16**. Verify the operator type, set parameters, and call the operator compute function.

    The implementation logic of the operator calculation function includes the following key steps:

    1.  Compute the shape and placeholder for the input and output tensors based on parameters.
    2.  Enable double buffering and AI Core parallelism by using the  **for\_range\( \)**  loop to tile input data, improving compute efficiency.
    3.  Call  **conv2d\(\)**  to implement the 2-D convolution operation.
    4.  Call  **fixpipe\(\)**  to move the compute result data.

    For details about the complete implementation code, see  [conv2d\_tik.py](../tbe/impl/conv2d_tik.py).

-   Operator Prototype Definition

    For details about the definition of the Conv2DTik prototype, see  [conv2d\_tik.h](../op_proto/conv2d_tik.h)  and  [conv2d\_tik.cc](../op_proto/conv2d_tik.cc).

-   Operator Information Library

    For details, see the  **conv2d\_tik.ini**  file in the corresponding chip version directory  [tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core).

-   Operator Plug-in

    Parse the operator whose type is ConvolutionTik in the original Caffe and map it to the Conv2DTik operator that adapts to the Ascend AI Processor. For details about the complete code, see  [caffe\_conv2d\_tik\_plugin.cc](../framework/caffe_plugin/caffe_conv2d_tik_plugin.cc).


## Supported SoCs<a name="section13382182116471"></a>

If the operator information library file of the corresponding chip version exists in the  [tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)  directory, the operator supports the corresponding chip version.
