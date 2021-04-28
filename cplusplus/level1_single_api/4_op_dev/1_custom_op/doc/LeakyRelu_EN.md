# LeakyRelu<a name="EN-US_TOPIC_0302083167"></a>

## Overview<a name="section7526288579"></a>

This sample describes how to implement the LeakyRelu operator in TBE DSL mode.

The mathematical expression of the LeakyRelu operator is as follows:

-   When  ![](https://images.gitee.com/uploads/images/2020/1223/172704_b8bfda6c_5474059.png)  = 0,

    f\(x\) = max\(0, x\)

-   When  ![](https://images.gitee.com/uploads/images/2020/1223/172704_b8bfda6c_5474059.png)! = 0,

    ![](https://images.gitee.com/uploads/images/2020/1223/172738_ce94c6e6_5474059.png
)

## Operator Analysis<a name="section1043174819574"></a>

1.  Specify the operator function and mathematical expression.

    For details about the operator function, see  [Overview](#section7526288579).

2.  Specify the input and output.
    -   The LeakyRelu operator has one input \(**x**\), one output \(**y**\), and one attribute \(![](https://images.gitee.com/uploads/images/2021/0426/102929_a40abaef_5474059.png)  in the operator expression\).
    -   The supported input data types include float16, float32, int32, and int8. The output has the same data type as the inputs.
    -   The operator input supports all shapes. The output has the same shape as the inputs.
    -   The operator input supports the following formats:  **NCHW**,  **NC1HWC0**, and  **NHWC**.

3.  Determine the operator development mode and the compute API.
    1.  The compute process involves several operations: obtaining the maximum value, obtaining the minimum value, and performing the multiplication method. The DSL APIs have corresponding functions:

        **te.lang.cce.vmuls\(\)**  is used for multiplication.

        **te.lang.cce.vmin\(\)**  is used to obtain the minimum value.

        **te.lang.cce.vmax\(\)**  is used to obtain the maximum value.

    2.  Some DSL APIs convert data types. Therefore, after the compute process, the result needs to be converted back to the original data type by using the  **te.lang.cce.cast\_to**  API.
    3.  When  ![](https://images.gitee.com/uploads/images/2021/0426/102929_a40abaef_5474059.png)  = 0, indicating the ReLU operation, its function of converting int8, int32, and float32 to float16 can be implemented by using the  **te.lang.cce.vrelu\(\)**  API. If the data type is int32 or float32, the precision loss may occur. In this case, use the  **te.lang.cce.vmax\(\)**  API to obtain the maximum value.

4.  Specify the operator implementation file name, operator implementation function name, and  _OpType_.

    -   _OpType_  is named in upper camel case.
    -   The implementation file name and implementation function name of the operator are converted based on the following rules:
        -   Replace the first uppercase letter with a lowercase letter.

            Example: Abc -\> abc

        -   Replace each uppercase letter following lowercase letters with an underscore \(\_\) and a lowercase letter.

            Example: AbcDef -\> abc\_def

        -   Uppercase letters following a digit or an uppercase letter are regarded as a character string. If there is a lowercase letter after this string, replace the last uppercase letter in this string with an underscore \(\_\) and a lowercase letter, and replace the other uppercase letters with lowercase letters. If there is no lowercase letter after the string, directly replace the string with lowercase letters.

            Examples: ABCDef -\> abc\_def; Abc2DEf -\> abc2d\_ef; Abc2DEF -\> abc2def; ABC2dEF -\> abc2d\_ef



    To avoid messing up with the built-in LeakyReluDemo operators, define the  _OpType_  as  **LeakyReluDemo**  and the implementation file name and implementation function name as  **leaky\_relu\_demo**.


## Code Implementation<a name="section657125913571"></a>

-   Operator Implementation

    The implementation logic of the operator calculation function is as follows. For details about the complete code implementation, see  [leaky\_relu\_demo.py](../tbe/impl/leaky_relu_demo.py).

    1.  When  **negative\_slope**  is  **0**, the output  **y**  is the larger value between the input  **x**  and 0.
        -   If the input data type is float16 or int8, the  **te.lang.cce.vrelu\(x\)**  API can be called for compute process.
        -   If the input data type is float32 or int32, precision loss may occur if the  **te.lang.cce.vrelu\(x\)**  API is directly called. This is because the  **te.lang.cce.vrelu\(x\)**  API converts float32 and int32 to float16 for compute process. Therefore, you need to compare the input data with  _tensor\_zero_  \(with the same shape as  **x**  and 0 elements\) by using the  **te.lang.cce.vmax\(\)**  API.

    2.  When  **negative\_slope**  is not 0, you need to distinguish the scenarios where  **negative\_slope**  is less than 1 and  **negative\_slope**  is larger than 1 according to  [Overview](#section7526288579).

        When  **negative\_slope**  is less than or equals to 1, the larger value between  **x**  and  **x\*negative\_slope**  is used. When  **negative\_slope**  is larger than 1, the smaller value between  **x**  and  **x\*negative\_slope**  is used.

        -   As required by the  **te.lang.cce.vmuls\(\)**  API, the input tensor should be of the same data type of the scalar. Therefore, you need to convert  **negative\_slope**  to the data type of input  **x**  by using the  **tvm.const**  API.
        -   When the  **te.lang.cce.vmuls\(\)**  API is used, data types int8 and int32 will be converted to float16. Therefore, after the  **te.lang.cce.vmuls\(\)**  operation is performed, you need to convert the result to the original data type by calling the  **te.lang.cce.cast\_to\(\)**  API and compare the value of  **negative\_slope**  with  **x**.


-   Operator Prototype Definition

    The key point of prototype definition is to infer the shape and dtype of the output tensor. The LeakyReLUDemo operator directly assigns the shape and dtype of the input tensor to the output tensor.

    For details about the complete code implementation, see  [leaky\_relu\_demo.h](../op_proto/leaky_relu_demo.h)  and  [leaky\_relu\_demo.cc](../op_proto/leaky_relu_demo.cc).

-   Operator Information Library

    For details, see the  **leaky\_relu\_demo.ini**  file in the corresponding chip version directory  [tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core).

-   Operator Plug-in

    You need to customize the  **ParseParamsLeakyRelu**  function to implement the attribute mapping from the Caffe LeakyReLUDemo operator to the LeakyReluDemo operator adapted to the Ascend AI Processor. For details about the complete code, see  [caffe\_leaky\_relu\_plugin.cc](../framework/caffe_plugin/caffe_leaky_relu_plugin.cc).


## Supported SoCs<a name="section13382182116471"></a>

If the operator information library file of the corresponding chip version exists in the  [tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)  directory, the operator supports the corresponding chip version.

