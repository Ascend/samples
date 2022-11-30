# Add<a name="EN-US_TOPIC_0302083439"></a>

## Overview<a name="section690154102412"></a>

This sample describes how to implement the Add operator in TBE DSL mode, and verifies the single-operator functions and single-operator network.

The Add operator returns the sum of its operands, as shown in the following figure.

**Figure  1**  Add operator function diagram<a name="en-us_topic_0229823836_fig1134425318216"></a>  
![](https://images.gitee.com/uploads/images/2021/0114/162517_9f334f35_5474059.png "add-operator-function-diagram.png")

## Operator Analysis<a name="section1672275111254"></a>

1.  The mathematical expression of the Add operator is as follows:

    ```
     z=x+y
    ```

    The Add operator adds two input parameters to obtain the final result  **z**  and return it.

2.  Specify the inputs and output.
    -   The Add operator has two inputs,  **x**  and  **y**, and outputs the result  **z**.
    -   The supported input data types include float16, float32, and int32. The output has the same data type as the inputs.
    -   The operator input supports all shapes. The output has the same shape as the inputs.
    -   The operator input supports the following formats:  **NCHW**,  **NC1HWC0**,  **NHWC**, and  **ND**.

3.  Determine the operator development mode and the compute API.
    1.  The compute process involves only the addition operation. The  **te.lang.cce.vadd\(lhs, rhs\)**  API can be used to implement "x + y" for preliminary analysis.
    2.  The  **te.lang.cce.vadd\(lhs, rhs\)**  API requires that the two input tensors to have the same shape. Therefore, you need to obtain the larger shape of the two input tensors, and then call the  **te.lang.cce.broadcast\(var, shape, output\_dtype=None\)**  API to broadcast the input argument to a specified shape.

4.  Specify the operator implementation file name, operator implementation function name, and  _OpType_.

    -   _OpType_  is named in upper camel case.
    -   The implementation file name and implementation function name of the operator are converted based on the following rules:
        -   Replace the first uppercase letter with a lowercase letter.

            Example: Abc -\> abc

        -   Replace each uppercase letter following lowercase letters with an underscore \(\_\) and a lowercase letter.

            Example: AbcDef -\> abc\_def

        -   Uppercase letters following a digit or an uppercase letter are regarded as a character string. If there is a lowercase letter after this string, replace the last uppercase letter in this string with an underscore \(\_\) and a lowercase letter, and replace the other uppercase letters with lowercase letters. If there is no lowercase letter after the string, directly replace the string with lowercase letters.

            Examples: ABCDef -\> abc\_def; Abc2DEf -\> abc2d\_ef; Abc2DEF -\> abc2def; ABC2dEF -\> abc2d\_ef



    In this example,  _OpType_  of the operator is defined as  **Add**. Uncapitalize the first letter to obtain the operator implementation file name and implementation function name, that is,  **add**.


## Code Implementation<a name="section781751919323"></a>

-   Operator Implementation

    The Add operator supports only three data types: float16, float32, and int32. Therefore, the input data type needs to be verified. The two inputs may have different shapes. This scenario is supported by the Add operator, but not supported by the operator compute API  **te.lang.cce.vadd\(\)**. As a result, the two input shapes need to be broadcast and verified. For details about the operator implementation code, see  [add_dsl.py](../tbe/impl/add_dsl.py).

-   Operator Prototype Definition

    The key point of prototype definition is to infer the shape of the output tensor and verify the internal association of the operator inputs.

    The principle of inferring the output shape is as follows: Obtain the two input shapes, broadcast them to the same shape, and assign the larger value of each dimension of the two inputs to form the output shape. For details about the code implementation of the  **InferShapeAndTypeAdd**  function, see  [add.cc](../op_proto/add.cc).

-   Operator Information Library

    For details, see the  **add.ini**  file in the corresponding chip version directory  [tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core).

-   Operator Plug-in

    Parse and map the Add operator developed under the TensorFlow framework to the Add operator supported by the Ascend AI Processor. You can implement the operator attribute mapping by calling  **AutoMappingFn\(\)**. For details about the complete code, see  [add\_plugin.cc](../framework/tf_plugin/add_plugin.cc).


## Supported SoCs<a name="section13382182116471"></a>

If the operator information library file of the corresponding chip version exists in the  [tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)  directory, the operator supports the corresponding chip version.
