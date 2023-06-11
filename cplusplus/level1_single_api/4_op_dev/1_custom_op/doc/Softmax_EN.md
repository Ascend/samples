# SoftmaxTik<a name="EN-US_TOPIC_0302083439"></a>

## Overview<a name="section690154102412"></a>

This sample describes how to implement the SoftmaxTik operator in TIK mode, and verifies the single-operator functions and single-operator network.

The Softmax operator is used in the multi-classification process. It normalizes the original data of the previous layer and maps it to the (0, 1) interval to perform multi-classification.

## Operator Analysis<a name="section1672275111254"></a>

1.  The mathematical expression of the Softmax operator is as follows, presented as Numpy code:

    ```
     y = np.exp(x) / np.sum(np.exp(x))
    ```

    The calculation process is: first convert the input into an exponential function with base  **e**, and then obtain the proportion of the sum of all inputs converted into the base  **e**  exponential function.

2.  Specify the input and output.
    -   The Softmax operator has one input,  **x**, and outputs the result  **y**.
    -   The supported input data types include float16 and float32. The output has the same data type as the input.
    -   The operator input supports one-dim shape, [128, ]. The output has the same shape as the input.
    -   The operator input supports the following format:  **ND**.

3.  Determine the operator development mode and the compute API.
    1.  The compute process involves the exponent, broadcast, reduce, and division operation. The  **vec_exp**,  **vector_dup**,  **vec_reduce_add**,  **vdiv**  API can be used to implement for preliminary analysis.
    2.  Since the calculation interface needs to input the number of calculated blocks, the number of repeat times, etc., it is necessary to additionally calculate  **burst**,  **repeat_times**, etc.

4.  Specify the operator implementation file name, operator implementation function name, and  _OpType_.

    -   _OpType_  is named in upper camel case.
    -   The implementation file name and implementation function name of the operator are converted based on the following rules:
        -   Replace the first uppercase letter with a lowercase letter.

            Example: Abc -\> abc

        -   Replace each uppercase letter following lowercase letters with an underscore \(\_\) and a lowercase letter.

            Example: AbcDef -\> abc\_def

        -   Uppercase letters following a digit or an uppercase letter are regarded as a character string. If there is a lowercase letter after this string, replace the last uppercase letter in this string with an underscore \(\_\) and a lowercase letter, and replace the other uppercase letters with lowercase letters. If there is no lowercase letter after the string, directly replace the string with lowercase letters.

            Examples: ABCDef -\> abc\_def; Abc2DEf -\> abc2d\_ef; Abc2DEF -\> abc2def; ABC2dEF -\> abc2d\_ef



    In this example,  _OpType_  of the operator is defined as  **SoftmaxTik**. Uncapitalize the first letter to obtain the operator implementation file name and implementation function name, that is,  **softmax_tik**.


## Code Implementation<a name="section781751919323"></a>

-   Operator Implementation

    The SoftmaxTik operator supports float16 and float32 data types; the SoftmaxTik operator shape supports one dimension, and the shape is [128,]. For details about the operator implementation code, see  [softmax_tik.py](../tbe/impl/softmax_tik.py).

-   Operator Prototype Definition

    The key point of prototype definition is to infer the shape of the output tensor and verify the internal association of the operator input.

    The principle of inferring the output shape is as follows: The shape of the input and output is consistent, both one-dimensional. For details about the code implementation of the  **SoftmaxTikInferShape**  function, see  [softmax_tik.cc](../op_proto/softmax_tik.cc).

-   Operator Information Library

    For details, see the  **softmax_tik.ini**  file in the corresponding chip version directory  [tbe/op\_info\_cfg/vector\_core](../tbe/op_info_cfg/vector_core).

## Supported SoCs<a name="section13382182116471"></a>

If the operator information library file of the corresponding chip version exists in the  [tbe/op\_info\_cfg/vector\_core](../tbe/op_info_cfg/vector_core)  directory, the operator supports the corresponding chip version.
