# Unique<a name="EN-US_TOPIC_0303147571"></a>

## Overview<a name="section6945232175"></a>

This sample describes the implementation of the AI CPU custom operator Unique. The original framework of the Unique operator is TensorFlow.

The Unique operator finds unique elements in the input 1D tensor  **x**  and returns tensor  **y**  containing all of the unique elements of  **x**  sorted in the same order that they occur in  **x**. The operator also returns a tensor  **idx**  the same size as  **x**  that contains the index of each value of  **x**  in the unique output  **y**.

For example:

-   Input tensor: \[1, 1, 2, 4, 4, 4, 7, 8, 8\]

-   Outputs:
    -   Tensor  **y**: \[1, 2, 4, 7, 8\]
    -   Tensor  **idx**: \[0, 0, 1, 2, 2, 2, 3, 4, 4\]


## Operator Analysis<a name="section5726181618178"></a>

1.  Specify the operator function.

    The Unique operator finds unique elements in the input 1D tensor  **x**  and returns tensor  **y**  containing all of the unique elements of  **x**  sorted in the same order that they occur in  **x**. The operator also returns a tensor  **idx**  the same size as  **x**  that contains the index of each value of  **x**  in the unique output  **y**.

2.  Specify the input and outputs.
    -   The Unique operator has one input:  **x**.

        In this sample, the input tensor supports the following data types: float16, float32, int8, int16, uint16, uint8, int32, int64, and float64.

    -   The operator has two outputs: tensor  **y**  and tensor  **idx**.

        Tensor  **y**  has the same data type as tensor  **x**.

        Tensor  **idx**  is of type int32 or int64.


3.  Specify the operator implementation file name and  _OpType_.

    -   Name  _**OpType**_  in upper camel case and indicate the separation of words with a single capitalized letter.
    -   Name the operator implementation file after  **_OpType_**  as follows:
        -   Replace the first uppercase letter with a lowercase letter.

            Example: Abc -\> abc

        -   Replace each uppercase letter following lowercase letters with an underscore \(\_\) and a lowercase letter.

            Example: AbcDef -\> abc\_def

        -   Uppercase letters following a digit or an uppercase letter are regarded as a character string. If there is a lowercase letter after this string, replace the last uppercase letter in this string with an underscore \(\_\) and a lowercase letter, and replace the other uppercase letters with lowercase letters. If there is no lowercase letter after the string, directly replace the string with lowercase letters.

            Examples: ABCDef -\> abc\_def; Abc2DEf -\> abc2d\_ef; Abc2DEF -\> abc2def; ABC2dEF -\> abc2d\_ef



    To avoid messing up with the built-in Unique operators, define the  _OpType_  as  **UniqueCust**  and the implementation file name as  **unique\_cust**.


## Code Implementation<a name="section83331113171811"></a>

-   Operator Implementation

    For details about the implementation code of the UniqueCust operator, see  [unique\_cust\_kernels.h](../cpukernel/impl/unique_cust_kernels.h)  and  [unique\_cust\_kernels.cc](../cpukernel/impl/unique_cust_kernels.cc).

    UniqueCust is a dynamic-shape operator. The shape of the output  **y**  is determined by input  **x**  and is inferred and updated during the compute process.

-   Operator Prototype Definition

    The key point of prototype definition is inferring the shape of the output tensor.

    The principle of the output shape of the UniqueCust operator is as follows: Output  **idx**  has the same shape as input  **x**, and the shape of output  **y**  is unknow and is related to the element repetition degree of input  **x**. For details about the code implementation of the UniqueCust operator prototype definition, see  [unique\_cust.h](../op_proto/unique_cust.h)  and  [unique\_cust.cpp](../op_proto/unique_cust.cpp).

-   Operator Information Library

    For details about the operator information library of UniqueCust, see  [unique\_cust.ini](../cpukernel/op_info_cfg/aicpu_kernel/unique_cust.ini).

-   Operator Plug-in

    This sample provides the TensorFlow-based operator adaptation plug-in to map the original TensorFlow operator to the operator that adapts to the Ascend AI Processor. For details about the complete code, see  [unique\_cust\_plugin.cpp](../framework/tf_plugin/unique_cust_plugin.cpp).


## Supported SoCs<a name="section13382182116471"></a>

All

