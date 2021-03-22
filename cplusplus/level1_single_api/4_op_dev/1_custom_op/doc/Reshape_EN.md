# Reshape<a name="EN-US_TOPIC_0303147570"></a>

## Overview<a name="section17348171613103"></a>

This sample describes the implementation of the AI CPU custom operator Reshape. The original framework of the Reshape operator is TensorFlow.

The Reshape operator reshapes the input tensor. It does not change the data.

For example:

-   Input tensor: \[1, 2, 3, 4, 5, 6, 7, 8, 9\]
-   Target shape: \[3, 3\]
-   Returned tensor: \[\[1,2,3\], \[4,5,6\], \[7,8,9\]\]

## Operator Analysis<a name="section118066248105"></a>

1.  Specify the operator function.

    The Reshape operator reshapes the input tensor to the target shape and returns the result tensor. It does not change the data.

2.  Specify the inputs and output.
    -   The Reshape operator has two inputs: input tensor and target shape.
    -   In this sample, the input tensor supports the following data types: bool, float16, float32, int8, int32, uint32, uint8, int64, int16, uint16, float64, complex64, complex128, qint8, qint16, quint16, and qint32.
    -   The target shape is of type int32 or int64.
    -   The result tensor has the same data type as the input tensor.

3.  Specify the operator implementation file name and  _OpType_.

    -   Name  _**OpType**_  in upper camel case and indicate the separation of words with a single capitalized letter.
    -   Name the operator implementation file after  **_OpType_**  as follows:
        -   Replace the first uppercase letter with a lowercase letter.

            Example: Abc -\> abc

        -   Replace each uppercase letter following lowercase letters with an underscore \(\_\) and a lowercase letter.

            Example: AbcDef -\> abc\_def

        -   Uppercase letters following a digit or an uppercase letter are regarded as a character string. If there is a lowercase letter after this string, replace the last uppercase letter in this string with an underscore \(\_\) and a lowercase letter, and replace the other uppercase letters with lowercase letters. If there is no lowercase letter after the string, directly replace the string with lowercase letters.

            Examples: ABCDef -\> abc\_def; Abc2DEf -\> abc2d\_ef; Abc2DEF -\> abc2def; ABC2dEF -\> abc2d\_ef



    To avoid messing up with the built-in Reshape operators, define the  _OpType_  as  **ReshapeCust**  and the implementation file name as  **reshape\_cust**.


## Code Implementation<a name="section1349813621017"></a>

-   Operator Implementation

    For details about the implementation code of the ReshapeCust operator, see  [reshape\_cust\_kernels.h](../cpukernel/impl/reshape_cust_kernels.h)  and  [reshape\_cust\_kernels.cc](../cpukernel/impl/reshape_cust_kernels.cc).

-   Operator Prototype Definition

    The key point of prototype definition is inferring the shape of the output tensor.

    The principle of output shape inference of the ReshapeCust operator is as follows: Obtain the input tensor and the target shape, check whether the element count of the input tensor is the same as that of the target shape. If yes, the output shape is set to the target shape.

    For details about the code implementation of the ReshapeCust operator prototype definition, see  [reshape\_cust.h](../op_proto/reshape_cust.h)  and  [reshape\_cust.cc](../op_proto/reshape_cust.cc).

-   Operator Information Library

    For details about the operator information library of ReshapeCust, see  [reshape\_cust.ini](../cpukernel/op_info_cfg/aicpu_kernel/reshape_cust.ini).

-   Operator Plug-in

    This sample provides the ReshapeCust operator adaptation plug-in for TensorFlow and Caffe to map the original TensorFlow/Caffe operators to the operators that adapt to the Ascend AI Processor. For details about the complete code, see the  [reshape\_cust\_plugin.cc](../framework/tf_plugin/reshape_cust_plugin.cc)  and  [caffe\_reshape\_cust\_plugin.cc](../framework/caffe_plugin/caffe_reshape_cust_plugin.cc)  files.


## Supported SoCs<a name="section13382182116471"></a>

All

