# Matmul<a name="EN-US_TOPIC_0302083453"></a>

## Overview<a name="section20275955152017"></a>

Implemented by using TIK, the Matmul operator performs matrix multiplication on the two input tensors A and B and outputs the result tensor.

## Operator Analysis<a name="section132184332113"></a>

1.  Specify the operator function and mathematical expression.

    The mathematical expression of the Matmul operator is as follows:

    ```
     y=x1*x2
    ```

    The Matmul operator multiplies two input matrices and returns a result matrix.

2.  Specify the inputs and output.
    -   The Matmul operator has two inputs  **x1**  and  **x2**, and one output  **y**.
    -   The input data type is  **float16**, and the output data type is  **float32**.
    -   The operator supports fixed input shapes \(16, 64\) and \(64, 1024\), and the output shape \(16, 1024\).
    -   The operator input supports  **ND**  format.

3.  Determine the operator development mode and the compute API.
    1.  The matrix multiplication \(x1\*x2\) can be implemented by calling the  **matmul\(\)**  API.
    2.  Data movement from the  Global Memory  to the  L1 Buffer  can be implemented by calling  **data\_move\(\)**.
    3.  The result data movement from the  L1OUT Buffer  to the  Global Memory  can be implemented by calling  **fixpipe\(\)**  for processing.

4.  Specify the operator implementation file name, operator implementation function name, and  _OpType_.

    -   _OpType_  is named in upper camel case.
    -   The implementation file name and implementation function name of the operator are converted based on the following rules:
        -   Replace the first uppercase letter with a lowercase letter.

            Example: Abc -\> abc

        -   Replace each uppercase letter following lowercase letters with an underscore \(\_\) and a lowercase letter.

            Example: AbcDef -\> abc\_def

        -   Uppercase letters following a digit or an uppercase letter are regarded as a character string. If there is a lowercase letter after this string, replace the last uppercase letter in this string with an underscore \(\_\) and a lowercase letter, and replace the other uppercase letters with lowercase letters. If there is no lowercase letter after the string, directly replace the string with lowercase letters.

            Examples: ABCDef -\> abc\_def; Abc2DEf -\> abc2d\_ef; Abc2DEF -\> abc2def; ABC2dEF -\> abc2d\_ef



    To avoid messing up with the built-in Matmul operators, define the  _OpType_  as  **MatmulTik**  and the implementation file name and implementation function name as  **matmul\_tik**.


## Code Implementation<a name="section657125913571"></a>

-   Operator Implementation

    For details about the implementation code of the MatmulTik operator, see  [matmul\_tik.py](../tbe/impl/matmul_tik.py).

-   Operator Prototype Definition

    For details about the definition of the MatmulTik prototype, see  [matmul\_tik.h](../op_proto/matmul_tik.h)  and  [matmul\_tik.cc](../op_proto/matmul_tik.cc).

-   Operator Information Library

    For details, see the  **matmul\_tik.ini**  file in the corresponding chip version directory  [tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core).

-   Operator Plug-in

    The corresponding operator plug-in is not implemented in this sample. To use this custom operator in the network model built based on a third-party framework, you need to customize an operator plug-in.


## Supported SoCs<a name="section13382182116471"></a>

If the operator information library file of the corresponding chip version exists in the  [tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)  directory, the operator supports the corresponding chip version.
