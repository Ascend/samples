# LSTM<a name="EN-US_TOPIC_0302083104"></a>

## Overview<a name="section638119317537"></a>

Implemented by using TIK.

The LSTM operator is used to process data with time series in a neural network. The formula is as follows:

![](https://images.gitee.com/uploads/images/2020/1223/175631_fc7718fc_5474059.png)

![](https://images.gitee.com/uploads/images/2020/1223/175710_a02cd86e_5474059.png)

![](https://images.gitee.com/uploads/images/2020/1223/175738_456c9f64_5474059.png)

![](https://images.gitee.com/uploads/images/2020/1223/175806_a8a10c06_5474059.png)

![](https://images.gitee.com/uploads/images/2020/1223/175831_cbfcef38_5474059.png)

![](https://images.gitee.com/uploads/images/2020/1223/175855_07b668ec_5474059.png)

## Operator Analysis<a name="section1328419555526"></a>

Before developing an LSTM operator by using TIK, you need to determine the operator functionality, inputs and outputs, operator type, and operator implementation function name.

1.  Specify the operator function.

    For details about the formula of the LSTM operator, see  [Overview](#section638119317537).

    Set  **forget\_bias**  to  **1.0**.

2.  Specify the inputs and outputs.
    -   The LSTM operator has five inputs and two outputs.

        Input:  **x**,  **init\_h**,  **init\_c**,  **weight**, and  **bias**

        Output:  **output\_h**  and  **output\_c**

    -   Except that the data type of  **bias**  is float32, the data type of other parameters is float16, including that of the outputs.
    -   The operator input supports only one fixed shape.
    -   The operator input format is  **ND**. The format of each input is as follows:
        -   <a name="li1773581262813"></a>Format of  **x**

            ![](https://images.gitee.com/uploads/images/2020/1223/181330_f6d956d0_5474059.png)

        -   <a name="li147104274289"></a>Format of  **init\_h**

            ![](https://images.gitee.com/uploads/images/2020/1223/181400_6e12da48_5474059.png)

        -   Format of  **init\_c**

            Has the same format as  [init\_h](#li147104274289).

        -   Format of  **weight**

            ![](https://images.gitee.com/uploads/images/2020/1223/181506_2bc63281_5474059.png)

        -   Format of  **bias**:

            \["4\*32"\]


    -   The output format of the operator is  **ND**.
        -   **output\_h**  has the same format as input  [x](#li1773581262813).
        -   **output\_c**  has the same format as input  [init\_h](#li147104274289).

    -   For details about the input and output formats of the operator, see the data generation file in the sample:

        "[acl\_execute\_lstm/run/out/test\_data/data/generate\_data.py](../../2_verify_op/acl_execute_lstm/run/out/test_data/data/generate_data.py)".


3.  Determine the operator development mode and the compute API.
    1.  The operator needs to read and write one tensor for multiple times in the time step loop, which is not supported by TBE DSL APIs. Therefore, implement the operator by using TIK.
    2.  The core computation process is as follows:
        1.  Use the  **data\_move\(\)**  API to read  **bias**  and  **init\_h**  into the L1 buffer and read  **init\_c**  into the  Unified Buffer.
        2.  Use the  **data\_move\(\)**  API to read the  **x**  and  **weight**  of the current time step into the L1 buffer.
        3.  Call the  **matmul\(\)**  and  **fixpipe\(\)**  APIs to compute matrices i, f, o, and j and move the matrices to the  Global Memory.
        4.  Use the  **data\_move\(\)**  API to transfer matrices i, f, o, and j to the  Unified Buffer.
        5.  Compute a new matrix c and a new matrix h according to the matrices i, f, o, and j.
        6.  Move the new matrix h to the  Global Memory.
        7.  Move the new matrix h from the  Global Memory  to the L1 Buffer and repeat the time steps starting from step b.
        8.  Use the  **data\_move\(\)**  API to move the obtained matrix C from the  Unified Buffer  to the  Global Memory.


4.  Specify the operator implementation file name, operator implementation function name, and  _OpType_.

    -   _OpType_  is named in upper camel case.
    -   The implementation file name and implementation function name of the operator are converted based on the following rules:
        -   Replace the first uppercase letter with a lowercase letter.

            Example: Abc -\> abc

        -   Replace each uppercase letter following lowercase letters with an underscore \(\_\) and a lowercase letter.

            Example: AbcDef -\> abc\_def

        -   Uppercase letters following a digit or an uppercase letter are regarded as a character string. If there is a lowercase letter after this string, replace the last uppercase letter in this string with an underscore \(\_\) and a lowercase letter, and replace the other uppercase letters with lowercase letters. If there is no lowercase letter after the string, directly replace the string with lowercase letters.

            Examples: ABCDef -\> abc\_def; Abc2DEf -\> abc2d\_ef; Abc2DEF -\> abc2def; ABC2dEF -\> abc2d\_ef



    To avoid messing up with the built-in LSTM operators, define the  _OpType_  as  **LSTMTik**  and the implementation file name and implementation function name as  **lstm\_tik**.


## Code Implementation<a name="section657125913571"></a>

-   Operator Implementation

    For details about the implementation code of the LSTMTik operator, see  [lstm\_tik.py](../tbe/impl/lstm_tik.py).

-   Operator Prototype Definition

    For the definition of the LSTMTik prototype, see  [lstm\_tik.h](../op_proto/lstm_tik.h)  and  [lstm\_tik.cpp](../op_proto/lstm_tik.cpp).

-   Operator Information Library

    For details, see the  [lstm\_tik.ini](../tbe/op_info_cfg/ai_core/ascend310/lstm_tik.ini)  file.

-   Operator Plug-in

    Parse and map the operator whose type is LSTMTik in the original TensorFlow model to the LSTMTik operator that adapts to the Ascend AI Processor. For details about the complete code, see  [lstm\_tik\_plugin.cpp](../framework/tf_plugin/lstm_tik_plugin.cpp).


## Supported SoCs<a name="section13382182116471"></a>

If the operator information library file of the corresponding chip version exists in the  [tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)  directory, the operator supports the corresponding chip version.
