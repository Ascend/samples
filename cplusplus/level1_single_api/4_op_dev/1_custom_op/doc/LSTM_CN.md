# LSTM<a name="ZH-CN_TOPIC_0302083104"></a>

## 功能描述<a name="section638119317537"></a>

本样例使用TIK方式进行了LSTM算子的实现。

LSTM算子在神经网络，常用于处理带有时间序列的数据，计算公式如下：

![](https://images.gitee.com/uploads/images/2020/1223/175631_fc7718fc_5474059.png)

![](https://images.gitee.com/uploads/images/2020/1223/175710_a02cd86e_5474059.png)

![](https://images.gitee.com/uploads/images/2020/1223/175738_456c9f64_5474059.png)

![](https://images.gitee.com/uploads/images/2020/1223/175806_a8a10c06_5474059.png)

![](https://images.gitee.com/uploads/images/2020/1223/175831_cbfcef38_5474059.png)

![](https://images.gitee.com/uploads/images/2020/1223/175855_07b668ec_5474059.png)

## 算子分析<a name="section1328419555526"></a>

使用TIK API开发LSTM算子前，我们需要确定算子功能、输入、输出，算子类型以及算子实现函数名称等。

1.  明确算子的功能。

    LSTM算子的计算公式请参见[功能描述](#section638119317537)。

    其中forget\_bias设置为1.0。

2.  明确输入和输出。
    -   LSTM算子有5个输入，2个输出。

        输入：x，init\_h，init\_c，weight，bias

        输出：output\_h，output\_c

    -   算子输入除了bias的数据类型为float32以外，其他参数的数据类型为float16；算子输出的数据类型为float16。
    -   算子输入只支持固定的一种shape。
    -   算子输入支持的format：ND，每个输入的format如下所示：
        -   <a name="li1773581262813"></a>x的格式：

            ![](https://images.gitee.com/uploads/images/2020/1223/181330_f6d956d0_5474059.png)

        -   <a name="li147104274289"></a>init\_h的格式：

            ![](https://images.gitee.com/uploads/images/2020/1223/181400_6e12da48_5474059.png)

        -   init\_c的格式：

            同[init\_h的格式](#li147104274289)。

        -   weight的格式:

            ![](https://images.gitee.com/uploads/images/2020/1223/181506_2bc63281_5474059.png)

        -   bias的格式：

            bias的格式为\[4\*32\]。


    -   算子输出的format为：ND。
        -   output\_h的格式与[输入x](#li1773581262813)的格式相同。
        -   output\_c的格式与[输入init\_h](#li147104274289)的格式相同。

    -   算子输入输出格式的详细信息还可以参考样例对应的数据生成文件：

        “[acl\_execute\_lstm/run/out/test\_data/data/generate\_data.py](../../2_verify_op/acl_execute_lstm/run/out/test_data/data/generate_data.py)”。


3.  确定算子开发方式及使用的计算接口。
    1.  涉及在时间步循环中多次读写同一个Tensor的场景，TBE DSL接口无法满足此算子的计算要求，所以考虑使用TIK方式进行此算子的实现。
    2.  该算子实现核心的计算流程如下：
        1.  使用data\_move\(\)接口将bias、init\_h读入到L1 Buffer中，将init\_c读入Unified Buffer。
        2.  使用data\_move\(\)接口将当前时间步的x和weight读入到L1 Buffer中。
        3.  使用matmul\(\)和fixpipe\(\)接口分别计算i、f、o、j矩阵并且搬运到Global Memory中。
        4.  使用data\_move\(\)接口将i、f、o、j矩阵搬运到Unified Buffer中。
        5.  根据i、f、o、j矩阵计算出新的c矩阵和h矩阵。
        6.  将新得到的h矩阵搬运到Global Memory中。
        7.  将新得到的h矩阵由Global Memory搬运到L1 Buffer中，并且从b步骤开始重复时间步次数。
        8.  使用data\_move\(\)接口把最终得到的c矩阵从Unified Buffer搬运数据到Global Memory中。


4.  明确算子实现文件名称、算子实现函数名称以及算子的类型（OpType）。

    -   算子类型采用大驼峰的命名方式。
    -   算子的实现文件名称及实现函数名称，将OpType采用如下规则进行转换：
        -   首字符的大写字符转换为小写字符。

            例如：Abc -\> abc

        -   小写字符后的大写字符转换为下划线+小写字符。

            例如：AbcDef -\> abc\_def

        -   紧跟数字以及大写字符后的大写字符，作为同一语义字符串，查找此字符串后的第一个小写字符，并将此小写字符的前一个大写字符转换为下划线+小写字符，其余大写字符转换为小写字符。若此字符串后不存在小写字符，则直接将此字符串中的大写字符转换为小写字符。

            例如：ABCDef -\> abc\_def；Abc2DEf -\> abc2d\_ef；Abc2DEF -\> abc2def；ABC2dEF -\> abc2d\_ef。



    本例中，为不影响内置的LSTM算子，算子类型定义为LSTMTik；算子的实现文件名称及实现函数名称定义为lstm\_tik。


## 代码实现<a name="section657125913571"></a>

-   算子实现

    LSTMTik算子的实现代码请参见[lstm\_tik.py](../tbe/impl/lstm_tik.py)。

-   算子原型定义

    LSTMTik的原型定义请参见[lstm\_tik.h](../op_proto/lstm_tik.h)与[lstm\_tik.cpp](../op_proto/lstm_tik.cpp)。

-   算子信息库

    请参见[lstm\_tik.ini](../tbe/op_info_cfg/ai_core/ascend310/lstm_tik.ini)文件。

-   算子适配插件

    将原始TensorFlow中Type为LSTMTik的算子解析并映射为适配昇腾AI处理器的LSTMTik算子，完整代码请参见[lstm\_tik\_plugin.cpp](../framework/tf_plugin/lstm_tik_plugin.cpp)。


## 支持的芯片类型<a name="section13382182116471"></a>

若[tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)目录下存在对应芯片版本的算子信息库文件，则说明此算子支持对应的芯片版本。

