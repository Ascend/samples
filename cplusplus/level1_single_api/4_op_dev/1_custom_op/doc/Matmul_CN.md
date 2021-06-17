# Matmul<a name="ZH-CN_TOPIC_0302083453"></a>

## 功能描述<a name="section20275955152017"></a>

本样例使用TIK方式进行了Matmul算子的实现，给定两个输入的张量A和B，进行矩阵相乘，输出结果张量。

## 算子分析<a name="section132184332113"></a>

1.  明确算子的功能以及数学表达式。

    Matmul算子的数学表达式为：

    ```
     y=x1*x2
    ```

    计算过程是：将两个输入矩阵相乘，得到最终结果矩阵z并将其返回。

2.  明确输入和输出。

    **说明：本样例基于固定排布格式和固定shape的输入，使用TIK方式进行Matmul算子的实现。** 

    -   Matmul算子有两个输入：x1与x2，输出为y。
    -   本样例中，算子输入的数据类型仅支持float16、int8、uint8。若输入的数据类型为float16，输出的数据类型为float32；若输入的数据类型为int8、uint8，输出的数据类型为int32。
    -   算子输入支持固定的format与shape。

        输入的format需要分别为：[k,m,c]与[k,n,c]，shape分别为[4,16,16]与[4,1024,16]。
        输入数据在传入算子前，需要分别reshape为（16,64）和（64,1024），详细请参见[generate_datatik.py](../../2_verify_op/acl_execute_matmul/run/out/test_data/data/generate_datatik.py)
        输出shape为（16,1024）。

3.  确定算子开发方式及使用的计算接口。
    1.  计算过程中主要涉及矩阵乘法操作，初分析可使用matmul\(\)接口实现矩阵x1\*x2的操作。
    2.  由于在整个Matmul的计算过程中会涉及到数据搬运操作，可使用data\_move\(\)接口把数据从Global Memory搬迁到L1 Buffer中。
    3.  矩阵计算完成后，对结果进行处理，可使用fixpipe\(\)把数据从L1OUT  Buffer搬迁到Global Memory中。

4.  明确算子实现文件名称、算子实现函数名称以及算子的类型（OpType）。

    -   算子类型采用大驼峰的命名方式。
    -   算子的实现文件名称及实现函数名称，将OpType采用如下规则进行转换：
        -   首字符的大写字符转换为小写字符。

            例如：Abc -\> abc

        -   小写字符后的大写字符转换为下划线+小写字符。

            例如：AbcDef -\> abc\_def

        -   紧跟数字以及大写字符后的大写字符，作为同一语义字符串，查找此字符串后的第一个小写字符，并将此小写字符的前一个大写字符转换为下划线+小写字符，其余大写字符转换为小写字符。若此字符串后不存在小写字符，则直接将此字符串中的大写字符转换为小写字符。

            例如：ABCDef -\> abc\_def；Abc2DEf -\> abc2d\_ef；Abc2DEF -\> abc2def；ABC2dEF -\> abc2d\_ef。



    本样例中，为了不影响内置的Matmul算子，算子类型定义为MatmulTik；算子的实现文件名称及实现函数名称定义为matmul\_tik。


## 代码实现<a name="section657125913571"></a>

-   算子实现

    MatmulTik算子的实现代码请参见[matmul\_tik.py](../tbe/impl/matmul_tik.py)。

-   算子原型定义

    MatmulTik的原型定义请参见[matmul\_tik.h](../op_proto/matmul_tik.h)与[matmul\_tik.cc](../op_proto/matmul_tik.cc)。

-   算子信息库

    请参见[tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)对应芯片版本目录下的matmul\_tik.ini文件。

-   算子适配插件

    此样例未实现算子适配插件，若开发者想在第三方框架网络模型中使用此自定义算子，需要自定义实现对应的算子适配插件。


## 支持的芯片类型<a name="section13382182116471"></a>

若[tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)目录下存在对应芯片版本的算子信息库文件，则说明此算子支持对应的芯片版本。
