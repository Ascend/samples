# Upsample<a name="ZH-CN_TOPIC_0302083325"></a>

## 功能描述<a name="section882342317476"></a>

本样例使用TIK方式进行了Upsample算子的实现，在神经网络中，用于扩大特征图的方法。

## 算子分析<a name="section3574144154711"></a>

1.  明确算子的功能。

    Upsample算子用于在神经网络中，使用插值方法扩大特征图。

2.  明确输入和输出。
    -   Upsample算子有1个输入x，1个输出y，3个属性。
    -   算子输入的数据类型为float16和float32，算子输出的数据类型为float16和float32。
    -   算子输入支持所有shape。
    -   算子输入支持的format：NC1HWC0。
    -   算子的三个属性为scale，stride\_h，stride\_w。

3.  确定算子开发方式及使用的计算接口。
    1.  涉及对tensor的不同维度上的不同元素同时操作，TBE DSL接口与TVM社区原生语言接口都无法满足此算子的计算要求，所以考虑使用TIK方式进行此算子的实现。
    2.  该算子实现核心的计算流程如下：
        1.  使用data\_move\(\)接口将数据读入到Unified Buffer中。
        2.  使用vec\_muls\(\)接口乘以缩放系数。
        3.  使用data\_move\(\)接口把数据从Unified Buffer搬运数据到Global Memory中。


4.  明确算子实现文件名称、算子实现函数名称以及算子的类型（OpType）。

    -   算子类型采用大驼峰的命名方式。
    -   算子的实现文件名称及实现函数名称，将OpType采用如下规则进行转换：
        -   首字符的大写字符转换为小写字符。

            例如：Abc -\> abc

        -   小写字符后的大写字符转换为下划线+小写字符。

            例如：AbcDef -\> abc\_def

        -   紧跟数字以及大写字符后的大写字符，作为同一语义字符串，查找此字符串后的第一个小写字符，并将此小写字符的前一个大写字符转换为下划线+小写字符，其余大写字符转换为小写字符。若此字符串后不存在小写字符，则直接将此字符串中的大写字符转换为小写字符。

            例如：ABCDef -\> abc\_def；Abc2DEf -\> abc2d\_ef；Abc2DEF -\> abc2def；ABC2dEF -\> abc2d\_ef。



    本例中，为了不影响内置的Upsample算子，算子类型定义为UpsampleTik；算子的实现文件名称及实现函数名称定义为upsample\_tik。


## 代码实现<a name="section657125913571"></a>

-   算子实现

    UpsampleTik算子的实现代码请参见[upsample\_tik.py](../tbe/impl/upsample_tik.py)。

-   算子原型定义

    UpsampleTik的原型定义请参见[upsample\_tik.h](../op_proto/upsample_tik.h)与[upsample\_tik.cc](../op_proto/upsample_tik.cc)。

-   算子信息库

    请参见[tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)对应芯片版本目录下的upsample\_tik.ini文件。

-   算子适配插件

    将原始Caffe中Type为UpsampleTik的算子解析并映射为适配昇腾AI处理器的UpsampleTik算子，完整代码请参见[upsample\_plugin.cc](../framework/caffe_plugin/upsample_plugin.cc)。


## 支持的芯片类型<a name="section13382182116471"></a>

若[tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)目录下存在对应芯片版本的算子信息库文件，则说明此算子支持对应的芯片版本。

