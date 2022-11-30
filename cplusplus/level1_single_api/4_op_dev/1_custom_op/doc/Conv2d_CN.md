# Conv2d<a name="ZH-CN_TOPIC_0302083411"></a>

## 功能描述<a name="section1973805311174"></a>

本样例使用TIK方式进行了Conv2d算子的实现，将输入的张量和一个权重张量执行卷积2-D操作，输出结果张量

## 算子分析<a name="section19621012171817"></a>

1.  明确算子的功能。

    Conv2d算子计算过程是：将输入的张量和一个权重张量执行卷积2-D操作，输出结果张量，如[图1](#fig2099591662413)所示。

    **图 1**  Conv2d算子计算过程<a name="fig2099591662413"></a>  
    ![](https://images.gitee.com/uploads/images/2020/1223/174800_6754ce07_5474059.png "Conv2d算子计算过程")

2.  明确输入和输出。
    -   Conv2d算子有2个输入x和filter，1个输出y，3个属性。
    -   算子输入的数据类型为float16，算子输出的数据类型为float16。
    -   算子输入支持固定shape，输出shape与输入shape需要满足算子的数学表达式。
    -   算子输入支持的format：NCHW。
    -   算子的三个属性为strides，pads，dilations，属性值分别为\[1,1,1,1\]。

3.  确定算子开发方式及使用的计算接口。
    1.  计算过程主要涉及到二维卷积运算，初步分析可使用conv2d\(\)接口实现二维卷积运算功能。

        conv2d接口在处理strides和dilations两个属性时，在NC两个维度的值必须设定为1，同时当前样例中HW两个维度上的值指定为1。

    2.  由于在整个conv2d计算过程中会涉及到数据搬运操作，可使用data\_move\(\)接口实现从Global Memory搬运数据到L1 Buffer中。
    3.  计算完成后，可使用fixpipe\(\)接口把数据从L1OUT  Buffer搬运数据到Global Memory中。

4.  明确算子实现文件名称、算子实现函数名称以及算子的类型（OpType）。

    -   算子类型采用大驼峰的命名方式。
    -   算子的实现文件名称及实现函数名称，将OpType采用如下规则进行转换：
        -   首字符的大写字符转换为小写字符。

            例如：Abc -\> abc

        -   小写字符后的大写字符转换为下划线+小写字符。

            例如：AbcDef -\> abc\_def

        -   紧跟数字以及大写字符后的大写字符，作为同一语义字符串，查找此字符串后的第一个小写字符，并将此小写字符的前一个大写字符转换为下划线+小写字符，其余大写字符转换为小写字符。若此字符串后不存在小写字符，则直接将此字符串中的大写字符转换为小写字符。

            例如：ABCDef -\> abc\_def；Abc2DEf -\> abc2d\_ef；Abc2DEF -\> abc2def；ABC2dEF -\> abc2d\_ef。



    本样例中，为不影响内置的Conv2D算子，算子类型定义为Conv2DTik；算子的实现文件名称及实现函数名称定义为conv2d\_tik。


## 代码实现<a name="section657125913571"></a>

-   算子实现

    样例中Conv2DTik算子接收的数据类型为"float16"，首先需要对算子类型进行校验，然后对参数进行设置，并调用算子计算函数。

    算子计算函数的实现逻辑主要包含如下关键步骤：

    1.  根据参数对输入和输出tensor进行shape计算和占位。
    2.  通过for\_range\( \)循环开启double buffer和多核，对输入数据进行切分，实现卷积的高效运算。
    3.  调用conv2d\(\)实现二维卷积计算。
    4.  调用fixpipe\(\)实现计算结果数据的搬运。

    完整的实现代码请参见[conv2d\_tik.py](../tbe/impl/conv2d_tik.py)。

-   算子原型定义

    Conv2DTik的原型定义请参见[conv2d\_tik.h](../op_proto/conv2d_tik.h)与[conv2d\_tik.cc](../op_proto/conv2d_tik.cc)。

-   算子信息库

    请参见[tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)对应芯片版本目录下的conv2d\_tik.ini文件。

-   算子适配插件

    将原始Caffe中Type为ConvolutionTik对的算子解析并映射为适配昇腾AI处理器的Conv2DTik算子，完整代码请参见[caffe\_conv2d\_tik\_plugin.cc](../framework/caffe_plugin/caffe_conv2d_tik_plugin.cc)。


## 支持的芯片类型<a name="section13382182116471"></a>

若[tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)目录下存在对应芯片版本的算子信息库文件，则说明此算子支持对应的芯片版本。
