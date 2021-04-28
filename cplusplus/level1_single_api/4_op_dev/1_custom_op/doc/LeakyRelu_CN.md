# LeakyRelu<a name="ZH-CN_TOPIC_0302083167"></a>

## 功能描述<a name="section7526288579"></a>

本样例使用TBE DSL方式进行了LeakyRelu（带泄漏线性整流）算子的实现。

LeakyRelu算子的数学表达式如下所示：

-   当![](https://images.gitee.com/uploads/images/2020/1223/172610_5cf5fb2a_5474059.png)=0时

    f\(x\) = max\(0, x\)

-   当![](https://images.gitee.com/uploads/images/2020/1223/172704_b8bfda6c_5474059.png)!=0时

    ![](https://images.gitee.com/uploads/images/2020/1223/172738_ce94c6e6_5474059.png)


## 算子分析<a name="section1043174819574"></a>

1.  明确算子的功能以及数学表达式。

    算子功能请参见[功能描述](#section7526288579)。

2.  明确输入和输出。
    -   LeakyRelu算子有1个输入x，1个输出y，1个属性（即算子表达式中的![](https://images.gitee.com/uploads/images/2021/0426/102929_a40abaef_5474059.png) )。
    -   本样例中算子的输入支持的数据类型为float16、float32、 int32、int8，算子输出的数据类型与输入数据类型相同。
    -   算子输入支持所有shape，输出shape与输入shape相同。
    -   算子输入支持的format为：NCHW,NC1HWC0,NHWC,。

3.  确定算子开发方式及使用的计算接口。
    1.  计算过程只涉及到取大值、取小值以及乘法法操作，初分析DSL接口如下所示：

        使用te.lang.cce.vmuls\( \)接口实现乘法功能。

        使用te.lang.cce.vmin\( \)接口实现取较小值的功能。

        使用te.lang.cce.vmax\( \)接口实现取较大值的功能。

    2.  由于某些DSL接口会进行数据类型的转换，所以结果计算出来后，需要使用te.lang.cce.cast\_to接口将计算结果转换为原数据类型。
    3.  当![](https://images.gitee.com/uploads/images/2021/0426/102929_a40abaef_5474059.png)  = 0时，即为relu操作，可使用te.lang.cce.vrelu\(\)接口进行功能实现，由于vrelu接口会将int8、int32、float32转换为float16，若数据类型为int32或者float32时，会造成精度丢失，所以可考虑使用te.lang.cce.vmax\(\)接口取大值。

4.  明确算子实现文件名称、算子实现函数名称以及算子的类型（OpType）。

    -   算子类型采用大驼峰的命名方式。
    -   算子的实现文件名称及实现函数名称，将OpType采用如下规则进行转换：
        -   首字符的大写字符转换为小写字符。

            例如：Abc -\> abc

        -   小写字符后的大写字符转换为下划线+小写字符。

            例如：AbcDef -\> abc\_def

        -   紧跟数字以及大写字符后的大写字符，作为同一语义字符串，查找此字符串后的第一个小写字符，并将此小写字符的前一个大写字符转换为下划线+小写字符，其余大写字符转换为小写字符。若此字符串后不存在小写字符，则直接将此字符串中的大写字符转换为小写字符。

            例如：ABCDef -\> abc\_def；Abc2DEf -\> abc2d\_ef；Abc2DEF -\> abc2def；ABC2dEF -\> abc2d\_ef。



    本样例中，为不影响内置的LeakyReluDemo算子，算子类型定义为LeakyReluDemo；算子的实现文件名称及实现函数名称定义为leaky\_relu\_demo。


## 代码实现<a name="section657125913571"></a>

-   算子实现

    算子计算函数的实现逻辑如下所示，完整的代码实现请参见[leaky\_relu\_demo.py](../tbe/impl/leaky_relu_demo.py)。

    1.  当negative\_slope为“0”时，输出y取输入x与0之间的较大值。
        -   如果输入数据的类型为float16与int8，可直接调用te.lang.cce.vrelu\(x\)接口进行计算。
        -   如果输入数据的类型为float32与int32，此时若直接使用te.lang.cce.vrelu\(x\)接口会造成计算过程中精度的损失（因为te.lang.cce.vrelu\(x\)接口会将float32与int32的数据类型转换为float16的数据类型进行计算），所以可使用te.lang.cce.vmax\(\)接口将输入数据与_tensor\_zero_（构造的shape与x相同，元素值为0的tensor）进行比较。

    2.  当negative\_slope为非“0”时，根据[功能描述](#section7526288579)，需要区分negative\_slope小于等于1及negative\_slope大于1的情况。

        若negative\_slope小于等于1，取x与x\*negative\_slope的大值；若negative\_slope大于1，取x与x\*negative\_slope的小值。

        -   由于te.lang.cce.vmuls\( \)接口要求输入tensor与标量的数据类型相同，所以需要将negative\_slope的数据类型转换为输入x的数据类型（使用tvm.const接口进行转换）。
        -   由于te.lang.cce.vmuls\( \)接口会将int8、int32的数据类型转换为float16，所以执行完te.lang.cce.vmuls\( \)操作后，需要将输出的数据类型调用te.lang.cce.cast\_to\( \)接口转换为原始数据类型，然后再根据negative\_slope的值进行比较操作。


-   算子原型定义

    原型定义的关键点是推理输出Tensor的shape及dtype，LeakyReluDemo算子直接将输入tensor的shape与dtype赋给输出tensor。

    完整的代码实现请参见[leaky\_relu\_demo.h](../op_proto/leaky_relu_demo.h)与[leaky\_relu\_demo.cc](../op_proto/leaky_relu_demo.cc)。

-   算子信息库

    请参见[tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)对应芯片版本目录下的leaky\_relu\_demo.ini文件。

-   算子适配插件

    自定义实现ParseParamsLeakyRelu函数，实现原始Caffe中Type为LeakyReLUDemo的算子到适配昇腾AI处理器的LeakyReluDemo算子的属性映射。完整代码请参见[caffe\_leaky\_relu\_plugin.cc](../framework/caffe_plugin/caffe_leaky_relu_plugin.cc)。


## 支持的芯片类型<a name="section13382182116471"></a>

若[tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)目录下存在对应芯片版本的算子信息库文件，则说明此算子支持对应的芯片版本。

