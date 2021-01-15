# Unique<a name="ZH-CN_TOPIC_0303147571"></a>

## 功能描述<a name="section6945232175"></a>

本样例描述了AI CPU自定义算子Unique的实现，Unique算子的原始框架为Tensorflow。

Unique算子的功能是：在输入的一维张量x中找到唯一的元素，并返回张量y，其中y包含所有x中的唯一元素，并按照与x相同的顺序排列。同时返回一个与张量x具有相同大小的张量idx，包含x中每个元素在张量y中的索引。

例如：

-   输入Tensor：\[1, 1, 2, 4, 4, 4, 7, 8, 8\]

-   进行UniqueCust计算后，输出如下：
    -   张量y：\[1, 2, 4, 7, 8\]
    -   张量idx：\[0, 0, 1, 2, 2, 2, 3, 4, 4\]


## 算子分析<a name="section5726181618178"></a>

1.  明确算子的功能。

    Unique算子的功能是：在输入的一维张量x中找到唯一的元素，并返回张量y，其中y包含所有x中的唯一元素，并按照与x相同的顺序排列。同时返回一个与张量x具有相同大小的张量idx，包含x中每个元素在张量y中的索引。

2.  明确输入和输出。
    -   Unique算子有1个输入：x。

        本样例中算子的输入x支持的数据类型为：float16, float32, int8, int16, uint16, uint8, int32, int64, float64。

    -   两个算子输出：张量y与张量idx。

        y支持的数据类型与输入x相同。

        idx支持的数据类型为int32与int64。


3.  明确算子实现文件名称以及算子的类型（OpType）。

    -   算子类型需要采用大驼峰的命名方式，即采用大写字符区分不同的语义。
    -   建议将OpType按照如下方式进行转换，得到算子文件名称。
        -   首字符的大写字符转换为小写字符。

            例如：Abc -\> abc

        -   小写字符后的大写字符转换为下划线+小写字符。

            例如：AbcDef -\> abc\_def

        -   紧跟数字以及大写字符后的大写字符，作为同一语义字符串，查找此字符串后的第一个小写字符，并将此小写字符的前一个大写字符转换为下划线+小写字符，其余大写字符转换为小写字符。若此字符串后不存在小写字符，则直接将此字符串中的大写字符转换为小写字符。

            例如：ABCDef -\> abc\_def；Abc2DEf -\> abc2d\_ef；Abc2DEF -\> abc2def；ABC2dEF -\> abc2d\_ef。



    为不影响内置算子，本样例中算子类型定义为UniqueCust；算子的实现文件名称定义为unique\_cust。


## 代码实现<a name="section83331113171811"></a>

-   算子实现

    UniqueCust算子的实现代码请参见[unique\_cust\_kernels.h](../cpukernel/impl/unique_cust_kernels.h)与[unique\_cust\_kernels.cc](../cpukernel/impl/unique_cust_kernels.cc)。

    UniqueCust的输出y的shape与输入x的内容相关，为动态shape算子，输出y的shape信息在计算过程中推导并更新。

-   算子原型定义

    原型定义的关键点是推理输出Tensor的shape。

    UniqueCust算子推理输出shape的原理为：根据功能描述可知，输出idx与输入x的shape相同，输出y与输入x的元素内容重复度相关，为未知shape。UniqueCust算子原型定义的代码实现请参见[unique\_cust.h](../op_proto/unique_cust.h)与[unique\_cust.cpp](../op_proto/unique_cust.cpp)。

-   算子信息库

    UniqueCust的算子信息库请参见[unique\_cust.ini](../cpukernel/op_info_cfg/aicpu_kernel/unique_cust.ini)。

-   算子适配插件

    本样例提供了TensorFlow算子适配插件，实现原始TensorFlow算子到适配昇腾AI处理器的算子的映射，完整代码请参见[unique\_cust\_plugin.cpp](../framework/tf_plugin/unique_cust_plugin.cpp)。


## 支持的芯片类型<a name="section13382182116471"></a>

All

