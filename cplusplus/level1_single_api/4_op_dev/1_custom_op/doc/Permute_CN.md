# Permute<a name="ZH-CN_TOPIC_0302083089"></a>

## 功能描述<a name="section1924110242164"></a>

本样例使用TIK方式进行了Permute算子的实现，用于置换索引轴顺序。

**说明：此样例当前仅支持NCHW转换为NHWC。**

## 算子分析<a name="section277922514179"></a>

1.  明确算子的功能。

    Permute算子，用于置换索引轴顺序

2.  明确输入和输出。
    -   Permute算子有1个输入x，1个输出y，1个属性order\(转换后的轴顺序, 样例仅支持\[0, 2, 3, 1\]，即NCHW-\>NHWC\)。
    -   算子输入的数据类型为float16，算子输出的数据类型为float16。
    -   算子输入支持所有shape，输出shape与输入shape相同。
    -   算子输入支持的format：NCHW。

3.  确定算子开发方式及使用的计算接口。

    由于Permute算子涉及对tensor的不同维度上的不同元素同时操作，TBE DSL接口与TVM社区原生语言接口都无法满足此算子的计算要求，所以考虑使用TIK方式进行此算子的实现。

    该算子实现核心的计算流程如下：

    1.  将数据读入到Unified Buffer中。
    2.  使用vec\_trans\_scatter\(\)接口实现从NCHW到NHWC的转换。
    3.  把数据从Unified Buffer搬运到Global Memory中。

4.  明确算子实现文件名称、算子实现函数名称以及算子的类型（OpType）。

    -   算子类型采用大驼峰的命名方式。
    -   算子的实现文件名称及实现函数名称，将OpType采用如下规则进行转换：
        -   首字符的大写字符转换为小写字符。

            例如：Abc -\> abc

        -   小写字符后的大写字符转换为下划线+小写字符。

            例如：AbcDef -\> abc\_def

        -   紧跟数字以及大写字符后的大写字符，作为同一语义字符串，查找此字符串后的第一个小写字符，并将此小写字符的前一个大写字符转换为下划线+小写字符，其余大写字符转换为小写字符。若此字符串后不存在小写字符，则直接将此字符串中的大写字符转换为小写字符。

            例如：ABCDef -\> abc\_def；Abc2DEf -\> abc2d\_ef；Abc2DEF -\> abc2def；ABC2dEF -\> abc2d\_ef。



    本样例中，为了不影响内置的Permute算子，算子类型定义为PermuteTik；算子的实现文件名称及实现函数名称定义为permute\_tik。


## 代码实现<a name="section657125913571"></a>

-   算子实现

    PermuteTik算子的实现代码请参见[permute\_tik.py](../tbe/impl/permute_tik.py)，计算函数的实现逻辑如下所示：

    1.  定义Permute类，并在初始化函数中初始化后续计算用到的参数。核心计算主要是计算每个输入的shape的大小，申请Global Memory大小。通过tbe\_platform.cce\_conf.get\_soc\_spec\(tbe\_platform.cce\_conf.UB\_SIZE\)接口获取到UB的实际物理空间。后续的步骤中，我们还会使用这些数据来计算data\_move、vec\_trans\_scatter等接口的参数。设置独立的tiling模块，将其与算子计算逻辑分离可以很好的做到算子的shape泛化。对于不同的shape，我们可以在不改变计算逻辑的情况下，只改变tiling参数来优化搬运和计算的次数，来做到泛化和高性能。

        ```
        class Permute:
            """
            Function: store permute parameters  and compute permute
            """
        
            def __init__(self, input_dict):
                ...
            def get_shape_info(self):
                ...
            ...
            ...
            def permute_compute(self):
                ...
        ```

    2.  根据shape信息，分三种场景去处理。
        -   场景1：C=1 或者 H\*W=1的场景，不需要转换。

            直接根据输入数据的大小进行tiling计算，如果要使用多核，通过for\_range循环开启double buffer和多核，对输入数据进行切分，实现高效运算。并且定义UB tensor的操作必须定义在多核循环内，防止编译时出现冲突。

        -   其他场景：C \>= 16的场景与C < 16场景。

            以上两种场景tiling策略一致，对N轴进行多核切分，转换过程中，尾块处理逻辑不同。

            get\_block\_num\_and\_loop\_cycle\( \)函数主要是确定是否开启多核和double buffer，确定每个核内处理的循环个数。

            compute\_c\_ge\_16\( \)函数计算C \>= 16场景，把HW当做一个轴处理，优先切C轴，确保放下足够大的C轴数据。

            compute\_c\_lt\_16\( \)函数计算C < 16场景，把HW当做一个轴处理，当H\*W放不下时，需要切H\*W数据。



-   算子原型定义

    PermuteTik的原型定义请参见[permute\_tik.h](../op_proto/permute_tik.h)与[permute\_tik.cc](../op_proto/permute_tik.cc)。

-   算子信息库

    请参见[tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)对应芯片版本目录下的permute\_tik.ini文件。

-   算子适配插件

    将原始Caffe中Type为Permute的算子解析并映射为适配昇腾AI处理器的PermuteTik算子，完整代码请参见[caffe\_permute\_tik\_plugin.cc](../framework/caffe_plugin/caffe_permute_tik_plugin.cc)。


## 支持的芯片类型<a name="section13382182116471"></a>

若[tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)目录下存在对应芯片版本的算子信息库文件，则说明此算子支持对应的芯片版本。
