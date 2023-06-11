# SoftmaxTik<a name="ZH-CN_TOPIC_0302083439"></a>

## 功能描述<a name="section690154102412"></a>

本样例使用TIK方式进行了Softmax算子的简单实现，并对Softmax算子进行了单算子功能验证及单算子网络验证。

Softmax用于多分类过程中，它将上一层的原始数据进行归一化，映射到(0, 1)区间内，从而来进行多分类。


## 算子分析<a name="section1672275111254"></a>

1.  Softmax算子的数学表达式如下，用Numpy代码的形式表示。

    ```
     y = np.exp(x) / np.sum(np.exp(x))
    ```

    计算过程是：首先将输入转化为以e作为底数的指数函数，然后再求得其在所有输入转化为e底数指数函数后的总和的比例。

2.  明确输入和输出。
    -   Softmax算子有一个输入：x，输出为y。
    -   本样例中算子的输入支持的数据类型为float16、float32，算子输出的数据类型与输入数据类型相同。
    -   算子输入支持的一维shape为[128,]，输出shape与输入shape相同。
    -   算子输入支持的format为：ND。

3.  确定算子开发方式及使用的计算接口。
    1.  计算过程涉及指数，广播，规约，除法等操作，初分析可使用vec_exp，vector_dup，vec_reduce_add，vdiv接口实现。
    2.  由于计算接口中需要输入计算的Block个数，repeat次数等，所以需要额外计算burst，repeat_times等。

4.  明确算子实现文件名称、算子实现函数名称以及算子的类型（OpType）。

    -   算子类型采用大驼峰的命名方式。
    -   算子的实现文件名称及实现函数名称，将OpType采用如下规则进行转换：
        -   首字符的大写字符转换为小写字符。

            例如：Abc -\> abc

        -   小写字符后的大写字符转换为下划线+小写字符。

            例如：AbcDef -\> abc\_def

        -   紧跟数字以及大写字符后的大写字符，作为同一语义字符串，查找此字符串后的第一个小写字符，并将此小写字符的前一个大写字符转换为下划线+小写字符，其余大写字符转换为小写字符。若此字符串后不存在小写字符，则直接将此字符串中的大写字符转换为小写字符。

            例如：ABCDef -\> abc\_def；Abc2DEf -\> abc2d\_ef；Abc2DEF -\> abc2def；ABC2dEF -\> abc2d\_ef。



    因此本例中，算子的OpType定义为SoftmaxTik，算子的实现文件名称及实现函数名称定义为softmax_tik。


## 代码实现<a name="section781751919323"></a>

-   算子实现

    SoftmaxTik算子支持float16和float32两种数据类型；SoftmaxTik算子shape支持一维，shape为[128,]，算子实现代码可参见[softmax_tik.py](../tbe/impl/softmax_tik.py)。

-   算子原型定义

    原型定义的关键点是推理输出Tensor的shape以及对算子输入的内在关联关系进行校验。

    SoftmaxTik算子推理输出shape的原理为：输入输出的shape保持一致，均为一维。SoftmaxTikInferShape函数的代码实现请参见[softmax_tik.cc](../op_proto/softmax_tik.cc)。

-   算子信息库

    请参见[tbe/op\_info\_cfg/vector\_core](../tbe/op_info_cfg/vector_core)对应芯片版本目录下的softmax_tik.ini文件。


## 支持的芯片类型<a name="section13382182116471"></a>

若[tbe/op\_info\_cfg/vector\_core](../tbe/op_info_cfg/vector_core)目录下存在对应芯片版本的算子信息库文件，则说明此算子支持对应的芯片版本。
