# Reshape<a name="ZH-CN_TOPIC_0303147570"></a>

## 功能描述<a name="section17348171613103"></a>

本样例描述了AI CPU自定义算子Reshape的实现，Reshape算子的原始框架为Tensorflow。

Reshape算子的功能是对输入张量进行整形的操作，只改变张量的形状，不改变数据。

例如：

-   输入 Tensor：\[1, 2, 3, 4, 5, 6, 7, 8, 9\]。
-   目标Shape：\[3,3\]。
-   则返回的Tensor为：\[\[1,2,3\], \[4,5,6\], \[7,8,9\]\]。

## 算子分析<a name="section118066248105"></a>

1.  明确算子的功能。

    Reshape算子的功能是根据输入的目标shape对输入tensor进行整形，仅仅改变tensor的形状，不改变数据，并返回整形后的tensor。

2.  明确输入和输出。
    -   Reshape算子有两个输入：输入tensor以及目标shape。
    -   本样例中算子的输入支持的数据类型为：bool, float16, float32, int8, int32, uint32, uint8, int64, int16, uint16, float64, complex64, complex128, qint8, qint16, quint16, qint32。
    -   目标shape的数据类型为int32与int64。
    -   算子输出tensor的数据类型与输入tensor相同。

3.  明确算子实现文件名称以及算子的类型（OpType）。

    -   算子类型需要采用大驼峰的命名方式，即采用大写字符区分不同的语义。
    -   建议将OpType按照如下方式进行转换，得到算子文件名称。
        -   首字符的大写字符转换为小写字符。

            例如：Abc -\> abc

        -   小写字符后的大写字符转换为下划线+小写字符。

            例如：AbcDef -\> abc\_def

        -   紧跟数字以及大写字符后的大写字符，作为同一语义字符串，查找此字符串后的第一个小写字符，并将此小写字符的前一个大写字符转换为下划线+小写字符，其余大写字符转换为小写字符。若此字符串后不存在小写字符，则直接将此字符串中的大写字符转换为小写字符。

            例如：ABCDef -\> abc\_def；Abc2DEf -\> abc2d\_ef；Abc2DEF -\> abc2def；ABC2dEF -\> abc2d\_ef。



    为不影响内置算子，本样例中算子类型定义为ReshapeCust；算子的实现文件名称定义为reshape\_cust。


## 代码实现<a name="section1349813621017"></a>

-   算子实现

    ReshapeCust算子的实现代码请参见[reshape\_cust\_kernels.h](../cpukernel/impl/reshape_cust_kernels.h)与[reshape\_cust\_kernels.cc](../cpukernel/impl/reshape_cust_kernels.cc)。

-   算子原型定义

    原型定义的关键点是推理输出Tensor的shape。

    ReshapeCust算子推理输出shape的原理为：首先获取两个输入，一个是输入tensor，一个是目标的shape，校验输入tensor元素个数是否与目标shape相同，校验成功则将输出shape设置为目标shape值。

    ReshapeCust算子原型定义的代码实现请参见[reshape\_cust.h](../op_proto/reshape_cust.h)与[reshape\_cust.cc](../op_proto/reshape_cust.cc)。

-   算子信息库

    ReshapeCust的算子信息库请参见[reshape\_cust.ini](../cpukernel/op_info_cfg/aicpu_kernel/reshape_cust.ini)。

-   算子适配插件

    本样例同时提供了TensorFlow与Caffe的ReshapeCust算子适配插件，实现原始Tensorflow/Caffe算子到适配昇腾AI处理器的算子的映射，完整代码可分别参见[reshape\_cust\_plugin.cc](../framework/tf_plugin/reshape_cust_plugin.cc)文件与[caffe\_reshape\_cust\_plugin.cc](../framework/caffe_plugin/caffe_reshape_cust_plugin.cc)文件。


## 支持的芯片类型<a name="section13382182116471"></a>

All

