# Add<a name="ZH-CN_TOPIC_0302083439"></a>

## 功能描述<a name="section690154102412"></a>

本样例使用TBE DSL方式进行了Add算子的实现，并对Add算子进行了单算子功能验证及单算子网络验证。

Add算子实现了两个数据相加，返回相加结果的功能，如下所示：

![](https://images.gitee.com/uploads/images/2020/1223/172326_9680591d_5474059.png "Add算子功能示例.png")


## 算子分析<a name="section1672275111254"></a>

1.  Add算子的数学表达式如下。

    ```
     z=x+y
    ```

    计算过程是：将两个输入参数相加，得到最终结果z并将其返回。

2.  明确输入和输出。
    -   Add算子有两个输入：x与y，输出为z。
    -   本样例中算子的输入支持的数据类型为float16、float32、 int32，算子输出的数据类型与输入数据类型相同。
    -   算子输入支持所有shape，输出shape与输入shape相同。
    -   算子输入支持的format为：NCHW,NC1HWC0,NHWC,ND。

3.  确定算子开发方式及使用的计算接口。
    1.  计算过程只涉及加法操作，初分析可使用te.lang.cce.vadd\(lhs, rhs\)接口实现x+y。
    2.  由于te.lang.cce.vadd\(lhs, rhs\)接口要求两个输入tensor的shape需要相同，所以需要首先获取两个输入tensor中较大的shape值，然后调用te.lang.cce.broadcast\(var, shape, output\_dtype=None\)接口将入参广播成指定的shape大小。

4.  明确算子实现文件名称、算子实现函数名称以及算子的类型（OpType）。

    -   算子类型采用大驼峰的命名方式。
    -   算子的实现文件名称及实现函数名称，将OpType采用如下规则进行转换：
        -   首字符的大写字符转换为小写字符。

            例如：Abc -\> abc

        -   小写字符后的大写字符转换为下划线+小写字符。

            例如：AbcDef -\> abc\_def

        -   紧跟数字以及大写字符后的大写字符，作为同一语义字符串，查找此字符串后的第一个小写字符，并将此小写字符的前一个大写字符转换为下划线+小写字符，其余大写字符转换为小写字符。若此字符串后不存在小写字符，则直接将此字符串中的大写字符转换为小写字符。

            例如：ABCDef -\> abc\_def；Abc2DEf -\> abc2d\_ef；Abc2DEF -\> abc2def；ABC2dEF -\> abc2d\_ef。



    因此本例中，算子的OpType定义为Add，算子的实现文件名称及实现函数名称定义为add。


## 代码实现<a name="section781751919323"></a>

-   算子实现

    Add算子仅支持float16, float32, int32三种数据类型，所以需要对算子的输入数据进行校验；由于Add算子允许两个输入数据的shape不同，但算子计算接口**te.lang.cce.vadd**要求两输入shape相同，因此需要对算子两个输入的shape进行广播并对其进行校验，算子实现代码可参见[add_dsl.py](../tbe/impl/add_dsl.py)。

-   算子原型定义

    原型定义的关键点是推理输出Tensor的shape以及对算子输入的内在关联关系进行校验。

    Add算子推理输出shape的原理为：首先获取两个输入的shape，然后将两个输入shape广播为相同的shape，输出shape取两个输入中每个维度的大值。InferShapeAndTypeAdd函数的代码实现请参见[add.cc](../op_proto/add.cc)。

-   算子信息库

    请参见[tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)对应芯片版本目录下的add.ini文件。

-   算子适配插件

    将原始TensorFlow的Add算子解析并映射为适配昇腾AI处理器的Add算子，算子属性的映射可直接调用AutoMappingFn\( \)接口进行实现，完整代码请参见[add\_plugin.cc](../framework/tf_plugin/add_plugin.cc)。


## 支持的芯片类型<a name="section13382182116471"></a>

若[tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)目录下存在对应芯片版本的算子信息库文件，则说明此算子支持对应的芯片版本。
