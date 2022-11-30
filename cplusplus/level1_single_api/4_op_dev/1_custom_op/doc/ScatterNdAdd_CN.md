# ScatterNdAdd<a name="ZH-CN_TOPIC_0302083117"></a>

## 功能描述<a name="section13485172361417"></a>

本样例使用TIK方式进行了ScatterNdAdd算子的实现。

ScatterNdAdd算子通过对输入数据中的单个值或切片应用稀疏算法，从而得到输出数据。

该算子具有var、indices和updates三个关键输入。其功能为使用updates更新var中indices指定位置的数据，即在var指定位置的数据上加上update的值。

三个输入之间的关系分别为：

-   张量var的shape的维度为P。
-   indices是整数张量，shape的维度（rank）为Q，索引为ref，最后一维的元素个数为K\(0<K<=P\)，shape为\[d\_0, ..., d\_\{Q-2\}, K\]。
-   张量updates的shape的维度（rank）为Q-1+P-K，shape为\[d\_0, ..., d\_\{Q-2\}, ref.shape\[K\], ..., ref.shape\[P-1\]\]。

例子:
![](https://images.gitee.com/uploads/images/2020/1223/172953_97dc7628_5474059.png "zh-cn_image_0302223512.png")

如上面示例所示，indices总共有4个index，每个index有两个数表示一个二维坐标，指示var中所要更新的位置。

第一个index\(0, 0\)表示更新的var的起始位置为\(0, 0, 0\)到\(0, 0, 3\)总共四个数。updates的第一个分片为\(1, 1, 1, 1\)，所以更新完之后的output的结果为\(2, 2, 2, 2\)。

第二个index\(0, 1\)表示更新的var的起始位置为\(0, 1, 0\)到\(0, 1, 3\)总共四个数。updates的第二个分片为\(2, 2, 2, 2\)，所以更新完之后的output的结果为\(3, 3, 3, 3\)。

第三个index\(0, 2\)表示更新的var的起始位置为\(0, 2, 0\)到\(0, 2, 3\)总共四个数。updates的第三个分片为\(3, 3, 3, 3\)，所以更新完之后的output的结果为\(4, 4, 4, 4\)。

第四个index\(1, 1\)表示更新的var的起始位置为\(1, 1, 0\)到\(1, 1, 3\)总共四个数。updates的第四个分片为\(4, 4, 4, 4\)，所以更新完之后的output的结果为\(5, 5, 5, 5\)。

## 算子分析<a name="section876395515144"></a>

1.  明确算子的功能。

    ScatterNdAdd算子的功能是通过对输入数据中的单个值或切片应用稀疏算法，从而得到输出数据，详细功能示例可参考[功能描述](#section13485172361417)。

2.  明确输入和输出。
    -   ScatterNdAdd算子有三个输入，一个输出。

        输入：var（需要更新的tensor），indices（指定需要更新的索引位置）,updates（更新数据）。

        输出：var（输出更新后的tensor）。

    -   本样例中算子的输入var支持的数据类型为float16、float32、int32、int8、uint8；输入indices支持的数据类型为int32；输入updates支持的数据类型为float16、float32、int32、int8、uint8；输出var支持的数据类型为：float16、float32、int32、int8、uint8。
    -   算子输入支持所有shape，输出shape与输入shape相同。
    -   算子输入支持的format为：ND。

3.  确定算子开发方式及使用的计算流程设计。

    由于ScatterNdAdd算子涉及对tensor的不同维度上的不同元素同时计算，TBE DSL接口无法满足此算子的计算要求，所以考虑使用TIK方式进行此算子的实现。

    该算子实现核心的计算流程如下：

    1.  将indices读入到UB Buffer中，然后遍历计算var中需要更新位置的index。
    2.  每算出一个index，就将相应的需要更新的var分片和updates分片搬入到UB Buffer中。
    3.  将两个分片每个对应位置的元素相加后再搬出到GM内存中。
    4.  当遍历完所有的index后，就可以得到最终的计算结果。

    针对这个核心计算流程，我们要设计相应的schedule策略。schedule策略在设计的时候主要考虑两个基本问题：

    1.  首先是shape的泛化。

        由于我们的UB空间有大小限制，所以不是所有的输入shape都能在UB上放下，需要考虑分片搬运入UB buffer进行计算。此时，就需要我们根据输入的shape大小，数据类型，UB buffer空间来计算每次分片搬运的大小和需要搬入的次数。在UB空间划分的时候，要充分合理的利用UB空间来提升性能。相同的输入shape，分10次搬入UB计算完之后再搬回到GM，比分100次搬运和计算性能更优。因此，要满足不同的shape泛化，我们要根据输入的shape来计算和划分UB buffer空间，计算各个指令的参数。

    2.  其次是多核，double buffer等策略。

        当前昇腾AI处理器有多个AI Core可以做并行计算，可以极大的提升计算的性能。对于ScatterNdAdd这个算子，总共需要三层循环，如下所示

        **表 1**  ScatterNdAdd算子循环列表

        <a name="zh-cn_topic_0229823847_table8448134862913"></a>
        <table><thead align="left"><tr id="zh-cn_topic_0229823847_row104481048192916"><th class="cellrowborder" valign="top" width="50%" id="mcps1.2.3.1.1"><p id="zh-cn_topic_0229823847_p11448248112913"><a name="zh-cn_topic_0229823847_p11448248112913"></a><a name="zh-cn_topic_0229823847_p11448248112913"></a>空间</p>
        </th>
        <th class="cellrowborder" valign="top" width="50%" id="mcps1.2.3.1.2"><p id="zh-cn_topic_0229823847_p7448184816291"><a name="zh-cn_topic_0229823847_p7448184816291"></a><a name="zh-cn_topic_0229823847_p7448184816291"></a>循环</p>
        </th>
        </tr>
        </thead>
        <tbody><tr id="zh-cn_topic_0229823847_row1844819484291"><td class="cellrowborder" valign="top" width="50%" headers="mcps1.2.3.1.1 "><p id="zh-cn_topic_0229823847_p74491483291"><a name="zh-cn_topic_0229823847_p74491483291"></a><a name="zh-cn_topic_0229823847_p74491483291"></a>GM (拆分var)</p>
        </td>
        <td class="cellrowborder" valign="top" width="50%" headers="mcps1.2.3.1.2 "><p id="zh-cn_topic_0229823847_p3449348132910"><a name="zh-cn_topic_0229823847_p3449348132910"></a><a name="zh-cn_topic_0229823847_p3449348132910"></a>多核LOOP</p>
        </td>
        </tr>
        <tr id="zh-cn_topic_0229823847_row17449194812914"><td class="cellrowborder" valign="top" width="50%" headers="mcps1.2.3.1.1 "><p id="zh-cn_topic_0229823847_p1449144822913"><a name="zh-cn_topic_0229823847_p1449144822913"></a><a name="zh-cn_topic_0229823847_p1449144822913"></a>GM2UB (indices)</p>
        </td>
        <td class="cellrowborder" valign="top" width="50%" headers="mcps1.2.3.1.2 "><p id="zh-cn_topic_0229823847_p844964852920"><a name="zh-cn_topic_0229823847_p844964852920"></a><a name="zh-cn_topic_0229823847_p844964852920"></a>indices LOOP</p>
        </td>
        </tr>
        <tr id="zh-cn_topic_0229823847_row1344954852912"><td class="cellrowborder" valign="top" width="50%" headers="mcps1.2.3.1.1 "><p id="zh-cn_topic_0229823847_p13449748112910"><a name="zh-cn_topic_0229823847_p13449748112910"></a><a name="zh-cn_topic_0229823847_p13449748112910"></a>GM2UB (var + update)</p>
        </td>
        <td class="cellrowborder" valign="top" width="50%" headers="mcps1.2.3.1.2 "><p id="zh-cn_topic_0229823847_p1449164872912"><a name="zh-cn_topic_0229823847_p1449164872912"></a><a name="zh-cn_topic_0229823847_p1449164872912"></a>Split indices LOOP</p>
        </td>
        </tr>
        </tbody>
        </table>

        核心是遍历indices，对每个index取出var和update做计算。为了实现多核并行计算，我们将var进行分拆，让不同AI Core进行不同的index位置的计算。例如，假设我们根据前面的公式算出index的取值范围为\[0, 6\]，这时候我们把index=0对应的updates分片放在第一个核处理，index=2对应的updates分片放在第二个核处理，以此类推，最后一个核处理index=6对应的updates分片，这样可以实现多个核的并行处理。


4.  明确算子实现文件名称、算子实现函数名称以及算子的类型（OpType）。

    -   算子类型采用大驼峰的命名方式。
    -   算子的实现文件名称及实现函数名称，将OpType采用如下规则进行转换：
        -   首字符的大写字符转换为小写字符。

            例如：Abc -\> abc

        -   小写字符后的大写字符转换为下划线+小写字符。

            例如：AbcDef -\> abc\_def

        -   紧跟数字以及大写字符后的大写字符，作为同一语义字符串，查找此字符串后的第一个小写字符，并将此小写字符的前一个大写字符转换为下划线+小写字符，其余大写字符转换为小写字符。若此字符串后不存在小写字符，则直接将此字符串中的大写字符转换为小写字符。

            例如：ABCDef -\> abc\_def；Abc2DEf -\> abc2d\_ef；Abc2DEF -\> abc2def；ABC2dEF -\> abc2d\_ef。



    因此本例中，算子类型定义为ScatterNdAdd；算子的实现文件名称及实现函数名称定义为scatter\_nd\_add。


## 代码实现<a name="section657125913571"></a>

-   算子实现

    ScatterNdAdd的算子实现的关键点是进行算子schedule策略的实现，包含tiling参数的计算、多核实现等，完整的实现代码请参见[scatter\_nd\_add.py](../tbe/impl/scatter_nd_add.py)。

    1.  定义Scatter类，并在初始化函数中进行tiling参数计算。

        核心计算主要是计算每个输入的shape的大小，再根据数据类型计算需要UB多少空间。我们可以通过tbe\_platform.cce\_conf.get\_soc\_spec\(\)接口获取到UB的实际物理空间后，根据UB大小来划分UB空间，为定义UB上的tensor做准备。后续的步骤中，我们还会使用这些数据来计算data\_move、vec\_add等接口的参数。设置独立的tiling模块，将其与算子计算逻辑分离可以很好的做到算子的shape泛化。对于不同的shape，我们可以在不改变计算逻辑的情况下，只改变tiling参数来优化搬运和计算的次数，来做到泛化和高性能。

        ```
        class Scatter():
            def __init__(self, var, indices, updates, var_out, nd_flag, kernel_name,
                         compute_type):
                ...
        ```

    2.  计算过程实现。

        根据tiling的计算结果，我们判断要不要使用多核。如果要使用多核，就需要设置多核循环。并且定义UB tensor的操作必须定义在多核循环内，防止编译时出现冲突。对于多核场景，每次循环都会遍历输入张量indices，在计算出index后判断该index是否在当前核的处理范围内再进行计算。

        ```
            def scatter_operator(self):
                # 根据tiling计算结果判断能否开多核，如果需要开多核，需要指定多核循环
                if self.block_num > 1:
                    with self.tik_instance.for_range(
                            0, self.block_num,
                            block_num=self.block_num) as indices_loop_index:
                      # 初始化UB中的tensor
                        self.init_ub_tensor()
                        self.indices_loop_index.set_as(indices_loop_index)
                        # 遍历indices索引计算
                        self.traversing_indices()
                else:
                    self.init_ub_tensor()
                    self.traversing_indices()
        
                # 通过BuildCCE接口进行算子编译，最终生成算子目标文件.o与算子描述文件.json
                self.tik_instance.BuildCCE(
                    kernel_name=self.kernel_name,
                    inputs=(self.var_gm, self.indices_gm, self.updates_gm),
                    outputs=(self.out_gm),
                    enable_l2=False)
        
                return self.tik_instance
        ```


-   算子原型定义

    ScatterNdAdd算子的输入tensor var与updates的数据类型要求相同，所以需要对齐进行校验。然后将输入tensor var的shape与数据类型更新到输出tensor。

    完整的代码实现请参见[scatter\_nd\_add.h](../op_proto/scatter_nd_add.h)与[scatter\_nd\_add.cc](../op_proto/scatter_nd_add.cc)。

-   算子信息库

    请参见[tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)对应芯片版本目录下的scatter\_nd\_add.ini文件。

-   算子适配插件

    将原始TensorFlow的ScatterNdAdd算子或者ResourceScatterNdAdd算子解析并映射为适配昇腾AI处理器的ScatterNdAdd算子，算子属性的映射可直接调用AutoMappingFn\( \)接口进行实现。完整代码请参见[scatter\_nd\_add\_plugin.cc](../framework/tf_plugin/scatter_nd_add_plugin.cc)。


## 支持的芯片类型<a name="section13382182116471"></a>

若[tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)目录下存在对应芯片版本的算子信息库文件，则说明此算子支持对应的芯片版本。

