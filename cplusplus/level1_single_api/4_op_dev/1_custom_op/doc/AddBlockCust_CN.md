# AddBlackCust

## 功能描述

本样例描述了AI CPU自定义算子AddBlockCust的实现，用以举例说明自定义AI CPU算子如何支持分块并行计算（开启多核并行计算）。

1.  开启多核的条件。
    -   数据在计算过程中，相互之间不存在关联，可以分割成多个数据块进行独立计算，若输入参数间存在数据依赖，则无法进行分块并行计算。
    -   每一块数据生成的结果在计算前可预知存放的位置和大小，如果不能预知，需要对计算结果进行拼接，拼接过程会带来额外的性能损耗。

2.  如何开启多核。
    -   使用此功能时，需要在算子信息库定义中设置opInfo.flagSupportBlockDim=True，并设置opInfo.functionName=RunCpuKernelWithBlock。
    -   opInfo.blockDimByIndex该字段表示根据第一个输入参数shape的某个维度来进行切分，默认为-1。

AddBlockCustk算子实现了两个数据相加，返回相加结果的功能，如下所示：

![](https://images.gitee.com/uploads/images/2020/1223/172326_9680591d_5474059.png "Add算子功能示例.png")


## 算子分析

1.  AddBlockCust算子的数学表达式如下。

    ```
     z=x+y
    ```

    计算过程是：将两个输入参数相加，得到最终结果z并将其返回。

2.  明确输入和输出。
    -   AddBlockCust算子有两个输入：x与y，输出为z。
    -   本样例中算子的输入支持的数据类型为float32、 int32、int64，算子输出的数据类型与输入数据类型相同。
    -   算子输入支持所有shape，输出shape与输入shape相同。
    -   算子输入支持的format为：NCHW,NC1HWC0,NHWC,ND。

3.  明确算子实现文件名称、算子实现函数名称以及算子的类型（OpType）。

    -   算子类型采用大驼峰的命名方式。
    -   算子的实现文件名称及实现函数名称，将OpType采用如下规则进行转换：
        -   首字符的大写字符转换为小写字符。

            例如：Abc -\> abc

        -   小写字符后的大写字符转换为下划线+小写字符。

            例如：AbcDef -\> abc\_def

        -   紧跟数字以及大写字符后的大写字符，作为同一语义字符串，查找此字符串后的第一个小写字符，并将此小写字符的前一个大写字符转换为下划线+小写字符，其余大写字符转换为小写字符。若此字符串后不存在小写字符，则直接将此字符串中的大写字符转换为小写字符。

            例如：ABCDef -\> abc\_def；Abc2DEf -\> abc2d\_ef；Abc2DEF -\> abc2def；ABC2dEF -\> abc2d\_ef。



    本样例中算子类型定义为AddBlockCust，算子的实现文件名称定义为add\_block\_cust。


## 代码实现

-   算子实现

    AddBlockCust算子的实现代码请参见[add\_block\_cust\_kernels.h](../cpukernel/impl/add_block_cust_kernels.h)与[add\_block\_cust\_kernels.cc](../cpukernel/impl/add_block_cust_kernels.cc)。

    此算子支持分块并行计算，详细计算逻辑可参见[add\_block\_cust\_kernels.cc](../cpukernel/impl/add_block_cust_kernels.cc)中的AddComputeWithBlock函数。

-   算子原型定义

    原型定义的关键点是推理输出Tensor的shape以及对算子输入的内在关联关系进行校验。

    AddBlockCust算子推理输出shape的原理为：首先获取两个输入的shape，然后将两个输入shape广播为相同的shape，输出shape取两个输入中每个维度的大值。InferShapeAndTypeAddBlock函数的代码实现请参见[add_block_cust.cc](../op_proto/add_block_cust.cc)。

-   算子信息库

    AddBlockCust的算子信息库请参见[add\_block\_cust.ini](../cpukernel/op_info_cfg/aicpu_kernel/add_block_cust.ini)。

## 支持的芯片类型<a name="section13382182116471"></a>

All