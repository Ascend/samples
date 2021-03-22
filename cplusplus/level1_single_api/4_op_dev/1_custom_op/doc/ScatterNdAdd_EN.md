# ScatterNdAdd<a name="EN-US_TOPIC_0302083117"></a>

## Overview<a name="section13485172361417"></a>

In this sample, the ScatterNdAdd operator is implemented in TIK mode.

The ScatterNdAdd operator applies the sparse algorithm to a single value or slice in the input data to obtain the output data.

This operator has three main inputs:  **var**,  **indices**, and  **updates**.  **updates**  is used to update the data at the location specified by  **indices**  in  **var**. In other words, the values of  **updates**  are added to the values at the specified locations in  **var**.

Description of the three inputs:

-   The number of dimensions of tensor  **var**'s shape is P.
-   **indices**  is an integer tensor, whose shape has Q dimensions \(rank\). The index is ref, the number of elements in the last dimension is K \(0 < K ≤ P\), and shape is \[d\_0, ..., d\_\{Q – 2\}, K\].
-   The shape of the tensor  **updates**  has \(Q – 1 + P – K\) dimensions \(rank\), that is, \[d\_0, ..., d\_\{Q – 2\}, ref.shape\[K\], ..., ref.shape\[P – 1\]\].

For example:

![](https://images.gitee.com/uploads/images/2020/1223/172953_97dc7628_5474059.png)

As shown in the preceding example,  **indices**  have four indexes. Each of them has two numbers \(a two-dimensional coordinate\), indicating the location to be updated in  **var**.

The first index \(0, 0\) indicates that the four values — \(0, 0, 0\) to \(0, 0, 3\)— in  **var**  are to be updated. The first slice of  **updates**  is \(1, 1, 1, 1\). After the update, the output result is \(2, 2, 2, 2\).

The second index \(0, 1\) indicates that the four values —\(0, 1, 0\) to \(0, 1, 3\)— in  **var**  are to be updated. The second slice of  **updates**  is \(2, 2, 2, 2\). After the update, the output result is \(3, 3, 3, 3\).

The third index \(0, 2\) indicates that the four values—\(0, 2, 0\) to \(0, 2, 3\)— in  **var**  are to be updated. The third slice of  **updates**  is \(3, 3, 3, 3\). After the update, the output result is \(4, 4, 4, 4\).

The fourth index \(1, 1\) indicates that the four values—\(1, 1, 0\) to \(1, 1, 3\)— in  **var**  are to be updated. The fourth slice of  **updates**  is \(4, 4, 4, 4\). After the update, the output result is \(5, 5, 5, 5\).

## Operator Analysis<a name="section876395515144"></a>

1.  Specify the operator function.

    The ScatterNdAdd operator applies the sparse algorithm to a single value or slice in the input data to obtain the output data. For details about the function example, see  [Overview](#section13485172361417).

2.  Specify the inputs and output.
    -   The ScatterNdAdd operator has three inputs and one output.

        Inputs:  **var**  \(tensor to be updated\),  **indices**  \(indexes specifying the locations to be updated\), and  **updates**  \(data to be updated\)

        Output:  **var**  \(output tensor after the update\)

    -   The inputs and output support different data types.  **var**: float16, float32, int32, int8, and uint8;  **indices**: int32;  **updates**: float16, float32, int32, int8, and uint8;  **var**: float16, float32, int32, int8, and uint8.
    -   The operator input supports all shapes. The output has the same shape as the inputs.
    -   The operator input supports  **ND**  format.

3.  Determine the operator development mode and compute process.

    The ScatterNdAdd operator needs to compute different elements in different dimensions of a tensor, which is not supported by TBE DSL APIs. Therefore, the operator is implemented by using TIK.

    The core computation process is as follows:

    1.  Read  **indices**  to the unified buffer \(UB\), and traverse and compute the indexes for specifying locations to be updated in  **var**.
    2.  Each time an index is obtained, move the corresponding  **var**  slice and  **updates**  slice to be updated to the UB.
    3.  Add the elements in the corresponding locations of the two slices and move the two slices to the GM.
    4.  Obtain the final compute process result after all indexes are traversed.

    A schedule strategy is required for this core compute process, and is designed based on the following considerations:

    1.  Shape generalization:

        The UB size is limited, so not all input shapes can be stored in the UB. In this case, split the input shape into slices before moving them into the UB for compute process, and compute the slice size to move each time and the number of times based on the input shape size, data type, and UB size. The UB must be properly divided and fully used to improve performance. For the same input shape, the fewer the movement times, the better the performance. To meet different shape generalization requirements, you need to divide the UB properly based on the input shapes, and compute the parameters of each instruction.

    2.  AI Core parallelism and double buffering policies:

        Computing on multiple AI Cores on the  Ascend AI Processor  in parallel greatly improves the compute performance. The ScatterNdAdd operator needs three layers of loops, as shown in the following table.

        **Table  1**  List of ScatterNdAdd operator loops

        <a name="en-us_topic_0229823847_table8448134862913"></a>
        <table><thead align="left"><tr id="en-us_topic_0229823847_row104481048192916"><th class="cellrowborder" valign="top" width="50%" id="mcps1.2.3.1.1"><p id="en-us_topic_0229823847_p11448248112913"><a name="en-us_topic_0229823847_p11448248112913"></a><a name="en-us_topic_0229823847_p11448248112913"></a>Space</p>
        </th>
        <th class="cellrowborder" valign="top" width="50%" id="mcps1.2.3.1.2"><p id="en-us_topic_0229823847_p7448184816291"><a name="en-us_topic_0229823847_p7448184816291"></a><a name="en-us_topic_0229823847_p7448184816291"></a>Loop</p>
        </th>
        </tr>
        </thead>
        <tbody><tr id="en-us_topic_0229823847_row1844819484291"><td class="cellrowborder" valign="top" width="50%" headers="mcps1.2.3.1.1 "><p id="en-us_topic_0229823847_p74491483291"><a name="en-us_topic_0229823847_p74491483291"></a><a name="en-us_topic_0229823847_p74491483291"></a>GM (splitting var)</p>
        </td>
        <td class="cellrowborder" valign="top" width="50%" headers="mcps1.2.3.1.2 "><p id="en-us_topic_0229823847_p3449348132910"><a name="en-us_topic_0229823847_p3449348132910"></a><a name="en-us_topic_0229823847_p3449348132910"></a>AI Core parallelism loop</p>
        </td>
        </tr>
        <tr id="en-us_topic_0229823847_row17449194812914"><td class="cellrowborder" valign="top" width="50%" headers="mcps1.2.3.1.1 "><p id="en-us_topic_0229823847_p1449144822913"><a name="en-us_topic_0229823847_p1449144822913"></a><a name="en-us_topic_0229823847_p1449144822913"></a>GM2UB (indices)</p>
        </td>
        <td class="cellrowborder" valign="top" width="50%" headers="mcps1.2.3.1.2 "><p id="en-us_topic_0229823847_p844964852920"><a name="en-us_topic_0229823847_p844964852920"></a><a name="en-us_topic_0229823847_p844964852920"></a>indices LOOP</p>
        </td>
        </tr>
        <tr id="en-us_topic_0229823847_row1344954852912"><td class="cellrowborder" valign="top" width="50%" headers="mcps1.2.3.1.1 "><p id="en-us_topic_0229823847_p13449748112910"><a name="en-us_topic_0229823847_p13449748112910"></a><a name="en-us_topic_0229823847_p13449748112910"></a>GM2UB (var + update)</p>
        </td>
        <td class="cellrowborder" valign="top" width="50%" headers="mcps1.2.3.1.2 "><p id="en-us_topic_0229823847_p1449164872912"><a name="en-us_topic_0229823847_p1449164872912"></a><a name="en-us_topic_0229823847_p1449164872912"></a>Split indices LOOP</p>
        </td>
        </tr>
        </tbody>
        </table>

        The purpose is to traverse  **indices**  and fetch  **var**  and  **updates**  values specified by each index for compute process. To implement AI Core parallelism,  **var**  is split so that different AI Cores can perform compute process at different locations specified by indexes. For example, supposing that the index value range is \[0, 6\], the  **updates**  slice specified by index=0 is processed by the first core, the  **updates**  slice specified by index=2 is processed by the second core, ..., the  **updates**  slice specified by index=6 is processed by the last core. In this way, parallel processing by multiple cores is implemented.


4.  Specify the operator implementation file name, operator implementation function name, and  _OpType_.

    -   _OpType_  is named in upper camel case.
    -   The implementation file name and implementation function name of the operator are converted based on the following rules:
        -   Replace the first uppercase letter with a lowercase letter.

            Example: Abc -\> abc

        -   Replace each uppercase letter following lowercase letters with an underscore \(\_\) and a lowercase letter.

            Example: AbcDef -\> abc\_def

        -   Uppercase letters following a digit or an uppercase letter are regarded as a character string. If there is a lowercase letter after this string, replace the last uppercase letter in this string with an underscore \(\_\) and a lowercase letter, and replace the other uppercase letters with lowercase letters. If there is no lowercase letter after the string, directly replace the string with lowercase letters.

            Examples: ABCDef -\> abc\_def; Abc2DEf -\> abc2d\_ef; Abc2DEF -\> abc2def; ABC2dEF -\> abc2d\_ef



    In this example,  _OpType_  of the operator is defined as  **ScatterNdAdd**. The first letter of the operator implementation file name and implementation function name is converted into lowercase letters, so the name is  **scatter\_nd\_add**.


## Code Implementation<a name="section657125913571"></a>

-   Operator Implementation

    The key of implementing the ScatterNdAdd operator is to implement the operator schedule strategies, including the compute process of tiling parameters and AI Core parallelism implementation. For details about the complete implementation code, see  [scatter\_nd\_add.py](../tbe/impl/scatter_nd_add.py).

    1.  Define the  **Scatter**  class and compute the tiling parameters in the initialization function.

        The key is to compute the shape size of each input and the space to be allocated in the UB based on the data type. First, call the  **tbe\_platform.cce\_conf.get\_soc\_spec\(\)**  API to obtain the actual physical space of the UB. Then, divide the UB as a preparation for defining the tensor on the UB. In subsequent steps, the calculated tiling parameters \(including the shape size and size of the UB\) will be used to configure the parameters of APIs such as  **data\_move**  and  **vec\_add**. Separate the tiling logic from the operator compute logic to implement shape generalization of the operator. In this way, data with different shapes can be computed in the same compute logic. You only need to change the tiling parameters to optimize the number of movement and compute process times, thereby achieving generalization and high performance.

        ```
        class Scatter():
            def __init__(self, var, indices, updates, var_out, nd_flag, kernel_name,
                         compute_type):
                ...
        ```

    2.  Implement the compute process.

        Determine whether AI Core parallelism is required based on the computation results of tiling parameters. If yes, you need to set an AI Core parallelism loop. In addition, UB tensors must be defined in the AI Core parallelism loop to avoid conflicts during build. In the AI Core parallelism scenario, the input tensor  **indices**  are traversed in each loop. Obtain an index, and determine whether the index belongs to the current AI Core before compute process.

        ```
            def scatter_operator(self):
                # Determine whether AI Core parallelism is required based on the computation results of tiling parameters. If yes, set a AI Core parallelism loop.
                if self.block_num > 1:
                    with self.tik_instance.for_range(
                            0, self.block_num,
                            block_num=self.block_num) as indices_loop_index:
                      # Initialize the tensor in the UB.
                        self.init_ub_tensor()
                        self.indices_loop_index.set_as(indices_loop_index)
                        # Traverse indexes of indices for compute process.
                        self.traversing_indices()
                else:
                    self.init_ub_tensor()
                    self.traversing_indices()
        
                # Call the BuildCCE API to build the operator and generate an operator binary (.o) and operator description file (.json).
                self.tik_instance.BuildCCE(
                    kernel_name=self.kernel_name,
                    inputs=(self.var_gm, self.indices_gm, self.updates_gm),
                    outputs=(self.out_gm),
                    enable_l2=False)
        
                return self.tik_instance
        ```


-   Operator Prototype Definition

    The data type of the input tensors  **var**  and  **updates**  of the ScatterNdAdd operator must be the same. Therefore, their data types need to be verified. Update the shape and data type of the input tensor  **var**  to the output tensor.

    For details about the complete code implementation, see  [scatter\_nd\_add.h](../op_proto/scatter_nd_add.h)  and  [scatter\_nd\_add.cc](../op_proto/scatter_nd_add.cc).

-   Operator Information Library

    For details, see the  **scatter\_nd\_add.ini**  file in the corresponding chip version directory  [tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core).

-   Operator Plug-in

    Parse and map the ScatterNdAdd operator or ResourceScatterNdAdd operator developed under the TensorFlow framework to the ScatterNdAdd operator supported by the Ascend AI Processor. You can implement the operator attribute mapping by directly calling  **AutoMappingFn\(\)**. For details about the complete code, see  [scatter\_nd\_add\_plugin.cc](../framework/tf_plugin/scatter_nd_add_plugin.cc).


## Supported SoCs<a name="section13382182116471"></a>

If the operator information library file of the corresponding chip version exists in the  [tbe/op\_info\_cfg/ai\_core](../tbe/op_info_cfg/ai_core)  directory, the operator supports the corresponding chip version.

