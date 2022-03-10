"""
Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the Apache License Version 2.0.You may not use
this file except in compliance with the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
Apache License for more details at
http://www.apache.org/licenses/LICENSE-2.0

permute_tik
"""

from tbe import tik
from tbe.common.platform import platform_info
from tbe.common.utils import para_check

# available ub size
UB_SIZE_B = platform_info.get_soc_spec(platform_info.UB_SIZE)
# available number of cores
AICORE_NUM = platform_info.get_soc_spec(platform_info.CORE_NUM)


# pylint: disable=invalid-name,too-many-locals,too-many-arguments
@para_check.check_op_params(para_check.REQUIRED_INPUT, para_check.REQUIRED_OUTPUT,
                            para_check.OPTION_ATTR_LIST_INT, para_check.KERNEL_NAME)
def permute_tik(x, y, order=(0), kernel_name="permute_tik"):
    """
    only support nchw->nhwc

    Parameters
    ----------
    x : dict
        shape and dtype of input
    y : dict
        shape and dtype of output, should be same shape and type as input
    order: tuple, list
           axis transformation order
    kernel_name : str
        kernel name, default value is "permute_tik"

    Returns
    -------
    None
    """
    shape = x.get("shape")
    dtype = y.get("dtype")
    input_dtype = dtype.lower()
    supported_dtype = ["float16"]
    input_format = x.get("format")
    check_pass = False
    if input_format == 'NCHW':
        if len(order) == 4 and order[0] == 0 \
                and order[1] == 2 and order[2] == 3 and order[3] == 1:
            check_pass = True
    if not check_pass:
        raise RuntimeError("only support nchw->nhwc")
    para_check.check_dtype_rule(input_dtype, supported_dtype)
    para_check.check_dtype_rule(dtype, supported_dtype)
    para_check.check_shape_rule(shape)
    para_check.check_shape_size(shape)
    para_check.check_kernel_name(kernel_name)

    input_dict = {
        "x": x,
        "y": y,
        "order": order
    }
    permute_process = Permute(input_dict)
    permute_process.permute_compute()
    permute_process.instance.BuildCCE(kernel_name=kernel_name,
                                      inputs=permute_process.x_gm,
                                      outputs=permute_process.y_gm)

    return permute_process.instance


def get_shape_size(shape):
    """
    get the number of element from the shape

    Parameters
    ----------
    shape: output shape

    Returns
    -------
    total_number: the number of element of the shape
    """
    total_number = 1
    for val in shape:
        total_number = total_number * val

    return total_number


def get_block_num_and_loop_cycle(shape):
    """
    get block dim and loop cycle

    Parameters
    ----------
    shape: input shape

    Returns
    -------
    block_num: the number of cores
    inner_loop: the number of cycles per core
    inner_loop_mod: the number of remaining cycles
    thread_num: whether to enable double buffer  1:false 2:true
    """
    batch, col_len, row_len = shape
    size = batch * col_len * row_len
    block_num = AICORE_NUM
    inner_loop = 1
    inner_loop_mod = 0
    thread_num = 1

    if size <= 16:
        block_num = 1
        return block_num, inner_loop, inner_loop_mod, thread_num

    all_block_num = shape[0]
    if col_len * row_len >= 16:
        if all_block_num < AICORE_NUM:
            block_num = all_block_num
    else:
        chw = col_len * row_len
        num = (16 + chw) // chw
        if batch // num < AICORE_NUM:
            block_num = batch // num
    inner_loop = all_block_num // block_num
    inner_loop_mod = all_block_num % block_num
    if inner_loop > 1:
        thread_num = 2
    return block_num, inner_loop, inner_loop_mod, thread_num


class Permute:
    """
    Function: store permute parameters  and compute permute
    """

    def __init__(self, input_dict):
        """
        init the permute parameters
        """
        self.instance = tik.Tik()
        self.dtype = input_dict.get("x").get("dtype").lower()
        self.dsize = 2
        size = get_shape_size(input_dict.get("x").get("shape"))
        self.x_gm = self.instance.Tensor(self.dtype, (size,), name="x_gm",
                                         scope=tik.scope_gm)
        self.y_gm = self.instance.Tensor(self.dtype, (size,), name="y_gm",
                                         scope=tik.scope_gm)
        ub_size = (UB_SIZE_B - 1024) // 4 // self.dsize // 256 * 256
        self.ub_size = ub_size
        self.input_dict = input_dict

    def get_shape_info(self):
        """
        determine whether to convert the shape based on the input shape
        """
        shape = self.input_dict.get("x").get("shape")
        if shape[1] == 1 or shape[2] * shape[3] == 1:
            shape_size = get_shape_size(shape)
            shape_new = [shape_size]
            order_new = [0]
            shape_out_new = [shape_size]
        else:
            n_i = shape[0]
            col_len = shape[1]
            row_len = shape[2] * shape[3]
            shape_new = [n_i, col_len, row_len]
            order_new = [0, 2, 1]
            shape_out_new = []
            for i in order_new:
                shape_out_new.append(shape_new[i])
        return shape_new, order_new, shape_out_new

    def move_without_transform(self, shape):
        """
        when C = 1 or H*W = 1, directly move data in and out
        """
        ub_size = (UB_SIZE_B - 1024) // 2 // self.dsize // 16 * 16
        if shape[0] <= 16:
            block_num = 1
        else:
            all_block_num = shape[0] // 16
            block_num = AICORE_NUM
            if all_block_num < AICORE_NUM:
                block_num = all_block_num
        each_len = shape[0] // block_num
        each_mod = shape[0] % block_num
        thread_num = 1
        if each_len // ub_size > 1:
            thread_num = 2

        with self.instance.for_range(0, block_num, block_num=block_num) \
                as block_id:
            each_size = self.instance.Scalar("int32")
            each_size.set_as(each_len)
            with self.instance.if_scope(block_id == block_num - 1):
                each_size.set_as(each_len + each_mod)
            ub_loop = each_size // ub_size
            ub_mod = each_size % ub_size
            with self.instance.for_range(0,
                                         ub_loop,
                                         thread_num=thread_num) as loop_id:
                src_ub = self.instance.Tensor(self.dtype, (ub_size,),
                                              name="src_ub",
                                              scope=tik.scope_ubuf)
                burst_len = ub_size // 16
                self.instance.data_move(
                    src_ub,
                    self.x_gm[each_len * block_id + loop_id * ub_size],
                    0, 1, burst_len, 0, 0)
                self.instance.data_move(
                    self.y_gm[each_len * block_id + loop_id * ub_size],
                    src_ub,
                    0, 1, burst_len, 0, 0)
            with self.instance.if_scope(ub_mod > 0):
                src_ub = self.instance.Tensor(self.dtype, (ub_size,),
                                              name="src_ub",
                                              scope=tik.scope_ubuf)
                with self.instance.if_scope(
                        tik.all(block_num > 1, ub_mod % 16 != 0)):
                    src_ub_1 = self.instance.Tensor(self.dtype, (16,),
                                                    name="src_ub_1",
                                                    scope=tik.scope_ubuf)
                    index = each_len * block_id + ub_loop * ub_size
                    with self.instance.if_scope(ub_mod >= 16):
                        burst_len = ub_mod // 16
                        self.instance.data_move(src_ub,
                                                self.x_gm[index],
                                                0, 1, burst_len, 0, 0)
                        self.instance.data_move(self.y_gm[index],
                                                src_ub,
                                                0, 1, burst_len, 0, 0)
                        offset = index + burst_len * 16 - 16 + ub_mod % 16
                        self.instance.data_move(src_ub_1,
                                                self.x_gm[offset],
                                                0, 1, 1, 0, 0)
                        self.instance.data_move(self.y_gm[offset],
                                                src_ub_1,
                                                0, 1, 1, 0, 0)
                    with self.instance.else_scope():
                        offset = index - 16 + ub_mod % 16
                        self.instance.data_move(src_ub_1,
                                                self.x_gm[offset],
                                                0, 1, 1, 0, 0)
                        self.instance.data_move(self.y_gm[offset],
                                                src_ub_1,
                                                0, 1, 1, 0, 0)
                with self.instance.else_scope():
                    burst_len = (ub_mod + 15) // 16
                    self.instance.data_move(
                        src_ub,
                        self.x_gm[
                            each_len * block_id + ub_loop * ub_size],
                        0, 1, burst_len, 0, 0)
                    self.instance.data_move(
                        self.y_gm[
                            each_len * block_id + ub_loop * ub_size],
                        src_ub,
                        0, 1, burst_len, 0, 0)

    def trans_scatter(self, col_len_ub, row_len_ub, src_ub, dst_ub):
        """
        transposes the data
        """
        c_zu = col_len_ub // 16
        r_zu = row_len_ub // 16
        with self.instance.for_range(0, r_zu) as num_r:
            repeat = c_zu
            src_stride = 0
            dst_stride = 0
            if repeat != 1:
                src_stride = 16 * r_zu
                dst_stride = 1
            dst_list = [dst_ub[16 * col_len_ub * num_r + 16 * c_zu * i] for i in range(16)]
            src_list = [src_ub[16 * num_r + 16 * r_zu * j] for j in range(16)]
            self.instance.vec_trans_scatter(False, False, dst_list,
                                            src_list, repeat,
                                            dst_stride, src_stride)

    def move_gm_to_ub(self, row_len, col_len_ub, row_len_ub, src_ub, index):
        """
        move data from gm to ub
        """
        stride = (row_len - row_len_ub) // 16
        row_len_ub_align = (row_len_ub + 15) // 16 * 16
        if row_len % 16 == 0 and stride < 65535:
            n_burst = col_len_ub
            burst_len = row_len_ub_align // 16
            self.instance.data_move(src_ub,
                                    self.x_gm[index],
                                    0, n_burst, burst_len, stride, 0)
        else:
            with self.instance.for_range(0, col_len_ub) as c_i:
                burst_len = row_len_ub_align // 16
                self.instance.data_move(
                    src_ub[c_i * row_len_ub_align],
                    self.x_gm[index + c_i * row_len],
                    0, 1, burst_len, 0, 0)

    def move_ub_to_gm(self, col_len, col_len_ub, row_len_ub, index, dst_ub):
        """
        move data from ub to gm when c >= 16
        """
        stride = (col_len - col_len_ub) // 16
        if col_len % 16 == 0 and stride < 65535:
            n_burst = row_len_ub
            burst_len = col_len_ub // 16
            self.instance.data_move(self.y_gm[index],
                                    dst_ub,
                                    0, n_burst, burst_len, 0, stride)
        else:
            with self.instance.for_range(0, row_len_ub) as r_i:
                burst_len = col_len_ub // 16
                self.instance.data_move(
                    self.y_gm[index + r_i * col_len],
                    dst_ub[r_i * col_len_ub],
                    0, 1, burst_len, 0, 0)

    def move_ub_to_gm_with_tail(self, input_dict):
        """
        move data from ub to gm when c < 16
        """
        shape = input_dict.get("shape")
        dst_ub = input_dict.get("dst_ub")
        ub_tail = input_dict.get("ub_tail")
        tail_offset = input_dict.get("tail_offset")
        tail_num = input_dict.get("tail_num")
        block_num = input_dict.get("block_num")
        row_index = input_dict.get("row_index")
        out_index = input_dict.get("out_index")
        tail_start = input_dict.get("tail_start")
        total_loop = input_dict.get("total_loop")
        r_i = input_dict.get("r_i")
        num = input_dict.get("num")
        _, col_len, row_len = shape
        col_len_align = (col_len + 15) // 16 * 16
        with self.instance.if_scope(
                tik.all(row_index >= num, block_num > 1)):
            scalar = self.instance.Scalar(ub_tail.dtype)
            with self.instance.for_range(0, col_len) as time:
                scalar.set_as(dst_ub[r_i * col_len_align + time])
                ub_tail[tail_offset + time].set_as(scalar)
            tail_offset.set_as(tail_offset + col_len)
            with self.instance.if_scope(
                    row_index == total_loop * row_len - 1):
                each_burst_num = 32 // self.dsize
                n_burst = self.instance.Scalar("int32")
                n_burst.set_as((tail_num * self.dsize) // 32)
                mod = self.instance.Scalar("int32")
                mod.set_as((tail_num * self.dsize) % 32)
                # 32b alignment
                with self.instance.if_scope(mod == 0):
                    self.instance.data_move(self.y_gm[tail_start], ub_tail, 0,
                                            1, n_burst, 0, 0)
                # bigger than 32b
                with self.instance.else_scope():
                    self.instance.data_move(self.y_gm[tail_start], ub_tail, 0,
                                            1, n_burst, 0, 0)
                    offset = tail_num - each_burst_num
                    scalar = self.instance.Scalar(ub_tail.dtype)
                    with self.instance.for_range(0, each_burst_num) as time:
                        scalar.set_as(ub_tail[offset + time])
                        ub_tail[time].set_as(scalar)
                    self.instance.data_move(self.y_gm[tail_start + offset],
                                            ub_tail, 0, 1, 1, 0, 0)
        with self.instance.else_scope():
            burst_len = col_len_align // 16
            self.instance.data_move(
                self.y_gm[out_index],
                dst_ub[r_i * col_len_align],
                0, 1, burst_len, 0, 0)

    def compute_c_lt_16(self, input_dict):
        """
        processing the scenario where c is less than 16
        """
        n_id = input_dict.get("n_id")
        total_loop = input_dict.get("each_loop")
        tail_offset = input_dict.get("tail_offset")
        ub_tail = input_dict.get("ub_tail")
        shape = input_dict.get("shape")
        x_index = input_dict.get("x_index")
        block_num = input_dict.get("block_num")
        _, col_len, row_len = shape
        col_len_align = (col_len + 15) // 16 * 16
        row_len_ub = self.ub_size // col_len_align // 16 * 16
        row_loop = row_len // row_len_ub
        row_mod = row_len % row_len_ub
        last_num = (16 + col_len - 1) // col_len
        num = total_loop * row_len - last_num
        src_ub = self.instance.Tensor(self.dtype, (self.ub_size,),
                                      name="src_ub",
                                      scope=tik.scope_ubuf)
        dst_ub = self.instance.Tensor(self.dtype, (self.ub_size,),
                                      name="dst_ub",
                                      scope=tik.scope_ubuf)
        if row_loop > 0:
            with self.instance.for_range(0, row_loop) as r_loop:
                in_index = x_index + n_id * col_len * row_len + \
                           row_len_ub * r_loop
                self.move_gm_to_ub(row_len, col_len, row_len_ub, src_ub,
                                   in_index)
                self.trans_scatter(col_len_align, row_len_ub, src_ub,
                                   dst_ub)
                with self.instance.for_range(0, row_len_ub) as r_i:
                    row_index = n_id * row_len + row_len_ub * r_loop + r_i
                    out_index = x_index + n_id * col_len * row_len + \
                                col_len * row_len_ub * r_loop + r_i * col_len
                    tail_start = x_index + total_loop * row_len * col_len - \
                                 last_num * col_len
                    input_dict = {
                        "shape": shape,
                        "dst_ub": dst_ub,
                        "ub_tail": ub_tail,
                        "tail_offset": tail_offset,
                        "tail_num": col_len * last_num,
                        "block_num": block_num,
                        "row_index": row_index,
                        "out_index": out_index,
                        "tail_start": tail_start,
                        "total_loop": total_loop,
                        "r_i": r_i,
                        "num": num,
                    }
                    self.move_ub_to_gm_with_tail(input_dict)

        if row_mod > 0:
            in_index = x_index + n_id * col_len * row_len + \
                       row_len_ub * row_loop
            self.move_gm_to_ub(row_len, col_len, row_mod, src_ub, in_index)
            row_mod_align = (row_mod + 15) // 16 * 16
            self.trans_scatter(col_len_align, row_mod_align, src_ub, dst_ub)
            with self.instance.for_range(0, row_mod) as r_i:
                row_index = n_id * row_len + row_len_ub * row_loop + r_i
                out_index = x_index + n_id * col_len * row_len + \
                            col_len * row_len_ub * row_loop + r_i * col_len
                tail_start = x_index + total_loop * row_len * col_len - \
                             last_num * col_len
                input_dict = {
                    "shape": shape,
                    "dst_ub": dst_ub,
                    "ub_tail": ub_tail,
                    "tail_offset": tail_offset,
                    "tail_num": col_len * last_num,
                    "block_num": block_num,
                    "row_index": row_index,
                    "out_index": out_index,
                    "tail_start": tail_start,
                    "total_loop": total_loop,
                    "r_i": r_i,
                    "num": num,
                }
                self.move_ub_to_gm_with_tail(input_dict)

    def compute_c_ge_16(self, shape, x_index):
        """
        processing the scenario where the value of c is greater than
        or equal to 16
        """
        _, col_len, row_len = shape
        ub_div_16 = self.ub_size // 16
        col_div_16 = col_len // 16 * 16
        col_len_ub = ub_div_16 if ub_div_16 < col_div_16 else col_div_16
        ub_div_col = self.ub_size // col_len_ub // 16 * 16
        row_len_ub = ub_div_col if ub_div_col < row_len else row_len
        row_len_ub_align = (row_len_ub + 15) // 16 * 16
        col_loop = col_len // col_len_ub
        col_mod = col_len % col_len_ub
        row_loop = row_len // row_len_ub
        row_mod = row_len % row_len_ub
        src_ub = self.instance.Tensor(self.dtype, (self.ub_size,),
                                      name="src_ub",
                                      scope=tik.scope_ubuf)
        dst_ub = self.instance.Tensor(self.dtype, (self.ub_size,),
                                      name="dst_ub",
                                      scope=tik.scope_ubuf)
        if col_loop > 0:
            with self.instance.for_range(0, col_loop) as c_loop:
                with self.instance.for_range(0, row_loop) as r_loop:
                    in_index = x_index + c_loop * col_len_ub * row_len + \
                               row_len_ub * r_loop
                    self.move_gm_to_ub(row_len, col_len_ub, row_len_ub, src_ub,
                                       in_index)
                    self.trans_scatter(col_len_ub, row_len_ub_align, src_ub,
                                       dst_ub)
                    out_index = x_index + col_len * row_len_ub * r_loop + \
                                c_loop * col_len_ub
                    self.move_ub_to_gm(col_len, col_len_ub, row_len_ub,
                                       out_index, dst_ub)
                if row_mod > 0:
                    in_index = x_index + c_loop * col_len_ub * row_len + \
                               row_len_ub * row_loop
                    row_mod_align = (row_mod + 15) // 16 * 16
                    self.move_gm_to_ub(
                        row_len, col_len_ub, row_mod, src_ub, in_index)
                    self.trans_scatter(col_len_ub, row_mod_align, src_ub,
                                       dst_ub)
                    out_index = x_index + col_len * row_len_ub * row_loop + \
                                c_loop * col_len_ub
                    self.move_ub_to_gm(col_len, col_len_ub, row_mod,
                                       out_index, dst_ub)
        if col_mod > 0:
            col_mod_align = (col_mod + 15) // 16 * 16
            offset = col_mod_align - col_mod
            with self.instance.for_range(0, row_loop) as r_loop:
                in_index = x_index + (col_loop * col_len_ub - offset) * \
                           row_len + row_len_ub * r_loop
                self.move_gm_to_ub(
                    row_len, col_mod_align, row_len_ub, src_ub, in_index)
                self.trans_scatter(col_mod_align, row_len_ub_align, src_ub,
                                   dst_ub)
                out_index = x_index + col_len * row_len_ub * r_loop + \
                            col_loop * col_len_ub - offset
                self.move_ub_to_gm(col_len, col_mod_align, row_len_ub,
                                   out_index, dst_ub)
            if row_mod > 0:
                in_index = x_index + (col_loop * col_len_ub - offset) * \
                           row_len + row_len_ub * row_loop
                self.move_gm_to_ub(row_len, col_mod_align, row_mod, src_ub,
                                   in_index)
                self.trans_scatter(col_mod_align, row_mod_align, src_ub,
                                   dst_ub)
                out_index = x_index + col_len * row_len_ub * row_loop + \
                            col_loop * col_len_ub - offset
                self.move_ub_to_gm(col_len, col_mod_align, row_mod,
                                   out_index, dst_ub)

    def permute_compute(self):
        """
        compute permute
        """
        shape, order, _ = self.get_shape_info()
        if order != [0, 2, 1]:
            self.move_without_transform(shape)
        else:
            _, col_len, row_len = shape
            block_num, inner_loop, tail, thread_num = \
                get_block_num_and_loop_cycle(shape)
            element_num = col_len * row_len

            with self.instance.for_range(0, block_num, block_num=block_num) \
                    as block_id:
                each_loop = self.instance.Scalar("int32")
                each_loop.set_as(inner_loop)
                offset = self.instance.Scalar("int32")
                if tail > 0:
                    with self.instance.if_scope(block_id < tail):
                        each_loop.set_as(each_loop + 1)
                offset.set_as(block_id * each_loop)
                if tail > 0:
                    with self.instance.if_scope(block_id >= tail):
                        offset.set_as(block_id * each_loop + tail)
                x_index = self.instance.Scalar("int32")
                x_index.set_as(offset * element_num)
                ub_tail = self.instance.Tensor(self.dtype, (256,),
                                               name="ub_tail",
                                               scope=tik.scope_ubuf)
                tail_offset = self.instance.Scalar("int32")
                tail_offset.set_as(0)
                with self.instance.for_range(0,
                                             each_loop,
                                             thread_num=thread_num) as n_id:
                    if col_len >= 16:
                        index = self.instance.Scalar("int32")
                        index.set_as(x_index + n_id * col_len * row_len)
                        self.compute_c_ge_16(shape, index)
                    else:
                        input_dict = {
                            "n_id": n_id,
                            "each_loop": each_loop,
                            "tail_offset": tail_offset,
                            "ub_tail": ub_tail,
                            "shape": shape,
                            "x_index": x_index,
                            "block_num": block_num
                        }
                        self.compute_c_lt_16(input_dict)
