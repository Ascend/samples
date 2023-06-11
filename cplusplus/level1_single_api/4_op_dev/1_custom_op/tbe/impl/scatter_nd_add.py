#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
Copyright (C) 2019. Huawei Technologies Co., Ltd. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the Apache License Version 2.0.You may not use this file

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
Apache License for more details at
http://www.apache.org/licenses/LICENSE-2.0

scatter_nd_add
"""
import math
from functools import reduce as functools_reduce

from tbe import tik
import tbe.common.platform as tbe_platform
from tbe.common.platform import platform_info
from tbe.common.utils import para_check

# neg two
NEG_TWO = -2

# neg one
NEG_ONE = -1


# pylint: disable=too-many-arguments,too-many-instance-attributes
class Scatter():
    """
       Function: use to store scatter base parameters
       Modify : 2019-10-28
    """

    # pylint: disable=too-many-statements
    def __init__(self, var, indices, updates, var_out, nd_flag, kernel_name,
                 compute_type):
        """
        Init scatter base parameters

        Parameters
        ----------
        var: dict
            data of input
            datatype suports float32,float16,int32,int8,uint8
        indices: dict
            data of indices
            datatype supports int32
        updates: dict
            data of updates
            datatype supports float32,float16,int32,int8,uint8
        var_out: dict
            data of input
        nd_flag: bool
            if this op is nd operator
        kernel_name: str
            the name of the operator
        compute_type: str
            the compute type of scatter
        Returns
        -------
        None
        """
        self.tik_instance = tik.Tik()
        self.nd_flag = nd_flag
        self.var_shape = var.get("shape")
        self.var_dtype = var.get("dtype").lower()
        self.indices_shape = indices.get("shape")
        self.indices_dtype = indices.get("dtype").lower()
        self.updates_shape = updates.get("shape")
        self.updates_dtype = updates.get("dtype").lower()
        self.var_ele_num = functools_reduce(lambda x, y: x * y, self.var_shape)
        self.indices_num = functools_reduce(lambda x, y: x * y,
                                            self.indices_shape)
        self.updates_num = functools_reduce(lambda x, y: x * y,
                                            self.updates_shape)
        self.kernel_name = kernel_name

        if self.indices_shape == (1,) and \
                len(self.var_shape)-len(self.updates_shape) == 1:
            if not nd_flag:
                self.updates_shape = (1,) + self.updates_shape

        self.check_param(var_out)
        if nd_flag:
            if self.indices_shape[-1] == len(self.var_shape):
                self.update_data_num = 1
            else:
                self.update_data_num = functools_reduce(
                    lambda x, y: x * y, self.var_shape[self.indices_shape[-1]:])
            self.max_indice = functools_reduce(
                lambda x, y: x * y, self.var_shape[0:self.indices_shape[-1]])
            self.index_dims = self.indices_shape[-1]
        else:
            if len(self.var_shape) > 1:
                self.update_data_num = functools_reduce(lambda x, y: x * y,
                                                        self.var_shape[1:])
            else:
                self.update_data_num = 1
            self.max_indice = self.var_shape[0]
            self.index_dims = 1

        self.compute_type = compute_type

        self.ub_size_bytes = (
                platform_info.get_soc_spec(
                    platform_info.UB_SIZE) - 8192)
        self.var_dtype_bytes_size = tbe_platform.get_bit_len(
            self.var_dtype) // 8
        self.indices_dtype_bytes_size = tbe_platform.get_bit_len(
            self.indices_dtype) // 8
        self.var_data_each_block = 32 // self.var_dtype_bytes_size
        self.indices_data_each_block = 32 // self.indices_dtype_bytes_size
        self.indices_ub_number = 0
        self.updates_ub_number = 0

        self.index_loop_num = 0

        self.max_num_one_repeat = 128
        if self.var_dtype in ("float32", "int32"):
            self.max_num_one_repeat = 64

        if self.update_data_num < self.var_data_each_block:
            self.block_num = 1
        else:
            ai_core_num = platform_info.get_soc_spec(
                platform_info.CORE_NUM)
            self.indice_step = math.ceil(self.max_indice / ai_core_num)
            self.block_num = math.ceil(self.max_indice / self.indice_step)

        self.var_gm = self.tik_instance.Tensor(
            self.var_dtype, self.var_shape, name="var_gm", scope=tik.scope_gm)
        self.indices_gm = self.tik_instance.Tensor(
            self.indices_dtype,
            self.indices_shape,
            name="indices_gm",
            scope=tik.scope_gm)
        self.updates_gm = self.tik_instance.Tensor(
            self.updates_dtype,
            self.updates_shape,
            name="updates_gm",
            scope=tik.scope_gm)
        self.out_gm = self.tik_instance.Tensor(
            self.var_dtype, self.var_shape, name="out_gm", scope=tik.scope_gm)

        self.vconv_dst_dtype = "float16"

        self.init_ub_tensor_para()
        self.var_vconv_ub = None
        self.updates_vconv_ub = None
        self.var_tile_vconv_ub = None
        self.updates_tile_vconv_ub = None

        self.var_ub = None
        self.updates_ub = None
        self.indices_ub = None
        self.var_tile_ub = None
        self.updates_tile_ub = None

        self.var_read_index = None
        self.updates_read_index = None
        self.indices_loop_index = None
        self.indices_tmp = None

    def init_ub_tensor_para(self):
        """
        Compute the ub size of tensors

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        updates_size_bytes = self.var_dtype_bytes_size * self.update_data_num
        indices_size_bytes = self.indices_dtype_bytes_size * self.indices_num

        need_vconv_dtype = ("int8", "uint8")
        if self.var_dtype in need_vconv_dtype:
            vconv_dtype_bytes_size = tbe_platform.get_bit_len(
                self.vconv_dst_dtype)
            vconv_data_each_block = 32 // vconv_dtype_bytes_size
            vconv_size_bytes = (
                    updates_size_bytes // self.var_dtype_bytes_size *
                    vconv_dtype_bytes_size)
            if (updates_size_bytes + vconv_size_bytes) * 2 < (
                    self.ub_size_bytes * 0.9):
                self.updates_ub_number = math.ceil(
                    self.update_data_num /
                    self.var_data_each_block) * self.var_data_each_block

                self.vconv_ub_number = math.ceil(
                    self.update_data_num /
                    vconv_data_each_block) * vconv_data_each_block

                self.indices_ub_number = (
                                                 self.ub_size_bytes - updates_size_bytes * 2 -
                                                 vconv_size_bytes * 2) // self.indices_dtype_bytes_size

                self.indices_ub_number = math.ceil(
                    self.indices_ub_number /
                    self.indices_data_each_block) * self.indices_data_each_block

            elif indices_size_bytes < (self.ub_size_bytes * 0.9):
                self.indices_ub_number = math.ceil(
                    self.indices_num /
                    self.indices_data_each_block) * self.indices_data_each_block
                self.updates_ub_number = (
                                                 self.ub_size_bytes -
                                                 indices_size_bytes) // self.var_dtype_bytes_size // 6

                self.updates_ub_number = math.ceil(
                    self.updates_ub_number /
                    self.var_data_each_block) * self.var_data_each_block

                self.vconv_ub_number = math.ceil(
                    self.updates_ub_number /
                    vconv_data_each_block) * vconv_data_each_block

            else:
                self.updates_ub_number = (
                        self.ub_size_bytes // 2 //
                        (vconv_dtype_bytes_size + self.var_dtype_bytes_size) // 2 //
                        self.var_data_each_block * self.var_data_each_block)
                self.indices_ub_number = (
                        self.ub_size_bytes // self.indices_dtype_bytes_size // 2 //
                        self.var_data_each_block * self.var_data_each_block)
                self.vconv_ub_number = self.updates_ub_number

        else:
            if updates_size_bytes * 2 < self.ub_size_bytes * 0.9:
                self.updates_ub_number = math.ceil(
                    self.update_data_num /
                    self.var_data_each_block) * self.var_data_each_block
                self.indices_ub_number = (
                                                 self.ub_size_bytes -
                                                 updates_size_bytes * 2) // self.indices_dtype_bytes_size
                self.indices_ub_number = math.ceil(
                    self.indices_ub_number /
                    self.indices_data_each_block) * self.indices_data_each_block
                if self.indices_num < self.indices_ub_number:
                    self.indices_ub_number = math.ceil(
                        self.indices_num / self.indices_data_each_block
                    ) * self.indices_data_each_block
            elif indices_size_bytes < self.ub_size_bytes * 0.9:
                self.indices_ub_number = math.ceil(
                    self.indices_num /
                    self.indices_data_each_block) * self.indices_data_each_block

                self.updates_ub_number = (
                                                 self.ub_size_bytes -
                                                 indices_size_bytes) // 2 // self.var_dtype_bytes_size

                self.updates_ub_number = math.ceil(
                    self.updates_ub_number /
                    self.var_data_each_block) * self.var_data_each_block
            else:
                self.indices_ub_number = (
                        self.ub_size_bytes // self.indices_dtype_bytes_size // 2 //
                        self.indices_data_each_block * self.indices_data_each_block)
                self.updates_ub_number = (
                        self.indices_ub_number // 2 // self.var_data_each_block *
                        self.var_data_each_block)

        last_num = self.update_data_num % self.updates_ub_number
        if (last_num < self.var_data_each_block and
                self.update_data_num > self.updates_ub_number):
            self.updates_ub_number -= self.var_data_each_block

    def init_ub_tensor(self):
        """
        Compute the ub size of tensors

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        need_vconv_dtype = ("int8", "uint8")
        if self.var_dtype in need_vconv_dtype:
            self.var_vconv_ub = self.tik_instance.Tensor(
                self.vconv_dst_dtype, (self.vconv_ub_number,),
                name="var_vconv_ub",
                scope=tik.scope_ubuf)
            self.updates_vconv_ub = self.tik_instance.Tensor(
                self.vconv_dst_dtype, (self.vconv_ub_number,),
                name="updates_vconv_ub",
                scope=tik.scope_ubuf)

            self.var_tile_vconv_ub = self.tik_instance.Tensor(
                self.vconv_dst_dtype, (self.var_data_each_block,),
                name="var_tile_vconv_ub",
                scope=tik.scope_ubuf)
            self.updates_tile_vconv_ub = self.tik_instance.Tensor(
                self.vconv_dst_dtype, (self.var_data_each_block,),
                name="updates_tile_vconv_ub",
                scope=tik.scope_ubuf)

        self.var_ub = self.tik_instance.Tensor(
            self.var_dtype, (self.updates_ub_number,),
            name="var_ub",
            scope=tik.scope_ubuf)
        self.updates_ub = self.tik_instance.Tensor(
            self.updates_dtype, (self.updates_ub_number,),
            name="updates_ub",
            scope=tik.scope_ubuf)
        self.indices_ub = self.tik_instance.Tensor(
            self.indices_dtype, (self.indices_ub_number,),
            name="indices_ub",
            scope=tik.scope_ubuf)

        self.var_tile_ub = self.tik_instance.Tensor(
            self.var_dtype, (self.var_data_each_block,),
            name="var_tile_ub",
            scope=tik.scope_ubuf)
        self.updates_tile_ub = self.tik_instance.Tensor(
            self.updates_dtype, (self.var_data_each_block,),
            name="updates_tile_ub",
            scope=tik.scope_ubuf)

        self.var_read_index = self.tik_instance.Scalar("int32")
        self.var_read_index.set_as(0)

        self.updates_read_index = self.tik_instance.Scalar("int32")
        self.updates_read_index.set_as(0)

        self.indices_loop_index = self.tik_instance.Scalar("int32")
        self.indices_loop_index.set_as(0)

        self.indices_tmp = self.tik_instance.Scalar("int32")
        self.indices_tmp.set_as(0)

    def get_var_read_index(self, indices_ub_index):
        """
        Calculate the index of the read var

        Parameters
        ----------
        indices_ub_index: int32
            the index of the currently traversed indices in UB

        Returns
        -------
        None
        """
        if not self.nd_flag:
            self.var_read_index.set_as(self.indices_ub[indices_ub_index])
        else:
            indices_ub_index = indices_ub_index * self.indices_shape[-1]
            self.var_read_index.set_as(0)
            if self.indices_shape[-1] == 1:
                self.var_read_index.set_as(self.indices_ub[indices_ub_index])
            else:
                for i in range(0, self.indices_shape[-1]):
                    self.indices_tmp.set_as(self.indices_ub[indices_ub_index +
                                                            i])
                    if i + 1 < self.indices_shape[-1]:
                        self.var_read_index.set_as(
                            self.var_read_index +
                            self.indices_tmp * functools_reduce(
                                lambda x, y: x * y,
                                self.var_shape[i + 1:self.indices_shape[-1]]))
                    else:
                        self.var_read_index.set_as(self.var_read_index +
                                                   self.indices_tmp)

    def get_updates_read_index(self, indices_ub_index):
        """
        Calculate the index of the read updates

        Parameters
        ----------
        indices_ub_index:int32
            the index of the currently traversed indices in UB

        Returns
        -------
        None
        """
        read_index = indices_ub_index * self.update_data_num
        self.updates_read_index.set_as(read_index)

    def updates_the_var(self, indices_in_index, indice_num):
        """
        Update the update fragment corresponding to the index

        Parameters
        ----------
        indices_in_index: int32
            Indices index on GM
        indice_num: int32
            the number of indexes in the indices on UB
        Returns
        -------
        None
        """
        indices_burst_len = math.ceil(indice_num / self.indices_data_each_block)
        if self.indices_num == 1:
            self.tik_instance.data_move(self.indices_ub, self.indices_gm, 0, 1,
                                        indices_burst_len, 0, 0)
        else:
            self.tik_instance.data_move(self.indices_ub,
                                        self.indices_gm[indices_in_index], 0, 1,
                                        indices_burst_len, 0, 0)
        if self.nd_flag:
            indice_loop_num = indice_num // self.indices_shape[-1]
        else:
            indice_loop_num = indice_num

        with self.tik_instance.for_range(0,
                                         indice_loop_num) as indices_ub_index:
            self.get_var_read_index(indices_ub_index)
            if self.block_num > 1:
                with self.tik_instance.if_scope(
                        self.indices_loop_index *
                        self.indice_step <= self.var_read_index):
                    with self.tik_instance.if_scope(
                            (self.indices_loop_index + 1) *
                            self.indice_step > self.var_read_index):
                        if self.nd_flag:
                            indices_in_index = indices_in_index // self.indices_shape[
                                -1]
                        self.get_updates_read_index(indices_ub_index +
                                                    indices_in_index)
                        self.var_read_index.set_as(self.var_read_index *
                                                   self.update_data_num)
                        self.calc_updates()
            else:
                if self.nd_flag:
                    indices_in_index = indices_in_index // self.indices_shape[-1]
                self.get_updates_read_index(indices_ub_index + indices_in_index)
                self.var_read_index.set_as(self.var_read_index *
                                           self.update_data_num)
                self.calc_updates()

    def calc_updates(self):
        """
        Calculate updates fragment

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        updates_loop = self.update_data_num // self.updates_ub_number
        if updates_loop > 0:
            with self.tik_instance.for_range(0, updates_loop) as loop_index:
                self.calc_updates_small(loop_index * self.updates_ub_number,
                                        self.updates_ub_number)

        last_num = self.update_data_num % self.updates_ub_number
        if last_num > 0:
            self.calc_updates_small(updates_loop * self.updates_ub_number,
                                    last_num)

    def calc_updates_small(self, read_index_offset, element_num):
        """
        Transfer update to UB and calculate

        Parameters
        ----------
        read_index_offset: int32
            the offset used to read the updates fragment
        element_num:
            the number of elements in the slice of updates

        Returns
        -------
        None
        """
        updates_burst_len = math.ceil(element_num / self.var_data_each_block)
        self.tik_instance.data_move(
            self.var_ub, self.var_gm[self.var_read_index + read_index_offset],
            0, 1, updates_burst_len, 0, 0)

        self.tik_instance.data_move(
            self.updates_ub,
            self.updates_gm[self.updates_read_index + read_index_offset], 0, 1,
            updates_burst_len, 0, 0)

        tile_ele_num = element_num % self.var_data_each_block
        align_offset = 0
        if (tile_ele_num != 0 and
                self.update_data_num > self.var_data_each_block):
            align_ele_num = (
                    element_num // self.var_data_each_block *
                    self.var_data_each_block)
            align_offset = (
                    read_index_offset + align_ele_num -
                    (self.var_data_each_block - tile_ele_num))
            self.tik_instance.data_move(
                self.var_tile_ub,
                self.var_gm[self.var_read_index + align_offset], 0, 1, 1, 0, 0)

            self.tik_instance.data_move(
                self.updates_tile_ub,
                self.updates_gm[self.updates_read_index + align_offset], 0, 1,
                1, 0, 0)

        compute_loop = element_num // self.max_num_one_repeat // 255

        if compute_loop > 0:
            with self.tik_instance.for_range(0, compute_loop) as index:
                index_offset = index * self.max_num_one_repeat * 255
                self.calc_process(self.max_num_one_repeat, index_offset,
                                  index_offset, index_offset, 255, False)
        last_loop = element_num % (self.max_num_one_repeat *
                                   255) // self.max_num_one_repeat

        if last_loop > 0:
            index_offset = compute_loop * self.max_num_one_repeat * 255
            self.calc_process(self.max_num_one_repeat, index_offset,
                              index_offset, index_offset, last_loop, False)

        compute_mask = element_num % self.max_num_one_repeat
        if compute_mask > 0:
            index_offset = (
                    element_num // self.max_num_one_repeat *
                    self.max_num_one_repeat)
            if (tile_ele_num == 0 or
                    self.update_data_num < self.var_data_each_block):
                self.calc_process(compute_mask, index_offset, index_offset,
                                  index_offset, 1, False)

                self.tik_instance.data_move(
                    self.out_gm[self.var_read_index + read_index_offset],
                    self.var_ub, 0, 1, updates_burst_len, 0, 0)
            else:
                self.calc_process(self.var_data_each_block, 0, 0, 0, 1, True)
                self.tik_instance.data_move(
                    self.out_gm[self.var_read_index + align_offset],
                    self.var_tile_ub, 0, 1, 1, 0, 0)
                self.calc_process(compute_mask, index_offset, index_offset,
                                  index_offset, 1, False)
                self.tik_instance.data_move(
                    self.out_gm[self.var_read_index + read_index_offset],
                    self.var_ub, 0, 1, updates_burst_len - 1, 0, 0)
        else:
            self.tik_instance.data_move(
                self.out_gm[self.var_read_index + read_index_offset],
                self.var_ub, 0, 1, updates_burst_len, 0, 0)

    def calc_process(self, mask, dest_addr, src_addr1, src_addr2, repeat_times,
                     is_tile):
        """
        Execute the corresponding calculation instruction

        Parameters
        ----------
        mask: int
            the mask of instruction
        dest_addr: int
            testination address offset
        src_addr1: int
            src1 address offset
        src_addr2: int
            src2 address offset
        repeat_times: int
            the repeat times of instruction
        is_tile: bool
            determine whether the currently calculated data is the tail of var
            and updates

        Returns
        -------
        None
        """
        need_vconv_dtype = ("int8", "uint8")
        if self.var_dtype in need_vconv_dtype:
            if is_tile:
                self.tik_instance.vconv(mask, "",
                                        self.var_tile_vconv_ub[dest_addr],
                                        self.var_tile_ub[src_addr1],
                                        repeat_times, 1, 1, 8, 4)
                self.tik_instance.vconv(mask, "",
                                        self.updates_tile_vconv_ub[dest_addr],
                                        self.updates_tile_ub[src_addr2],
                                        repeat_times, 1, 1, 8, 4)
                compute_repeat_strid = 8
                src1_ub = self.var_tile_vconv_ub
                src2_ub = self.updates_tile_vconv_ub
                dst_ub = self.var_tile_vconv_ub
                mask = self.var_data_each_block
            else:
                self.tik_instance.vconv(mask, "", self.var_vconv_ub[dest_addr],
                                        self.var_ub[src_addr1], repeat_times, 1,
                                        1, 8, 4)
                self.tik_instance.vconv(mask, "",
                                        self.updates_vconv_ub[dest_addr],
                                        self.updates_ub[src_addr2],
                                        repeat_times, 1, 1, 8, 4)
                compute_repeat_strid = 8
                src1_ub = self.var_vconv_ub[src_addr1]
                src2_ub = self.updates_vconv_ub[src_addr2]
                dst_ub = self.var_vconv_ub[dest_addr]

        else:
            if is_tile:
                compute_repeat_strid = (
                        self.max_num_one_repeat // self.var_data_each_block)
                src1_ub = self.var_tile_ub
                src2_ub = self.updates_tile_ub
                dst_ub = self.var_tile_ub
                mask = self.var_data_each_block
            else:
                compute_repeat_strid = (
                        self.max_num_one_repeat // self.var_data_each_block)
                src1_ub = self.var_ub[src_addr1]
                src2_ub = self.updates_ub[src_addr2]
                dst_ub = self.var_ub[dest_addr]

        if self.compute_type == "vadd":
            self.tik_instance.vec_add(mask, dst_ub, src1_ub, src2_ub, repeat_times,
                                   compute_repeat_strid,compute_repeat_strid,
                                   compute_repeat_strid)
        elif self.compute_type == "vsub":
            self.tik_instance.vec_sub(mask, dst_ub, src1_ub, src2_ub, repeat_times,
                                   compute_repeat_strid, compute_repeat_strid,
                                   compute_repeat_strid)
        elif self.compute_type == "vdiv":
            if platform_info.api_check_support("tik.vdiv", "float32"):
                self.tik_instance.vdiv(mask, dst_ub, src1_ub, src2_ub,
                                       repeat_times, 1, 1, 1,
                                       compute_repeat_strid,
                                       compute_repeat_strid,
                                       compute_repeat_strid)
            else:
                tmp_tensor = self.tik_instance.Tensor(
                    src2_ub.dtype, (mask * repeat_times,),
                    scope=tik.scope_ubuf,
                    name="tmp_tensor")
                self.tik_instance.vrec(mask, tmp_tensor, src2_ub, repeat_times,
                                       1, 1, compute_repeat_strid,
                                       compute_repeat_strid)
                self.tik_instance.vec_mul(mask, src2_ub, src2_ub, tmp_tensor,
                                       repeat_times,
                                       compute_repeat_strid,
                                       compute_repeat_strid,
                                       compute_repeat_strid)
                self.tik_instance.vec_adds(mask, src2_ub, src2_ub, NEG_TWO,
                                        repeat_times,
                                        compute_repeat_strid,
                                        compute_repeat_strid)
                self.tik_instance.vec_mul(mask, src2_ub, src2_ub, tmp_tensor,
                                       repeat_times,
                                       compute_repeat_strid,
                                       compute_repeat_strid,
                                       compute_repeat_strid)
                self.tik_instance.vec_muls(mask, src2_ub, src2_ub, NEG_ONE,
                                        repeat_times,
                                        compute_repeat_strid,
                                        compute_repeat_strid)
                # src1_ub * (1/src2_ub)
                self.tik_instance.vec_mul(mask, dst_ub, src1_ub, src2_ub,
                                       repeat_times,
                                       compute_repeat_strid,
                                       compute_repeat_strid,
                                       compute_repeat_strid)
        elif self.compute_type == "vmax":
            self.tik_instance.vmax(mask, dst_ub, src1_ub, src2_ub, repeat_times,
                                   1, 1, 1, compute_repeat_strid,
                                   compute_repeat_strid, compute_repeat_strid)
        elif self.compute_type == "vmin":
            self.tik_instance.vmin(mask, dst_ub, src1_ub, src2_ub, repeat_times,
                                   1, 1, 1, compute_repeat_strid,
                                   compute_repeat_strid, compute_repeat_strid)
        elif self.compute_type == "vmul":
            self.tik_instance.vec_mul(mask, dst_ub, src1_ub, src2_ub, repeat_times,
                                      compute_repeat_strid, compute_repeat_strid,
                                      compute_repeat_strid)
        elif self.compute_type == "update":
            self.tik_instance.vec_muls(mask, dst_ub, src1_ub, 0, repeat_times,
                                    compute_repeat_strid,
                                    compute_repeat_strid)
            self.tik_instance.vec_add(mask, dst_ub, src1_ub, src2_ub, repeat_times,
                                   compute_repeat_strid, compute_repeat_strid,
                                   compute_repeat_strid)
        else:
            raise RuntimeError("the operater [%s] is not supported" %
                               self.compute_type)
        if self.var_dtype in need_vconv_dtype:
            if is_tile:
                self.tik_instance.vconv(mask, "", self.var_tile_ub,
                                        self.var_tile_vconv_ub, repeat_times, 1,
                                        1, 4, 8)
            else:
                self.tik_instance.vconv(mask, "", self.var_ub[src_addr1],
                                        self.var_vconv_ub[dest_addr],
                                        repeat_times, 1, 1, 4, 8)

    def traversing_indices(self):
        """
        Traversing the index in the indices

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        max_ub_idx_num = (
                self.indices_ub_number // self.index_dims * self.index_dims)
        indices_loop_num = self.indices_num // max_ub_idx_num

        if indices_loop_num > 0:
            with self.tik_instance.for_range(
                    0, indices_loop_num) as indices_loop_index:
                self.updates_the_var(indices_loop_index * max_ub_idx_num,
                                     max_ub_idx_num)

        indices_last_num = self.indices_num % max_ub_idx_num
        if indices_last_num > 0:
            self.updates_the_var(indices_loop_num * max_ub_idx_num,
                                 indices_last_num)

    def check_param(self, var_out):
        """
        Check parameter

        Parameters
        ----------
        var_out: dict
            data of input
            datatype suports float32,float16,int32,int8,uint8

        Returns
        -------
        None
        """
        var_out_shape = var_out.get("shape")
        var_out_dtype = var_out.get("dtype").lower()
        if var_out_dtype == "bool":
            var_out_dtype = "int8"
        para_check.check_kernel_name(self.kernel_name)
        para_check.check_shape_rule(self.var_shape)
        para_check.check_shape_rule(self.indices_shape)
        para_check.check_shape_rule(self.updates_shape)
        para_check.check_shape_rule(var_out_shape)

        para_check.check_shape_size(self.var_shape)
        para_check.check_shape_size(self.indices_shape)
        para_check.check_shape_size(self.updates_shape)
        para_check.check_shape_size(var_out_shape)

        check_list_var = ("float16", "float32", "int32", "int8", "uint8")
        check_list_indices = ("int32")
        para_check.check_dtype_rule(self.var_dtype, check_list_var)
        para_check.check_dtype_rule(self.indices_dtype, check_list_indices)
        para_check.check_dtype_rule(self.updates_dtype, check_list_var)
        para_check.check_dtype_rule(var_out_dtype, check_list_var)

        if var_out_shape != self.var_shape:
            raise RuntimeError(
                "var_out's shape must be the same as var's shape")

        if (self.updates_dtype != self.var_dtype or
                var_out_dtype != self.var_dtype):
            raise RuntimeError(
                "updates's datatype and var_out's datatype must be the"
                " same as var's datatype")

        if self.nd_flag:
            if len(self.indices_shape) < 2:
                raise RuntimeError(
                    "the lenth of indices_shape must be large than 2")
            k = self.indices_shape[-1]
            updates_len = len(self.indices_shape) - 1 + len(self.var_shape) - k
            if k > len(self.var_shape):
                raise RuntimeError(
                    "indices_shape[-1] can not be large than var's rank")
            if len(self.updates_shape) != updates_len:
                raise RuntimeError("the lenth of update must be len(indices_"
                                   "shape)-1+len(var_shape)-indices_shape[-1]")
            updates_true_shape = self.indices_shape[:-1] + self.var_shape[k:]
        else:
            updates_true_shape = self.indices_shape + self.var_shape[1:]

        if self.updates_shape != updates_true_shape:
            raise RuntimeError("updates's shape is illegal")

    def scatter_operator(self):
        """
        Scatter operation

        Parameters
        ----------
        None

        Returns:
        ----------
        tik_instance: tik instance
        """
        if self.block_num > 1:
            with self.tik_instance.for_range(
                    0, self.block_num,
                    block_num=self.block_num) as indices_loop_index:
                self.init_ub_tensor()
                self.indices_loop_index.set_as(indices_loop_index)
                self.traversing_indices()
        else:
            self.init_ub_tensor()
            self.traversing_indices()

        self.tik_instance.BuildCCE(
            kernel_name=self.kernel_name,
            inputs=(self.var_gm, self.indices_gm, self.updates_gm),
            outputs=(self.out_gm),
            enable_l2=False)

        return self.tik_instance


# pylint: disable=too-many-arguments,unused-argument
def scatter_nd_add(var,
                   indices,
                   updates,
                   var_out,
                   use_locking=False,
                   kernel_name="scatter_nd_add"):
    """
    Applies sparse addition to individual values or slices in a Variable.

    Parameters
    ----------
    var: dict
        data of input.
        source data type, support "int8", "uint8", "int32", "float16", "float32"
    indices: dict
         A tensor of indices into var, support "int32"
    updates: dict
        data of updates
        source data type should ne same as var
    var_out: dict
        data of output.
    use_locking: bool
        not used in this compute
    kernel_name: str
        kernel name, default value is "scatter_nd_add"

    Returns:
    None
    """
    scatter_nd = Scatter(var, indices, updates, var_out, True, kernel_name,
                         "vadd")

    scatter_nd.scatter_operator()
