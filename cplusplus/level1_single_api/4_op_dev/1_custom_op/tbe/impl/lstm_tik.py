"""
Copyright (C) 2018. Huawei Technologies Co., Ltd. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the Apache License Version 2.0.You may not use this
file except in compliance with the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
Apache License for more details at
http://www.apache.org/licenses/LICENSE-2.0

lstm tik demo
"""
from te import tik


FP16_MASK = 128


def newton_iteration(tik_instance, rec, src_rec, src, repeats):
    """
    newton iteration method :
    new_rec = -1 * old_rec * (old_rec * src - 2)
    """
    with tik_instance.new_stmt_scope():
        newton_mul = tik_instance.Tensor(
            src.dtype, src.shape, name="newton_mul",
            scope=tik.scope_ubuf)

        tik_instance.vec_mul(FP16_MASK, newton_mul, src_rec, src, repeats,
                          8, 8, 8)
        tik_instance.vec_adds(FP16_MASK, newton_mul, newton_mul, -2, repeats,
                           8, 8)
        tik_instance.vec_mul(FP16_MASK, newton_mul, newton_mul, src_rec, repeats,
                           8, 8, 8)
        tik_instance.vec_muls(FP16_MASK, rec, newton_mul, -1, repeats,
                           8, 8)


def sigmoid(tik_instance, dst, src, repeats):
    """
    dst = 1 / (1 + exp(-x))
    """
    with tik_instance.new_stmt_scope():
        tik_instance.vec_muls(FP16_MASK, dst, src, -1, repeats, 8, 8)
        tik_instance.vec_exp(FP16_MASK, dst, dst, repeats, 8, 8)
        tik_instance.vec_adds(FP16_MASK, dst, dst, 1, repeats, 8, 8)
        sigmoid_rec = tik_instance.Tensor(
            src.dtype, src.shape, name="sigmoid_rec",
            scope=tik.scope_ubuf)
        tik_instance.vec_rec(FP16_MASK, sigmoid_rec, dst, repeats, 8, 8)
        newton_iteration(tik_instance, dst, sigmoid_rec, dst, repeats)


def tanh(tik_instance, dst, src, repeats):
    """
    dst = (exp(2x) - 1) / (exp(2x) + 1)
    """
    with tik_instance.new_stmt_scope():
        _2x = tik_instance.Tensor(dst.dtype, dst.shape, name="_2x",
                                  scope=tik.scope_ubuf)
        tik_instance.vec_muls(FP16_MASK, _2x, src, 2, repeats, 8, 8)
        tik_instance.vec_exp(FP16_MASK, _2x, _2x, repeats, 8, 8)
        tik_instance.vec_adds(FP16_MASK, dst, _2x, -1, repeats, 8, 8)
        tik_instance.vec_adds(FP16_MASK, _2x, _2x, 1, repeats, 8, 8)
        # mini does not support vdiv, use vrec instead
        tanh_rec = tik_instance.Tensor(
            src.dtype, src.shape, name="tanh_rec",
            scope=tik.scope_ubuf)
        tik_instance.vec_rec(FP16_MASK, tanh_rec, _2x, repeats, 8, 8)
        newton_iteration(tik_instance, tanh_rec, tanh_rec, _2x, repeats)
        tik_instance.vec_mul(FP16_MASK, dst, dst, tanh_rec,
                          repeats, 8, 8, 8)


class LSTMDemo(object):
    """
    lstm demo class
    """

    def __init__(self, kernel_name):
        self.hidden_size = 32
        self.feature_size = 32
        self.block_size = 16
        self.feature_block_num = self.feature_size // self.block_size
        self.hidden_block_size = self.hidden_size // self.block_size
        self.batch_size = 32
        self.batch_blocks = self.batch_size // self.block_size
        self.num_step = 16

        self.forget_bias = 1.0

        self.use_fixpipe = True
        self.matmul_init_l1out = self.use_fixpipe

        self.feature_hidden_size = self.feature_size + self.hidden_size
        self.feature_hidden_block = self.feature_hidden_size // self.block_size

        self.tik_instance = tik.Tik(tik.Dprofile())

        self.fixpipe_workspace = self.tik_instance.Tensor(
            "float16", (1, 4 * self.hidden_block_size, self.batch_blocks,
                        self.block_size, self.block_size),
            name="fixpipe_workspace", scope=tik.scope_gm, is_workspace=True)

        self.declare_gm_tensor()

        self.init_core()

        self.tik_instance.BuildCCE(
            kernel_name,
            inputs=[self.gm_x, self.gm_init_h,
                    self.gm_init_c, self.gm_weight, self.gm_b],
            outputs=[self.gm_output_h, self.gm_output_c])

    def declare_gm_tensor(self):
        """
        declare gm tensor
        """
        self.gm_x = self.tik_instance.Tensor("float16",
                                             (self.num_step,
                                              self.batch_blocks,
                                              self.feature_block_num,
                                              self.block_size,
                                              self.block_size),
                                             name="gm_x",
                                             scope=tik.scope_gm)

        self.gm_init_h = self.tik_instance.Tensor("float16",
                                                  (self.batch_blocks,
                                                   self.hidden_block_size,
                                                   self.block_size,
                                                   self.block_size),
                                                  name="gm_init_h",
                                                  scope=tik.scope_gm)

        self.gm_init_c = self.tik_instance.Tensor("float16",
                                                  (self.batch_blocks,
                                                   self.hidden_block_size,
                                                   self.block_size,
                                                   self.block_size),
                                                  name="gm_init_c",
                                                  scope=tik.scope_gm)

        self.gm_weight = self.tik_instance.Tensor(
            "float16", (self.feature_hidden_size // self.block_size,
                        4 * self.hidden_block_size,
                        self.block_size, self.block_size),
            name="gm_weight", scope=tik.scope_gm)
        self.gm_b = self.tik_instance.Tensor(
            "float32", (4 * self.hidden_size,),
            name="gm_b", scope=tik.scope_gm)

        self.gm_output_h = self.tik_instance.Tensor("float16",
                                                    (self.num_step,
                                                     self.batch_blocks,
                                                     self.hidden_block_size,
                                                     self.block_size,
                                                     self.block_size),
                                                    name="gm_output_h",
                                                    scope=tik.scope_gm)

        self.gm_output_c = self.tik_instance.Tensor(
            "float16", (self.batch_blocks, self.hidden_block_size,
                        self.block_size, self.block_size),
            name="gm_output_c",
            scope=tik.scope_gm)

    def init_core(self):
        """
        init core
        """
        self.l1_bias = self.tik_instance.Tensor(
            "float32",
            (4 * self.hidden_size, ),
            name="l1_bias",
            scope=tik.scope_cbuf)

        self.l1_xh = self.tik_instance.Tensor("float16",
                                              (self.feature_hidden_block,
                                               self.batch_blocks,
                                               self.block_size,
                                               self.block_size),
                                              name="l1_xh",
                                              scope=tik.scope_cbuf)

        self.ub_c_next = self.tik_instance.Tensor("float16",
                                                  (self.batch_blocks,
                                                   self.hidden_block_size,
                                                   self.block_size,
                                                   self.block_size),
                                                  name="ub_c",
                                                  scope=tik.scope_ubuf)

        self.ub_h_next = self.tik_instance.Tensor(
            "float16", (self.batch_blocks, self.hidden_block_size,
                        self.block_size, self.block_size),
            name="ub_h", scope=tik.scope_ubuf)

        self.init_params()

        with self.tik_instance.for_range(0, self.num_step - 1) as t:
            self.step(t)

        self.last_step()

    def init_params(self):
        """
        these param only load once
        init c
        """
        self.tik_instance.data_move(
            self.ub_c_next, self.gm_init_c, 0, 1,
            self.batch_blocks * self.hidden_size, 0, 0)

        # init bias
        self.tik_instance.data_move(
            self.l1_bias, self.gm_b, 0,
            1, 4 * self.hidden_size // 8,
            0, 0)

        # init h
        with self.tik_instance.for_range(0, self.batch_blocks) as batch_i:
            self.tik_instance.data_move(
                self.l1_xh[self.feature_block_num, batch_i, :, :],
                self.gm_init_h, 0,
                self.hidden_block_size,
                self.block_size, 0, self.block_size * (self.batch_blocks - 1))

    def _step_impl(self, t, last_time):
        # on the fly permutation
        with self.tik_instance.for_range(0, self.batch_blocks) as batch_i:
            self.tik_instance.data_move(
                self.l1_xh[:, batch_i, :, :],
                self.gm_x[t, batch_i, :, :, :], 0,
                self.feature_block_num,
                self.block_size, 0, self.block_size * (self.batch_blocks - 1))

        self._step_tile(t, last_time)

        self.tik_instance.data_move(
            self.gm_output_h[t, 0, :, :, :],
            self.ub_h_next,
            0, 1, self.batch_blocks * self.hidden_size,
            0, 0)

    def _step_tile_init(self):
        self.mat_cc_i = self.tik_instance.Tensor(
            "float32", (self.hidden_block_size,
                        self.batch_blocks, self.block_size, self.block_size),
            name="mat_cc_i", scope=tik.scope_cbuf_out)

        self.mat_cc_j = self.tik_instance.Tensor(
            "float32", (self.hidden_block_size,
                        self.batch_blocks, self.block_size, self.block_size),
            name="mat_cc_j", scope=tik.scope_cbuf_out)

        self.mat_cc_f = self.tik_instance.Tensor(
            "float32", (self.hidden_block_size,
                        self.batch_blocks, self.block_size, self.block_size),
            name="mat_cc_f", scope=tik.scope_cbuf_out)

        self.mat_cc_o = self.tik_instance.Tensor(
            "float32", (self.hidden_block_size,
                        self.batch_blocks, self.block_size, self.block_size),
            name="mat_cc_o", scope=tik.scope_cbuf_out)

    def _step_tile(self, t, last_time):
        self._step_tile_init()

        vector_repeat = self.hidden_block_size * 2 * self.batch_blocks
        sigmoid_i = self._step_tile_i(t, last_time)
        tanh_j = self._step_tile_j(t, last_time)
        ub_temp_1 = self.tik_instance.Tensor(
            "float16",
            (self.batch_blocks, self.hidden_block_size,
             self.block_size, self.block_size),
            name="ub_temp_1", scope=tik.scope_ubuf)
        # ub_temp_1 = sigmoid_i * tanh_j
        self.tik_instance.vec_mul(
            FP16_MASK, ub_temp_1, sigmoid_i, tanh_j, vector_repeat,
            8, 8, 8)
        # sigmod_f = sigmoid(f+forget_bias)
        sigmoid_f = self._step_tile_f(t, last_time)
        # calculate next c
        self.tik_instance.vec_mul(
            FP16_MASK,
            self.ub_h_next,
            sigmoid_f,
            self.ub_c_next,
            vector_repeat, 8, 8, 8)
        # c = sigmoid_f * c_prev + ub_temp1
        self.tik_instance.vec_add(
            FP16_MASK,
            self.ub_c_next,
            ub_temp_1,
            self.ub_h_next,
            vector_repeat, 8, 8, 8)
        sigmoid_o = self._step_tile_o(t, last_time)
        # calculate next h

        # ub_temp1 = tanh(c)
        tanh(self.tik_instance,
             ub_temp_1,
             self.ub_c_next,
             vector_repeat)

        # h = sigmoid_o * ub_temp1
        self.tik_instance.vec_mul(
            FP16_MASK,
            self.ub_h_next,
            sigmoid_o,
            ub_temp_1,
            vector_repeat, 8, 8, 8)

    def _step_tile_i(self, t, last_time):
        l1_weight = self.tik_instance.Tensor(
            "float16",
            (
                self.feature_hidden_block,
                self.hidden_block_size,
                self.block_size,
                self.block_size),
            name="l1_weight_i",
            scope=tik.scope_cbuf)

        i = 0
        with self.tik_instance.for_range(0, self.hidden_block_size) as tb_i:
            weight_offset = tb_i * self.block_size * self.block_size +\
                i * self.hidden_size * self.block_size
            weight_burst_num = self.feature_hidden_size // self.block_size
            weight_burst_len = self.block_size
            weight_src_gap = 4 * self.hidden_size - self.block_size
            weight_dst_gap = self.hidden_size - self.block_size

            l1_offset = tb_i * self.block_size * self.block_size

            self.tik_instance.data_move(
                l1_weight.flatten()[l1_offset], self.gm_weight.flatten()[
                    weight_offset], 0,
                weight_burst_num, weight_burst_len,
                weight_src_gap, weight_dst_gap)

        self.tik_instance.matmul(
            self.mat_cc_i, self.l1_xh, l1_weight, self.batch_size,
            self.feature_size + self.hidden_size,
            self.hidden_size, init_l1out=self.matmul_init_l1out)

        self.tik_instance.fixpipe(
            self.fixpipe_workspace[0, i *
                                   self.hidden_block_size, :, :, :],
            self.mat_cc_i,
            self.hidden_block_size,
            self.batch_blocks * self.block_size * 2, 0, 0,
            {"quantize_params": {"mode": "fp322fp16", "mode_param": None},
             "bias": self.l1_bias[i * self.hidden_size]}
        )

        # num of frac of i,j,f,o
        l0c_sub_frac_num = self.hidden_block_size
        vector_repeat = l0c_sub_frac_num * 2 * self.batch_blocks

        ub_temp_i = self.tik_instance.Tensor(
            "float16",
            (self.batch_blocks, self.hidden_block_size,
             self.block_size, self.block_size),
            name="ub_temp_i", scope=tik.scope_ubuf)

        with self.tik_instance.for_range(0, self.batch_blocks) as batch_i:
            self.tik_instance.data_move(
                ub_temp_i[batch_i, 0, :, :],
                self.fixpipe_workspace[0, i * self.hidden_block_size,
                                       batch_i, :, :],
                0, l0c_sub_frac_num, self.block_size, (self.batch_blocks - 1) * self.block_size, 0)

        ub_sigmoid_i = self.tik_instance.Tensor(
            "float16",
            (self.batch_blocks, self.hidden_block_size,
             self.block_size, self.block_size),
            name="ub_sigmoid_i", scope=tik.scope_ubuf)

        sigmoid(self.tik_instance, ub_sigmoid_i,
                ub_temp_i, vector_repeat)

        return ub_sigmoid_i

    def _step_tile_j(self, t, last_time):
        l1_weight = self.tik_instance.Tensor(
            "float16",
            (
                self.feature_hidden_block,
                self.hidden_block_size,
                self.block_size,
                self.block_size),
            name="l1_weight_j",
            scope=tik.scope_cbuf)

        i = 1
        with self.tik_instance.for_range(0,
                                         self.hidden_block_size) as tb_i:
            weight_offset = tb_i * self.block_size * self.block_size + \
                i * self.hidden_size * self.block_size
            weight_burst_num = self.feature_hidden_size // self.block_size
            weight_burst_len = self.block_size
            weight_src_gap = 4 * self.hidden_size - self.block_size
            weight_dst_gap = self.hidden_size - self.block_size

            l1_offset = tb_i * self.block_size * self.block_size

            self.tik_instance.data_move(
                l1_weight.flatten()[l1_offset],
                self.gm_weight.flatten()[weight_offset], 0,
                weight_burst_num, weight_burst_len,
                weight_src_gap, weight_dst_gap)

        self.tik_instance.matmul(
            self.mat_cc_j, self.l1_xh, l1_weight, self.batch_size,
            self.feature_size + self.hidden_size,
            self.hidden_size, init_l1out=self.matmul_init_l1out)

        self.tik_instance.fixpipe(
            self.fixpipe_workspace[0,
                                   i * self.hidden_block_size, :, :, :],
            self.mat_cc_j,
            self.hidden_block_size,
            self.batch_blocks * self.block_size * 2, 0,
            0,
            {"quantize_params": {"mode": "fp322fp16", "mode_param": None},
                "bias": self.l1_bias[
                i * self.hidden_size]}
        )

        # num of frac of i,j,f,o
        l0c_sub_frac_num = self.hidden_block_size
        vector_repeat = l0c_sub_frac_num * 2 * self.batch_blocks

        ub_temp_j = self.tik_instance.Tensor(
            "float16",
            (self.batch_blocks, self.hidden_block_size,
             self.block_size, self.block_size),
            name="ub_temp_j", scope=tik.scope_ubuf)

        with self.tik_instance.for_range(0, self.batch_blocks) as batch_i:
            self.tik_instance.data_move(
                ub_temp_j[batch_i, 0, :, :],
                self.fixpipe_workspace[0, i * self.hidden_block_size,
                                       batch_i, :, :],
                0, l0c_sub_frac_num, self.block_size, (self.batch_blocks - 1) * self.block_size, 0)

        ub_tanh_j = self.tik_instance.Tensor(
            "float16",
            (self.batch_blocks, self.hidden_block_size,
             self.block_size, self.block_size),
            name="ub_tanh_j", scope=tik.scope_ubuf)

        tanh(self.tik_instance, ub_tanh_j,
             ub_temp_j, vector_repeat)

        return ub_tanh_j

    def _step_tile_f(self, t, last_time):
        l1_weight = self.tik_instance.Tensor(
            "float16",
            (
                self.feature_hidden_block,
                self.hidden_block_size,
                self.block_size,
                self.block_size),
            name="l1_weight_f",
            scope=tik.scope_cbuf)

        i = 2
        with self.tik_instance.for_range(0,
                                         self.hidden_block_size) as tb_i:
            weight_offset = tb_i * self.block_size * self.block_size + \
                i * self.hidden_size * self.block_size
            weight_burst_num = self.feature_hidden_size // self.block_size
            weight_burst_len = self.block_size
            weight_src_gap = 4 * self.hidden_size - self.block_size
            weight_dst_gap = self.hidden_size - self.block_size

            l1_offset = tb_i * self.block_size * self.block_size

            self.tik_instance.data_move(
                l1_weight.flatten()[l1_offset],
                self.gm_weight.flatten()[weight_offset], 0,
                weight_burst_num, weight_burst_len,
                weight_src_gap, weight_dst_gap)

        self.tik_instance.matmul(
            self.mat_cc_f, self.l1_xh, l1_weight, self.batch_size,
            self.feature_size + self.hidden_size,
            self.hidden_size, init_l1out=self.matmul_init_l1out)

        self.tik_instance.fixpipe(
            self.fixpipe_workspace[0,
                                   i * self.hidden_block_size, :, :, :],
            self.mat_cc_f,
            self.hidden_block_size,
            self.batch_blocks * self.block_size * 2, 0,
            0,
            {"quantize_params": {"mode": "fp322fp16", "mode_param": None},
                "bias": self.l1_bias[
                    i * self.hidden_size]}
        )

        # num of frac of i,j,f,o
        l0c_sub_frac_num = self.hidden_block_size
        vector_repeat = l0c_sub_frac_num * 2 * self.batch_blocks

        ub_temp_f = self.tik_instance.Tensor(
            "float16",
            (self.batch_blocks, self.hidden_block_size,
             self.block_size, self.block_size),
            name="ub_temp_f", scope=tik.scope_ubuf)

        with self.tik_instance.for_range(0, self.batch_blocks) as batch_i:
            self.tik_instance.data_move(
                ub_temp_f[batch_i, 0, :, :],
                self.fixpipe_workspace[0, i * self.hidden_block_size,
                                       batch_i, :, :],
                0, l0c_sub_frac_num,
                self.block_size, (self.batch_blocks - 1) * self.block_size, 0)

        self.tik_instance.vec_adds(FP16_MASK, ub_temp_f, ub_temp_f,
                                self.forget_bias, vector_repeat, 8, 8)

        ub_sigmoid_f = self.tik_instance.Tensor(
            "float16",
            (self.batch_blocks, self.hidden_block_size,
             self.block_size, self.block_size),
            name="ub_sigmoid_f", scope=tik.scope_ubuf)

        sigmoid(self.tik_instance, ub_sigmoid_f,
                ub_temp_f, vector_repeat)

        return ub_sigmoid_f

    def _step_tile_o(self, t, last_time):
        l1_weight = self.tik_instance.Tensor(
            "float16",
            (
                self.feature_hidden_block,
                self.hidden_block_size,
                self.block_size,
                self.block_size),
            name="l1_weight_o",
            scope=tik.scope_cbuf)

        i = 3
        with self.tik_instance.for_range(0,
                                         self.hidden_block_size) as tb_i:
            weight_offset = tb_i * self.block_size * self.block_size + \
                i * self.hidden_size * self.block_size
            weight_burst_num = self.feature_hidden_size // self.block_size
            weight_burst_len = self.block_size
            weight_src_gap = 4 * self.hidden_size - self.block_size
            weight_dst_gap = self.hidden_size - self.block_size

            l1_offset = tb_i * self.block_size * self.block_size

            self.tik_instance.data_move(
                l1_weight.flatten()[l1_offset],
                self.gm_weight.flatten()[weight_offset], 0,
                weight_burst_num, weight_burst_len,
                weight_src_gap, weight_dst_gap)

        self.tik_instance.matmul(
            self.mat_cc_o, self.l1_xh, l1_weight, self.batch_size,
            self.feature_size + self.hidden_size,
            self.hidden_size, init_l1out=self.matmul_init_l1out)

        self.tik_instance.fixpipe(
            self.fixpipe_workspace[0,
                                   i * self.hidden_block_size, :, :, :],
            self.mat_cc_o,
            self.hidden_block_size,
            self.batch_blocks * self.block_size * 2, 0,
            0,
            {"quantize_params": {"mode": "fp322fp16", "mode_param": None},
                "bias": self.l1_bias[
                    i * self.hidden_size]}
        )

        # num of frac of i,j,f,o
        l0c_sub_frac_num = self.hidden_block_size
        vector_repeat = l0c_sub_frac_num * 2 * self.batch_blocks

        ub_temp_o = self.tik_instance.Tensor(
            "float16",
            (self.batch_blocks, self.hidden_block_size,
             self.block_size, self.block_size),
            name="ub_temp_o", scope=tik.scope_ubuf)

        with self.tik_instance.for_range(0, self.batch_blocks) as batch_i:
            self.tik_instance.data_move(
                ub_temp_o[batch_i, 0, :, :],
                self.fixpipe_workspace[0, i *
                                       self.hidden_block_size, batch_i, :, :],
                0, l0c_sub_frac_num, self.block_size, (self.batch_blocks - 1) * self.block_size, 0)

        ub_sigmoid_o = self.tik_instance.Tensor(
            "float16",
            (self.batch_blocks, self.hidden_block_size,
             self.block_size, self.block_size),
            name="ub_sigmoid_o", scope=tik.scope_ubuf)

        sigmoid(self.tik_instance, ub_sigmoid_o,
                ub_temp_o, vector_repeat)

        return ub_sigmoid_o

    def step(self, t):
        """
        lstm cell
        """
        self._step_impl(t, False)

        # Copy h to L1 for next step
        with self.tik_instance.for_range(0, self.batch_blocks) as batch_i:
            self.tik_instance.data_move(
                self.l1_xh[self.feature_block_num, batch_i, :, :],
                self.gm_output_h[t, batch_i, :, :, :], 0,
                self.hidden_block_size,
                self.block_size, 0, self.block_size * (self.batch_blocks - 1))

    def last_step(self):
        """
        last t cell
        """
        self._step_impl(self.num_step - 1, True)

        # In the last step, we need to output c
        self.tik_instance.data_move(
            self.gm_output_c,
            self.ub_c_next, 0,
            1, self.batch_blocks * self.hidden_size, 0, 0)


def lstm_tik(x, init_h, init_c,
             weight, bias, output_h, output_c, kernel_name="lstm_tik"):
    """
    This is a fixed shape demo, only the kernel_name param is used
    """
    LSTMDemo(kernel_name)
