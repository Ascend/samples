"""
Copyright (C) 2019. Huawei Technologies Co., Ltd. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the Apache License Version 2.0.You may not use
this file except in compliance with the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
Apache License for more details at
http://www.apache.org/licenses/LICENSE-2.0
"""

import os
import warnings

from tbe import tik
from tbe.common.platform import set_current_compile_soc_info

# tiling mode 3 ub size
TILING_2_UB_SIZE = 112*1024

# batch for N
MAX_BATCH = 1

# channel for C
MAX_CHANNEL = 1024

# width for W
MAX_WIDTH = 512

# height for H
MAX_HEIGHT = 1024


# ceil div by value
def ceil_div_offline(value, factor):
    """Fuction to get ceil number."""
    return ((value) + (factor)-1) // (factor)


# ceil div by value
def ceil_div_mul(value, factor):
    """Fuction to get ceil number."""
    return (((value) + (factor)-1) // (factor))*(factor)


# floor div by value
def floor_div_mul(value, factor):
    """Fuction to get floor number."""
    return (((value) // (factor))) * (factor)


# pylint: disable=locally-disabled,too-many-instance-attributes
# pylint: disable=locally-disabled,too-many-arguments,too-many-statements
# pylint: disable=locally-disabled,too-many-locals,invalid-name,unused-argument
class BatchNorm(object):
    """
    function_desciption:
     calc tensor batch normlize with function_desciption: (x - mean)/var.
    Parameters
    ----------
    kernel_name : kernel name, default value is "BatchNorm"
    input0: dict shape and dtype of input
    gamma0: mean tensor
    beta0:  var tensor
    output0 : dict shape and dtype of output,
              should be same shape and type as input
    kernel_name : str kernel name
    default value is "BatchNorm" Returns ------- None
    Returns
    -------
    None
    """
    def __init__(self, input0, gamma0, beta0, output0, kernel_name="BatchNorm"):

        # SET Ascend AI Processor version.
        soc_version = os.getenv("SOC_VERSION")
        if soc_version is None:
            warnings.warn("SOC_VERSION no environment variable is set,"
                          " the default value of Ascend310 will be used.")
            soc_version = "Ascend310"
        set_current_compile_soc_info(soc_version)

        self.tik_instance = tik.Tik()

        self.sclar_gamma = self.tik_instance.Scalar("float16")
        self.sclar_beta = self.tik_instance.Scalar("float16")

        self.input_n = self.tik_instance.InputScalar(dtype="int32",
                                                     name="inputscalar_n")
        self.input_c = self.tik_instance.InputScalar(dtype="int32",
                                                     name="inputscalar_c")
        self.input_h = self.tik_instance.InputScalar(dtype="int32",
                                                     name="inputscalar_h")
        self.input_w = self.tik_instance.InputScalar(dtype="int32",
                                                     name="inputscalar_w")
        self.inputtype = \
            self.tik_instance.InputScalar(dtype="int32",
                                          name="inputscalar_dtype")
        self.output_n = self.tik_instance.InputScalar(dtype="int32",
                                                      name="outputscalar_n")
        self.output_c = self.tik_instance.InputScalar(dtype="int32",
                                                      name="outputscalar_c")
        self.output_h = self.tik_instance.InputScalar(dtype="int32",
                                                      name="outputscalar_h")
        self.output_w = self.tik_instance.InputScalar(dtype="int32",
                                                      name="outputscalar_w")
        self.outputtype = \
            self.tik_instance.InputScalar(dtype="int32",
                                          name="outputscalar_dtype")
        self.gamma_c = self.tik_instance.InputScalar(dtype="int32",
                                                     name="gammascalar")
        self.gammatype = \
            self.tik_instance.InputScalar(dtype="int32",
                                          name="gammascalar_dtype")
        self.beta_c = self.tik_instance.InputScalar(dtype="int32",
                                                    name="betascalar")
        self.betatype = self.tik_instance.InputScalar(dtype="int32",
                                                      name="betascalar_dtype")
        self.param1 = self.tik_instance.InputScalar(dtype="int32",
                                                    name="param1")
        self.param2 = self.tik_instance.InputScalar(dtype="int32",
                                                    name="param2")
        self.param3 = self.tik_instance.InputScalar(dtype="int32",
                                                    name="param3")
        self.param4 = self.tik_instance.InputScalar(dtype="int32",
                                                    name="param4")
        self.param5 = self.tik_instance.InputScalar(dtype="int32",
                                                    name="param5")
        self.param6 = self.tik_instance.InputScalar(dtype="int32",
                                                    name="param6")
        self.param7 = self.tik_instance.InputScalar(dtype="int32",
                                                    name="param7")
        self.param8 = self.tik_instance.InputScalar(dtype="int32",
                                                    name="param8")
        self.param9 = self.tik_instance.InputScalar(dtype="int32",
                                                    name="param9")
        self.param10 = self.tik_instance.InputScalar(dtype="int32",
                                                     name="param10")

        self.byte_fp16 = 2
        self.input_dtype = "float16"
        self.kernel_name = kernel_name
        align_c = ceil_div_mul(self.input_c, 16)

        # gm buffer
        self.gamma_gm = self.tik_instance.Tensor("float16", (MAX_CHANNEL, ),
                                                 name="gamma_gm",
                                                 scope=tik.scope_gm)
        self.beta_gm = self.tik_instance.Tensor("float16", (MAX_CHANNEL, ),
                                                name="beta_gm",
                                                scope=tik.scope_gm)
        self.input_gm = self.tik_instance.Tensor(
            "float16", (MAX_BATCH*MAX_CHANNEL*MAX_HEIGHT*MAX_WIDTH,),
            name="input_gm", scope=tik.scope_gm)
        self.output_gm = self.tik_instance.Tensor(
            "float16", (MAX_BATCH*MAX_CHANNEL*MAX_HEIGHT*MAX_WIDTH,),
            name="output_gm", scope=tik.scope_gm)

        self.gamma_ub = self.tik_instance.Tensor("float16", (MAX_CHANNEL, ),
                                                 name="gamma_ub",
                                                 scope=tik.scope_ubuf)
        self.beta_ub = self.tik_instance.Tensor("float16", (MAX_CHANNEL, ),
                                                name="beta_ub",
                                                scope=tik.scope_ubuf)

        # clear to zero
        self.tik_instance.vec_muls(128, self.gamma_ub, self.gamma_ub, 0,
                                   MAX_CHANNEL // 128, 8, 8)
        self.tik_instance.vec_muls(128, self.beta_ub, self.beta_ub, 0,
                                   MAX_CHANNEL // 128, 8, 8)

        self.tik_instance.data_move(self.gamma_ub, self.gamma_gm, 0, 1,
                                    align_c // 16, 0, 0)
        self.tik_instance.data_move(self.beta_ub, self.beta_gm, 0, 1,
                                    align_c // 16, 0, 0)

        self.tik_instance.vec_rec(16, self.beta_ub, self.beta_ub,
                                  align_c // 16, 1, 1)  # 1/var
        self.tik_instance.vec_muls(16, self.gamma_ub, self.gamma_ub, -1.0,
                                   align_c // 16, 1, 1) # -mean

    def batchnorm_compute(self):
        """Function to compute batch_norm"""
        self.batchnorm_compute_tiling_wh_muti_c()

        self.tik_instance.BuildCCE(kernel_name=self.kernel_name,
                                   inputs=[self.input_gm,
                                           self.gamma_gm,
                                           self.beta_gm],
                                   outputs=[self.output_gm],
                                   flowtable=[self.input_n, self.input_c,
                                              self.input_h, self.input_w,
                                              self.inputtype, self.output_n,
                                              self.output_c, self.output_h,
                                              self.output_w, self.outputtype,
                                              self.gamma_c, self.gammatype,
                                              self.beta_c, self.betatype,
                                              self.param1, self.param2,
                                              self.param3, self.param4,
                                              self.param5, self.param6,
                                              self.param7, self.param8,
                                              self.param9, self.param10],
                                   config={"double_buffer_non_reuse": True,
                                           "out_of_bound_sync_check": True})
        return self.tik_instance

    def batchnorm_compute_tiling_wh_muti_c(self):
        """tiling function"""
        with self.tik_instance.new_stmt_scope():
            tiling_num = TILING_2_UB_SIZE // self.byte_fp16 // 2
            input_wh = self.param1
            align_wh = self.param2
            single_chnum = self.param3
            iter_cnum = self.param4

            input_gm = self.input_gm
            output_gm = self.output_gm
            gamma_ub = self.gamma_ub
            beta_ub = self.beta_ub

            # align with 16, move buffer only one, reduce move times
            with self.tik_instance.if_scope(align_wh == input_wh):

                repeat_length = self.param5
                repeat_mask = self.param6
                res_ch_num = self.param9
                res_repeat_length = self.param10

                iter_num_double = floor_div_mul(iter_cnum, 2)

                # algin with 16
                with self.tik_instance.for_range(0, iter_num_double,
                                                 thread_num=2) as i:
                    index_cur = i*single_chnum
                    index = i*single_chnum*input_wh
                    ping_ub = self.tik_instance.Tensor(self.input_dtype,
                                                       (tiling_num + 128, ),
                                                       name="ping_ub",
                                                       scope=tik.scope_ubuf)
                    temp_ub = self.tik_instance.Tensor(self.input_dtype,
                                                       (tiling_num + 128, ),
                                                       name="temp_ub",
                                                       scope=tik.scope_ubuf)
                    # ping buffer
                    self.tik_instance.data_move(ping_ub[0], input_gm[index],
                                                0, 1, repeat_length // 16,
                                                0, 0)
                    with self.tik_instance.for_range(0, single_chnum) as j:
                        idx = index_cur + j
                        idx1 = j*input_wh
                        self.sclar_gamma.set_as(gamma_ub[idx])
                        self.sclar_beta.set_as(beta_ub[idx])
                        self.tik_instance.vec_adds(128, temp_ub[idx1],
                                                   ping_ub[idx1],
                                                   self.sclar_gamma,
                                                   repeat_mask + 1, 8, 8)
                        self.tik_instance.vec_muls(128, temp_ub[idx1],
                                                   temp_ub[idx1],
                                                   self.sclar_beta,
                                                   repeat_mask + 1, 8, 8)
                    self.tik_instance.data_move(output_gm[index], temp_ub, 0,
                                                1, repeat_length // 16, 0, 0)

                with self.tik_instance.for_range(iter_num_double,
                                                 iter_cnum) as i:
                    index_cur = i*single_chnum
                    index = i*single_chnum*input_wh
                    ping_ub = self.tik_instance.Tensor(self.input_dtype,
                                                       (tiling_num + 128, ),
                                                       name="ping_ub",
                                                       scope=tik.scope_ubuf)
                    temp_ub = self.tik_instance.Tensor(self.input_dtype,
                                                       (tiling_num + 128, ),
                                                       name="temp_ub",
                                                       scope=tik.scope_ubuf)
                    # ping buffer
                    self.tik_instance.data_move(ping_ub[0], input_gm[index], 0,
                                                1, repeat_length // 16, 0, 0)
                    with self.tik_instance.for_range(0, single_chnum) as j:
                        idx = index_cur + j
                        idx1 = j*input_wh
                        self.sclar_gamma.set_as(gamma_ub[idx])
                        self.sclar_beta.set_as(beta_ub[idx])
                        self.tik_instance.vec_adds(128, temp_ub[idx1],
                                                   ping_ub[idx1],
                                                   self.sclar_gamma,
                                                   repeat_mask + 1, 8, 8)
                        self.tik_instance.vec_muls(128, temp_ub[idx1],
                                                   temp_ub[idx1],
                                                   self.sclar_beta,
                                                   repeat_mask + 1, 8, 8)
                    self.tik_instance.data_move(output_gm[index], temp_ub, 0,
                                                1, repeat_length // 16, 0, 0)

                with self.tik_instance.if_scope(res_ch_num > 0):
                    index_new = single_chnum * iter_cnum
                    index2 = index_new * input_wh
                    ping_ub = self.tik_instance.Tensor(self.input_dtype,
                                                       (tiling_num + 128, ),
                                                       name="ping_ub",
                                                       scope=tik.scope_ubuf)
                    temp_ub = self.tik_instance.Tensor(self.input_dtype,
                                                       (tiling_num + 128, ),
                                                       name="temp_ub",
                                                       scope=tik.scope_ubuf)
                    # ping buffer
                    self.tik_instance.data_move(ping_ub[0], input_gm[index2],
                                                0, 1, repeat_length // 16,
                                                0, 0)

                    with self.tik_instance.for_range(0, res_ch_num) as j:
                        index = index_new + j
                        self.sclar_gamma.set_as(gamma_ub[index])
                        self.sclar_beta.set_as(beta_ub[index])
                        self.tik_instance.vec_adds(128, temp_ub[j*input_wh],
                                                   ping_ub[j*input_wh],
                                                   self.sclar_gamma,
                                                   repeat_mask + 1, 8, 8)
                        self.tik_instance.vec_muls(128, temp_ub[j*input_wh],
                                                   temp_ub[j*input_wh],
                                                   self.sclar_beta,
                                                   repeat_mask + 1, 8, 8)
                    self.tik_instance.data_move(output_gm[index2], temp_ub,
                                                0, 1, res_repeat_length // 16,
                                                0, 0)
                with self.tik_instance.else_scope():
                    pass
            with self.tik_instance.else_scope():
                repeat_mask128 = self.param5
                iter_num_double = floor_div_mul(self.input_c, 2)

                with self.tik_instance.for_range(0, iter_num_double,
                                                 thread_num=2) as i:
                    self.sclar_gamma.set_as(gamma_ub[i])
                    self.sclar_beta.set_as(beta_ub[i])
                    ping_ub = self.tik_instance.Tensor(self.input_dtype,
                                                       (tiling_num + 128, ),
                                                       name="ping_ub",
                                                       scope=tik.scope_ubuf)
                    temp_ub = self.tik_instance.Tensor(self.input_dtype,
                                                       (tiling_num + 128, ),
                                                       name="temp_ub",
                                                       scope=tik.scope_ubuf)
                    self.tik_instance.data_move(
                        ping_ub, input_gm[i * input_wh], 0, 1,
                        align_wh // 16, 0, 0)
                    self.tik_instance.vec_adds(128, temp_ub, ping_ub,
                                               self.sclar_gamma,
                                               repeat_mask128 + 1, 8, 8)
                    self.tik_instance.vec_muls(128, temp_ub, temp_ub,
                                               self.sclar_beta,
                                               repeat_mask128 + 1, 8, 8)
                    self.tik_instance.data_move(output_gm[i * input_wh],
                                                temp_ub, 0, 1, align_wh // 16,
                                                0, 0)

                with self.tik_instance.for_range(iter_num_double,
                                                 self.input_c) as i:
                    self.sclar_gamma.set_as(gamma_ub[i])
                    self.sclar_beta.set_as(beta_ub[i])
                    ping_ub = self.tik_instance.Tensor(self.input_dtype,
                                                       (tiling_num + 128, ),
                                                       name="ping_ub",
                                                       scope=tik.scope_ubuf)
                    temp_ub = self.tik_instance.Tensor(self.input_dtype,
                                                       (tiling_num + 128, ),
                                                       name="temp_ub",
                                                       scope=tik.scope_ubuf)
                    self.tik_instance.data_move(
                        ping_ub, input_gm[i * input_wh], 0, 1,
                        align_wh // 16, 0, 0)
                    self.tik_instance.vec_adds(128, temp_ub, ping_ub,
                                               self.sclar_gamma,
                                               repeat_mask128 + 1, 8, 8)
                    self.tik_instance.vec_muls(128, temp_ub, temp_ub,
                                               self.sclar_beta,
                                               repeat_mask128 + 1, 8, 8)
                    self.tik_instance.data_move(output_gm[i * input_wh],
                                                temp_ub, 0, 1, align_wh // 16,
                                                0, 0)


def batch_norm(input0, gamma0, beta0, output0, kernel_name="BatchNorm"):
    """entrance function"""
    obj = BatchNorm(input0, gamma0, beta0, output0, kernel_name)
    obj.batchnorm_compute()


N = MAX_BATCH
C = MAX_CHANNEL
H = MAX_HEIGHT
W = MAX_WIDTH
INPUT = {"shape": [N, C, H, W], "dtype": "float16"}
GAMMA = {"shape": [C, ], "dtype": "float16"}
BETA = {"shape": [C, ], "dtype": "float16"}
OUTPUT = {"shape": [N, C, H, W], "dtype": "float16"}
NAME = "tiling_mode_3"
batch_norm(INPUT, GAMMA, BETA, OUTPUT, NAME)
