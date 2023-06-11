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

#tiling mode 2 ub size
TILING_1_UB_SIZE = 112*1024

#max allow general mem size
TOTAL_GM_SIZE = 512*1024*1024   #GM_MAX_SIZE

#batch for N
MAX_BATCH = 1

#channel for C
MAX_CHANNEL = 1024

#width for W
MAX_WIDTH = 512

#height for H
MAX_HEIGHT = 1024


#(a + align - 1) // align
def ceil_div_offline(value, factor):
    """Fuction to get ceil number."""
    return ((value) + (factor)-1) // (factor)


#((a + align - 1) // align) * align
def ceil_div_mul(value, factor):
    """Fuction to get ceil number."""
    return (((value) + (factor)-1) // (factor))*(factor)


# floor div by value
def floor_div_mul(value, factor):
    """Fuction to get floor number."""
    return (((value) // (factor))) *(factor)


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
        #
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

        #self.mode.set_as(1)
        self.input_dtype = "float16"
        self.byte_fp16 = 2
        self.kernel_name = kernel_name
        align_c = ceil_div_mul(self.input_c, 16)

        # gm buffer
        self.gamma_gm = self.tik_instance.Tensor(self.input_dtype,
                                                 (MAX_CHANNEL, ),
                                                 name="gamma_gm",
                                                 scope=tik.scope_gm)
        self.beta_gm = self.tik_instance.Tensor(self.input_dtype,
                                                (MAX_CHANNEL, ),
                                                name="beta_gm",
                                                scope=tik.scope_gm)
        self.input_gm = self.tik_instance.\
            Tensor(self.input_dtype,
                   (MAX_BATCH*MAX_CHANNEL*MAX_HEIGHT*MAX_WIDTH,),
                   name="input_gm", scope=tik.scope_gm)
        self.output_gm = self.tik_instance.\
            Tensor(self.input_dtype,
                   (MAX_BATCH*MAX_CHANNEL*MAX_HEIGHT*MAX_WIDTH,),
                   name="output_gm", scope=tik.scope_gm)

        # ub buffer
        self.gamma_ub = self.tik_instance.Tensor(self.input_dtype,
                                                 (MAX_CHANNEL, ),
                                                 name="gamma_ub",
                                                 scope=tik.scope_ubuf)
        self.beta_ub = self.tik_instance.Tensor(self.input_dtype,
                                                (MAX_CHANNEL, ),
                                                name="beta_ub",
                                                scope=tik.scope_ubuf)

        #clear to zero
        self.tik_instance.vec_muls(128, self.gamma_ub, self.gamma_ub,
                                   0, MAX_CHANNEL // 128, 8, 8)
        self.tik_instance.vec_muls(128, self.beta_ub, self.beta_ub,
                                   0, MAX_CHANNEL // 128, 8, 8)

        self.tik_instance.data_move(self.gamma_ub, self.gamma_gm, 0,
                                    1, align_c // 16, 0, 0)
        self.tik_instance.data_move(self.beta_ub, self.beta_gm, 0,
                                    1, align_c // 16, 0, 0)

        self.tik_instance.vec_rec(16, self.beta_ub, self.beta_ub,
                                  align_c // 16, 1, 1)
        self.tik_instance.vec_muls(16, self.gamma_ub, self.gamma_ub,
                                   -1.0, align_c // 16, 1, 1)

        self.tik_instance.data_move(self.output_gm[0], self.gamma_ub,
                                    0, 1, 1, 0, 0)


    def batchnorm_compute(self):
        """Function to compute batch_norm"""
        self.batchnorm_compute_tiling_wh_single_c()

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

    def batchnorm_compute_tiling_wh_single_c(self):
        """Tiling function"""
        with self.tik_instance.new_stmt_scope():
            tiling_num = TILING_1_UB_SIZE // self.byte_fp16 // 2
            maxiternum = tiling_num // 128
            input_gm = self.input_gm
            output_gm = self.output_gm
            gamma_ub = self.gamma_ub
            beta_ub = self.beta_ub
            input_wh = self.param1
            align_wh = self.param2
            iter_ceil = self.param3

            with self.tik_instance.if_scope(iter_ceil < 2):
                repeat_mask128 = self.param5
                repeat_res_mask128 = self.param6
                iter_num_double = floor_div_mul(self.input_c, 2)

                with self.tik_instance.for_range(0, iter_num_double,
                                                 thread_num=2) as i:
                    ping_ub = self.tik_instance.Tensor(self.input_dtype,
                                                       (tiling_num, ),
                                                       name="ping_ub",
                                                       scope=tik.scope_ubuf)
                    temp_ub = self.tik_instance.Tensor(self.input_dtype,
                                                       (tiling_num, ),
                                                       name="temp_ub",
                                                       scope=tik.scope_ubuf)
                    self.sclar_gamma.set_as(gamma_ub[i])
                    self.sclar_beta.set_as(beta_ub[i])
                    self.tik_instance.data_move(ping_ub, input_gm[i*input_wh],
                                                0, 1, align_wh // 16, 0, 0)
                    self.tik_instance.vec_adds(128, temp_ub[0], ping_ub[0],
                                               self.sclar_gamma,
                                               maxiternum, 8, 8)
                    self.tik_instance.vec_muls(128, temp_ub[0], temp_ub[0],
                                               self.sclar_beta,
                                               maxiternum, 8, 8)
                    self.tik_instance.data_move(output_gm[i*input_wh],
                                                temp_ub, 0, 1,
                                                align_wh // 16, 0, 0)

                with self.tik_instance.for_range(iter_num_double,
                                                 self.input_c) as i:
                    ping_ub = self.tik_instance.Tensor(self.input_dtype,
                                                       (tiling_num, ),
                                                       name="ping_ub",
                                                       scope=tik.scope_ubuf)
                    temp_ub = self.tik_instance.Tensor(self.input_dtype,
                                                       (tiling_num, ),
                                                       name="temp_ub",
                                                       scope=tik.scope_ubuf)
                    self.sclar_gamma.set_as(gamma_ub[i])
                    self.sclar_beta.set_as(beta_ub[i])
                    self.tik_instance.data_move(ping_ub, input_gm[i*input_wh],
                                                0, 1, align_wh // 16, 0, 0)
                    self.tik_instance.vec_adds(128, temp_ub[0], ping_ub[0],
                                               self.sclar_gamma,
                                               maxiternum, 8, 8)
                    self.tik_instance.vec_muls(128, temp_ub[0], temp_ub[0],
                                               self.sclar_beta,
                                               maxiternum, 8, 8)
                    self.tik_instance.data_move(output_gm[i*input_wh],
                                                temp_ub, 0, 1,
                                                align_wh // 16, 0, 0)

            with self.tik_instance.else_scope(): #tiling model

                iter_h = self.param4
                iter_align16 = self.param5
                iter_res_align16 = self.param6
                repeat_mask128 = self.param7
                repeat_res_mask128 = self.param8
                res_h = self.param9

                with self.tik_instance.if_scope(res_h == 0):
                    with self.tik_instance.for_range(0, self.input_c) as i:
                        self.sclar_gamma.set_as(gamma_ub[i])
                        self.sclar_beta.set_as(beta_ub[i])
                        idx = i*input_wh
                        iter_num_double = floor_div_mul(iter_ceil, 2)
                        with self.tik_instance.for_range(0, iter_num_double,
                                                         thread_num=2) as j:
                            ping_ub = \
                                self.tik_instance.Tensor(self.input_dtype,
                                                         (tiling_num + 128, ),
                                                         name="ping_ub",
                                                         scope=tik.scope_ubuf)
                            temp_ub = \
                                self.tik_instance.Tensor(self.input_dtype,
                                                         (tiling_num + 128, ),
                                                         name="temp_ub",
                                                         scope=tik.scope_ubuf)
                            self.tik_instance.\
                                data_move(ping_ub,
                                          input_gm[idx + j*iter_h*self.input_w],
                                          0, 1, iter_align16 // 16, 0, 0)
                            self.tik_instance.vec_adds(128, temp_ub, ping_ub,
                                                       self.sclar_gamma,
                                                       repeat_mask128 // 128,
                                                       8, 8)
                            self.tik_instance.vec_muls(128, temp_ub, temp_ub,
                                                       self.sclar_beta,
                                                       repeat_mask128 // 128,
                                                       8, 8)
                            self.tik_instance.data_move(
                                self.output_gm[idx + j*iter_h*self.input_w],
                                temp_ub, 0, 1, iter_align16 // 16, 0, 0)

                        with self.tik_instance.for_range(iter_num_double,
                                                         iter_ceil) as j:
                            ping_ub = \
                                self.tik_instance.Tensor(self.input_dtype,
                                                         (tiling_num + 128, ),
                                                         name="ping_ub",
                                                         scope=tik.scope_ubuf)
                            temp_ub = \
                                self.tik_instance.Tensor(self.input_dtype,
                                                         (tiling_num + 128, ),
                                                         name="temp_ub",
                                                         scope=tik.scope_ubuf)
                            self.tik_instance.\
                                data_move(ping_ub,
                                          input_gm[idx + j*iter_h*self.input_w],
                                          0, 1, iter_align16 // 16, 0, 0)
                            self.tik_instance.vec_adds(128, temp_ub, ping_ub,
                                                       self.sclar_gamma,
                                                       repeat_mask128 // 128,
                                                       8, 8)
                            self.tik_instance.vec_muls(128, temp_ub, temp_ub,
                                                       self.sclar_beta,
                                                       repeat_mask128 // 128,
                                                       8, 8)
                            self.tik_instance.data_move(
                                self.output_gm[idx + j*iter_h*self.input_w],
                                temp_ub, 0, 1, iter_align16 // 16, 0, 0)

                with self.tik_instance.else_scope():
                    with self.tik_instance.for_range(0, self.input_c) as i:
                        self.sclar_gamma.set_as(gamma_ub[i])
                        self.sclar_beta.set_as(beta_ub[i])
                        idx = i*input_wh
                        iter_num_double = floor_div_mul(iter_ceil - 1, 2)
                        #pingpong buffer
                        with self.tik_instance.for_range(0, iter_num_double,
                                                         thread_num=2) as j:
                            ping_ub = \
                                self.tik_instance.Tensor(self.input_dtype,
                                                         (tiling_num + 128, ),
                                                         name="ping_ub",
                                                         scope=tik.scope_ubuf)
                            temp_ub = \
                                self.tik_instance.Tensor(self.input_dtype,
                                                         (tiling_num + 128, ),
                                                         name="temp_ub",
                                                         scope=tik.scope_ubuf)
                            index_h = j*iter_h
                            self.tik_instance.data_move(
                                ping_ub, input_gm[idx + index_h*self.input_w],
                                0, 1, iter_align16 // 16, 0, 0)
                            self.tik_instance.vec_adds(128, temp_ub, ping_ub,
                                                       self.sclar_gamma,
                                                       repeat_mask128 // 128,
                                                       8, 8)
                            self.tik_instance.vec_muls(128, temp_ub, temp_ub,
                                                       self.sclar_beta,
                                                       repeat_mask128 // 128,
                                                       8, 8)
                            self.tik_instance.data_move(
                                self.output_gm[idx + index_h*self.input_w],
                                temp_ub, 0, 1, iter_align16 // 16, 0, 0)
                        with self.tik_instance.for_range(iter_num_double,
                                                         iter_ceil - 1) as j:
                            ping_ub = \
                                self.tik_instance.Tensor(self.input_dtype,
                                                         (tiling_num + 128, ),
                                                         name="ping_ub",
                                                         scope=tik.scope_ubuf)
                            temp_ub = \
                                self.tik_instance.Tensor(self.input_dtype,
                                                         (tiling_num + 128, ),
                                                         name="temp_ub",
                                                         scope=tik.scope_ubuf)
                            index_h = j*iter_h
                            self.tik_instance.data_move(
                                ping_ub, input_gm[idx + index_h*self.input_w],
                                0, 1, iter_align16 // 16, 0, 0)
                            self.tik_instance.vec_adds(128, temp_ub, ping_ub,
                                                       self.sclar_gamma,
                                                       repeat_mask128 // 128,
                                                       8, 8)
                            self.tik_instance.vec_muls(128, temp_ub, temp_ub,
                                                       self.sclar_beta,
                                                       repeat_mask128 // 128,
                                                       8, 8)
                            self.tik_instance.data_move(
                                self.output_gm[idx + index_h*self.input_w],
                                temp_ub, 0, 1, iter_align16 // 16, 0, 0)

                        #process res
                        with self.tik_instance.if_scope(iter_res_align16 > 0):
                            index3 = idx + (iter_ceil - 1)*iter_h*self.input_w
                            pong_ub = \
                                self.tik_instance.Tensor(self.input_dtype,
                                                         (tiling_num + 128, ),
                                                         name="pong_ub",
                                                         scope=tik.scope_ubuf)
                            temp_ub = \
                                self.tik_instance.Tensor(self.input_dtype,
                                                         (tiling_num + 128, ),
                                                         name="temp_ub",
                                                         scope=tik.scope_ubuf)
                            self.tik_instance.data_move(pong_ub,
                                                        input_gm[index3], 0, 1,
                                                        iter_res_align16 // 16,
                                                        0, 0)

                            self.tik_instance.vec_adds(
                                128, temp_ub, pong_ub, self.sclar_gamma,
                                repeat_res_mask128 // 128, 8, 8)
                            self.tik_instance.vec_muls(
                                128, temp_ub, temp_ub, self.sclar_beta,
                                repeat_res_mask128 // 128, 8, 8)

                            self.tik_instance.data_move(
                                self.output_gm[index3], temp_ub, 0, 1,
                                iter_res_align16 // 16, 0, 0)
                        with self.tik_instance.else_scope():
                            pass

def batch_norm(input0, gamma0, beta0, output0, kernel_name="BatchNorm"):
    """entrance function"""
    obj = BatchNorm(input0, gamma0, beta0, output0, kernel_name)
    obj.batchnorm_compute()

#maxinum 112 KB
N = MAX_BATCH
C = MAX_CHANNEL
H = MAX_HEIGHT
W = MAX_WIDTH
INPUT = {"shape": [N, C, H, W], "dtype": "float16"}
GAMMA = {"shape": [C,], "dtype": "float16"}
BETA = {"shape": [C,], "dtype": "float16"}
OUTPUT = {"shape": [N, C, H, W], "dtype": "float16"}
NAME = "tiling_mode_2"
batch_norm(INPUT, GAMMA, BETA, OUTPUT, NAME)
