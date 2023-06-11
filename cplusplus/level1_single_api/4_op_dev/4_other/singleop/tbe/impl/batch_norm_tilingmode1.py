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
from tbe.common.platform import UB_SIZE
from tbe.common.platform import get_soc_spec
from tbe.common.platform import set_current_compile_soc_info

#batch for N
MAX_BATCH = 1

#channel for C
MAX_CHANNEL = 1024

#width for W
MAX_WIDTH = 32

#height for H
MAX_HEIGHT = 32

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
# pylint: disable=locally-disabled,too-many-arguments,too-many-locals
# pylint: disable=locally-disabled,unused-argument,too-many-statements
# pylint: disable=locally-disabled,invalid-name
class BatchNorm(object):
    """
    function desciption:
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

        self.byte_fp16 = 2
        self.input_dtype = "float16"
        self.kernel_name = kernel_name

        # gm buffer
        self.gamma_gm = self.tik_instance.Tensor("float16",
                                                 (MAX_CHANNEL, ),
                                                 name="gamma_gm",
                                                 scope=tik.scope_gm)
        self.beta_gm = self.tik_instance.Tensor("float16",
                                                (MAX_CHANNEL, ),
                                                name="beta_gm",
                                                scope=tik.scope_gm)
        self.input_gm = self.tik_instance.\
            Tensor("float16",
                   (MAX_BATCH*MAX_CHANNEL*MAX_HEIGHT*MAX_WIDTH,),
                   name="input_gm", scope=tik.scope_gm)
        self.output_gm = self.tik_instance.\
            Tensor("float16",
                   (MAX_BATCH*MAX_CHANNEL*MAX_HEIGHT*MAX_WIDTH,),
                   name="output_gm", scope=tik.scope_gm)
        self.gamma_ub = self.tik_instance.\
            Tensor("float16", (MAX_CHANNEL, ),
                   name="gamma_ub", scope=tik.scope_ubuf)
        self.beta_ub = self.tik_instance.\
            Tensor("float16", (MAX_CHANNEL, ),
                   name="beta_ub", scope=tik.scope_ubuf)

        align_c = ceil_div_mul(self.input_c, 16)

        #clear to zero
        self.tik_instance.vec_muls(128, self.gamma_ub, self.gamma_ub, 0,
                                   MAX_CHANNEL // 128, 8, 8)
        self.tik_instance.vec_muls(128, self.beta_ub, self.beta_ub, 0,
                                   MAX_CHANNEL // 128, 8, 8)

        self.tik_instance.data_move(self.gamma_ub, self.gamma_gm, 0, 1,
                                    align_c // 16, 0, 0)
        self.tik_instance.data_move(self.beta_ub, self.beta_gm, 0, 1,
                                    align_c // 16, 0, 0)

        # 1/var
        self.tik_instance.vec_rec(16, self.beta_ub, self.beta_ub,
                                  align_c // 16, 1, 1)
        # -mean
        self.tik_instance.vec_muls(16, self.gamma_ub, self.gamma_ub,
                                   -1.0, align_c // 16, 1, 1)

    #main process
    def batchnorm_compute(self):
        """Function to compute batch_norm"""
        self.batchnorm_compute_tiling_c()
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

    def batchnorm_compute_tiling_c(self):
        """
        Parameters
        ----------
        kernel_name : kernel name, default value is "BatchNorm"
        function_desciption :
         calc tensor batch normlize with function_desciption: (x - mean)/var.
        input0: dict shape and dtype of input
        gamma0: mean tensor
        beta0:  var tensor
        output0 :
         dict shape and dtype of output, should be same shape and type as input
        kernel_name : str kernel name
        default value is "BatchNorm" Returns ------- None
        Returns
        -------
        None
        """
        ub_size = get_soc_spec(UB_SIZE)

        with self.tik_instance.new_stmt_scope():
            chn_num = MAX_CHANNEL
            input_gm = self.input_gm
            output_gm = self.output_gm
            gamma_ub = self.gamma_ub
            beta_ub = self.beta_ub
            align_16 = ceil_div_mul(self.input_w*self.input_h, 16)
            total_use_ub = \
                chn_num*align_16*2*self.byte_fp16 + chn_num*self.byte_fp16

            with self.tik_instance.if_scope(total_use_ub <= ub_size):

                align_wh = self.param2
                align_c = self.param3

                data_ub = self.tik_instance.Tensor(self.input_dtype, (60928,),
                                                   name="data_ub",
                                                   scope=tik.scope_ubuf)
                trans_ub = self.tik_instance.Tensor(self.input_dtype, (60928,),
                                                    name="trans_ub",
                                                    scope=tik.scope_ubuf)
                tmp_ub = self.tik_instance.Tensor(self.input_dtype,
                                                  (chn_num, ),
                                                  name="tmp_ub",
                                                  scope=tik.scope_ubuf)

                self.tik_instance.data_move(data_ub[0], input_gm[0], 0, 1,
                                            align_c * align_wh // 16, 0, 0)

                with self.tik_instance.for_range(0, align_wh // 16,
                                                 thread_num=2) as k:
                    #transfer (chn_num, 16) to (16, chn_num)
                    #[align_c, 16]
                    src_list = [data_ub[0*align_wh + k*16],
                                data_ub[1*align_wh + k*16],
                                data_ub[2*align_wh + k*16],
                                data_ub[3*align_wh + k*16],
                                data_ub[4*align_wh + k*16],
                                data_ub[5*align_wh + k*16],
                                data_ub[6*align_wh + k*16],
                                data_ub[7*align_wh + k*16],
                                data_ub[8*align_wh + k*16],
                                data_ub[9*align_wh + k*16],
                                data_ub[10*align_wh + k*16],
                                data_ub[11*align_wh + k*16],
                                data_ub[12*align_wh + k*16],
                                data_ub[13*align_wh + k*16],
                                data_ub[14*align_wh + k*16],
                                data_ub[15*align_wh + k*16]]
                    #[align_wh, chn_num]
                    dst_list = [trans_ub[(k*16 + 0)*align_c],
                                trans_ub[(k*16 + 1)*align_c],
                                trans_ub[(k*16 + 2)*align_c],
                                trans_ub[(k*16 + 3)*align_c],
                                trans_ub[(k*16 + 4)*align_c],
                                trans_ub[(k*16 + 5)*align_c],
                                trans_ub[(k*16 + 6)*align_c],
                                trans_ub[(k*16 + 7)*align_c],
                                trans_ub[(k*16 + 8)*align_c],
                                trans_ub[(k*16 + 9)*align_c],
                                trans_ub[(k*16 + 10)*align_c],
                                trans_ub[(k*16 + 11)*align_c],
                                trans_ub[(k*16 + 12)*align_c],
                                trans_ub[(k*16 + 13)*align_c],
                                trans_ub[(k*16 + 14)*align_c],
                                trans_ub[(k*16 + 15)*align_c]]
                    self.tik_instance.vec_trans_scatter(True, True,
                                                        dst_list, src_list,
                                                        align_c // 16, 1,
                                                        align_wh)

                #calc (x - mean)/var
                with self.tik_instance.for_range(0, align_wh,
                                                 thread_num=2) as m:
                    self.tik_instance.vec_add(16, tmp_ub, trans_ub[m*align_c],
                                              gamma_ub, align_c // 16, 1, 1, 1)
                    self.tik_instance.vec_mul(16, trans_ub[m*align_c], tmp_ub,
                                              beta_ub, align_c // 16, 1, 1, 1)

                with self.tik_instance.for_range(0, align_wh // 16,
                                                 thread_num=2) as k:
                    #[align_wh, chn_num]
                    src_list = [trans_ub[(k*16 + 0)*align_c],
                                trans_ub[(k*16 + 1)*align_c],
                                trans_ub[(k*16 + 2)*align_c],
                                trans_ub[(k*16 + 3)*align_c],
                                trans_ub[(k*16 + 4)*align_c],
                                trans_ub[(k*16 + 5)*align_c],
                                trans_ub[(k*16 + 6)*align_c],
                                trans_ub[(k*16 + 7)*align_c],
                                trans_ub[(k*16 + 8)*align_c],
                                trans_ub[(k*16 + 9)*align_c],
                                trans_ub[(k*16 + 10)*align_c],
                                trans_ub[(k*16 + 11)*align_c],
                                trans_ub[(k*16 + 12)*align_c],
                                trans_ub[(k*16 + 13)*align_c],
                                trans_ub[(k*16 + 14)*align_c],
                                trans_ub[(k*16 + 15)*align_c]]

                    #[chn_num, align_wh]
                    dst_list = [data_ub[0*align_wh + k*16],
                                data_ub[1*align_wh + k*16],
                                data_ub[2*align_wh + k*16],
                                data_ub[3*align_wh + k*16],
                                data_ub[4*align_wh + k*16],
                                data_ub[5*align_wh + k*16],
                                data_ub[6*align_wh + k*16],
                                data_ub[7*align_wh + k*16],
                                data_ub[8*align_wh + k*16],
                                data_ub[9*align_wh + k*16],
                                data_ub[10*align_wh + k*16],
                                data_ub[11*align_wh + k*16],
                                data_ub[12*align_wh + k*16],
                                data_ub[13*align_wh + k*16],
                                data_ub[14*align_wh + k*16],
                                data_ub[15*align_wh + k*16]]

                    self.tik_instance.vec_trans_scatter(True, True, dst_list,
                                                        src_list,
                                                        align_c // 16,
                                                        align_wh, 1)

                #move ub->gm
                self.tik_instance.data_move(output_gm[0], data_ub[0], 0, 1,
                                            align_wh*self.input_c // 16, 0, 0)

            with self.tik_instance.else_scope():
                iterwh_align16 = self.param2
                repeat_alignc = self.param3
                align_wh = self.param4
                align_c = self.param5
                stride_alignwh = self.param6
                iter_num_double = floor_div_mul(iterwh_align16, 2)

                with self.tik_instance.for_range(0, iter_num_double,
                                                 thread_num=2) as i:
                    ping_ub = self.tik_instance.Tensor(self.input_dtype,
                                                       (1024, 16),
                                                       name="ping_ub",
                                                       scope=tik.scope_ubuf)
                    trans_ub = self.tik_instance.Tensor(self.input_dtype,
                                                        (16, 1024),
                                                        name="trans_ub",
                                                        scope=tik.scope_ubuf)
                    vconv_ub = self.tik_instance.Tensor(self.input_dtype,
                                                        (1024, 16),
                                                        name="vconv_ub",
                                                        scope=tik.scope_ubuf)
                    tmp_ub = self.tik_instance.Tensor(self.input_dtype,
                                                      (1024,),
                                                      name="tmp_ub",
                                                      scope=tik.scope_ubuf)

                    self.tik_instance.data_move(ping_ub[0, 0],
                                                input_gm[16 * i], 0,
                                                self.input_c, 1,
                                                stride_alignwh, 0)

                    #align_c & 16
                    src_list = [ping_ub[0, 0], ping_ub[1, 0], ping_ub[2, 0],
                                ping_ub[3, 0], ping_ub[4, 0], ping_ub[5, 0],
                                ping_ub[6, 0], ping_ub[7, 0], ping_ub[8, 0],
                                ping_ub[9, 0], ping_ub[10, 0], ping_ub[11, 0],
                                ping_ub[12, 0], ping_ub[13, 0], ping_ub[14, 0],
                                ping_ub[15, 0]]
                    #16 & align_c
                    dst_list = [trans_ub[0, 0], trans_ub[1, 0], trans_ub[2, 0],
                                trans_ub[3, 0], trans_ub[4, 0], trans_ub[5, 0],
                                trans_ub[6, 0], trans_ub[7, 0], trans_ub[8, 0],
                                trans_ub[9, 0], trans_ub[10, 0],
                                trans_ub[11, 0], trans_ub[12, 0],
                                trans_ub[13, 0], trans_ub[14, 0],
                                trans_ub[15, 0]]
                    self.tik_instance.vec_trans_scatter(True, True, dst_list,
                                                        src_list,
                                                        repeat_alignc, 1, 16)

                    #calc (x - mean)/var
                    with self.tik_instance.for_range(0, 16) as m:
                        self.tik_instance.vec_add(16, tmp_ub, trans_ub[m, 0],
                                                  gamma_ub, repeat_alignc,
                                                  1, 1, 1)
                        self.tik_instance.vec_mul(16, trans_ub[m, 0], tmp_ub,
                                                  beta_ub, repeat_alignc,
                                                  1, 1, 1)

                    #16 & align_c
                    src_list = [trans_ub[0, 0], trans_ub[1, 0], trans_ub[2, 0],
                                trans_ub[3, 0], trans_ub[4, 0], trans_ub[5, 0],
                                trans_ub[6, 0], trans_ub[7, 0], trans_ub[8, 0],
                                trans_ub[9, 0], trans_ub[10, 0],
                                trans_ub[11, 0], trans_ub[12, 0],
                                trans_ub[13, 0], trans_ub[14, 0],
                                trans_ub[15, 0]]
                    #align_c & 16
                    dst_list = [vconv_ub[0, 0], vconv_ub[1, 0], vconv_ub[2, 0],
                                vconv_ub[3, 0], vconv_ub[4, 0], vconv_ub[5, 0],
                                vconv_ub[6, 0], vconv_ub[7, 0], vconv_ub[8, 0],
                                vconv_ub[9, 0], vconv_ub[10, 0],
                                vconv_ub[11, 0], vconv_ub[12, 0],
                                vconv_ub[13, 0], vconv_ub[14, 0],
                                vconv_ub[15, 0]]
                    self.tik_instance.vec_trans_scatter(True, True, dst_list,
                                                        src_list,
                                                        repeat_alignc, 16, 1)

                    #move ub->gm
                    self.tik_instance.data_move(output_gm[16 * i],
                                                vconv_ub[0, 0], 0,
                                                self.input_c, 1, 0,
                                                stride_alignwh)

                with self.tik_instance.for_range(iter_num_double,
                                                 iterwh_align16) as i:
                    ping_ub = self.tik_instance.Tensor(self.input_dtype,
                                                       (1024, 16),
                                                       name="ping_ub",
                                                       scope=tik.scope_ubuf)
                    trans_ub = self.tik_instance.Tensor(self.input_dtype,
                                                        (16, 1024),
                                                        name="trans_ub",
                                                        scope=tik.scope_ubuf)
                    vconv_ub = self.tik_instance.Tensor(self.input_dtype,
                                                        (1024, 16),
                                                        name="vconv_ub",
                                                        scope=tik.scope_ubuf)
                    tmp_ub = self.tik_instance.Tensor(self.input_dtype,
                                                      (1024,),
                                                      name="tmp_ub",
                                                      scope=tik.scope_ubuf)

                    self.tik_instance.data_move(ping_ub[0, 0],
                                                input_gm[16 * i], 0, 1,
                                                self.input_c,
                                                stride_alignwh, 0)

                    #transfer (chn_num, 16) to (16, chn_num)
                    #align_c & 16
                    src_list = [ping_ub[0, 0], ping_ub[1, 0], ping_ub[2, 0],
                                ping_ub[3, 0], ping_ub[4, 0], ping_ub[5, 0],
                                ping_ub[6, 0], ping_ub[7, 0], ping_ub[8, 0],
                                ping_ub[9, 0], ping_ub[10, 0], ping_ub[11, 0],
                                ping_ub[12, 0], ping_ub[13, 0],
                                ping_ub[14, 0], ping_ub[15, 0]]
                    #16 & align_c
                    dst_list = [trans_ub[0, 0], trans_ub[1, 0], trans_ub[2, 0],
                                trans_ub[3, 0], trans_ub[4, 0], trans_ub[5, 0],
                                trans_ub[6, 0], trans_ub[7, 0], trans_ub[8, 0],
                                trans_ub[9, 0], trans_ub[10, 0],
                                trans_ub[11, 0], trans_ub[12, 0],
                                trans_ub[13, 0], trans_ub[14, 0],
                                trans_ub[15, 0]]
                    self.tik_instance.vec_trans_scatter(True, True, dst_list,
                                                        src_list,
                                                        repeat_alignc, 1, 16)

                    #calc (x - mean)/var
                    with self.tik_instance.for_range(0, 16) as m:
                        self.tik_instance.vec_add(16, tmp_ub, trans_ub[m, 0],
                                                  gamma_ub, repeat_alignc,
                                                  1, 1, 1)
                        self.tik_instance.vec_mul(16, trans_ub[m, 0],
                                                  tmp_ub, beta_ub,
                                                  repeat_alignc, 1, 1, 1)

                    #16 & align_c
                    src_list = [trans_ub[0, 0], trans_ub[1, 0], trans_ub[2, 0],
                                trans_ub[3, 0], trans_ub[4, 0], trans_ub[5, 0],
                                trans_ub[6, 0], trans_ub[7, 0], trans_ub[8, 0],
                                trans_ub[9, 0], trans_ub[10, 0],
                                trans_ub[11, 0], trans_ub[12, 0],
                                trans_ub[13, 0], trans_ub[14, 0],
                                trans_ub[15, 0]]
                    #align_c & 16
                    dst_list = [vconv_ub[0, 0], vconv_ub[1, 0], vconv_ub[2, 0],
                                vconv_ub[3, 0], vconv_ub[4, 0], vconv_ub[5, 0],
                                vconv_ub[6, 0], vconv_ub[7, 0], vconv_ub[8, 0],
                                vconv_ub[9, 0], vconv_ub[10, 0],
                                vconv_ub[11, 0], vconv_ub[12, 0],
                                vconv_ub[13, 0], vconv_ub[14, 0],
                                vconv_ub[15, 0]]
                    self.tik_instance.vec_trans_scatter(True, True, dst_list,
                                                        src_list,
                                                        repeat_alignc, 16, 1)

                    #move ub->gm
                    self.tik_instance.data_move(output_gm[16*i],
                                                vconv_ub[0, 0], 0,
                                                self.input_c, 1, 0,
                                                stride_alignwh)

def batch_norm(input_x, gamma, beta, output, kernel_name="BatchNorm"):
    """entrance function"""
    obj = BatchNorm(input_x, gamma, beta, output, kernel_name)
    obj.batchnorm_compute()


SHAPE_N = MAX_BATCH
SHAPE_C = MAX_CHANNEL
SHAPE_H = MAX_HEIGHT
SHAPE_W = MAX_WIDTH
INPUT = {"shape": [SHAPE_N, SHAPE_C, SHAPE_H, SHAPE_W], "dtype": "float16"}
GAMMA = {"shape": [SHAPE_C,], "dtype": "float16"}
BETA = {"shape": [SHAPE_C,], "dtype": "float16"}
OUTPUT = {"shape": [SHAPE_N, SHAPE_C, SHAPE_H, SHAPE_W], "dtype": "float16"}
NAME = "tiling_mode_1"
batch_norm(INPUT, GAMMA, BETA, OUTPUT, NAME)
