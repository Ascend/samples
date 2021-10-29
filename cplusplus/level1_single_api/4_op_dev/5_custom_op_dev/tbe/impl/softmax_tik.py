"""
Copyright 2020 Huawei Technologies Co., Ltd. All rights reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

softmax_tik
"""
import os
from te import tik
import numpy as np
from tbe.common.platform import set_current_compile_soc_info

def softmax_compute(shape, data_type, kernel_name):
    """Run softmax case
    Returns result
    """
    set_current_compile_soc_info("Ascend910")
    tik_instance = tik.Tik()
    # declare calculate params
    height = shape[0]
    width = shape[1]
    DIM_LEN = shape[0]
    x_gm = tik_instance.Tensor(data_type, shape, scope=tik.scope_gm, name="x_gm")
    w_gm = tik_instance.Tensor(data_type, shape, scope=tik.scope_gm, name="w_gm")
    result_gm = tik_instance.Tensor(data_type, shape, scope=tik.scope_gm, name="result_gm")
    with tik_instance.for_range(0, DIM_LEN) as i:
        x_ub = tik_instance.Tensor(data_type, (DIM_LEN, width), scope=tik.scope_ubuf, name="x_ub")
        w_ub = tik_instance.Tensor(data_type, (DIM_LEN, width), scope=tik.scope_ubuf, name="w_ub")
        result_ub = tik_instance.Tensor(data_type, (DIM_LEN, width),
            scope=tik.scope_ubuf, name="result_ub")
        exp_ub = tik_instance.Tensor(data_type, (DIM_LEN, width),
            scope=tik.scope_ubuf, name="exp_ub")
        reduce_ub = tik_instance.Tensor(data_type, (DIM_LEN,),
            scope=tik.scope_ubuf, name="reduce_ub")
        broadcast_ub = tik_instance.Tensor(data_type, (DIM_LEN, width),
            scope=tik.scope_ubuf, name="broadcast_ub")
        tik_instance.h_data_move(x_ub[:, :], x_gm[DIM_LEN * i: DIM_LEN * (i + 1), :])
        tik_instance.h_data_move(w_ub[:, :], w_gm[DIM_LEN * i: DIM_LEN * (i + 1), :])
        # get max value of one row
        tik_instance.h_mul(exp_ub, x_ub, w_ub)
        tik_instance.h_reduce_max(reduce_ub, exp_ub, axis=(1))

        # broadcast max_value to max_value_brc
        broadcast_scalar = tik_instance.Scalar(dtype=data_type)
        with tik_instance.for_range(0, DIM_LEN) as j:
            broadcast_scalar.set_as(reduce_ub[j])
            tik_instance.h_duplicate(broadcast_ub[j: j + 1, :], broadcast_scalar)
        tik_instance.h_sub(exp_ub, exp_ub, broadcast_ub)

        # get exponent value by elementwise
        tik_instance.h_exp(exp_ub, exp_ub)
        tik_instance.h_duplicate(reduce_ub, 0.0)
        tik_instance.h_reduce_sum(reduce_ub, exp_ub, axis=(1))

        # x/sum, need to broadcast
        with tik_instance.for_range(0, DIM_LEN) as j:
            broadcast_scalar.set_as(reduce_ub[j])
            tik_instance.h_duplicate(broadcast_ub[j: j + 1, :], broadcast_scalar)
        tik_instance.h_div(result_ub, exp_ub, broadcast_ub)

        tik_instance.h_data_move(result_gm[DIM_LEN * i: DIM_LEN * (i + 1), :], result_ub[:, :])

    # move result ub to reslut gm
    tik_instance.BuildCCE(kernel_name=kernel_name, inputs=[x_gm, w_gm,],
                          outputs=[result_gm,], config={"save_temp_cce_file": False,})

    return tik_instance


def softmax_tik(input_x1, input_x2, output_y, kernel_name="simple_softmax"):
    """
    softmax_tik main func with tik1.5
    Parameters
    ----------
    input_x1: input data 1
    input_x2: input data 2
    output_y: output dta
    """    
    shape_a = input_x1.get("ori_shape")
    data_type = input_x2.get("dtype").lower()
    output_y = output_y
    return softmax_compute(shape_a, data_type, kernel_name)
