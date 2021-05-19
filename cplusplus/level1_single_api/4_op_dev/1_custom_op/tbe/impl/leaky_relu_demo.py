#!/usr/bin/env python
# -*- coding: UTF-8 -*-
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

cce extended operator builder wrapper
"""

import tbe.dsl as tbe
from tbe import tvm
from tbe.common.register import register_op_compute
from tbe.common.utils import para_check
from tbe.common.utils import shape_util

# pylint: disable=locally-disabled,unused-argument,invalid-name
@register_op_compute("LeakyReluDemo", op_mode="dynamic", support_fusion=True)
def leaky_relu_demo_compute(x, y, negative_slope=0, kernel_name="leaky_relu"):
    """
    compute for caffe_relu_layer_cce
    """
    inp_dtype = x.dtype.lower()
    shape = x.shape

    # The original relu logic remains unchanged.
    if negative_slope == 0:
        if inp_dtype in ("float32", "int32"):
            tensor_zero = tbe.broadcast(tvm.const(0, inp_dtype), shape)
            data_res = tbe.vmax(x, tensor_zero)
        else:
            data_res = tbe.vrelu(x)

        data_res = tbe.cast_to(data_res, inp_dtype)

        return data_res
    # negative_slope != 0
    if inp_dtype in ("float16", "float32"):
        slope_tmp = tvm.const(negative_slope, dtype=inp_dtype)
        tmp = tbe.vmuls(x, slope_tmp)
        if negative_slope <= 1:
            res = tbe.vmax(x, tmp)
        else:
            res = tbe.vmin(x, tmp)
    else:
        # inp_dtype in ("int32", "int8")
        slope_tmp = tvm.const(negative_slope, dtype=inp_dtype)
        tmp = tbe.vmuls(x, slope_tmp)
        tmp_oritype = tbe.cast_to(tmp, inp_dtype)
        if negative_slope <= 1:
            res = tbe.vmax(x, tmp_oritype)
        else:
            res = tbe.vmin(x, tmp_oritype)

        res = tbe.cast_to(res, inp_dtype)

    return res


@para_check.check_op_params(para_check.REQUIRED_INPUT, para_check.REQUIRED_OUTPUT,
                            para_check.OPTION_ATTR_FLOAT, para_check.KERNEL_NAME)
def leaky_relu_demo(x, y, negative_slope=0, kernel_name="leaky_relu"):
    """leaky_relu op for input tensor

       f(x)= x(x>=0) or negative_slope*x(x<0) equal to
       f(x)=negative_slope*x

    Parameters
    ----------
    x : TVM tensor
        input tensor has shape and dtype attributes
    y : dict
        dict with keys(shape and dtype) of output

    negative_slope : float or int
        allow non-zero slope for negative inputs to speed up optimization

    kernel_name : str
        cce kernel name, default value is "leaky_relu"

    Returns
    ------
    None
    """

    # check input tensor shape
    shape = x.get("shape")
    dtype = x.get("dtype")

    # check input tensor data_type
    check_list = ["float16", "float32", "int32", "int8"]
    para_check.check_dtype(dtype.lower(), check_list, param_name="x")

    inp_dtype = dtype.lower()
    input_data_x = tvm.placeholder(shape, name="input_data_x", dtype=inp_dtype)

    with tvm.target.cce():

        res = leaky_relu_demo_compute(input_data_x, y, negative_slope, kernel_name)
        sch = tbe.auto_schedule(res)

    config = {"name": kernel_name,
              "tensor_list": [input_data_x, res]}
    tbe.build(sch, config)
