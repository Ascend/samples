"""
Copyright (C) 2019. Huawei Technologies Co., Ltd. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the Apache License Version 2.0.You may not use this file
except in compliance with the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
Apache License for more details at
http://www.apache.org/licenses/LICENSE-2.0

conv2d_tik
"""
from __future__ import absolute_import
import numpy as np
from tbe import tik
from tbe.common.platform import get_soc_spec

DTYPE_SIZE = {
    'bool': 1,
    'uint8': 1,
    'int8': 1,
    'uint16': 2,
    'int16': 2,
    'int24': 3,
    'uint32': 4,
    'int32': 4,
    'float16': 2,
    'float32': 4,
    'int48': 6,
    'int64': 8,
    'uint64': 8,
    'float64':8
}


def conv2d_tik_compute(params):
    """
    conv2d tik compute
    @param params: conv2d data
    @return: tik instance
    """
    tik_instance = tik.Tik()

    # get shape of feature map and weight
    n, c1, h, w, c0 = params["fm_shape"]
    c1, kh, kw, cout, c0 = params["weight_shape"]

    # get value of stride, dilation, pad
    stride_h, stride_w = params["stride_list"]
    dilation_h, dilation_w = params["dilation_list"]
    pad_top, pad_bot, pad_left, pad_right = params["pad_list"]

    # calculate height and width
    kh_dilation = (kh - 1) * dilation_h + 1
    kw_dilation = (kw - 1) * dilation_w + 1
    ho = int(np.ceil((h + pad_top + pad_bot - kh_dilation + 1) / stride_h)) 
    wo = int(np.ceil((w + pad_right + pad_left - kw_dilation + 1) / stride_w))
    round_howo = ((ho * wo + 16 - 1) // 16) * 16

    fm_gm = tik_instance.Tensor(params['fm_dtype'], (n, c1, h, w, c0),
                                name='fm_gm', scope=tik.scope_gm)
    weight_gm = tik_instance.Tensor(params['weight_type'],
                                    (c1, kh, kw, cout, c0), name='weight_gm',
                                    scope=tik.scope_gm)
    dst_gm = tik_instance.Tensor(params['dst_gm_type'],
                                 [n, cout // 16, ho, wo, 16],
                                 name='dst_gm', scope=tik.scope_gm)

    core_num = params['core_num']
    pre_core_cout = cout // core_num
    cout_iter_num = pre_core_cout // params["cout_split_factor"]
    Cin_blocks = c1

    with tik_instance.for_range(0, core_num, block_num=core_num) as cout_o:
        with tik_instance.for_range(0, cout_iter_num, thread_num=1) as cout_i:
            weight_L1 = tik_instance.Tensor(
                params['weight_type'], (Cin_blocks, kh, kw,
                                        params["cout_split_factor"], c0),
                name='weight_l1', scope=tik.scope_cbuf)
            tik_instance.data_move(
                weight_L1,
                weight_gm.flatten()[cout_o * pre_core_cout * c0 +
                                    params["cout_split_factor"] * cout_i * c0],
                                    0, Cin_blocks * kh * kw, 
                                    params["cout_split_factor"],
                                    (cout - params["cout_split_factor"]), 0)

            with tik_instance.for_range(0, n, thread_num=2) as n_index:
                feature_map_l1 = tik_instance.Tensor(params['fm_dtype'],
                                                     (c1, h, w, c0),
                                                     name='feature_map_l1',
                                                     scope=tik.scope_cbuf)
                tik_instance.data_move(feature_map_l1,
                                        fm_gm[n_index, :, :, :, :],
                                        0, 1, c1 * h * w, 0, 0)
                dst_l0c = tik_instance.Tensor(
                    params['dst_l0c_type'], [params["cout_split_factor"] // 16,
                                             round_howo, 16],
                    name='dst_l0c', scope=tik.scope_cbuf_out)

                tik_instance.conv2d(dst_l0c, feature_map_l1,
                                    weight_L1, (c1, h, w, c0),
                                    (Cin_blocks, kh, kw,
                                    params["cout_split_factor"], c0),
                                    params['stride_list'],
                                    [pad_left, pad_right, pad_top, pad_bot],
                                    params['dilation_list'],
                                    params['pad_value'])

                tik_instance.fixpipe(
                    dst_gm[n_index, (cout_o * pre_core_cout + params["cout_split_factor"] * cout_i) //
                           (32 // DTYPE_SIZE[params['dst_gm_type']]), 0, 0, 0],
                    dst_l0c, params["cout_split_factor"] // 16,
                    ho * wo * 16 * DTYPE_SIZE[params['dst_l0c_type']] // 32, 0, 0,
                    extend_params={"quantize_params": params["quantize_params"]})

    tik_instance.BuildCCE(kernel_name=params["kernel_name"], inputs=[fm_gm, weight_gm], outputs=[dst_gm])

    return tik_instance


def conv2d_tik(inputs, weights, outputs, strides, pads, dilations, kernel_name="conv2d_tik"):
    in_dtype = inputs.get("dtype")
    w_dtype = weights.get("dtype")
    res_dtype = outputs.get("dtype")
    in_shape = inputs.get("shape")
    w_shape = weights.get("ori_shape")
            
    if len(strides) != 4:
        raise RuntimeError("strides shape should be 4d.")
    if len(dilations) != 4:
        raise RuntimeError("dilations shape should be 4d.")
    if len(pads) != 4:
        raise RuntimeError("pads shape should be 4d.")
    if in_dtype != "float16" or w_dtype != "float16" or res_dtype != "float16":
        raise RuntimeError("dtype shape should be float16.")
    if weights.get("ori_format") != "NCHW":
        raise RuntimeError("format should be NCHW.")

    if get_soc_spec("SOC_VERSION") in ["SD3403", "OPTG", "Hi3796CV300CS", "TsnsC"]:
        loc_dtype = "float16"
        quantize_params = None
    else:
        loc_dtype = "float32"
        quantize_params = {"mode": "fp322fp16", "mode_param": None}

    stride_list = [strides[2], strides[3]]
    dilation_list = [dilations[2], dilations[3]]                    
    w5hd_shape = [w_shape[1] // 16, w_shape[2], w_shape[3], w_shape[0], 16]

    params = {
        "fm_shape": in_shape,
        "weight_shape": w5hd_shape,
        "fm_dtype": in_dtype,
        "weight_type": w_dtype,
        "dst_l0c_type": loc_dtype,
        "dst_gm_type": res_dtype,
        "quantize_params": quantize_params,
        "pad_list": pads,
        "pad_value": 0,
        "stride_list": stride_list,
        "dilation_list": dilation_list,
        "cout_split_factor": 64,
        "core_num": 2,
        "kernel_name": kernel_name}

    conv2d_tik_compute(params)
