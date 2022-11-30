"""
* @file check_out.py
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
"""
import numpy as np
import tensorflow as tf
import sys
import os

FEAUTRE_MAP = np.fromfile("input_0.bin", dtype=np.float16)
WEIGHT = np.fromfile("input_1.bin", dtype=np.float16)
FEAUTRE_MAP = FEAUTRE_MAP.reshape(8, 256, 7, 7).transpose(0, 2, 3, 1)
WEIGHT = WEIGHT.reshape(256, 256, 3, 3).transpose(2, 3, 1, 0)
FM_SHAPE = [8, 16, 7, 7, 16]
W_SHAPE = [16, 3, 3, 256, 16]
FM_DTYPE = "float16"
L0C_DTYPE = "float32"
GM_DTYPE = "float16"
PAD = [1, 1, 1, 1]

                        
def tik_conv2d(feature_map_tensor_dtype, output_dtype, output_global_memory_dtype,
               stride_list=None, pad_list=None, pad_value=0, dilation_list=None, deq=1):
    if stride_list is None:
        stride_list = [1, 1]
    if dilation_list is None:
        dilation_list = [1, 1]
    if pad_list is None:
        pad_list = [0, 0, 0, 0]
    output = tf_conv2d(stride_list, pad_list, dilation_list, pad_value, feature_map_tensor_dtype)
    tf_res = deq_dtype(output, deq, output_dtype, output_global_memory_dtype)
    tf_res = tf_res.transpose(0, 3, 1, 2)
    tik_res = np.fromfile("../../result_files/output_0.bin", dtype=np.float16)
    tik_res = tik_res.reshape(8, 256, 7, 7)
    print("The conv2dTik result is: \n", tik_res)
    if np.sum(abs(tik_res-tf_res) > precision_err_ratios*abs(tf_res)) > num_err_ratios*8*256*7*7:
        print("Compared with the tf conv2d method, the result is wrong.")  
    else:
        print("Compared with the tf conv2d method, the result is correct.")
    return tik_res


def tf_conv2d(stride, pad, dilation, value, dtype):
    stride_h, stride_w = stride
    dilation_h, dilation_w = dilation
    pad_top, pad_bot, pad_left, pad_right = pad
    with tf.Session() as sess:
        # padding
        if dtype == "float16":
            fm_new = FEAUTRE_MAP.astype("float32")
        else:
            fm_new = FEAUTRE_MAP
        fm_padding = tf.pad(fm_new, [[0, 0], [pad_top, pad_bot],
                            [pad_left, pad_right], [0, 0]],
                            constant_values=value)
        tf_output = tf.nn.conv2d(fm_padding, WEIGHT,
                           (1, stride_h, stride_w, 1), 'VALID',
                           dilations=[1, dilation_h, dilation_w, 1]).eval()
    return tf_output


def deq_dtype(ret, deq, output_dtype, output_global_memory_dtype):
    deq_value = np.float16(deq)
    if output_global_memory_dtype in ("int8", "uint8"):
        # s322f16
        with np.errstate(over="warn"):
            output = ret.astype("float16") * deq_value
        if output_global_memory_dtype == "int8":
            output = np.maximum(np.minimum(output, 127), -128)
        elif output_global_memory_dtype == "uint8":
            output = np.maximum(np.minimum(output, 255), 0)
        # fp162s8/fp162u8
        output = np.around(output).astype(output_global_memory_dtype)
    elif output_dtype == "int32" and output_global_memory_dtype == "float16":
        # s322f16
        output = ret.astype(output_global_memory_dtype) * deq_value
    elif output_dtype == "float32" and output_global_memory_dtype == "float16":
        # f322f16
        output = ret.astype(output_global_memory_dtype)
    else:
        output = ret.astype(output_global_memory_dtype)
    return output


if __name__ == "__main__":

    try:
        soc_version = sys.argv[1].upper()
    except IndexError:
        print("WARNING: The current soc_version is default Ascend310.")
        soc_version = "ASCEND310"

    if soc_version in ["SD3403", "OPTG", "HI3796CV300CS", "TSNSC"]:
        L0C_DTYPE = "float16"
        precision_err_ratios = 0.1
        num_err_ratios = 0.01
    elif soc_version == "ASCEND310":
        L0C_DTYPE = "float32"
        precision_err_ratios = 0.001
        num_err_ratios = 0.001
    else:
        raise RuntimeError("Conv2dTik not support the soc_version:%s." % soc_version)

    tik_conv2d(FM_DTYPE, L0C_DTYPE, GM_DTYPE, pad_list=PAD)
