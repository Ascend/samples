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

upsample
"""
import te.platform as tbe_platform
from tbe.common.utils import para_check
from tbe import tik

# size of 5HD format
DIM_5HD = 5
# size of c0 for fp16 fp32
C0 = 16
MAX_REPEAT = 255
RESERVE_SIZE = 16 * 1024
BLOCK_SIZE = 32


def cal_tilling(x_shape, y_shape, c0_size_in_ub):
    """
    calculate tilling

    Parameters
    ----------
    x_shape: input x shape
    y_shape: output y shape
    c0_size_in_ub: number of C0 that can be stored in the UB

    Returns
    -------
    out_loop, in_loop, axis, x_shape_in_ub, y_shape_in_ub
    """
    y_shape_in_ub = [1] * len(y_shape)
    x_shape_in_ub = [1] * len(x_shape)
    x_shape_in_ub[-1] = x_shape[-1]
    y_shape_in_ub[-1] = y_shape[-1]

    in_loop = 1
    tmp_x_size = 1
    tmp_y_size = 1
    for i in range(len(x_shape) - 2, 0, -1):
        axis = i
        stride = y_shape[i] // x_shape[i]
        pre_x_size = tmp_x_size
        pre_y_size = tmp_y_size
        tmp_x_size = tmp_x_size * x_shape[i]
        tmp_y_size = tmp_y_size * y_shape[i]

        if tmp_x_size + tmp_y_size > c0_size_in_ub:
            x_shape_in_ub[i] = c0_size_in_ub // (pre_x_size + pre_y_size)
            if x_shape_in_ub[i] > x_shape[i]:
                x_shape_in_ub[i] = x_shape[i]
            y_shape_in_ub[i] = x_shape_in_ub[i]
            if c0_size_in_ub >= pre_x_size + stride * pre_y_size:
                x_shape_in_ub[i] = c0_size_in_ub // (pre_x_size + stride * pre_y_size)
                y_shape_in_ub[i] = x_shape_in_ub[i] * stride
            in_loop = (x_shape[i] + x_shape_in_ub[i] - 1) // x_shape_in_ub[i]
            break
        x_shape_in_ub[i] = x_shape[i]
        y_shape_in_ub[i] = y_shape[i]
    out_loop = 1

    for i in range(1, axis):
        out_loop = out_loop * x_shape[i]
    return out_loop, in_loop, axis, x_shape_in_ub, y_shape_in_ub


def check_shape_dtype_format(input_shape, input_dtype, input_format):
    """
    check shape, dtype and format

    Parameters
    ----------
    input_shape: input dic shape
    input_dtype: input dtype
    input_format: input format, NC1HWC0
    The common check rule for tensor shape, just for 5hd

    Returns
    -------
    None
    """
    tik_name = tbe_platform.cce_conf.get_soc_spec("SOC_VERSION")
    if tik_name == "Hi3796CV300ES":
        check_list = ["float16"]
    else:
        check_list = ["float16", "float32"]
    if input_dtype not in check_list:
        raise RuntimeError("upsample only support %s while dtype is %s"
                           % (",".join(check_list), input_dtype))

    para_check.check_shape_rule(input_shape)
    if len(input_shape) != DIM_5HD:
        raise RuntimeError(
            "The dim of tensor must be %d"
            ", actual dim is %d" % (DIM_5HD, len(input_shape)))
    n, c1, h, w, c0 = input_shape
    shape_c0 = C0
    if input_shape[DIM_5HD - 1] != shape_c0:
        raise RuntimeError(
            "The value of C0 must be 16")

    if input_format != "NC1HWC0":
        raise RuntimeError(
            "The format must be NC1HWC0, while actual format is %s" % (input_format))


def upsample_check(input_x, stride_h, stride_w, kernel_name="upsample"):
    """
    parameters check

    Parameters
    ----------
    input_x: dict, include shape dtype and format
    stride_h: the shape change axis h
    stride_w: the shape change axis w
    kernel_name: str, kernel_name

    Returns
    -------
    None
    """
    input_shape = input_x.get("shape")
    input_format = input_x.get("format")
    input_dtype = input_x.get("dtype").lower()
    if stride_h <= 0 or stride_w <= 0:
        raise RuntimeError(
            "The stride must be greater than 0")
    check_shape_dtype_format(input_shape, input_dtype, input_format)
    para_check.check_kernel_name(kernel_name)


def get_axis_size_shape(shape, axis):
    """
    get size of shape from axis
    """
    size = 1
    for i in range(axis + 1, len(shape)):
        size = size * shape[i]
    return size


def cal_out_shape(shape, stride_h, stride_w):
    """
    calculate output shape
    """
    n, c1, h, w, c0 = shape
    out_shape = (n, c1, h * stride_h, w * stride_w, c0)
    return out_shape


def get_data_size(dtype):
    """
    get data size
    """
    if dtype == "float16":
        dsize = 2
    else:
        dsize = 4
    return dsize


class Upsample:
    def __init__(self, input_dict, stride_h, stride_w):
        self.tik_instance = tik.Tik()
        self.ub_size = tbe_platform.cce_conf.get_soc_spec(tbe_platform.cce_conf.UB_SIZE)
        self.dtype = input_dict.get("x").get("dtype").lower()
        self.x_shape = input_dict.get("x").get("shape")
        self.dsize = get_data_size(self.dtype)
        self.y_shape = cal_out_shape(self.x_shape, stride_h, stride_w)
        self.x_gm = self.tik_instance.Tensor(self.dtype, self.x_shape, name="x_gm",
                                             scope=tik.scope_gm)
        self.y_gm = self.tik_instance.Tensor(self.dtype, self.y_shape, name="y_gm",
                                             scope=tik.scope_gm)

    def upsample_compute(self, stride_h, stride_w, scale):
        """
        calculating data

        Parameters
        ----------
        stride_h: the shape change axis h
        stride_w: the shape change axis w
        scale: the value of tensor change axis

        Returns
        -------
        None
        """
        # ub_size_in_byte = tbe_platform.cce_conf.get_soc_spec(tbe_platform.cce_conf.UB_SIZE) - RESERVE_SIZE
        ub_size_in_byte = self.ub_size - RESERVE_SIZE

        c0_size_in_ub = ub_size_in_byte // self.dsize // self.x_shape[-1] // 2
        out_loop, in_loop, axis, x_shape_in_ub, y_shape_in_ub = cal_tilling(self.x_shape, self.y_shape, c0_size_in_ub)

        x_axis_num = get_axis_size_shape(self.x_shape, axis)
        y_axis_num = get_axis_size_shape(self.y_shape, axis)

        n, c1, y_h, y_w, c0 = self.y_shape
        c0_stride = c0 * self.dsize // BLOCK_SIZE
        block_size = BLOCK_SIZE // self.dsize

        mask = c0

        if in_loop * out_loop > 1:
            thread_num = 2
        else:
            thread_num = 1

        with self.tik_instance.for_range(0, n, block_num=n) as blockid:
            c1_id = self.tik_instance.Scalar("uint32", name="c1_id", init_value=0)
            c1_size = self.tik_instance.Scalar("uint32", name="c1_size", init_value=x_shape_in_ub[1])
            x_h_id = self.tik_instance.Scalar("uint32", name="x_h_id", init_value=0)
            x_h_size = self.tik_instance.Scalar("uint32", name="x_h_size", init_value=x_shape_in_ub[2])
            x_w_id = self.tik_instance.Scalar("uint32", name="x_w_id", init_value=0)
            x_w_size = self.tik_instance.Scalar("uint32", name="x_w_size", init_value=x_shape_in_ub[3])
            x_axis_size = self.tik_instance.Scalar("uint32", name="x_axis_size")
            y_axis_size = self.tik_instance.Scalar("uint32", name="y_axis_size")
            repeats = self.tik_instance.Scalar("uint32", name="repeats")
            loop = self.tik_instance.Scalar("uint32", name="loop")

            with self.tik_instance.for_range(0, out_loop * in_loop, thread_num=thread_num) as out_loopid:
                x_in_ub = self.tik_instance.Tensor(self.dtype, x_shape_in_ub, scope=tik.scope_ubuf, name="x_in_ub")
                y_in_ub = self.tik_instance.Tensor(self.dtype, y_shape_in_ub, scope=tik.scope_ubuf, name="y_in_ub")

                if axis == 1:
                    # store the c1 start of x
                    c1_id.set_as(out_loopid * x_shape_in_ub[1])
                    c1_size.set_as(x_shape_in_ub[1])
                    with self.tik_instance.if_scope(c1_id + c1_size > self.x_shape[axis]):
                        c1_size.set_as(self.x_shape[axis] - c1_id)
                    x_axis_size.set_as(c1_size)
                    y_axis_size.set_as(c1_size * y_shape_in_ub[axis] // x_shape_in_ub[axis])
                    x_h_id.set_as(0)
                    x_w_id.set_as(0)
                elif axis == 2:
                    c1_id.set_as(out_loopid // in_loop)
                    x_h_id.set_as(out_loopid % in_loop * x_shape_in_ub[axis])
                    x_w_id.set_as(0)
                    c1_size.set_as(1)
                    x_h_size.set_as(x_shape_in_ub[2])
                    with self.tik_instance.if_scope(x_h_size + x_h_id > self.x_shape[axis]):
                        x_h_size.set_as(self.x_shape[axis] - x_h_id)
                    x_axis_size.set_as(x_h_size)
                    y_axis_size.set_as(x_axis_size * y_shape_in_ub[axis] // x_shape_in_ub[axis])
                else:
                    c1_id.set_as(out_loopid // in_loop // self.x_shape[2] * x_shape_in_ub[1])
                    x_h_id.set_as(out_loopid // in_loop % self.x_shape[2] * x_shape_in_ub[2])
                    x_w_id.set_as(out_loopid % in_loop * x_shape_in_ub[3])
                    c1_size.set_as(1)
                    x_h_size.set_as(1)
                    x_w_size.set_as(x_shape_in_ub[3])
                    with self.tik_instance.if_scope(x_w_size + x_w_id > self.x_shape[3]):
                        x_w_size.set_as(self.x_shape[3] - x_w_id)
                    x_axis_size.set_as(x_w_size)
                    y_axis_size.set_as(x_w_size * y_shape_in_ub[axis] // x_shape_in_ub[axis])
                self.tik_instance.data_move(x_in_ub[0, 0, 0, 0, 0], self.x_gm[blockid, c1_id, x_h_id, x_w_id, 0],
                                            0, 1, x_axis_size * x_axis_num * self.dsize // BLOCK_SIZE, 0, 0)

                loop.set_as(x_w_size // MAX_REPEAT)
                repeats.set_as(x_w_size % MAX_REPEAT)

                with self.tik_instance.for_range(0, c1_size) as c1_index:

                    if axis == 3 or (axis == 2 and y_shape_in_ub[2] == x_shape_in_ub[2]):
                        for i in range(0, stride_h):
                            with self.tik_instance.for_range(0, x_h_size) as x_h_index:
                                with self.tik_instance.for_range(0, stride_w) as stride_w_index:
                                    with self.tik_instance.for_range(0, loop) as loop_w_id:
                                        self.tik_instance.vec_muls(
                                            mask,
                                            y_in_ub[0, c1_index, x_h_index,
                                                    MAX_REPEAT * loop_w_id * stride_w + stride_w_index, 0],
                                            x_in_ub[0, c1_index, (x_h_index + i * x_h_size) // stride_h,
                                                    MAX_REPEAT * loop_w_id, 0],
                                            scale, MAX_REPEAT, c0_stride * stride_w, c0_stride)
                                    with self.tik_instance.if_scope(repeats > 0):
                                        self.tik_instance.vec_muls(
                                            mask,
                                            y_in_ub[0, c1_index, x_h_index,
                                                    MAX_REPEAT * loop * stride_w + stride_w_index, 0],
                                            x_in_ub[0, c1_index, (x_h_index + i * x_h_size) // stride_h,
                                                    (MAX_REPEAT * mask * loop) // block_size, 0],
                                            scale, repeats, c0_stride * stride_w, c0_stride)

                            self.tik_instance.data_move(
                                self.y_gm[blockid, c1_id, x_h_id * stride_h + i * x_h_size, x_w_id * stride_w, 0],
                                y_in_ub[0, 0, 0, 0, 0],
                                0, 1, y_axis_size * y_axis_num * self.dsize // BLOCK_SIZE, 0, 0)

                    else:
                        with self.tik_instance.for_range(0, x_h_size * stride_h) as y_h_index:
                            with self.tik_instance.for_range(0, stride_w) as stride_w_index:
                                with self.tik_instance.for_range(0, loop) as loop_w_id:
                                    self.tik_instance.vec_muls(
                                        mask,
                                        y_in_ub[0, c1_index, y_h_index,
                                                MAX_REPEAT * loop_w_id * stride_w + stride_w_index, 0],
                                        x_in_ub[0, c1_index, y_h_index // stride_h, (MAX_REPEAT * loop_w_id), 0],
                                        scale, MAX_REPEAT, c0_stride * stride_w, c0_stride)

                                with self.tik_instance.if_scope(repeats > 0):
                                    self.tik_instance.vec_muls(
                                        mask, y_in_ub[0, c1_index, y_h_index,
                                                      MAX_REPEAT * loop * stride_w + stride_w_index, 0],
                                        x_in_ub[0, c1_index, y_h_index // stride_h, (MAX_REPEAT * loop), 0],
                                        scale, repeats, c0_stride * stride_w, c0_stride)

                        with self.tik_instance.if_scope(c1_size == 1):
                            self.tik_instance.data_move(
                                self.y_gm[blockid, c1_id, x_h_id * stride_h, x_w_id * stride_w, 0], y_in_ub, 0, 1,
                                y_axis_size * y_axis_num * self.dsize // BLOCK_SIZE, 1, 1)

                with self.tik_instance.if_scope(c1_size > 1):
                    self.tik_instance.data_move(
                        self.y_gm[blockid, c1_id, x_h_id * stride_h, x_w_id * stride_w, 0],
                        y_in_ub, 0, 1, y_axis_size * y_axis_num * self.dsize // BLOCK_SIZE, 1, 1)


@para_check.check_op_params(para_check.REQUIRED_INPUT, para_check.REQUIRED_OUTPUT,
                            para_check.OPTION_ATTR_FLOAT, para_check.OPTION_ATTR_INT,
                            para_check.OPTION_ATTR_INT, para_check.KERNEL_NAME)
def upsample_tik(input_x, output_y, scale=1, stride_h=2, stride_w=2, kernel_name="upsample"):
    """
    the interface of upsample op

    Parameters
    ----------
    input_x: dict
        shape and dtype of input
    output_y: dict
        shape and dtype of output, should be same shape and type as input
    scale: float
        the value of tensor change axis, default value is 1
    stride_h: int
        the shape change axis h
    stride_w: int
        the shape change axis w
    kernel_name: str
        kernel name, default value is "upsample"

    Returns
    -------
    None
    """
    upsample_check(input_x, stride_h, stride_w, kernel_name)

    input_dict = {
        "x": input_x,
        "y": output_y,
        "kernel_name": kernel_name
    }
    upsample_instance = Upsample(input_dict, stride_h, stride_w)
    upsample_instance.upsample_compute(stride_h, stride_w, scale)
    upsample_instance.tik_instance.BuildCCE(kernel_name=kernel_name,
                                            inputs=upsample_instance.x_gm,
                                            outputs=upsample_instance.y_gm)

    return upsample_instance.tik_instance
