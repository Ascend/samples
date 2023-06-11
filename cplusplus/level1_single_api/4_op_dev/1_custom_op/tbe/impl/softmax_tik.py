"""
Copyright 2022 Huawei Technologies Co., Ltd. All rights reserved.

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
from tbe import tik
from tbe.common.platform import set_current_compile_soc_info

dtype2size = {
    "float16": 2,
    "float32": 4
}
PARALLEL_SIZE = 256
BLOCK_SIZE = 32


def softmax_compute(shape, dtype, kernel_name):
    """Run softmax case
    Returns result
    """
    set_current_compile_soc_info("BS9SX1AA", "VectorCore")
    tik_instance = tik.Tik()

    x = tik_instance.Tensor(dtype, shape, scope=tik.scope_gm, name="x")
    y = tik_instance.Tensor(dtype, shape, scope=tik.scope_gm, name="y")
    x_ub = tik_instance.Tensor(dtype, shape, scope=tik.scope_ubuf, name="x_ub")
    y_ub = tik_instance.Tensor(dtype, shape, scope=tik.scope_ubuf, name="y_ub")
    sum_ub = tik_instance.Tensor(
        dtype, shape, scope=tik.scope_ubuf, name="sum_ub")
    work_tensor_ub = tik_instance.Tensor(
        dtype, shape, scope=tik.scope_ubuf, name="work_tensor_ub")

    burst = shape[0] * dtype2size.get(dtype) // BLOCK_SIZE
    mask = PARALLEL_SIZE // dtype2size.get(dtype)
    repeat_times = shape[0] // mask

    # this case only show full mask, if tail exists, tail handle functions should be added.
    tik_instance.data_move(x_ub, x, 0, 1, burst, 0, 0)
    tik_instance.vec_exp(mask, y_ub, x_ub, repeat_times, 8, 8)
    tik_instance.vector_dup(mask, sum_ub, 0, repeat_times, 1, 8)
    tik_instance.vec_reduce_add(
        mask, sum_ub, y_ub, work_tensor_ub, repeat_times, 8)
    temp_scalar = tik_instance.Scalar(dtype=dtype, init_value=sum_ub[0])
    tik_instance.vector_dup(mask, sum_ub, temp_scalar, repeat_times, 1, 8)
    tik_instance.vdiv(mask, y_ub, y_ub, sum_ub, repeat_times, 1, 1, 1, 8, 8, 8)
    tik_instance.data_move(y, y_ub, 0, 1, burst, 0, 0)

    tik_instance.BuildCCE(kernel_name=kernel_name, inputs=[x, ],
                          outputs=[y, ], config={"save_temp_cce_file": False, })

    return tik_instance


def softmax_tik(input_x, output_y, kernel_name="softmax_tik"):
    """
    softmax_tik main func
    Parameters
    ----------
    input_x: input data
    output_y: output data
    """
    shape = input_x.get("ori_shape")
    dtype = input_x.get("dtype").lower()
    output_y = output_y
    return softmax_compute(shape, dtype, kernel_name)


if __name__ == "__main__":
    _ = {"ori_shape": [256, ], "dtype": "float32"}
    softmax_tik(_, _)
