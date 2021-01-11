# Copyright 2020 Huawei Technologies Co., Ltd
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ============================================================================
"""
assign.py
"""
from te import tik
from te import platform as tbe_platform
import te.lang.dynamic
from te.utils import para_check
from te.utils.error_manager import error_manager_vector


# max int32
MAX_INT32 = 2 ** 31 - 1
# tiling param num
TILING_ARG_NUM = 16
# reserved ub size
RESERVED_UB_SIZE = 8 * 1024
# MAX REPEAT NUM
MAX_REPEAT_NUM = 254


# pylint: disable=too-many-instance-attributes
class Assign:
    """
    Class for Dynamic shape operator Assign
    """
    def __init__(self, ref, value, output, kernel_name):
        self.tik_instance = tik.Tik(tik.Dprofile)
        self.ref_dtype = ref.get("dtype").lower()
        self.value_dtype = value.get("dtype").lower()
        self.out_dtype = output.get("dtype").lower()

        # check dtype
        para_check.check_dtype(self.ref_dtype,
                               ("float16", "float32", "int8", "int32", "int64", "uint8"), param_name="ref")
        para_check.check_dtype(self.value_dtype,
                               ("float16", "float32", "int8", "int32", "int64", "uint8"), param_name="value")
        if self.ref_dtype != self.value_dtype:
            error_manager_vector.raise_err_inputs_dtype_not_equal("Assign", "ref", "value",
                                                                  self.ref_dtype, self.value_dtype)
        self.kernel_name = kernel_name

        self.ai_core_num = tbe_platform.cce_conf.get_soc_spec(tbe_platform.cce_conf.CORE_NUM)
        self.ub_size_bytes = (tbe_platform.cce_conf.get_soc_spec(tbe_platform.cce_conf.UB_SIZE) - RESERVED_UB_SIZE)

        if self.ref_dtype == "int8" or self.ref_dtype == "uint8":
            self.ele_per_block = 32
        elif self.ref_dtype == "float16":
            self.ele_per_block = 16
        elif self.ref_dtype == "float32" or self.ref_dtype == "int32":
            self.ele_per_block = 8
        else:
            self.ele_per_block = 4

        self.max_block_num = MAX_REPEAT_NUM
        self.max_ele_num = self.max_block_num * self.ele_per_block

        self.tiling_gm = self.tik_instance.Tensor("int64", (TILING_ARG_NUM,), name="tiling_gm", scope=tik.scope_gm)
        self.ref_gm = self.tik_instance.Tensor(self.ref_dtype, (MAX_INT32,), name="ref_gm", scope=tik.scope_gm)
        self.value_gm = self.tik_instance.Tensor(self.value_dtype, (MAX_INT32,), name="value_gm", scope=tik.scope_gm)
        self.out_gm = self.tik_instance.Tensor(self.ref_dtype, (MAX_INT32,), name="out_gm", scope=tik.scope_gm)

        self.tiling_ub = None
        self.value_ub = None
        self.out_ub = None

        self.core_used_num = self.tik_instance.Scalar("int64", name="core_used_num")
        self.block_per_core = self.tik_instance.Scalar("int64", name="block_per_core")
        self.block_tail_core = self.tik_instance.Scalar("int64", name="block_tail_core")

    def _tiling_args(self):
        """
        get runtime tiling parameters from tiling
        """
        # read tiling int64 scalar
        self.core_used_num.set_as(self.tiling_ub[0])
        self.block_per_core.set_as(self.tiling_ub[1])
        self.block_tail_core.set_as(self.tiling_ub[2])

    def _init_ub_tensor(self):
        """
        compute the ub size of tensors
        """
        self.value_ub = self.tik_instance.Tensor(self.value_dtype, (self.max_ele_num,),
                                                 name="value_ub", scope=tik.scope_ubuf)
        self.out_ub = self.tik_instance.Tensor(self.ref_dtype, (self.max_ele_num,),
                                               name="out_ub", scope=tik.scope_ubuf)

    def _run_one_loop(self, gm_offset, block_num):
        self.tik_instance.data_move(self.value_ub, self.value_gm[gm_offset], 0, 1, block_num, 0, 0)
        self.tik_instance.data_move(self.out_gm[gm_offset], self.value_ub, 0, 1, block_num, 0, 0)

    def run_one_core(self, _core_idx, block_num):
        """
        run assign in one core
        """
        copy_loop = block_num // self.max_block_num
        copy_tail = block_num % self.max_block_num

        with self.tik_instance.for_range(0, copy_loop) as _copy_idx:
            copy_gm_offset = _core_idx * self.block_per_core + _copy_idx * self.max_block_num
            copy_gm_offset = copy_gm_offset * self.ele_per_block
            self._run_one_loop(copy_gm_offset, self.max_block_num)

        with self.tik_instance.if_scope(copy_tail > 0):
            copy_gm_offset = _core_idx * self.block_per_core + copy_loop * self.max_block_num
            copy_gm_offset = copy_gm_offset * self.ele_per_block
            self._run_one_loop(copy_gm_offset, copy_tail)

    def assign_compute(self):
        """
        The tik implementation of operator Assign
        """
        with self.tik_instance.for_range(0, self.ai_core_num, block_num=self.ai_core_num) as _core_idx:
            self.tiling_ub = self.tik_instance.Tensor("int64", (TILING_ARG_NUM,),
                                                      name="tiling_ub", scope=tik.scope_ubuf)
            self.tik_instance.data_move(self.tiling_ub, self.tiling_gm, 0, 1, 2, 0, 0)
            self._tiling_args()
            self._init_ub_tensor()

            with self.tik_instance.if_scope(_core_idx < (self.core_used_num - 1)):
                self.run_one_core(_core_idx, self.block_per_core)
            with self.tik_instance.if_scope(_core_idx == (self.core_used_num - 1)):
                self.run_one_core(_core_idx, self.block_tail_core)

        opt_config = {"out_of_bound_sync_check": True}
        self.tik_instance.BuildCCE(kernel_name=self.kernel_name,
                                   inputs=(self.ref_gm, self.value_gm),
                                   outputs=(self.out_gm,),
                                   flowtable=(self.tiling_gm,), config=opt_config)
        te.op.add_compile_info("vars", {"ub_size": self.ub_size_bytes, "core_num": self.ai_core_num})


@te.op.register_operator("Assign")
@para_check.check_op_params(para_check.REQUIRED_INPUT, para_check.REQUIRED_INPUT,
                            para_check.REQUIRED_OUTPUT, para_check.KERNEL_NAME)
def assign(ref, value, output, kernel_name="assign"):
    """
    algorithm: assign
    calculating: update 'ref' by assigning 'value' to it

    Parameters
    ----------
    ref: dict
        dict of input_ref, include shape and dtype,
    value: dict
        dict of input_value, include shape and dtype,
        Must have the same shape and dtype as input_ref
    output: dict
        dict of output
    kernel_name : str
        cce kernel name, default value is assign

    Returns
    -------
    None
    """
    obj = Assign(ref, value, output, kernel_name)
    obj.assign_compute()

