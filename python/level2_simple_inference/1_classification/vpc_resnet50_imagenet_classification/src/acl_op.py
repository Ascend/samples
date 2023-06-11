# -*- coding:utf-8 -*-
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
import os
import acl
import numpy as np
from constant import ACL_MEM_MALLOC_NORMAL_ONLY, \
    ACL_MEMCPY_DEVICE_TO_HOST, ACL_INT32, \
    ACL_FORMAT_ND, ACL_FLOAT, ACL_FLOAT16
from acl_util import check_ret


class SingleOp(object):
    """
    单算子运行
    """
    def __init__(self, stream, input_shape=1000, output_shape=1):
        # single_op
        self.tensor_size_cast = 0
        self.tensor_size_arg_max_v2 = 0
        self.dev_buffer_cast = None
        self.dev_buffer_arg_max_v2 = None
        self.input_shape = input_shape
        self.output_shape = output_shape
        self.output_buffer_cast = None
        self.input_buffer_arg_max_v2_second = None
        self.output_buffer_arg_max_v2 = None
        self.stream = stream
        self.input_desc_arg_max_v2 = None
        self.input_desc_arg_max_v2_second = None
        self.output_desc_arg_max_v2 = None
        self.op_type_name = ""
        self.op_attr = None
        self.init_resource()

    def __del__(self):
        if self._input_desc:
            acl.destroy_tensor_desc(self._input_desc)

        if self._output_desc:
            acl.destroy_tensor_desc(self._output_desc)

        if self.dev_buffer_cast:
            ret = acl.rt.free(self.dev_buffer_cast)
            check_ret("acl.rt.free", ret)

        if self.input_desc_arg_max_v2:
            acl.destroy_tensor_desc(self.input_desc_arg_max_v2)

        if self.input_desc_arg_max_v2_second:
            acl.destroy_tensor_desc(self.input_desc_arg_max_v2_second)

        if self.output_desc_arg_max_v2:
            acl.destroy_tensor_desc(self.output_desc_arg_max_v2)

        if self.op_attr:
            acl.op.destroy_attr(self.op_attr)

        if self.dev_buffer_arg_max_v2:
            ret = acl.rt.free(self.dev_buffer_arg_max_v2)
            check_ret("acl.rt.free", ret)

        if self.output_buffer_cast:
            ret = acl.destroy_data_buffer(self.output_buffer_cast)
            check_ret("acl.destroy_data_buffer", ret)

        if self.input_buffer_arg_max_v2_second:
            ret = acl.destroy_data_buffer(self.input_buffer_arg_max_v2_second)
            check_ret("acl.destroy_data_buffer", ret)

        if self.output_buffer_arg_max_v2:
            ret = acl.destroy_data_buffer(self.output_buffer_arg_max_v2)
            check_ret("acl.destroy_data_buffer", ret)

        print("[SingOp] class SingOp release source success")

    def init_resource(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        ret = acl.op.set_model_dir(os.path.join(current_dir, "../op_models"))
        check_ret("acl.op.set_model_dir", ret)
        self.init_resource_cast()
        self.init_resource_arg_max()

    def init_resource_cast(self):
        # settings of cast operator
        self._input_desc = acl.create_tensor_desc(ACL_FLOAT,
                                                  [self.input_shape],
                                                  ACL_FORMAT_ND)
        self._output_desc = acl.create_tensor_desc(ACL_FLOAT16,
                                                   [self.input_shape],
                                                   ACL_FORMAT_ND)

        tensor_size = acl.get_tensor_desc_size(self._output_desc)
        self.dev_buffer_cast, ret = acl.rt.malloc(tensor_size,
                                                  ACL_MEM_MALLOC_NORMAL_ONLY)
        check_ret("acl.rt.malloc", ret)

        self.output_buffer_cast = acl.create_data_buffer(self.dev_buffer_cast,
                                                         tensor_size)

    def init_resource_arg_max(self):
        self.op_type_name = "ArgMaxV2"
        self.op_attr = acl.op.create_attr()

        # settings of arg_max operator
        self.input_desc_arg_max_v2 = \
            acl.create_tensor_desc(ACL_FLOAT16,
                                   [self.input_shape, ],
                                   ACL_FORMAT_ND)
        self.input_desc_arg_max_v2_second = \
            acl.create_tensor_desc(ACL_INT32, [1, ], ACL_FORMAT_ND)
        dimension_size = acl.data_type_size(ACL_INT32)
        dimension = bytes(dimension_size)
        ret = acl.set_tensor_const(self.input_desc_arg_max_v2_second, \
            acl.util.bytes_to_ptr(dimension), dimension_size)
        self.output_desc_arg_max_v2 = \
            acl.create_tensor_desc(ACL_INT32,
                                   [self.output_shape, ],
                                   ACL_FORMAT_ND)

        self.tensor_size_arg_max_v2 = \
            acl.get_tensor_desc_size(self.output_desc_arg_max_v2)
        self.dev_buffer_arg_max_v2, ret = \
            acl.rt.malloc(self.tensor_size_arg_max_v2,
                          ACL_MEM_MALLOC_NORMAL_ONLY)
        check_ret("acl.rt.malloc", ret)

        self.input_buffer_arg_max_v2_second = \
            acl.create_data_buffer(0, 0)
        self.output_buffer_arg_max_v2 = \
            acl.create_data_buffer(self.dev_buffer_arg_max_v2,
                                   self.tensor_size_arg_max_v2)

    def run(self, dataset_ptr):
        self._gen_input_buffer(dataset_ptr)
        self._forward_op_cast()
        self._forward_op_arg_max_v2()
        self._print_label()

    def _gen_input_buffer(self, dataset_ptr):
        self.input_buffer = acl.mdl.get_dataset_buffer(dataset_ptr, 0)

    def _forward_op_cast(self):
        ret = acl.op.cast(self._input_desc,
                          self.input_buffer,
                          self._output_desc,
                          self.output_buffer_cast,
                          0,
                          self.stream)
        check_ret("acl.op.cast", ret)

        ret = acl.rt.synchronize_stream(self.stream)
        check_ret("acl.rt.synchronize_stream", ret)
        print("[SingleOP] single op cast success")

    def _forward_op_arg_max_v2(self):
        ret = acl.op.execute_v2(
            self.op_type_name,
            [self.input_desc_arg_max_v2, self.input_desc_arg_max_v2_second],
            [self.output_buffer_cast, self.input_buffer_arg_max_v2_second],
            [self.output_desc_arg_max_v2],
            [self.output_buffer_arg_max_v2],
            self.op_attr,
            self.stream)
        check_ret("acl.op.execute_v2", ret)
        ret = acl.rt.synchronize_stream(self.stream)
        check_ret("acl.rt.synchronize_stream", ret)
        print("[SingleOp] get top 1 label success")

    def _print_label(self):
        host_buffer, ret = acl.rt.malloc_host(self.tensor_size_arg_max_v2)
        check_ret("acl.rt.malloc_host", ret)
        ret = acl.rt.memcpy(host_buffer,
                            self.tensor_size_arg_max_v2,
                            self.dev_buffer_arg_max_v2,
                            self.tensor_size_arg_max_v2,
                            ACL_MEMCPY_DEVICE_TO_HOST)
        check_ret("acl.rt.memcpy", ret)

        if "ptr_to_bytes" in dir(acl.util):
            bytes_data = acl.util.ptr_to_bytes(host_buffer, self.tensor_size_arg_max_v2)
            data = np.frombuffer(bytes_data, dtype=np.int32)
        else:
            data = acl.util.ptr_to_numpy(host_buffer, (self.output_shape,), 5)
        print("[SingleOP][ArgMaxOp] label of classification result is:{}"
              .format(data[0]))
        ret = acl.rt.free_host(host_buffer)
        check_ret("acl.rt.free_host", ret)
