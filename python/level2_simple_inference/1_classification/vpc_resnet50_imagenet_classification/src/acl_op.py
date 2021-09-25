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
import acl
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
        self.tensor_size_arg_max_d = 0
        self.dev_buffer_cast = None
        self.dev_buffer_arg_max_d = None
        self.input_shape = input_shape
        self.output_shape = output_shape
        self.output_buffer_cast = None
        self.output_buffer_arg_max_d = None
        self.stream = stream
        self.input_desc_arg_max_d = None
        self.output_desc_arg_max_d = None
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

        if self.input_desc_arg_max_d:
            acl.destroy_tensor_desc(self.input_desc_arg_max_d)

        if self.output_desc_arg_max_d:
            acl.destroy_tensor_desc(self.output_desc_arg_max_d)

        if self.op_attr:
            acl.op.destroy_attr(self.op_attr)

        if self.dev_buffer_arg_max_d:
            ret = acl.rt.free(self.dev_buffer_arg_max_d)
            check_ret("acl.rt.free", ret)

        if self.output_buffer_cast:
            ret = acl.destroy_data_buffer(self.output_buffer_cast)
            check_ret("acl.destroy_data_buffer", ret)

        if self.output_buffer_arg_max_d:
            ret = acl.destroy_data_buffer(self.output_buffer_arg_max_d)
            check_ret("acl.destroy_data_buffer", ret)

        print("[SingOp] class SingOp release source success")

    def init_resource(self):
        ret = acl.op.set_model_dir("./op_models")
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
        self.op_type_name = "ArgMaxD"
        self.op_attr = acl.op.create_attr()
        ret = acl.op.set_attr_int(self.op_attr, "dimension", 0)
        check_ret("acl.op.set_attr_int", ret)

        # settings of arg_max operator
        self.input_desc_arg_max_d = \
            acl.create_tensor_desc(ACL_FLOAT16,
                                   [self.input_shape, ],
                                   ACL_FORMAT_ND)
        self.output_desc_arg_max_d = \
            acl.create_tensor_desc(ACL_INT32,
                                   [self.output_shape, ],
                                   ACL_FORMAT_ND)

        self.tensor_size_arg_max_d = \
            acl.get_tensor_desc_size(self.output_desc_arg_max_d)
        self.dev_buffer_arg_max_d, ret = \
            acl.rt.malloc(self.tensor_size_arg_max_d,
                          ACL_MEM_MALLOC_NORMAL_ONLY)
        check_ret("acl.rt.malloc", ret)

        self.output_buffer_arg_max_d = \
            acl.create_data_buffer(self.dev_buffer_arg_max_d,
                                   self.tensor_size_arg_max_d)

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

    def _forward_op_arg_max_d(self):
        ret = acl.op.execute_v2(
            self.op_type_name,
            [self.input_desc_arg_max_d],
            [self.output_buffer_cast],
            [self.output_desc_arg_max_d],
            [self.output_buffer_arg_max_d],
            self.op_attr,
            self.stream)
        check_ret("acl.op.execute_v2", ret)
        ret = acl.rt.synchronize_stream(self.stream)
        check_ret("acl.rt.synchronize_stream", ret)
        print("[SingleOp] get top 1 label success")

    def run(self, dataset_ptr):
        self._gen_input_buffer(dataset_ptr)
        self._forward_op_cast()
        self._forward_op_arg_max_d()
        self._print_label()

    def _print_label(self):
        host_buffer, ret = acl.rt.malloc_host(self.tensor_size_arg_max_d)
        check_ret("acl.rt.malloc_host", ret)
        ret = acl.rt.memcpy(host_buffer,
                            self.tensor_size_arg_max_d,
                            self.dev_buffer_arg_max_d,
                            self.tensor_size_arg_max_d,
                            ACL_MEMCPY_DEVICE_TO_HOST)
        check_ret("acl.rt.memcpy", ret)

        data = acl.util.ptr_to_numpy(host_buffer, (self.output_shape,), 5)
        print("[SingleOP][ArgMaxOp] label of classification result is:{}"
              .format(data[0]))
        ret = acl.rt.free_host(host_buffer)
        check_ret("acl.rt.free_host", ret)
