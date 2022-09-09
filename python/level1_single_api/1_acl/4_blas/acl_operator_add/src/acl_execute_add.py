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
import numpy as np
import os
import acl

from constant import ACL_MEMCPY_HOST_TO_DEVICE, \
    ACL_MEMCPY_DEVICE_TO_HOST, \
    ACL_MEM_MALLOC_NORMAL_ONLY, ACL_FORMAT_ND, \
    acl_dtype, NPY_INT, ACL_SUCCESS


def check_ret(message, ret):
    """
    功能简介：检测pyACL函数返回值是否正常，如果非0则会抛出异常
    参数：ret，pyACL函数返回值
    返回值：无
    """
    if ret != ACL_SUCCESS:
        raise Exception("{} failed ret={}"
                        .format(message, ret))


class Operator(object):
    """
    实现矩阵-矩阵相加运算
    """
    def __init__(self, device_id,
                 factor_a,
                 factor_b,
                 op_model_path,
                 format_type=ACL_FORMAT_ND,
                 config_path=None,
                 op_type="Add"):
        self.device_id = device_id  # int
        self.op_model_path = op_model_path  # string
        self.config_path = config_path
        self.context = None  # pointer
        self.stream = None
        self.op_attr = None
        self.op_type = op_type
        self.format_type = format_type
        self.factor_a = factor_a
        self.factor_b = factor_b

        self._inputs_desc = []
        self._inputs_device = []
        self._inputs_device_buffer = []
        self.host_inputs = []

        self.output_desc = []
        self.device_outputs = []
        self.device_buffer_outputs = []
        self.host_outputs = []
        self.init_resource()

    def release_resource(self):
        print('release source stage:')
        while self._inputs_desc:
            ret = acl.destroy_data_buffer(self._inputs_device_buffer.pop())
            check_ret("acl.destroy_data_buffer", ret)
            ret = acl.rt.free(self._inputs_device.pop())
            check_ret("acl.rt.free", ret)
            acl.destroy_tensor_desc(self._inputs_desc.pop())

        while self.output_desc:
            ret = acl.destroy_data_buffer(self.device_buffer_outputs.pop())
            check_ret("acl.destroy_data_buffer", ret)
            ret = acl.rt.free(self.device_outputs.pop())
            check_ret("acl.rt.free", ret)
            acl.destroy_tensor_desc(self.output_desc.pop())

        if self.op_attr:
            acl.op.destroy_attr(self.op_attr)
            self.op_attr = None

        if self.stream:
            ret = acl.rt.destroy_stream(self.stream)
            check_ret("acl.rt.destroy_stream", ret)
            self.stream = None

        if self.context:
            ret = acl.rt.destroy_context(self.context)
            check_ret("acl.rt.destroy_context", ret)
            self.context = None

        ret = acl.rt.reset_device(self.device_id)
        check_ret("acl.rt.reset_device", ret)
        ret = acl.finalize()
        check_ret("acl.finalize", ret)
        print('release source success')

    def init_resource(self):
        """
        功能简介：初始化所需资源
        参数：无
        返回值：无
        异常：会校验两个矩阵的shape和dtype，不一致会抛出异常
        """
        print("init resource stage:")
        if isinstance(self.config_path, str) \
                and os.path.exists(self.config_path):
            ret = acl.init(self.config_path)
            check_ret("acl.init", ret)
        elif self.config_path is None:
            ret = acl.init()
            check_ret("acl.init", ret)
        ret = acl.rt.set_device(self.device_id)
        check_ret("acl.rt.set_device", ret)

        self.context, ret = acl.rt.create_context(self.device_id)
        check_ret("acl.rt.create_context", ret)

        self.stream, ret = acl.rt.create_stream()
        check_ret("acl.rt.create_stream", ret)

        self.shape = self.factor_a.shape
        if self.factor_b.shape != self.shape:
            raise ValueError("factor_a:{} factor_b:{} isn't same shape!!!"
                             .format(self.shape, self.factor_b.shape))
        self.shape = list(self.shape)
        self.data_type = str(self.factor_a.dtype)
        if str(self.factor_b.dtype) != self.data_type:
            raise ValueError("factor_a:{} factor_b:{} isn't same dtype!!!"
                             .format(self.factor_a.dtype, self.factor_b.dtype))

        self.op_attr = acl.op.create_attr()
        ret = acl.op.set_model_dir(self.op_model_path)
        check_ret("acl.op.set_model_dir", ret)
        print("init resource success")

    def _gen_input_tensor(self):
        print("gen input data stage:")
        for factor in [self.factor_a, self.factor_b]:
            tensor = acl.create_tensor_desc(acl_dtype[self.data_type],
                                            self.shape,
                                            self.format_type)
            factor_size = acl.get_tensor_desc_size(tensor)
            factor_device, ret = acl.rt.malloc(
                factor_size, ACL_MEM_MALLOC_NORMAL_ONLY)
            check_ret("acl.rt.malloc", ret)
            if "bytes_to_ptr" in dir(acl.util):
                bytes_data = factor.tobytes()
                factor_ptr = acl.util.bytes_to_ptr(bytes_data)
            else:
                factor_ptr = acl.util.numpy_to_ptr(factor)

            ret = acl.rt.memcpy(factor_device,
                                factor_size,
                                factor_ptr,
                                factor_size,
                                ACL_MEMCPY_HOST_TO_DEVICE)
            check_ret("acl.rt.memcpy", ret)
            factor_buffer = acl.create_data_buffer(factor_device, factor_size)
            self._inputs_device.append(factor_device)
            self._inputs_device_buffer.append(factor_buffer)
            self._inputs_desc.append(tensor)
        print("gen input data success")

    def _gen_output_tensor(self):
        print("gen output data stage:")
        self.operator_output = acl.create_tensor_desc(
            acl_dtype[self.data_type],
            self.shape,
            self.format_type)
        for factor in [self.operator_output]:
            factor_size = acl.get_tensor_desc_size(factor)
            factor_device, ret = acl.rt.malloc(
                factor_size, ACL_MEM_MALLOC_NORMAL_ONLY)
            check_ret("acl.rt.malloc", ret)
            self.device_outputs.append(factor_device)
            self.device_buffer_outputs.append(
                acl.create_data_buffer(factor_device, factor_size)
            )
            self.host_outputs.append(acl.rt.malloc_host(factor_size)[0])
            self.output_desc.append(factor)
            print("gen output data success")

    def run(self):
        self._gen_input_tensor()
        self._gen_output_tensor()
        self._forward()
        result = self._get_operator_result()
        self.release_resource()
        return result

    def _forward(self):
        print('execute stage:')
        ret = acl.op.execute_v2(
            self.op_type,
            self._inputs_desc,
            self._inputs_device_buffer,
            self.output_desc,
            self.device_buffer_outputs,
            self.op_attr,
            self.stream)
        check_ret("acl.op.execute_v2", ret)
        ret = acl.rt.synchronize_stream(self.stream)
        check_ret("acl.rt.synchronize_stream", ret)
        print('execute success')

    def _get_operator_result(self):
        print('get operator result stage:')
        result = []
        for index in range(len(self.output_desc)):
            factor = self.output_desc[index]
            factor_size = acl.get_tensor_desc_size(factor)
            ret = acl.rt.memcpy(self.host_outputs[index],
                                factor_size,
                                self.device_outputs[index],
                                factor_size,
                                ACL_MEMCPY_DEVICE_TO_HOST)
            check_ret("acl.rt.memcpy", ret)

            print("shape:", tuple(self.shape))
            if "ptr_to_bytes" in dir(acl.util):
                bytes_data = acl.util.ptr_to_bytes(self.host_outputs[index], factor_size)
                data = np.frombuffer(bytes_data, dtype=np.int32).reshape(tuple(self.shape))
            else:
                data = acl.util.ptr_to_numpy(self.host_outputs[index],
                                            tuple(self.shape),
                                            NPY_INT)
            print("ACL output:\n", data)
            result.append(data)
        print('get operator result success')
        return result


if __name__ == '__main__':
    a = np.random.randint(100, size=(8, 16)).astype(np.int32)
    b = np.random.randint(100, size=(8, 16)).astype(np.int32)
    print("factor_a:\n{} \nfactor_b:\n{}".format(a, b))

    current_dir = os.path.dirname(os.path.abspath(__file__))
    op = Operator(0, a, b, os.path.join(current_dir, "../test_data/op_models"))
    result_list = op.run()
