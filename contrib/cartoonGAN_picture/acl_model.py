"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2020-6-04 20:12:13
MODIFIED: 2020-6-28 14:04:45
"""
import acl
import numpy as np

from constants import *
from utils import *


class Model(object):
    def __init__(self, run_mode, model_path):
        self._run_mode = run_mode
        self.model_path = model_path    # string
        self.model_id = None            # pointer
        self.input_dataset = None
        self.output_dataset = None
        self._output_info = []
        self.model_desc = None          # pointer when using
        self.init_resource()

    def __del__(self):
        self._release_dataset(self.output_dataset)
        if self.model_id:
            ret = acl.mdl.unload(self.model_id)
            check_ret("acl.mdl.unload", ret)
        if self.model_desc:
            ret = acl.mdl.destroy_desc(self.model_desc)
            check_ret("acl.mdl.destroy_desc", ret)
        print("[Model] Model release source success")

    def init_resource(self):
        print("[Model] class Model init resource stage:")
        self.model_id, ret = acl.mdl.load_from_file(self.model_path)
        check_ret("acl.mdl.load_from_file", ret)
        self.model_desc = acl.mdl.create_desc()
        ret = acl.mdl.get_desc(self.model_desc, self.model_id)
        check_ret("acl.mdl.get_desc", ret)
        output_size = acl.mdl.get_num_outputs(self.model_desc)
        self._gen_output_dataset(output_size)
        self._gen_output_tensor(output_size)
        print("[Model] class Model init resource stage success")

        return SUCCESS

    def _gen_output_tensor(self, output_size):
        for i in range(output_size):
            dims = acl.mdl.get_output_dims(self.model_desc, i)
            shape = tuple(dims[0]["dims"])
            datatype = acl.mdl.get_output_data_type(self.model_desc, i)
            size = acl.mdl.get_output_size_by_index(self.model_desc, i)

            if datatype == ACL_FLOAT:
                np_type = np.float32            
            elif datatype == ACL_FLOAT16:
                np_type = np.float16
            elif datatype == ACL_UINT8:
                np_type = np.uint8
            else:
                raise Exception("Unspport model output datatype ", datatype)

            if np_type == np.float32:
                output_tensor = np.zeros(size // 4, dtype=np_type).reshape(shape)
            elif np_type == np.float16:
                output_tensor = np.zeros(size // 2, dtype=np_type).reshape(shape)
            else:
                output_tensor = np.zeros(size, dtype=np_type).reshape(shape)

            if not output_tensor.flags['C_CONTIGUOUS']:
                output_tensor = np.ascontiguousarray(output_tensor)

            tensor_ptr = acl.util.numpy_to_ptr(output_tensor)           
            self._output_info.append({"ptr": tensor_ptr,
                                      "tensor": output_tensor})            

    def _gen_output_dataset(self, size):
        print("[Model] create model output dataset:")
        dataset = acl.mdl.create_dataset()
        for i in range(size):
            size = acl.mdl.get_output_size_by_index(self.model_desc, i)
            buffer, ret = acl.rt.malloc(size, ACL_MEM_MALLOC_NORMAL_ONLY)
            check_ret("acl.rt.malloc", ret)
            dataset_buffer = acl.create_data_buffer(buffer, size)
            _, ret = acl.mdl.add_dataset_buffer(dataset, dataset_buffer)
            if ret:
                acl.rt.free(buffer)
                acl.destroy_data_buffer(dataset)
                check_ret("acl.destroy_data_buffer", ret)
        self.output_dataset = dataset
        print("[Model] create model output dataset success")

    def _gen_input_dataset(self, data, data_size):
        self.input_dataset = acl.mdl.create_dataset()
        input_dataset_buffer = acl.create_data_buffer(data, data_size)
        _, ret = acl.mdl.add_dataset_buffer(
            self.input_dataset,
            input_dataset_buffer)
        if ret:
            ret = acl.destroy_data_buffer(self.input_dataset)
            check_ret("acl.destroy_data_buffer", ret)

    def execute(self, data, data_size):
        self._gen_input_dataset(data, data_size)
        ret = acl.mdl.execute(self.model_id,
                              self.input_dataset,
                              self.output_dataset)
        check_ret("acl.mdl.execute", ret)
        self._release_dataset(self.input_dataset)
        return self._output_dataset_to_numpy()

    def _output_dataset_to_numpy(self):
        dataset = []
        plicy = None
        if self._run_mode == ACL_HOST:
            policy = ACL_MEMCPY_DEVICE_TO_HOST
        else:
            policy = ACL_MEMCPY_DEVICE_TO_DEVICE

        num = acl.mdl.get_dataset_num_buffers(self.output_dataset)
        for i in range(num):
            buffer = acl.mdl.get_dataset_buffer(self.output_dataset, i)
            data = acl.get_data_buffer_addr(buffer)
            size = int(acl.get_data_buffer_size(buffer))
            output_ptr = self._output_info[i]["ptr"]
            output_tensor = self._output_info[i]["tensor"]

            ret = acl.rt.memcpy(output_ptr,
                                output_tensor.size*output_tensor.itemsize,
                                data, size, policy)
            if ret != ACL_ERROR_NONE:
                print("Memcpy inference output to local failed")
                return None

            dataset.append(output_tensor)

        return dataset  

    def _release_dataset(self, dataset):
        if not dataset:
            return
        print("[Model] destroy dataset")
        num = acl.mdl.get_dataset_num_buffers(dataset)
        print("[Model] data buffer num ", num)
        for i in range(num):
            data_buf = acl.mdl.get_dataset_buffer(dataset, i)
            if data_buf:
                ret = acl.destroy_data_buffer(data_buf)
                if ret != ACL_ERROR_NONE:
                    print("Destroy data buffer error ", ret)
        ret = acl.mdl.destroy_dataset(dataset)
        if ret != ACL_ERROR_NONE:
            print("Destroy data buffer error ", ret)
        print("[Model] destroy dataset finish")


