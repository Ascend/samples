"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2020-6-04 20:12:13
MODIFIED: 2020-6-28 14:04:45
"""

import acl
import struct
import numpy as np
import datetime
import sys
import os

import atlas_utils.constants as const
import atlas_utils.utils as utils
from atlas_utils.acl_logger import log_error, log_info
from atlas_utils.acl_image import AclImage
from atlas_utils.resource_list import resource_list


class Model(object):
    """
    wrap acl model inference interface, include input dataset construction,
    execute, and output transform to numpy array
    Attributes:
        model_path: om offline mode file path
    """

    def __init__(self, model_path):
        self._run_mode, ret = acl.rt.get_run_mode()
        utils.check_ret("acl.rt.get_run_mode", ret)
        self._copy_policy = const.ACL_MEMCPY_DEVICE_TO_DEVICE
        if self._run_mode == const.ACL_HOST:
            self._copy_policy = const.ACL_MEMCPY_DEVICE_TO_HOST

        self._model_path = model_path    # string
        self._model_id = None            # pointer
        self._input_num = 0
        self._input_buffer = []
        self._input_dataset = None
        self._output_dataset = None
        self._model_desc = None          # pointer when using
        self._output_size = 0
        self._init_resource()
        self._is_destroyed = False
        resource_list.register(self)

    def _init_resource(self):
        log_info("Init model resource start...")
        if not os.path.isfile(self._model_path):
            log_error(
                "model_path failed, please check. model_path=%s" %
                self._model_path)
            return const.FAILED

        self._model_id, ret = acl.mdl.load_from_file(self._model_path)
        utils.check_ret("acl.mdl.load_from_file", ret)
        self._model_desc = acl.mdl.create_desc()
        ret = acl.mdl.get_desc(self._model_desc, self._model_id)
        utils.check_ret("acl.mdl.get_desc", ret)
        # get outputs num of model
        self._output_size = acl.mdl.get_num_outputs(self._model_desc)
        # create output dataset
        self._gen_output_dataset(self._output_size)
        # recode input data address,if need malloc memory,the memory will be
        # reuseable
        self._init_input_buffer()

        log_info("Init model resource success")

        return const.SUCCESS

    def _gen_output_dataset(self, ouput_num):
        log_info("[Model] create model output dataset:")
        dataset = acl.mdl.create_dataset()
        for i in range(ouput_num):
            # malloc device memory for output
            size = acl.mdl.get_output_size_by_index(self._model_desc, i)
            buf, ret = acl.rt.malloc(size, const.ACL_MEM_MALLOC_NORMAL_ONLY)
            utils.check_ret("acl.rt.malloc", ret)
            # crate oputput data buffer
            dataset_buffer = acl.create_data_buffer(buf, size)
            _, ret = acl.mdl.add_dataset_buffer(dataset, dataset_buffer)
            log_info("malloc output %d, size %d" % (i, size))
            if ret:
                acl.rt.free(buf)
                acl.destroy_data_buffer(dataset_buffer)
                utils.check_ret("acl.destroy_data_buffer", ret)
        self._output_dataset = dataset
        log_info("Create model output dataset success")

    def _init_input_buffer(self):
        self._input_num = acl.mdl.get_num_inputs(self._model_desc)
        for i in range(self._input_num):
            item = {"addr": None, "size": 0}
            self._input_buffer.append(item)

    def _gen_input_dataset(self, input_list):
        ret = const.SUCCESS
        if len(input_list) != self._input_num:
            log_error("Current input data num %d unequal to model "
                      "input num %d" % (len(input_list), self._input_num))
            return const.FAILED

        self._input_dataset = acl.mdl.create_dataset()
        for i in range(self._input_num):
            item = input_list[i]
            data, size = self._parse_input_data(item, i)
            if (data is None) or (size == 0):
                ret = const.FAILED
                log_error("The %d input is invalid" % (i))
                break
            dataset_buffer = acl.create_data_buffer(data, size)
            _, ret = acl.mdl.add_dataset_buffer(self._input_dataset,
                                                dataset_buffer)
            if ret:
                log_error("Add input dataset buffer failed")
                acl.destroy_data_buffer(self._input_dataset)
                ret = const.FAILED
                break
        if ret == const.FAILED:
            self._release_dataset(self._input_dataset)
            self._input_dataset = None

        return ret

    def _parse_input_data(self, input_data, index):
        data = None
        size = 0
        if isinstance(input_data, AclImage):
            size = input_data.size
            data = input_data.data()
        elif isinstance(input_data, np.ndarray):
            ptr = acl.util.numpy_to_ptr(input_data)
            size = input_data.size * input_data.itemsize
            data = self._copy_input_to_device(ptr, size, index)
            if data is None:
                size = 0
                log_error("Copy input to device failed")
        elif (isinstance(input_data, dict) and
              ('data' in input_data.keys()) and ('size' in input_data.keys())):
            size = input_data['size']
            data = input_data['data']
        else:
            log_error("Unsupport input")

        return data, size

    def _copy_input_to_device(self, input_ptr, size, index):
        buffer_item = self._input_buffer[index]
        data = None
        if buffer_item['addr'] is None:
            data = utils.copy_data_device_to_device(input_ptr, size)
            if data is None:
                log_error("Malloc memory and copy model %dth "
                          "input to device failed" % (index))
                return None
            buffer_item['addr'] = data
            buffer_item['size'] = size
        elif size == buffer_item['size']:
            ret = acl.rt.memcpy(buffer_item['addr'], size,
                                input_ptr, size,
                                const.ACL_MEMCPY_DEVICE_TO_DEVICE)
            if ret != const.ACL_ERROR_NONE:
                log_error("Copy model %dth input to device failed" % (index))
                return None
            data = buffer_item['addr']
        else:
            log_error("The model %dth input size %d is change,"
                      " before is %d" % (index, size, buffer_item['size']))
            return None

        return data

    def execute(self, input_list):
        """
        inference input data
        Args:
            input_list: input data list, support AclImage,
            numpy array and {'data': ,'size':} dict
        returns:
            inference result data, which is a numpy array list,
            each corresponse to a model output
        """
        ret = self._gen_input_dataset(input_list)
        if ret == const.FAILED:
            log_error("Gen model input dataset failed")
            return None

        ret = acl.mdl.execute(self._model_id,
                              self._input_dataset,
                              self._output_dataset)
        if ret != const.ACL_ERROR_NONE:
            log_error("Execute model failed for acl.mdl.execute error ", ret)
            return None

        self._release_dataset(self._input_dataset)
        self._input_dataset = None

        return self._output_dataset_to_numpy()

    def _output_dataset_to_numpy(self):
        dataset = []
        output_tensor_list = self._gen_output_tensor()
        num = acl.mdl.get_dataset_num_buffers(self._output_dataset)

        for i in range(num):
            buf = acl.mdl.get_dataset_buffer(self._output_dataset, i)
            data = acl.get_data_buffer_addr(buf)
            size = int(acl.get_data_buffer_size(buf))
            output_ptr = output_tensor_list[i]["ptr"]
            output_tensor = output_tensor_list[i]["tensor"]
            ret = acl.rt.memcpy(output_ptr,
                                output_tensor.size * output_tensor.itemsize,
                                data, size, self._copy_policy)
            if ret != const.ACL_ERROR_NONE:
                log_error("Memcpy inference output to local failed")
                return None

            dataset.append(output_tensor)

        return dataset

    def _gen_output_tensor(self):
        output_tensor_list = []
        for i in range(self._output_size):
            dims = acl.mdl.get_output_dims(self._model_desc, i)
            shape = tuple(dims[0]["dims"])
            datatype = acl.mdl.get_output_data_type(self._model_desc, i)
            size = acl.mdl.get_output_size_by_index(self._model_desc, i)

            if datatype == const.ACL_FLOAT:
                np_type = np.float32
                output_tensor = np.zeros(
                    size // 4, dtype=np_type).reshape(shape)
            elif datatype == const.ACL_INT32:
                np_type = np.int32
                output_tensor = np.zeros(
                    size // 4, dtype=np_type).reshape(shape)
            elif datatype == const.ACL_UINT32:
                np_type = np.uint32
                output_tensor = np.zeros(
                    size // 4, dtype=np_type).reshape(shape)
            elif datatype == const.ACL_FLOAT16:
                np_type = np.float16
                output_tensor = np.zeros(
                    size // 2, dtype=np_type).reshape(shape)
            else:
                print("Unspport model output datatype ", datatype)
                return None

            if not output_tensor.flags['C_CONTIGUOUS']:
                output_tensor = np.ascontiguousarray(output_tensor)

            tensor_ptr = acl.util.numpy_to_ptr(output_tensor)
            output_tensor_list.append({"ptr": tensor_ptr,
                                       "tensor": output_tensor})

        return output_tensor_list

    def _release_dataset(self, dataset, free_memory=False):
        if not dataset:
            return

        num = acl.mdl.get_dataset_num_buffers(dataset)
        for i in range(num):
            data_buf = acl.mdl.get_dataset_buffer(dataset, i)
            if data_buf:
                self._release_databuffer(data_buf, free_memory)

        ret = acl.mdl.destroy_dataset(dataset)
        if ret != const.ACL_ERROR_NONE:
            log_error("Destroy data buffer error ", ret)

    def _release_databuffer(self, data_buffer, free_memory=False):
        if free_memory:
            data_addr = acl.get_data_buffer_addr(data_buffer)
            if data_addr:
                acl.rt.free(data_addr)

        ret = acl.destroy_data_buffer(data_buffer)
        if ret != const.ACL_ERROR_NONE:
            log_error("Destroy data buffer error ", ret)

    def destroy(self):
        """
        release resource of model inference
        Args:
            null
        Returns:
            null
        """
        if self._is_destroyed:
            return

        self._release_dataset(self._output_dataset, free_memory=True)
        if self._model_id:
            ret = acl.mdl.unload(self._model_id)
            if ret != const.ACL_ERROR_NONE:
                log_info("acl.mdl.unload error:", ret)

        if self._model_desc:
            ret = acl.mdl.destroy_desc(self._model_desc)
            if ret != const.ACL_ERROR_NONE:
                log_info("acl.mdl.destroy_desc error:", ret)

        self._is_destroyed = True
        resource_list.unregister(self)
        log_info("Model release source success")

    def __del__(self):
        self.destroy()
