"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2020-6-04 20:12:13
MODIFIED: 2020-6-28 14:04:45
"""
import sys
import acl
import struct
import numpy as np
import datetime
from atlas_utils import constants
from atlas_utils import acl_model
from atlas_utils import utils
from atlas_utils import acl_dvpp


class Model(object):
    """
    Perform model loading,
    Initialization, reasoning process
    """
    def __init__(self, run_mode, model_path):
        print("load model ", model_path)
#         self.acl_resource = acl_resource
        self._run_mode = run_mode
        self.model_path = model_path    # string
        self.model_id = None            # pointer
        self.input_dataset = None
        self.output_dataset = None
        self._input_num = None
        self._input_buffer = None
        self._output_info = []
        self.model_desc = None          # pointer when using
        self._init_resource()
        self._is_released = False
#         acl_resource.register_resource(self)
    
    def __del__(self):
        if self._is_released:
            return
        #unregister acl resource
#         self.acl_resource.unregister_resource(self)

        print("Model start release...")
        #Destroys input and output data of the aclmdlDataset type
        self._release_dataset(self.input_dataset)
        self._release_dataset(self.output_dataset)
        if self.model_id:
            #Uninstalls the model and releases resources
            ret = acl.mdl.unload(self.model_id)
            if ret != constants.ACL_ERROR_NONE:
                print("acl.mdl.unload error:", ret)

        if self.model_desc:
            #Destroys data of the aclmdlDesc type
            ret = acl.mdl.destroy_desc(self.model_desc)
            if ret != constants.ACL_ERROR_NONE:
                print("acl.mdl.destroy_desc error:", ret)
        self._is_released = True
        print("Model release source success")

    def destroy(self):
        """
        destroy
        """
        self.__del__()

    def _init_resource(self):
        print("Init model resource")
        #load model 
        self.model_id, ret = acl.mdl.load_from_file(self.model_path)
        acl_model.check_ret("acl.mdl.load_from_file", ret)
        self.model_desc = acl.mdl.create_desc()
        ret = acl.mdl.get_desc(self.model_desc, self.model_id)
        acl_model.check_ret("acl.mdl.get_desc", ret)
        #get input num
        output_size = acl.mdl.get_num_outputs(self.model_desc)
        #create output dataset
        self._gen_output_dataset(output_size)
        print("[Model] class Model init resource stage success")
        #get output tensor 
        self._get_output_desc(output_size)
        #creat input buffer table
        self._init_input_buffer()

        return constants.SUCCESS

    def _get_output_desc(self, output_size):
        for i in range(output_size):
            #get input tensor
            dims = acl.mdl.get_output_dims(self.model_desc, i)
            datatype = acl.mdl.get_output_data_type(self.model_desc, i)
            self._output_info.append({"shape": tuple(dims[0]["dims"]),
                                      "type": datatype})

    def _gen_output_dataset(self, size):
        print("[Model] create model output dataset:")
        dataset = acl.mdl.create_dataset()
        for i in range(size):
            temp_buffer_size = acl.mdl.\
                get_output_size_by_index(self.model_desc, i)
            temp_buffer, ret = acl.rt.malloc(temp_buffer_size,
                                             constants.ACL_MEM_MALLOC_NORMAL_ONLY)
            acl_model.check_ret("acl.rt.malloc", ret)
            
            dataset_buffer = acl.create_data_buffer(temp_buffer,
                                                    temp_buffer_size)
                                                    
            _, ret = acl.mdl.add_dataset_buffer(dataset, dataset_buffer)
            if ret:
                acl.rt.free(temp_buffer)
                acl.destroy_data_buffer(dataset)
                acl_model.check_ret("acl.destroy_data_buffer", ret)
        self.output_dataset = dataset
        print("[Model] create model output dataset success")

    def _init_input_buffer(self):
        self._input_num = acl.mdl.get_num_inputs(self.model_desc)
        self._input_buffer = []
        for i in range(self._input_num):
            item = {"addr":None, "size":0}
            self._input_buffer.append(item)

    def _gen_input_dataset(self, input_list):
        ret = constants.SUCCESS
        
        if len(input_list) != self._input_num:
            print("Current input data num %d unequal to"
                  " model input num % d" % (len(input_list), self._input_num))
            return constants.FAILED

        self.input_dataset = acl.mdl.create_dataset()
        for i in range(self._input_num):
            item = input_list[i]
            data, size = self._parse_input_data(item, i)            
            if (data is None) or (size == 0):
            
                ret = constants.FAILED
                print("The % d input is invalid" % (i))
                break
            dataset_buffer = acl.create_data_buffer(data, size)
            
            _, ret = acl.mdl.add_dataset_buffer(self.input_dataset,
                                                dataset_buffer)
            if ret:
                print("Add input dataset buffer failed")
                acl.destroy_data_buffer(self.input_dataset)
                ret = constants.FAILED
                break
        if ret == constants.FAILED:
            self._release_dataset(self.input_dataset)

        return ret

    def _parse_input_data(self, inputs, index):
        data = None
        size = 0
        if isinstance(inputs, np.ndarray):
        
            ptr = acl.util.numpy_to_ptr(inputs)
            size = inputs.size * inputs.itemsize
            data = self._copy_input_to_device(ptr, size, index)
            if data is None:
                size = 0
                print("Copy input to device failed")
        elif (isinstance(inputs, dict) and
              inputs in ('data') and inputs in ('size')):
            size = inputs['size']
            data = inputs['data']
        else:
            print("Unsupport input")

        return data, size

    def _copy_input_to_device(self, input_ptr, size, index):
        buffer_item = self._input_buffer[index]
        data = None
        if buffer_item['addr'] is None:
            data = acl_model.copy_data_device_to_device(input_ptr, size)
            if data is None:
                print("Malloc memory and copy model % dth "
                      "input to device failed" % (index))
                return None
            buffer_item['addr'] = data
            buffer_item['size'] = size
        elif size == buffer_item['size']:
            ret = acl.rt.memcpy(buffer_item['addr'], size,
                                input_ptr, size,
                                constants.ACL_MEMCPY_DEVICE_TO_DEVICE)
            if ret != constants.ACL_ERROR_NONE:
                print("Copy model % dth input to device failed" % (index))
                return None
            data = buffer_item['addr']
        else:
            print("The model % dth input size % d is change,"
                  " before is % d" % (index, size, buffer_item['size']))
            return None

        return data

    def execute(self, input_list):
        """
        input list
        """
        ret = self._gen_input_dataset(input_list)
        if ret == constants.FAILED:
            print("Gen model input dataset failed")
            return None
        ret = acl.mdl.execute(self.model_id,
                              self.input_dataset,
                              self.output_dataset)
        if ret != constants.ACL_ERROR_NONE:
            print("Execute model failed for acl.mdl.execute error ", ret)
            return None
        self._release_dataset(self.input_dataset)
        self.input_dataset = None
        return self._output_dataset_to_numpy()
    
    def _get_datatype(self, datatype):
        outdatatype = np.float32
        if datatype == constants.ACL_FLOAT:
            return np.float32
        elif datatype == constants.ACL_INT32:
            return np.int32
        elif datatype == constants.ACL_UINT32:
            return np.uint32
            
        else:
            print("unsurpport datatype ", datatype)            
            return np.float32        
        
    def _output_dataset_to_numpy(self):        
        dataset = []        
        num = acl.mdl.get_dataset_num_buffers(self.output_dataset)
        #loop through each output
        for i in range(num):
            
            buffers = acl.mdl.get_dataset_buffer(self.output_dataset, i)
            data = acl.get_data_buffer_addr(buffers)
            size = acl.get_data_buffer_size(buffers) 
            outdatatype = self._get_datatype(self._output_info[i]["type"])

            
            narray = np.zeros(self._output_info[i]["shape"], dtype=outdatatype)                        
            narray_ptr = acl.util.numpy_to_ptr(narray)  
            ret = acl.rt.memcpy(narray_ptr, narray.size * narray.itemsize, 
                                data, size, constants.ACL_MEMCPY_DEVICE_TO_DEVICE)
            if ret != constants.ACL_ERROR_NONE:
                print("Memcpy inference output to local failed")
                return None
            dataset.append(narray)
        return dataset 

    def _unpack_bytes_array(self, byte_array, shape, datatype):
        tag = ""
        np_type = None
        if datatype == constants.ACL_FLOAT:
            tag = 'f'
            np_type = np.float32
        elif datatype == constants.ACL_INT32:
            tag = 'i'
            np_type = np.int32
        elif datatype == constants.ACL_UINT32:
            tag = 'I'
            np_type = np.uint32
        else:
            print("unsurpport datatype ", datatype)
            return
        size = byte_array.size / 4
        unpack_tag = str(int(size)) + tag
        st = struct.unpack(unpack_tag, bytearray(byte_array))
        return np.array(st).astype(np_type).reshape(shape)

    def _release_dataset(self, dataset):
        if not dataset:
            return
        num = acl.mdl.get_dataset_num_buffers(dataset)
        for i in range(num):
            data_buf = acl.mdl.get_dataset_buffer(dataset, i)
            if data_buf:
                ret = acl.destroy_data_buffer(data_buf)
                if ret != constants.ACL_ERROR_NONE:
                    print("Destroy data buffer error ", ret)
        ret = acl.mdl.destroy_dataset(dataset)
        if ret != constants.ACL_ERROR_NONE:
            print("Destroy data buffer error ", ret)

