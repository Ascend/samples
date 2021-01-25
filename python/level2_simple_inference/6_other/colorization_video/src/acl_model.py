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
import utils
from constants import *


class Model(object):
    """
    Perform model loading,
    Initialization, reasoning process
    """
    def __init__(self, acl_resource, model_path):
        print("load model ", model_path)
        self.acl_resource = acl_resource
        self._run_mode = acl_resource.run_mode
        self.model_path = model_path    # string
        self.model_id = None            # pointer
        self.input_dataset = None
        self.output_dataset = None
        self._output_info = []
        self.model_desc = None          # pointer when using
        self._init_resource()
        self._is_released = False
        acl_resource.register_resource(self)
    
    def __del__(self):
        if self._is_released:
            return
        self.acl_resource.unregister_resource(self)

        print("Model start release...")
        self._release_dataset(self.input_dataset)
        self._release_dataset(self.output_dataset)
        if self.model_id:
            ret = acl.mdl.unload(self.model_id)
            if ret != ACL_ERROR_NONE:
                print("acl.mdl.unload error:", ret)

        if self.model_desc:
            ret = acl.mdl.destroy_desc(self.model_desc)
            if ret != ACL_ERROR_NONE:
                print("acl.mdl.destroy_desc error:", ret)
        self._is_released = True
        print("Model release source success")

    def _init_resource(self):
        print("Init model resource")
        #load model
        self.model_id, ret = acl.mdl.load_from_file(self.model_path)
        utils.check_ret("acl.mdl.load_from_file", ret)
        self.model_desc = acl.mdl.create_desc()
        ret = acl.mdl.get_desc(self.model_desc, self.model_id)
        utils.check_ret("acl.mdl.get_desc", ret)
        #model output number
        output_size = acl.mdl.get_num_outputs(self.model_desc)
        #Create model output dataset structure
        self._gen_output_dataset(output_size)
        print("[Model] class Model init resource stage success")
        #Get the type and shape of each output of the model
        self._get_output_desc(output_size)
        self._init_input_buffer()

        return SUCCESS

    def _get_output_desc(self, output_size):
        for i in range(output_size):
            #Get the shape and data type of each output
            dims = acl.mdl.get_output_dims(self.model_desc, i)
            datatype = acl.mdl.get_output_data_type(self.model_desc, i)
            self._output_info.append({"shape": tuple(dims[0]["dims"]),
                                      "type": datatype})

    def _gen_output_dataset(self, size):
        print("[Model] create model output dataset:")
        dataset = acl.mdl.create_dataset()
        for i in range(size):
            #Request device memory for each output
            temp_buffer_size = acl.mdl.\
                get_output_size_by_index(self.model_desc, i)
            temp_buffer, ret = acl.rt.malloc(temp_buffer_size,
                                             ACL_MEM_MALLOC_NORMAL_ONLY)
            utils.check_ret("acl.rt.malloc", ret)
            #Create the output data buffer structure
            # fill the requested memory into the data buffer
            dataset_buffer = acl.create_data_buffer(temp_buffer,
                                                    temp_buffer_size)

            #Add the data buffer to the output dataset
            _, ret = acl.mdl.add_dataset_buffer(dataset, dataset_buffer)
            if ret:
                acl.rt.free(temp_buffer)
                acl.destroy_data_buffer(dataset)
                utils.check_ret("acl.destroy_data_buffer", ret)
        self.output_dataset = dataset
        print("[Model] create model output dataset success")

    def _init_input_buffer(self):
        # Create a table to record the input data memory requested by the user.
        # when the user inputs the numpy array, the numpy data needs to be copied to the device side,
        # and the memory is required to be applied
        self._input_num = acl.mdl.get_num_inputs(self.model_desc)
        self._input_buffer = []
        for i in range(self._input_num):
            #Initially, all input units did not request memory
            item = {"addr":None, "size":0}
            self._input_buffer.append(item)

    def _gen_input_dataset(self, input_list):
        # organizing dataset structure of input data
        ret = SUCCESS
        #If the number of input data does not match the model requirements, return
        if len(input_list) != self._input_num:
            print("Current input data num %d unequal to"
                  " model input num %d " % (len(input_list), self._input_num))
            return FAILED

        self.input_dataset = acl.mdl.create_dataset()
        for i in range(self._input_num):
            item = input_list[i]
            #Parsing input, supports input of AclImage type, ACL pointer and numpy array
            data, size = self._parse_input_data(item, i)            
            if (data is None) or (size == 0):
                #The remaining data will not be parsed when the data fails to be parsed
                ret = FAILED
                print("The %d input is invalid" % (i))
                break
            #Create the input dataset buffer structure and fill in the input data
            dataset_buffer = acl.create_data_buffer(data, size)
            #Add dataset buffer to the dataset
            _, ret = acl.mdl.add_dataset_buffer(self.input_dataset,
                                                dataset_buffer)
            if ret:
                print("Add input dataset buffer failed")
                acl.destroy_data_buffer(self.input_dataset)
                ret = FAILED
                break
        if ret == FAILED:
            #Release dataset on failure
            self._release_dataset(self.input_dataset)

        return ret

    def _parse_input_data(self, input, index):
        data = None
        size = 0
        if isinstance(input, np.ndarray):
            #If the input is numpy data, the device memory is requested for the data,
            # and the data is copied to the device side
            #The applied memory can be reused, and it is not necessary to apply for memory every time
            ptr = acl.util.numpy_to_ptr(input)
            size = input.size * input.itemsize
            data = self._copy_input_to_device(ptr, size, index)
            if data is None:
                size = 0
                print("Copy input to device failed")
        elif (isinstance(input, dict) and 'data' in input and  'size' in input):
            size = input['size']
            data = input['data']
        else:
            print("Unsupport input")

        return data, size

    def _copy_input_to_device(self, input_ptr, size, index):
        #Request device side memory for input and copy data to this memory
        buffer_item = self._input_buffer[index]
        data = None
        #According to the subscript position of the data in the model input,
        # check whether the input has applied for memory
        if buffer_item['addr'] is None:
            #If not, request memory, copy data, and record memory for next reuse
            data = utils.copy_data_device_to_device(input_ptr, size)
            if data is None:
                print("Malloc memory and copy model %dth "
                      "input to device failed" % (index))
                return None
            buffer_item['addr'] = data
            buffer_item['size'] = size
        elif size == buffer_item['size']:
            #If memory has been applied for this input,
            # and the memory size is consistent with the current input data size,
            # the data will be copied to the memory for this execute
            ret = acl.rt.memcpy(buffer_item['addr'], size,
                                input_ptr, size,
                                ACL_MEMCPY_DEVICE_TO_DEVICE)
            if ret != ACL_ERROR_NONE:
                print("Copy model %dth input to device failed" % (index))
                return None
            data = buffer_item['addr']
        else:
            #If the memory has been applied for the input,
            # but the memory size is inconsistent with the current input data size,
            # it is considered as abnormal at this time,
            # because each input size of the model is fixed and immutable
            print("The model %dth input size %d is change,"
                  " before is %d " % (index, size, buffer_item['size']))
            return None

        return data

    def execute(self, input_list):
        """
        Execute reasoning process
        """
        ret = self._gen_input_dataset(input_list)
        if ret == FAILED:
            print("Gen model input dataset failed")
            return None
        #execute
        start = datetime.datetime.now()
        ret = acl.mdl.execute(self.model_id,
                              self.input_dataset,
                              self.output_dataset)
        if ret != ACL_ERROR_NONE:
            print("Execute model failed for acl.mdl.execute error ", ret)
            return None
        end = datetime.datetime.now()        
        print("acl.mdl.execute exhaust ", end - start)
        #Release the input dataset object instance, and the input data memory will not be released
        self._release_dataset(self.input_dataset)
        self.input_dataset = None
        #The binary data stream is decoded into numpy array.
        # The shape and type of the array are consistent with the model output specification
        return self._output_dataset_to_numpy()

    def _output_dataset_to_numpy(self):
        dataset = []
        num = acl.mdl.get_dataset_num_buffers(self.output_dataset)
        #loop through output
        for i in range(num):
            #Get memory address of outputData from output buffer
            buffer = acl.mdl.get_dataset_buffer(self.output_dataset, i)
            data = acl.get_data_buffer_addr(buffer)
            size = acl.get_data_buffer_size(buffer)
            #Create a numpy array to copy the output memory data
            narray = np.zeros(size, dtype=np.byte)
            narray_ptr = acl.util.numpy_to_ptr(narray)
            ret = acl.rt.memcpy(narray_ptr, narray.size * narray.itemsize, 
                                data, size, ACL_MEMCPY_DEVICE_TO_DEVICE)
            if ret != ACL_ERROR_NONE:
                print("Memcpy inference output to local failed")
                return None
            #According to the shape and data type of the model output,
            # the memory data is decoded into numpy array
            output_nparray = self._unpack_bytes_array(
                narray, self._output_info[i]["shape"],
                self._output_info[i]["type"])
            dataset.append(output_nparray)

        return dataset  

    def _unpack_bytes_array(self, byte_array, shape, datatype):
        tag = ""
        np_type = None
        #Get the numpy array type and decoding sign corresponding to the output data type
        if datatype == ACL_FLOAT:
            tag = 'f'
            np_type = np.float32
        elif datatype == ACL_INT32:
            tag = 'i'
            np_type = np.int32
        elif datatype == ACL_UINT32:
            tag = 'I'
            np_type = np.uint32
        else:
            print("unsurpport datatype ", datatype)
            return
        #get the actual data length by output data bytes length dividing the data type bytes length
        size = byte_array.size / 4
        #Organize decode tag string: data length + type
        unpack_tag = str(int(size)) + tag
        #Decoding data
        st = struct.unpack(unpack_tag, bytearray(byte_array))
        #Organize the decoded data into numpy array, and set shape and type
        return np.array(st).astype(np_type).reshape(shape)

    def _release_dataset(self, dataset):
        if not dataset:
            return
        num = acl.mdl.get_dataset_num_buffers(dataset)
        for i in range(num):
            data_buf = acl.mdl.get_dataset_buffer(dataset, i)
            if data_buf:
                ret = acl.destroy_data_buffer(data_buf)
                if ret != ACL_ERROR_NONE:
                    print("Destroy data buffer error ", ret)
        ret = acl.mdl.destroy_dataset(dataset)
        if ret != ACL_ERROR_NONE:
            print("Destroy data buffer error ", ret)
