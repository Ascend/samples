"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
"""
import acl
import struct
import numpy as np
import datetime
from utils import check_ret, copy_data_device_to_device
from constants import ACL_MEM_MALLOC_NORMAL_ONLY, SUCCESS
from constants import ACL_ERROR_NONE, FAILED, ACL_MEMCPY_DEVICE_TO_DEVICE
from constants import ACL_FLOAT, ACL_INT32, ACL_UINT32

class Model(object):
    """Model"""
    def __init__(self, acl_resource, model_path):
        print("load model ", model_path)
        self.acl_resource = acl_resource
        self._run_mode = acl_resource.run_mode
        self.model_path = model_path    # string
        self.model_id = None            # pointer
        self.input_dataset = None
        self._input_num = 0
        self.output_dataset = None
        self._input_buffer = []
        self._output_info = []
        self.model_desc = None          # pointer when using
        self._init_resource()
        self._is_released = False
        acl_resource.register_resource(self)
        return 
    
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
        return

    def _init_resource(self):
        print("Init model resource")
        #load model file
        self.model_id, ret = acl.mdl.load_from_file(self.model_path)
        check_ret("acl.mdl.load_from_file", ret)
        self.model_desc = acl.mdl.create_desc()
        ret = acl.mdl.get_desc(self.model_desc, self.model_id)
        check_ret("acl.mdl.get_desc", ret)

        #get model output
        output_size = acl.mdl.get_num_outputs(self.model_desc)

        #create model output dataset
        self._gen_output_dataset(output_size)
        print("[Model] class Model init resource stage success")

        #get model output description
        self._get_output_desc(output_size)

        #create input buffer 
        self._init_input_buffer()
        return SUCCESS

    def _get_output_desc(self, output_size):
        for i in range(output_size):
            #get output dims and datatype
            dims = acl.mdl.get_output_dims(self.model_desc, i)
            datatype = acl.mdl.get_output_data_type(self.model_desc, i)
            self._output_info.append({"shape": tuple(dims[0]["dims"]),
                                      "type": datatype})
        return 

    def _gen_output_dataset(self, size):
        print("[Model] create model output dataset:")
        dataset = acl.mdl.create_dataset()
        for i in range(size):
            # create output memory 
            temp_buffer_size = acl.mdl.\
                get_output_size_by_index(self.model_desc, i)
            temp_buffer, ret = acl.rt.malloc(temp_buffer_size,
                                             ACL_MEM_MALLOC_NORMAL_ONLY)
            check_ret("acl.rt.malloc", ret)
            #create output data buffer
            dataset_buffer = acl.create_data_buffer(temp_buffer,
                                                    temp_buffer_size)
            #add data buffer to output dataset
            _, ret = acl.mdl.add_dataset_buffer(dataset, dataset_buffer)
            if ret:
                #free resource 
                acl.rt.free(temp_buffer)
                acl.destroy_data_buffer(dataset)
                check_ret("acl.destroy_data_buffer", ret)
        self.output_dataset = dataset
        print("[Model] create model output dataset success")
        return

    def _init_input_buffer(self):
        self._input_num = acl.mdl.get_num_inputs(self.model_desc)
        self._input_buffer = []
        for i in range(self._input_num):
            item = {"addr":None, "size":0}
            self._input_buffer.append(item)
        return

    def _gen_input_dataset(self, input_list):
        ret = SUCCESS
        if len(input_list) != self._input_num:
            print("Current input data num %d unequal to"
                  " model input num %d" % (len(input_list), self._input_num))
            return FAILED

        self.input_dataset = acl.mdl.create_dataset()
        for i in range(self._input_num):
            item = input_list[i]
            data, size = self._parse_input_data(item, i)            
            if (data is None) or (size == 0):
                ret = FAILED
                print("The %d input is invalid" % (i))
                break

            dataset_buffer = acl.create_data_buffer(data, size)
            _, ret = acl.mdl.add_dataset_buffer(self.input_dataset,
                                                dataset_buffer)
            if ret:
                print("Add input dataset buffer failed")
                acl.destroy_data_buffer(self.input_dataset)
                ret = FAILED
                break
        if ret == FAILED:
            self._release_dataset(self.input_dataset)
        return ret

    def _parse_input_data(self, inputitem, index):
        data = None
        size = 0
        if isinstance(inputitem, np.ndarray):
            ptr = acl.util.numpy_to_ptr(inputitem)
            size = inputitem.size * inputitem.itemsize
            data = self._copy_input_to_device(ptr, size, index)
            if data is None:
                size = 0
                print("Copy input to device failed")

        elif (isinstance(inputitem, dict) and ('data' in inputitem) and ('size' in inputitem)):
            size = inputitem['size']
            data = inputitem['data']
        else:
            print("Unsupport input")

        return data, size

    def _copy_input_to_device(self, input_ptr, size, index):
        buffer_item = self._input_buffer[index]
        data = None
        if buffer_item['addr'] is None:
            data = copy_data_device_to_device(input_ptr, size)
            if data is None:
                print("Malloc memory and copy model %dth "
                      "input to device failed" % (index))
                return None
            buffer_item['addr'] = data
            buffer_item['size'] = size
        elif size == buffer_item['size']:
            ret = acl.rt.memcpy(buffer_item['addr'], size,
                                input_ptr, size,
                                ACL_MEMCPY_DEVICE_TO_DEVICE)
            if ret != ACL_ERROR_NONE:
                print("Copy model %dth input to device failed" % (index))
                return None
            data = buffer_item['addr']
        else:
            print("The model %dth input size %d is change,"
                  " before is %d" % (index, size, buffer_item['size']))
            return None

        return data

    def execute(self, input_list):
        """execute"""
        ret = self._gen_input_dataset(input_list)
        if ret == FAILED:
            print("Gen model input dataset failed")
            return None
            
        start = datetime.datetime.now()
        ret = acl.mdl.execute(self.model_id,
                              self.input_dataset,
                              self.output_dataset)
        if ret != ACL_ERROR_NONE:
            print("Execute model failed for acl.mdl.execute error ", ret)
            return None
        end = datetime.datetime.now()        
        print("acl.mdl.execute exhaust ", end - start)
        self._release_dataset(self.input_dataset)
        self.input_dataset = None
        return self._output_dataset_to_numpy()

    def _output_dataset_to_numpy(self):
        dataset = []
        num = acl.mdl.get_dataset_num_buffers(self.output_dataset)
        for i in range(num):
            outbuffer = acl.mdl.get_dataset_buffer(self.output_dataset, i)
            data = acl.get_data_buffer_addr(outbuffer)
            size = acl.get_data_buffer_size(outbuffer)
            narray = np.zeros(size, dtype=np.byte)
            narray_ptr = acl.util.numpy_to_ptr(narray)
            ret = acl.rt.memcpy(narray_ptr, narray.size * narray.itemsize, 
                                data, size, ACL_MEMCPY_DEVICE_TO_DEVICE)
            if ret != ACL_ERROR_NONE:
                print("Memcpy inference output to local failed")
                return None
            output_nparray = self._unpack_bytes_array(
                narray, self._output_info[i]["shape"],
                self._output_info[i]["type"])
            dataset.append(output_nparray)

        return dataset  

    def _unpack_bytes_array(self, byte_array, shape, datatype):
        tag = ""
        np_type = None
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
                if ret != ACL_ERROR_NONE:
                    print("Destroy data buffer error ", ret)
        ret = acl.mdl.destroy_dataset(dataset)
        if ret != ACL_ERROR_NONE:
            print("Destroy data buffer error ", ret)

