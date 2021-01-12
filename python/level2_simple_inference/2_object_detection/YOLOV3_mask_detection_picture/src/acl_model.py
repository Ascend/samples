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
from utils import *


class Model(object):
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
        #加载模型文件
        self.model_id, ret = acl.mdl.load_from_file(self.model_path)
        check_ret("acl.mdl.load_from_file", ret)
        self.model_desc = acl.mdl.create_desc()
        ret = acl.mdl.get_desc(self.model_desc, self.model_id)
        check_ret("acl.mdl.get_desc", ret)
        #获取模型输出个数
        output_size = acl.mdl.get_num_outputs(self.model_desc)
        #创建模型输出dataset结构
        self._gen_output_dataset(output_size)
        print("[Model] class Model init resource stage success")
        #获取模型每个输出的类型和shape
        self._get_output_desc(output_size)
        #创建记录输入数据内存地址的表,如果需要为输入申请内存,则记录到该表以便复用
        self._init_input_buffer()

        return SUCCESS

    def _get_output_desc(self, output_size):
        for i in range(output_size):
            #获取每个输出的shape和数据类型
            dims = acl.mdl.get_output_dims(self.model_desc, i)
            datatype = acl.mdl.get_output_data_type(self.model_desc, i)
            self._output_info.append({"shape": tuple(dims[0]["dims"]),
                                      "type": datatype})

    def _gen_output_dataset(self, size):
        print("[Model] create model output dataset:")
        dataset = acl.mdl.create_dataset()
        for i in range(size):
            #为每个输出申请device内存
            temp_buffer_size = acl.mdl.\
                get_output_size_by_index(self.model_desc, i)
            temp_buffer, ret = acl.rt.malloc(temp_buffer_size,
                                             ACL_MEM_MALLOC_NORMAL_ONLY)
            check_ret("acl.rt.malloc", ret)
            #创建输出的data buffer结构,将申请的内存填入data buffer
            dataset_buffer = acl.create_data_buffer(temp_buffer,
                                                    temp_buffer_size)
            #将data buffer加入输出dataset
            _, ret = acl.mdl.add_dataset_buffer(dataset, dataset_buffer)
            if ret:
                #如果失败,则释放资源
                acl.rt.free(temp_buffer)
                acl.destroy_data_buffer(dataset)
                check_ret("acl.destroy_data_buffer", ret)
        self.output_dataset = dataset
        print("[Model] create model output dataset success")

    def _init_input_buffer(self):
        #创建一个表，记录为用户申请的输入数据内存,当前只有用户输入numpy数组时,
        #需要将numpy数据拷贝到device侧,需要申请内存
        self._input_num = acl.mdl.get_num_inputs(self.model_desc)
        self._input_buffer = []
        for i in range(self._input_num):
            #初始时所有输入单元都没有申请内存
            item = {"addr":None, "size":0}
            self._input_buffer.append(item)

    def _gen_input_dataset(self, input_list):
        #组织输入数据的dataset结构
        ret = SUCCESS
        #如果输入的数据个数与模型要求的不匹配,则直接返回
        if len(input_list) != self._input_num:
            print("Current input data num %d unequal to"
                  " model input num %d"%(len(input_list), self._input_num))
            return FAILED

        self.input_dataset = acl.mdl.create_dataset()
        for i in range(self._input_num):
            item = input_list[i]
            #解析输入,当前支持输入AclImage类型、Acl指针和numpy数组
            data, size = self._parse_input_data(item, i)            
            if (data is None) or (size == 0):
                #解析数据失败时不再解析剩余数据
                ret = FAILED
                print("The %d input is invalid"%(i))
                break
            #创建输入dataset buffer结构,填入输入的数据
            dataset_buffer = acl.create_data_buffer(data, size)
            #将dataset buffer加入dataset
            _, ret = acl.mdl.add_dataset_buffer(self.input_dataset,
                                                dataset_buffer)
            if ret:
                print("Add input dataset buffer failed")
                acl.destroy_data_buffer(self.input_dataset)
                ret = FAILED
                break
        if ret == FAILED:
            #失败时释放dataset
            self._release_dataset(self.input_dataset)

        return ret

    def _parse_input_data(self, input, index):
        data = None
        size = 0
        if isinstance(input, np.ndarray):
            #如果输入为numpy数据,则为数据申请device内存,并将数据拷贝到device侧
            #申请的内存是可以复用的,不需要每次都申请内存
            ptr = acl.util.numpy_to_ptr(input)
            size = input.size * input.itemsize
            data = self._copy_input_to_device(ptr, size, index)
            if data == None:
                size = 0
                print("Copy input to device failed")
        #如果直接输入内存指针,要求组织为{"data":, "size":}的dict,并且默认内存为device侧
        elif (isinstance(input, dict) and
              input.has_key('data') and input.has_key('size')):
            size = input['size']
            data = input['data']
        else:
            print("Unsupport input")

        return data, size

    def _copy_input_to_device(self, input_ptr, size, index):
        #为输入申请device侧内存,并将数据拷贝到该内存
        buffer_item = self._input_buffer[index]
        data = None
        #根据数据在模型输入中的下标位置,查看该输入是否已经申请过内存
        if buffer_item['addr'] is None:
            #如果没有,这申请内存,拷贝数据,并记录内存以供下次复用
            data = copy_data_device_to_device(input_ptr, size)
            if data is None:
                print("Malloc memory and copy model %dth "
                      "input to device failed"%(index))
                return None
            buffer_item['addr'] = data
            buffer_item['size'] = size
        elif size == buffer_item['size']:
            #如果曾经为该输入申请过内存,并且内存大小与当前输入数据大小一致,
            #则将数据拷贝到该内存以供本次推理
            ret = acl.rt.memcpy(buffer_item['addr'], size,
                                input_ptr, size,
                                ACL_MEMCPY_DEVICE_TO_DEVICE)
            if ret != ACL_ERROR_NONE:
                print("Copy model %dth input to device failed"%(index))
                return None
            data = buffer_item['addr']
        else:
            #如果曾经为该输入申请过内存,但是内存大小与当前输入数据大小不一致,则认为
            #时异常.因为模型的每个输入大小时固定的，不过可变的
            print("The model %dth input size %d is change,"
                  " before is %d"%(index, size, buffer_item['size']))
            return None

        return data

    def execute(self, input_list):
        #创建离线模型推理需要的dataset对象实例
        ret = self._gen_input_dataset(input_list)
        if ret == FAILED:
            print("Gen model input dataset failed")
            return None
        #调用离线模型的execute推理数据
        start = datetime.datetime.now()
        ret = acl.mdl.execute(self.model_id,
                              self.input_dataset,
                              self.output_dataset)
        if ret != ACL_ERROR_NONE:
            print("Execute model failed for acl.mdl.execute error ", ret)
            return None
        end = datetime.datetime.now()        
        print("acl.mdl.execute exhaust ", end - start)
        #释放输入dataset对象实例,不会释放输入数据内存
        self._release_dataset(self.input_dataset)
        self.input_dataset = None
        #将推理输出的二进制数据流解码为numpy数组，数组的shape和类型与模型输出规格一致
        return self._output_dataset_to_numpy()

    def _output_dataset_to_numpy(self):
        dataset = []
        num = acl.mdl.get_dataset_num_buffers(self.output_dataset)
        #遍历每个输出
        for i in range(num):
            #从输出buffer中获取输出数据内存地址
            buffer = acl.mdl.get_dataset_buffer(self.output_dataset, i)
            data = acl.get_data_buffer_addr(buffer)
            size = acl.get_data_buffer_size(buffer)
            #创建一个numpy数组用于拷贝输出内存数据
            narray = np.zeros(size, dtype=np.byte)
            narray_ptr = acl.util.numpy_to_ptr(narray)
            ret = acl.rt.memcpy(narray_ptr, narray.size * narray.itemsize, 
                                data, size, ACL_MEMCPY_DEVICE_TO_DEVICE)
            if ret != ACL_ERROR_NONE:
                print("Memcpy inference output to local failed")
                return None
            #根据模型输出的shape和数据类型,将内存数据解码为numpy数组
            output_nparray = self._unpack_bytes_array(
                narray, self._output_info[i]["shape"],
                self._output_info[i]["type"])
            dataset.append(output_nparray)

        return dataset  

    def _unpack_bytes_array(self, byte_array, shape, datatype):
        tag = ""
        np_type = None
        #获取输出数据类型对应的numpy数组类型和解码标记
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
        #输出数据字节数除以数据类型字节长度, 得到实际数据长度
        size = byte_array.size / 4
        #组织解码标签字符串:数据长度+类型
        unpack_tag = str(int(size)) + tag
        #解码数据
        st = struct.unpack(unpack_tag, bytearray(byte_array))
        #将解码后的数据组织为numpy数组,并设置shape和类型
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
