import os
import numpy as np
from PIL import Image

import acl
from atlas_utils.utils import *
from atlas_utils.acl_logger import log_error, log_info
from atlas_utils.constants import *

class AclImage(object):
    _run_mode, _ = acl.rt.get_run_mode()
    def __init__(self, image, width=0, height=0,
                 size=0, memory_type=MEMORY_NORMAL):    
        self._data = None
        self._np_array = None
        self._memory_type = memory_type   
        self.width = 0
        self.height = 0
        self.size = 0
        self._encode_format = ENCODE_FORMAT_UNKNOW
        self._load_ok = True

        if isinstance(image, str):
            self._instance_by_image_file(image, width, height)
        elif isinstance(image, int):
            self._instance_by_buffer(image, width, height, size)        
        else:
            log_error("Create instance failed for unknow image data type")

    def _instance_by_image_file(self, image_path, width, height):
        #根据文件后缀判断文件格式
        self._encode_format = self._get_image_format_by_suffix(image_path)
        if self._encode_format == ENCODE_FORMAT_UNKNOW: 
            log_error("Load image %s failed"%(image_path))
            self._load_ok = False
            return

        self._data = np.fromfile(image_path, dtype=np.byte)
        self._type = IMAGE_DATA_NUMPY
        self.size = self._data.itemsize * self._data.size
        #如果是jpeg或者png,通过pillow获取图片尺寸
        if ((self._encode_format == ENCODE_FORMAT_JPEG) or 
            (self._encode_format == ENCODE_FORMAT_PNG)):
            image = Image.open(image_path)
            self.width, self.height = image.size
        else:
            #如果时yuv图片,要求调用者输入宽高
            self.width = width
            self.height = height


    def _get_image_format_by_suffix(self, filename): 
        suffix = os.path.splitext(filename)[-1].strip().lower()       
        if (suffix == ".jpg") or (suffix == ".jpeg"):
            image_format = ENCODE_FORMAT_JPEG
        elif suffix == ".png":
            image_format = ENCODE_FORMAT_PNG
        elif suffix == ".yuv":
            image_format = ENCODE_FORMAT_YUV420_SP
        else:
            log_error("Unsupport image format: ", suffix)
            image_format = ENCODE_FORMAT_UNKNOW
        return image_format

    def is_loaded(self):
        return self._load_ok

    def _instance_by_buffer(self, image_buffer, width, height, size):
        self.width = width
        self.height = height
        self.size = size
        self._data = image_buffer
        self._type = IMAGE_DATA_BUFFER

    def nparray(self):
        if self._type == IMAGE_DATA_NUMPY:
            return self._data
        else: 
            return acl.util.ptr_to_numpy(self._data, (self.size, ), NPY_BYTE)

    def data(self):
        if self._type == IMAGE_DATA_NUMPY:
            return acl.util.numpy_to_ptr(self._data)
        else:
            return self._data

    def copy_to_dvpp(self):
        device_ptr = copy_data_to_dvpp(self.data(), self.size, self._run_mode)
        if device_ptr is None:
            log_error("Copy image to dvpp failed")
            return None
        return AclImage(device_ptr, self.width, self.height, self.size, MEMORY_DVPP)

    def is_local(run_model):
        '''
        判断图片内存是否为可以直接访问的内存
        '''
        #atlas200dk的device内存,dvpp内存,numpy数组图片都是可以直接访问的
        if run_model == ACL_DEVICE:
            return True
        #atlas300环境只有host内存或numpy数组图片才能直接访问
        elif ((run_model == ACL_HOST) and 
              ((self._memory_type == MEMORY_HOST) or 
               (self._memory_type == MEMORY_NORMAL))):
               return True
        else:
            return False

    def destroy(self):
        if (self._data is None) or (self.size == 0):
            log_error("Release image abnormaly, data is None")
            return

        if self._memory_type == MEMORY_DEVICE: 
            acl.rt.free(self._data)  
        elif self._memory_type == MEMORY_HOST:
            acl.rt.free_host(self._data)  
        elif self._memory_type == MEMORY_DVPP:
            acl.media.dvpp_free(self._data)

        self._data = None
        self.size = 0

    def __del__(self):
        self.destroy()


        
      
        
        