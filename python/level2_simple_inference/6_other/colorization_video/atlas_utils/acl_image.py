"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
"""
import numpy as np
from PIL import Image
import acl
import utils
import acl_logger
from constants import *


class AclImage(object):
    """
    Processing images
    """
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
            acl_logger.log_error("Create instance failed for unknow image data type")

    def _instance_by_image_file(self, image_path, width, height):
        #Judging file format according to file suffix
        self._encode_format = _get_image_format_by_suffix(image_path)
        if self._encode_format == ENCODE_FORMAT_UNKNOW: 
            acl_logger.log_error("Load image %s failed" % (image_path))
            self._load_ok = False
            return

        self._data = np.fromfile(image_path, dtype=np.byte)
        self._type = IMAGE_DATA_NUMPY
        self.size = self._data.itemsize * self._data.size
        #If it is JPEG or PNG, get the size of the image by pillow
        if ((self._encode_format == ENCODE_FORMAT_JPEG) or 
            (self._encode_format == ENCODE_FORMAT_PNG)):
            image = Image.open(image_path)
            self.width, self.height = image.size
        else:
            #If the picture is YUV, ask the caller to enter the width and height
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
            acl_logger.log_error("Unsupport image format: %s " % suffix)
            image_format = ENCODE_FORMAT_UNKNOW
        return image_format

    def is_loaded(self):
        """
        return self._load_ok
        """
        return self._load_ok

    def _instance_by_buffer(self, image_buffer, width, height, size):
        self.width = width
        self.height = height
        self.size = size
        self._data = image_buffer
        self._type = IMAGE_DATA_BUFFER

    def nparray(self):
        """
        Judging type
        """
        if self._type == IMAGE_DATA_NUMPY:
            return self._data
        else: 
            return acl.util.ptr_to_numpy(self._data, (self.size, ), NPY_BYTE)

    def data(self):      
        """
        return self._data
        """
        if self._type == IMAGE_DATA_NUMPY:
            return acl.util.numpy_to_ptr(self._data)
        else:
            return self._data

    def copy_to_dvpp(self, run_mode):
        """
        Copy image to dvpp
        """
        device_ptr = utils.copy_data_to_dvpp(self.data(), self.size, run_mode)
        if device_ptr is None:
            print("Copy image to dvpp failed")
            return None
        return AclImage(device_ptr, self.width, self.height, self.size, MEMORY_DVPP)

    def is_local(self, run_model):
        '''
        Determine whether the image memory is directly accessible memory
        '''
        #Atlas 200dk device memory, dvpp memory, numpy array images are directly accessible
        if run_model == ACL_DEVICE:
            return True
        #Only host memory or numpy array images can be accessed directly in atlas 300 environment
        elif ((run_model == ACL_HOST) and 
              ((self._memory_type == MEMORY_HOST) or 
               (self._memory_type == MEMORY_NORMAL))):
            return True
        else:
            return False

    def destroy(self):
        """
        Determine the storage type and release
        """
        if (self._data is None) or (self.size == 0):
            acl_logger.log_error("Release image abnormaly, data is None")
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


        
      
        
        