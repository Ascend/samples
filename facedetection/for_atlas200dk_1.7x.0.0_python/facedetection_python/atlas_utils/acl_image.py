import numpy as np
from PIL import Image

import acl
from utils import *
from atlas_utils.constants import *

class AclImage():
    def __init__(self, image, width=0, height=0,
                 size=0, memory_type=MEMORY_NORMAL):    
        self._data = None
        self._np_array = None
        self._memory_type = memory_type   
        self.width = 0
        self.height = 0
        self.channels = 0
        self.size = 0

        if isinstance(image, str):
            self._instance_by_image_file(image)
        elif isinstance(image, int):
            self._instance_by_buffer(image, width, height, size)        
        else:
            print("Create instance failed for unknow image data type")

    def _instance_by_image_file(self, image_path):
        self._data = np.fromfile(image_path, dtype=np.byte)
        self._type = IMAGE_DATA_NUMPY
        self.size = self._data.itemsize * self._data.size
        image = Image.open(image_path)
        self.width, self.height = image.size

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

    def copy_to_device(self, run_mode):
        ptr = acl.util.numpy_to_ptr(self._data)
        device_ptr = None
        if run_mode == ACL_HOST:
            device_ptr = copy_data_host_to_device(ptr, self.size)
        else:
            device_ptr = copy_data_device_to_device(ptr, self.size)

        return AclImage(device_ptr, self.width,
                        self.height, self.size, MEMORY_DEVICE)

    def copy_as_nparray(self):
        if self._type == IMAGE_DATA_BUFFER:
            np_output = np.zeros(self.size, dtype=np.byte)
            np_output_ptr = acl.util.numpy_to_ptr(np_output)
            ret = acl.rt.memcpy(np_output_ptr,
                                np_output.size * np_output.itemsize,
                                self._data, self.size,
                                ACL_MEMCPY_DEVICE_TO_DEVICE)
            if (ret != ACL_ERROR_NONE):
                print("Copy mage to np array failed for memcpy error ", ret)
                return None
            return np_output
        else:
            return self._data.copy()

    def destroy(self):
        if self._memory_type == MEMORY_DEVICE:
            acl.rt.free(self._data)  
        elif self._memory_type == MEMORY_HOST:
            acl.rt.free_host(self._data)  
        elif self._memory_type == MEMORY_DVPP:
            acl.media.dvpp_free(self._data)

    def __del__(self):
        self.destroy()


        
      
        
        