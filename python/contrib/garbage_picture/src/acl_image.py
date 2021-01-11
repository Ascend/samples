import numpy as np
from PIL import Image

import acl
from utils import *
from constants import *

IMAGE_DATA_NUMPY = 0
IMAGE_DATA_BUFFER = 1

class AclImage():
    def __init__(self, image, width=0, height=0, 
                 size=0, memory_type=MEMORY_NORMAL):    
        print("instance image")
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
            return None

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

        return  AclImage(device_ptr, self.width, self.height, self.size, MEMORY_DEVICE) 

    def destroy(self):
        if self._memory_type == MEMORY_DEVICE:
            acl.rt.free(self._data)  
        elif self._memory_type == MEMORY_HOST:
            acl.rt.free_host(self._data)  
        elif self._memory_type == MEMORY_DVPP:
            acl.rt.free_dvpp(self._data)  

    def __del__(self):
        print("destory image")
        self.destroy()


        
      
        
        