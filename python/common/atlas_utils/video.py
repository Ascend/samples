import ctypes
from ctypes import *
import os
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)

from constants import *
from acl_image import AclImage
from lib.atlasutil_so import libatlas
from resource_list import resource_list
from acl_logger import log_error, log_info

RTSP_TRANSPORT_UDP = 0
RTSP_TRANSPORT_UDP = 1
INVALID_CHANNEL_ID = -1


class ImageDataC(Structure):
    _fields_ = [
        ('format',        c_uint),
        ('width',         c_uint),
        ('height',        c_uint),        
        ('alignWidth',    c_uint),
        ('alignHeight',   c_uint),
        ('size',          c_uint),
        ('data',          POINTER(c_ubyte))
    ]

class VideoInfoC(Structure):
    _fields_ = [
        ('width',         c_uint),
        ('height',        c_uint),        
        ('fps',           c_uint)
    ]

class AclVideo():
    def __init__(self, name):
        self._name = name
        self._channel_id = self._open(name)
        if self._channel_id == INVALID_CHANNEL_ID:
            self._is_opened = False
            log_error("Open video %s failed"%(name))
        else:
            self._is_opened = True
        
        self._width = 0
        self._height = 0
        self._fps = 0
        self._get_param()
   
        self._is_destroyed = False     
        resource_list.register(self)

    def _open(self, name):
        return libatlas.OpenVideo(name.encode())

    def _get_param(self):
        param = VideoInfoC()
        libatlas.GetVideoInfo(self._channel_id, byref(param))
        self._width = param.width
        self._height = param.height
        self._fps = param.fps

    def is_opened(self):
        return self._is_opened

    def width(self):
        return self._width
    
    def height(self):
        return self._height

    def fps(self):
        return self._fps

    def read(self):
        frame = ImageDataC()
        image = None
        ret = libatlas.ReadDecodedFrame(self._channel_id, byref(frame))
        if ret == READ_VIDEO_OK:
            image = AclImage(addressof(frame.data.contents),
                             frame.width, frame.height, 
                             frame.size, MEMORY_DVPP)
        elif ret == VIDEO_DECODE_FINISH:
            log_info("Video has been read finished")
        else: 
            log_error("Read frame error: ", ret)
            
        return ret, image

    def close(self):
        ret = libatlas.CloseVideo(self._channel_id)
        if ret != SUCCESS:
            log_error("Close %s failed"%(self._name))

    def destroy(self):       
        if self._is_destroyed is False:
            self.close()
            self._is_destroyed = True
            resource_list.unregister(self)
            log_info("Video decoder ok")
        

    def __del__(self):        
        self.destroy()

