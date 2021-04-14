"""
@Date: 2020-12-09 23:28:01
@LastEditTime: 2020-12-17 23:21:36
@FilePath: /colorization_video_python/atlas_utils/video.py
"""
import ctypes
from ctypes import *
import os
import sys

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)

from constants import *
import acl_image
from lib.atlasutil_so import libatlas
from resource_list import resource_list
import acl_logger

RTSP_TRANSPORT_UDP = 0
RTSP_TRANSPORT_UDP = 1
INVALID_CHANNEL_ID = -1


class ImageDataC(Structure):
    """
    image data message
    """
    _fields_ = [
        ('format', c_uint),
        ('width', c_uint),
        ('height', c_uint),        
        ('alignWidth', c_uint),
        ('alignHeight', c_uint),
        ('size', c_uint),
        ('data', POINTER(c_ubyte))
    ]


class AclVideo(object):
    """
    Encapsulate the camera input as aclvideo
    """
    def __init__(self, name):
        self._name = name
        self._channel_id = self._open(name)
        if self._channel_id == INVALID_CHANNEL_ID:
            self._is_opened = False
            acl_logger.log_error("Open video %s failed" % (name))
        else:
            self._is_opened = True
        self._is_destroyed = False
        resource_list.register(self)

    def _open(self, name):
        """
        open video
        """
        return libatlas.open_video(name.encode())

    def is_opened(self):
        """
        return status
        """
        return self._is_opened

    def read(self):
        """
        read decode frame and output status information
        """
        frame = ImageDataC()
        image = None
        ret = libatlas.ReadDecodedFrame(self._channel_id, byref(frame))
        if ret == READ_VIDEO_OK:
            image = acl_image.AclImage(addressof(frame.data.contents),
                             frame.width, frame.height, 
                             frame.size, MEMORY_DVPP)
        if ret == READ_VIDEO_ERROR:
            acl_logger.log_error("Read frame error")
        elif ret == READ_VIDEO_NOFRAME:
            acl_logger.log_info("No frame to read")
        elif ret == READ_VIDEO_FINISHED:
            acl_logger.log_info("Read video finished")
            
        return ret, image

    def close(self):
        """
        close video
        """
        ret = libatlas.CloseVideo(self._channel_id)
        if ret != SUCCESS:
            acl_logger.log_info("ERROR:Close %s failed" % (self._name))

    def destroy(self):
        """
        destroy resource
        """
        if self._is_destroyed is False:
            self.close()
            self._is_destroyed = True
            resource_list.unregister(self)

    def __del__(self):
        self.destroy()

