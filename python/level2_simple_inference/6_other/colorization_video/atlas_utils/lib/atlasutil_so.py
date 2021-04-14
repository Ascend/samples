"""
@Date: 2020-12-09 23:28:01
@LastEditTime: 2020-12-17 19:48:37
@FilePath: /colorization_video_python/atlas_utils/lib/atlasutil_so.py
"""
import threading
import ctypes
import os

import acl

from atlas_utils.constants import ACL_HOST, ACL_DEVICE

g_run_mode, ret = acl.rt.get_run_mode()

def _lib_relative_path():
    global g_run_mode
    so_path = ""    
    if g_run_mode == ACL_HOST:
        so_path = '/aisc/libatlasutil.so'
    elif g_run_mode == ACL_DEVICE:
        so_path = '/atlas200dk/libatlasutil.so'
    else:
        raise Exception("Invalid run mode value ")

    return so_path


class _AtlasutilLib(object):
    _instance_lock = threading.Lock() 
    lib = ctypes.CDLL(os.path.dirname(os.path.abspath(__file__))+ 
                                      _lib_relative_path())

    def __init__(self):
        pass

    def __new__(cls):
        if not hasattr(_AtlasutilLib, "_instance"):
            with _AtlasutilLib._instance_lock:
                if not hasattr(_AtlasutilLib, "_instance"):
                    _AtlasutilLib._instance = object.__new__(cls)
        return _AtlasutilLib._instance

libatlas = _AtlasutilLib.lib
