import ctypes
from ctypes import *
import numpy as np

from lib.atlasutil_so import libatlas, g_run_mode

class DataBufC(Structure):
    _fields_ = [
        ('size',        c_int),
        ('data',        POINTER(c_ubyte))
    ]

class DataBuf():
    def __init__(self, data, data_size):
        self.data = data
        self.size = data_size
        self.nparray = None

    def copy_to_local(self):
        src_data = DataBufC()
        src_data.data = cast(self.data, POINTER(c_ubyte))
        src_data.size = self.size
        dest_data = DataBufC()
        ret = libatlas.CopyDataBufToLocal(byref(dest_data), byref(src_data), 
                                          g_run_mode)
        if ret:
            log_error("Copy data to local failed")
            return None

        return DataBuf(dest_data.data, dest_data.size)
    
    def tobytes(self):
        self.nparray = np.frombuffer((ctypes.c_ubyte * self.size).from_address(
            ctypes.addressof(self.data.contents)), dtype=np.uint8)
        return self.nparray.tobytes()

    def destroy(self):
        data_buf = DataBufC()
        data_buf.data = cast(self.data, POINTER(c_ubyte))
        data_buf.size = self.size
        libatlas.ReleaseDataBuf(byref(data_buf))
        self.data = None
        self.size = 0
            