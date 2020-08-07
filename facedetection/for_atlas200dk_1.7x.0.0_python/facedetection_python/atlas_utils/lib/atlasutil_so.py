import threading
import ctypes
import os


class _AtlasutilLib(object):
    _instance_lock = threading.Lock()
    lib = ctypes.CDLL(os.path.dirname(os.path.abspath(__file__)) + '/libatlasutil.so')

    def __init__(self):
        pass

    def __new__(cls, *args, **kwargs):
        if not hasattr(_AtlasutilLib, "_instance"):
            with _AtlasutilLib._instance_lock:
                if not hasattr(_AtlasutilLib, "_instance"):
                    _AtlasutilLib._instance = object.__new__(cls)
        return _AtlasutilLib._instance

libatlas = _AtlasutilLib.lib
