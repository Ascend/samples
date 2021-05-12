import threading
import ctypes
import os
import platform

import acl

from atlas_utils.constants import ACL_HOST, ACL_DEVICE


def _load_lib_atlasutil():
    run_mode, ret = acl.rt.get_run_mode()

    lib = None
    if run_mode == ACL_DEVICE:
        cur_dir = os.path.dirname(os.path.abspath(__file__))
        so_path = os.path.join(cur_dir, 'atlas200dk/libatlasutil.so')
        lib=ctypes.CDLL(so_path)

    return lib


class _AtlasutilLib(object):
    _instance_lock=threading.Lock()
    lib=_load_lib_atlasutil()

    def __init__(self):
        pass

    def __new__(cls, *args, **kwargs):
        if not hasattr(_AtlasutilLib, "_instance"):
            with _AtlasutilLib._instance_lock:
                if not hasattr(_AtlasutilLib, "_instance"):
                    _AtlasutilLib._instance=object.__new__(
                        cls, *args, **kwargs)
        return _AtlasutilLib._instance

libatlas=_AtlasutilLib.lib
