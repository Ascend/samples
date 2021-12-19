import threading
import ctypes
import os
import platform

import acl

from constants import ACL_HOST, ACL_DEVICE


def _load_lib_acllite():
    run_mode, ret = acl.rt.get_run_mode()

    lib = None
    if run_mode == ACL_DEVICE:
        cur_dir = os.path.dirname(os.path.abspath(__file__))
        so_path = os.path.join(cur_dir, 'atlas200dk/libpython_acllite.so')
        lib=ctypes.CDLL(so_path)

    return lib


class _AclLiteLib(object):
    _instance_lock=threading.Lock()
    lib=_load_lib_acllite()

    def __init__(self):
        pass

    def __new__(cls, *args, **kwargs):
        if not hasattr(_AclLiteLib, "_instance"):
            with _AclLiteLib._instance_lock:
                if not hasattr(_AclLiteLib, "_instance"):
                    _AclLiteLib._instance=object.__new__(
                        cls, *args, **kwargs)
        return _AclLiteLib._instance

libacllite=_AclLiteLib.lib
