"""
@Date: 2020-12-09 23:28:01
@LastEditTime: 2020-12-17 23:12:41
@FilePath: /colorization_video_python/atlas_utils/resource_list.py
"""
import threading

REGISTER = 0
UNREGISTER = 1

class _ResourceList(object):
    _instance_lock = threading.Lock()    

    def __init__(self):
        self.resources = []

    def __new__(cls):
        if not hasattr(_ResourceList, "_instance"):
            with _ResourceList._instance_lock:
                if not hasattr(_ResourceList, "_instance"):
                    _ResourceList._instance = object.__new__(cls)
        return _ResourceList._instance

    def register(self, resource):
        """
        resource status is register
        """
        item = {"resource":resource, "status":REGISTER}
        self.resources.append(item)

    def unregister(self, resource):
        """
        resource status is unregister
        """
        for item in self.resources:
            if resource == item["resource"]:
                item["status"] = UNREGISTER

    def destroy(self):
        """
        destroy register
        """
        for item in self.resources:
            if item["status"] == REGISTER:
                item["resource"].destroy()
                item["status"] = UNREGISTER


resource_list = _ResourceList()