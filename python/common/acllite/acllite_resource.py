"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2021-01-20 20:12:13
MODIFIED: 2021-02-03 14:04:45
"""
import threading
import acl

import acllite_utils as utils

REGISTER = 0
UNREGISTER = 1

class _ResourceList(object):
    """Acl resources of current application
    This class provide register inferace of acl resource, when application
    exit, all register resource will release befor acl.rt.reset_device to
    avoid program abnormal 
    """
    _instance_lock = threading.Lock()

    def __init__(self):
        self.resources = []

    def __new__(cls, *args, **kwargs):
        if not hasattr(_ResourceList, "_instance"):
            with _ResourceList._instance_lock:
                if not hasattr(_ResourceList, "_instance"):
                    _ResourceList._instance = object.__new__(
                        cls, *args, **kwargs)
        return _ResourceList._instance

    def register(self, resource):
        """Resource register interface
        Args:
            resource: object with acl resource, the object must be has
                      method destroy()
        """
        item = {"resource": resource, "status": REGISTER}
        self.resources.append(item)

    def unregister(self, resource):
        """Resource unregister interface
        If registered resource release by self and no need _ResourceList 
        release, the resource object should unregister self
        Args:
            resource: registered resource
        """
        for item in self.resources:
            if resource == item["resource"]:
                item["status"] = UNREGISTER

    def destroy(self):
        """Destroy all register resource"""
        for item in self.resources:
            if item["status"] == REGISTER:
                item["resource"].destroy()
                item["status"] = UNREGISTER

resource_list = _ResourceList()

class AclLiteResource(object):
    """
    AclLiteResource
    """

    def __init__(self, device_id=0):
        self.device_id = device_id
        self.context = None
        self.stream = None
        self.run_mode = None

    def init(self):
        """
        init resource
        """
        print("init resource stage:")
        ret = acl.init()
        utils.check_ret("acl.init", ret)

        ret = acl.rt.set_device(self.device_id)
        utils.check_ret("acl.rt.set_device", ret)

        self.context, ret = acl.rt.create_context(self.device_id)
        utils.check_ret("acl.rt.create_context", ret)

        self.stream, ret = acl.rt.create_stream()
        utils.check_ret("acl.rt.create_stream", ret)

        self.run_mode, ret = acl.rt.get_run_mode()
        utils.check_ret("acl.rt.get_run_mode", ret)

        print("Init resource success")

    def __del__(self):
        print("acl resource release all resource")
        resource_list.destroy()
        if self.stream:
            print("acl resource release stream")
            acl.rt.destroy_stream(self.stream)

        if self.context:
            print("acl resource release context")
            acl.rt.destroy_context(self.context)

        print("Reset acl device ", self.device_id)
        acl.rt.reset_device(self.device_id)
        acl.finalize()
        print("Release acl resource success")
