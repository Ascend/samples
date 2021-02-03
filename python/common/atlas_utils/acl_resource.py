"""
Copyright (R) @huawei.com, all rights reserved
-*- coding:utf-8 -*-
CREATED:  2021-01-20 20:12:13
MODIFIED: 2021-02-03 14:04:45
"""
import acl

import atlas_utils.utils as utils
from atlas_utils.resource_list import resource_list

class AclResource(object):
    """
    AclResource
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
        utils.check_ret("acl.rt.set_device", ret)

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
            acl.rt.destroy_stream(self.stream)
            print("acl resource release stream")
        if self.context:
            acl.rt.destroy_context(self.context)
            print("acl resource release context")
        acl.rt.reset_device(self.device_id)
        acl.finalize()
        print("Release acl resource success")
