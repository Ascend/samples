import acl

from utils import *
from resource_list import resource_list

class AclResource(object):
    def __init__(self, device_id=0):
        self.device_id = device_id
        self.context = None
        self.stream = None
        self.run_mode = None

    def init(self):
        print("init resource stage:")
        ret = acl.init()
        check_ret("acl.rt.set_device", ret)

        ret = acl.rt.set_device(self.device_id)
        check_ret("acl.rt.set_device", ret)

        self.context, ret = acl.rt.create_context(self.device_id)
        check_ret("acl.rt.create_context", ret)

        self.stream, ret = acl.rt.create_stream()
        check_ret("acl.rt.create_stream", ret)

        self.run_mode, ret = acl.rt.get_run_mode()
        check_ret("acl.rt.get_run_mode", ret)

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