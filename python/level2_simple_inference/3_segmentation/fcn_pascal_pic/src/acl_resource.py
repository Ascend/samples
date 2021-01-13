import acl

from atlas_utils.constants import *
from atlas_utils.utils import *

DICT_KEY_RESOURCE = "resource"
DICT_KEY_STATUS = "status"
DICT_VAL_REG = "register"
DICT_VAL_UNREG = "unregister"



class AclResource(object):
    def __init__(self, device_id=0):
        self.device_id = device_id
        self.context = None
        self.stream = None
        self.run_mode = None
        self.is_destroyed = False
        self.other_resource_list = []

    def init(self):
        print("[Sample] init resource stage:")
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
        return self.stream

    def register_resource(self, resource):
        self.other_resource_list.append({DICT_KEY_RESOURCE:resource, 
                                         DICT_KEY_STATUS:DICT_VAL_REG})

    def unregister_resource(self, resource):
        for i in range(len(self.other_resource_list)):            
            if self.other_resource_list[i] == resource:
                self.other_resource_list[i][DICT_KEY_STATUS] = DICT_VAL_UNREG
                break
    
    def destroy(self):
        self.__del__()
        self.is_destroyed = True
    
    def __del__(self):
        if self.is_destroyed:
            return

        print("Release acl resource, ", len(self.other_resource_list))
        for i in range(len(self.other_resource_list)): 
            print("Start relase resource ", i)           
            if self.other_resource_list[i][DICT_KEY_STATUS] == DICT_VAL_REG:
                del self.other_resource_list[i][DICT_KEY_RESOURCE]

        if self.stream:
            acl.rt.destroy_stream(self.stream)
        if self.context:
            acl.rt.destroy_context(self.context)
        acl.rt.reset_device(self.device_id)
        acl.finalize()
        print("Release acl resource success")