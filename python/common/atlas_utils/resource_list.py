import threading

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
