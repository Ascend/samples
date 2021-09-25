from constant import ACL_ERROR_NONE


def check_ret(message, ret):
    """
    功能简介：检测pyACL函数返回值是否正常，如果非0则会抛出异常
    参数：ret，pyACL函数返回值
    返回值：无
    """
    if ret != ACL_ERROR_NONE:
        raise Exception("{} failed ret = {}".format(message, ret))
