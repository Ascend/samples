import numpy as np
import acl
import atlas_utils.constants as constants
from atlas_utils.acl_logger import log_error, log_info
import time

from functools import wraps
DEBUG = True

def check_ret(message, ret_int):
    if ret_int != 0:
        raise Exception("{} failed ret_int={}"
                        .format(message, ret_int))


def check_none(message, ret_none):
    if ret_none is None:
        raise Exception("{} failed"
                        .format(message))


def copy_data_device_to_host(device_data, data_size):
    """
    Copy device data to host
    """
    host_buffer, ret = acl.rt.malloc_host(data_size)
    if ret != constants.ACL_ERROR_NONE:
        log_error("Malloc host memory failed, error: ", ret)
        return None

    ret = acl.rt.memcpy(host_buffer, data_size,
                        device_data, data_size,
                        constants.ACL_MEMCPY_DEVICE_TO_HOST)
    if ret != constants.ACL_ERROR_NONE:
        log_error("Copy device data to host memory failed, error: ", ret)
        acl.rt.free_host(host_buffer)
        return None

    return host_buffer


def copy_data_device_to_device(device_data, data_size):
    """
    Copy device data to device
    """
    device_buffer, ret = acl.rt.malloc(data_size, constants.ACL_MEM_MALLOC_NORMAL_ONLY)
    if ret != constants.ACL_ERROR_NONE:
        log_error("Malloc device memory failed, error: ", ret)
        return None


    ret = acl.rt.memcpy(device_buffer, data_size,
                        device_data, data_size,
                        constants.ACL_MEMCPY_DEVICE_TO_DEVICE)
    if ret != constants.ACL_ERROR_NONE:
        log_error("Copy device data to device memory failed, error: ", ret)
        acl.rt.free(device_buffer)
        return None

    return device_buffer


def copy_data_host_to_device(host_data, data_size):
    """
    Copy device host to device
    """
    device_buffer, ret = acl.rt.malloc(data_size, constants.ACL_MEM_MALLOC_NORMAL_ONLY)
    if ret != constants.ACL_ERROR_NONE:
        log_error("Malloc device memory failed, error: ", ret)
        return None

    ret = acl.rt.memcpy(device_buffer, data_size,
                        host_data, data_size,
                        constants.ACL_MEMCPY_HOST_TO_DEVICE)
    if ret != constants.ACL_ERROR_NONE:
        log_error("Copy device data to device memory failed, error: ", ret)
        acl.rt.free(device_buffer)
        return None

    return device_buffer


def copy_data_to_dvpp(data, size, run_mode):
    """
    Copy data to dvpp
    """
    policy = constants.ACL_MEMCPY_HOST_TO_DEVICE
    if run_mode == constants.ACL_DEVICE:
        policy = constants.ACL_MEMCPY_DEVICE_TO_DEVICE

    buffer, ret = acl.media.dvpp_malloc(size)
    check_ret("acl.rt.malloc_host", ret)

    ret = acl.rt.memcpy(buffer, size, data, size, policy)
    check_ret("acl.rt.memcpy", ret)

    return buffer


def copy_data_as_numpy(data, size, run_mode):
    """
    copy data as numpy
    """
    np_data = np.zeros(size, dtype=np.byte)
    np_data_ptr = acl.util.numpy_to_ptr(np_data)
    policy = constants.ACL_MEMCPY_HOST_TO_DEVICE
    if run_mode == constants.ACL_DEVICE:
        policy = constants.ACL_MEMCPY_DEVICE_TO_DEVICE
    ret = acl.rt.memcpy(np_data_ptr, size, data, size, policy)
    check_ret("acl.rt.memcpy", ret)

    return np_data


def align_up(value, align):
    """
    :param value:input data
    :param align: align data
    :return: aligned data
    """
    return int(int((value + align - 1) / align) * align)


def align_up16(value):
    """
    :param value:input data
    :return: 16 aligned data
    """
    return align_up(value, 16)


def align_up128(value):
    """
    :param value:input data
    :return: 128 aligned data
    """
    return align_up(value, 128)


def align_up2(value):
    """
    :param value:input data
    :return: 2 aligned data
    """
    return align_up(value, 2)


def yuv420sp_size(width, height):
    """
    :param width: input width
    :param height: input width
    :return: yuv420sp size
    """
    return int(width * height * 3 / 2)

def display_time(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        if DEBUG:
            btime = time.time()
            res = func(*args, **kwargs)
            use_time = time.time() - btime
            print("in %s, use time:%s" % (func.__name__, use_time))
            return res
        else:
            return func(*args, **kwargs)

    return wrapper

