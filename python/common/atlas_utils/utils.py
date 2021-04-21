import numpy as np
import acl
import atlas_utils.constants as const
from atlas_utils.acl_logger import log_error, log_info
import time

from functools import wraps
DEBUG = True


def check_ret(message, ret_int):
    """Check int value is 0 or not
    Args:
        message: output log str
        ret_int: check value that type is int
    """
    if ret_int != 0:
        raise Exception("{} failed ret_int={}"
                        .format(message, ret_int))


def check_none(message, ret_none):
    """Check object is None or not
    Args:
        message: output log str
        ret_none: check object
    """
    if ret_none is None:
        raise Exception("{} failed"
                        .format(message))


def copy_data_device_to_host(device_data, data_size):
    """Copy device data to host
    Args:
        device_data: data that to be copyed
        data_size: data size
    Returns:
        None: copy failed
        others: host data which copy from device_data
    """
    host_buffer, ret = acl.rt.malloc_host(data_size)
    if ret != const.ACL_ERROR_NONE:
        log_error("Malloc host memory failed, error: ", ret)
        return None

    ret = acl.rt.memcpy(host_buffer, data_size,
                        device_data, data_size,
                        const.ACL_MEMCPY_DEVICE_TO_HOST)
    if ret != const.ACL_ERROR_NONE:
        log_error("Copy device data to host memory failed, error: ", ret)
        acl.rt.free_host(host_buffer)
        return None

    return host_buffer


def copy_data_device_to_device(device_data, data_size):
    """Copy device data to device
    Args:
        device_data: data that to be copyed
        data_size: data size
    Returns:
        None: copy failed
        others: device data which copy from device_data
    """
    device_buffer, ret = acl.rt.malloc(data_size,
                                       const.ACL_MEM_MALLOC_NORMAL_ONLY)
    if ret != const.ACL_ERROR_NONE:
        log_error("Malloc device memory failed, error: ", ret)
        return None

    ret = acl.rt.memcpy(device_buffer, data_size,
                        device_data, data_size,
                        const.ACL_MEMCPY_DEVICE_TO_DEVICE)
    if ret != const.ACL_ERROR_NONE:
        log_error("Copy device data to device memory failed, error: ", ret)
        acl.rt.free(device_buffer)
        return None

    return device_buffer


def copy_data_host_to_device(host_data, data_size):
    """Copy host data to device
    Args:
        host_data: data that to be copyed
        data_size: data size
    Returns:
        None: copy failed
        others: device data which copy from host_data
    """
    device_buffer, ret = acl.rt.malloc(data_size,
                                       const.ACL_MEM_MALLOC_NORMAL_ONLY)
    if ret != const.ACL_ERROR_NONE:
        log_error("Malloc device memory failed, error: ", ret)
        return None

    ret = acl.rt.memcpy(device_buffer, data_size,
                        host_data, data_size,
                        const.ACL_MEMCPY_HOST_TO_DEVICE)
    if ret != const.ACL_ERROR_NONE:
        log_error("Copy device data to device memory failed, error: ", ret)
        acl.rt.free(device_buffer)
        return None

    return device_buffer


def copy_data_host_to_host(host_data, data_size):
    """Copy host data to host
    Args:
        host_data: data that to be copyed
        data_size: data size
    Returns:
        None: copy failed
        others: host data which copy from host_data
    """
    host_buffer, ret = acl.rt.malloc_host(data_size)
    if ret != const.ACL_ERROR_NONE:
        log_error("Malloc host memory failed, error: ", ret)
        return None

    ret = acl.rt.memcpy(host_buffer, data_size,
                        host_data, data_size,
                        const.ACL_MEMCPY_HOST_TO_HOST)
    if ret != const.ACL_ERROR_NONE:
        log_error("Copy host data to host memory failed, error: ", ret)
        acl.rt.free_host(host_buffer)
        return None

    return host_buffer


def copy_data_to_dvpp(data, size, run_mode):
    """Copy data to dvpp
    Args:
        data: data that to be copyed
        data_size: data size
        run_mode: device run mode
    Returns:
        None: copy failed
        others: data which copy from host_data
    """
    policy = const.ACL_MEMCPY_HOST_TO_DEVICE
    if run_mode == const.ACL_DEVICE:
        policy = const.ACL_MEMCPY_DEVICE_TO_DEVICE

    dvpp_buf, ret = acl.media.dvpp_malloc(size)
    check_ret("acl.rt.malloc_host", ret)

    ret = acl.rt.memcpy(dvpp_buf, size, data, size, policy)
    check_ret("acl.rt.memcpy", ret)

    return dvpp_buf


def copy_data_as_numpy(data, size, data_mem_type, run_mode):
    """Copy data as numpy array
    Args:
        data: data that to be copyed
        size: data size
        data_mem_type: src data memory type
        run_mode: device run mode
    Returns:
        None: copy failed
        others: numpy array whoes data copy from host_data
    """
    np_data = np.zeros(size, dtype=np.byte)
    np_data_ptr = acl.util.numpy_to_ptr(np_data)

    policy = const.ACL_MEMCPY_DEVICE_TO_DEVICE
    if run_mode == const.ACL_HOST:
        if ((data_mem_type == const.MEMORY_DEVICE) or
                (data_mem_type == const.MEMORY_DVPP)):
            policy = const.ACL_MEMCPY_DEVICE_TO_HOST
        elif data_mem_type == const.MEMORY_HOST:
            policy = const.ACL_MEMCPY_HOST_TO_HOST

    ret = acl.rt.memcpy(np_data_ptr, size, data, size, policy)
    check_ret("acl.rt.memcpy", ret)

    return np_data


def align_up(value, align):
    """Align up int value
    Args:
        value:input data
        align: align data
    Return:
        aligned data
    """
    return int(int((value + align - 1) / align) * align)


def align_up16(value):
    """Align up data with 16
    Args:
        value:input data
    Returns:
        16 aligned data
    """
    return align_up(value, 16)


def align_up128(value):
    """Align up data with 128
    Args:
        value:input data
    Returns:
        128 aligned data
    """
    return align_up(value, 128)


def align_up2(value):
    """Align up data with 2
    Args:
        value:input data
    Returns:
        2 aligned data
    """
    return align_up(value, 2)


def yuv420sp_size(width, height):
    """Calculate yuv420sp image size
    Args:
        width: image width
        height: image height
    Returns:
        image data size
    """
    return int(width * height * 3 / 2)


def rgbu8_size(width, height):
    """Calculate rgb 24bit image size
    Args:
        width: image width
        height: image height
    Returns:
        rgb 24bit image data size
    """
    return int(width * height * 3)


def display_time(func):
    """print func execute time"""
    @wraps(func)
    def wrapper(*args, **kwargs):
        """wrapper caller"""
        if DEBUG:
            btime = time.time()
            res = func(*args, **kwargs)
            use_time = time.time() - btime
            print("in %s, use time:%s" % (func.__name__, use_time))
            return res
        else:
            return func(*args, **kwargs)

    return wrapper
