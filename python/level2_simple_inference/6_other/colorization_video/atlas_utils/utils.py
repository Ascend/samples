"""
@Date: 2020-12-09 23:28:01
@LastEditTime: 2020-12-17 23:32:26
@FilePath: /colorization_video_python/atlas_utils/utils.py
"""
import acl
from atlas_utils.constants import *

def check_ret(message, ret):
    """
    check ret,raise exception
    """
    if ret != ACL_ERROR_NONE:
        raise Exception("{} failed ret={}"
                        .format(message, ret))


def copy_data_device_to_host(device_data, data_size):
    """
    Copy device data to host memory
    """
    host_buffer, ret = acl.rt.malloc_host(data_size)
    if ret != ACL_ERROR_NONE:
        print("Malloc host memory failed, error: ", ret)
        return None

    ret = acl.rt.memcpy(host_buffer, data_size,
                        device_data, data_size,
                        ACL_MEMCPY_DEVICE_TO_HOST)
    if ret != ACL_ERROR_NONE:
        print("Copy device data to host memory failed, error: ", ret)
        acl.rt.free_host(host_buffer)
        return None

    return host_buffer


def copy_data_device_to_device(device_data, data_size):
    """
    Copy device data to device memory
    """
    device_buffer, ret = acl.rt.malloc(data_size, ACL_MEM_MALLOC_NORMAL_ONLY)
    if ret != ACL_ERROR_NONE:
        print("Malloc device memory failed, error: ", ret)
        return None

    ret = acl.rt.memcpy(device_buffer, data_size,
                        device_data, data_size,
                        ACL_MEMCPY_DEVICE_TO_DEVICE)
    if ret != ACL_ERROR_NONE:
        print("Copy device data to device memory failed, error: ", ret)
        acl.rt.free(device_buffer)
        return None

    return device_buffer


def copy_data_host_to_device(host_data, data_size):
    """
    Copy device data to device from host
    """
    device_buffer, ret = acl.rt.malloc(data_size, ACL_MEM_MALLOC_NORMAL_ONLY)
    if ret != ACL_ERROR_NONE:
        print("Malloc device memory failed, error: ", ret)
        return None

    ret = acl.rt.memcpy(device_buffer, data_size,
                        host_data, data_size,
                        ACL_MEMCPY_HOST_TO_DEVICE)
    if ret != ACL_ERROR_NONE:
        print("Copy device data to device memory failed, error: ", ret)
        acl.rt.free(device_buffer)
        return None

    return device_buffer


def copy_data_to_dvpp(data, size, run_mode):
    """
    Copy  data to dvpp
    """
    policy = ACL_MEMCPY_HOST_TO_DEVICE
    if run_mode == ACL_DEVICE:
        policy = ACL_MEMCPY_DEVICE_TO_DEVICE

    buffer, ret = acl.media.dvpp_malloc(size)
    check_ret("acl.rt.malloc_host", ret)
    print("dvpp buffer ", buffer, ", size ", size, ", src data ", data)
    ret = acl.rt.memcpy(buffer, size, data, size, policy)
    check_ret("acl.rt.memcpy", ret)

    return buffer


def align_up(value, align):
    """
    alignment
    """
    return int(int((value + align - 1) / align) * align)


def align_up16(value):
    """
    alignment
    """
    return align_up(value, 16)


def align_up2(value):
    """
    alignment
    """
    return align_up(value, 2)


def yuv420sp_size(width, height):
    """
    return yuv420sp size
    """
    return int(width * height * 3 / 2)