import acl
from atlas_utils.constants import *

def check_ret(message, ret_int):
    if ret_int != 0:
        raise Exception("{} failed ret_int={}"
                        .format(message, ret_int))


def check_none(message, ret_none):
    if ret_none is None:
        raise Exception("{} failed"
                        .format(message))

def copy_data_device_to_host(device_data, data_size):
    host_buffer, ret = acl.rt.malloc_host(data_size)
    if ret != ACL_ERROR_NONE:
        log_error("Malloc host memory failed, error: ", ret)
        return None

    ret = acl.rt.memcpy(host_buffer, data_size,
                        device_data, data_size,
                        ACL_MEMCPY_DEVICE_TO_HOST)
    if ret != ACL_ERROR_NONE:
        log_error("Copy device data to host memory failed, error: ", ret)
        acl.rt.free_host(host_buffer)
        return None

    return host_buffer

def copy_data_device_to_device(device_data, data_size):
    device_buffer, ret = acl.rt.malloc(data_size, ACL_MEM_MALLOC_NORMAL_ONLY)
    if ret != ACL_ERROR_NONE:
        log_error("Malloc device memory failed, error: ", ret)
        return None

    ret = acl.rt.memcpy(device_buffer, data_size,
                        device_data, data_size,
                        ACL_MEMCPY_DEVICE_TO_DEVICE)
    if ret != ACL_ERROR_NONE:
        log_error("Copy device data to device memory failed, error: ", ret)
        acl.rt.free(device_buffer)
        return None

    return device_buffer

def copy_data_host_to_device(host_data, data_size):
    device_buffer, ret = acl.rt.malloc(data_size, ACL_MEM_MALLOC_NORMAL_ONLY)
    if ret != ACL_ERROR_NONE:
        log_error("Malloc device memory failed, error: ", ret)
        return None

    ret = acl.rt.memcpy(device_buffer, data_size,
                        host_data, data_size,
                        ACL_MEMCPY_HOST_TO_DEVICE)
    if ret != ACL_ERROR_NONE:
        log_error("Copy device data to device memory failed, error: ", ret)
        acl.rt.free(device_buffer)
        return None

    return device_buffer

def copy_data_to_dvpp(data, size, run_mode):
    policy = ACL_MEMCPY_HOST_TO_DEVICE
    if run_mode == ACL_DEVICE:
        policy = ACL_MEMCPY_DEVICE_TO_DEVICE

    buffer, ret = acl.media.dvpp_malloc(size)
    check_ret("acl.rt.malloc_host", ret)

    ret = acl.rt.memcpy(buffer, size, data, size, policy)
    check_ret("acl.rt.memcpy", ret)

    return buffer

def copy_data_as_numpy(data, size, run_mode):
    np_data = np.zeros(size, dtype=np.byte)
    np_data_ptr = acl.util.numpy_to_ptr(np_data)
    policy = ACL_MEMCPY_HOST_TO_DEVICE
    if run_mode == ACL_DEVICE:
        policy = ACL_MEMCPY_DEVICE_TO_DEVICE
    ret = acl.rt.memcpy(np_data_ptr, size, data, size, policy)
    check_ret("acl.rt.memcpy", ret)

    return np_data

def align_up(value, align):
    return int(int((value + align - 1) / align) * align)

def align_up16(value):
    return align_up(value, 16)

def align_up2(value):
    return align_up(value, 2)

def yuv420sp_size(width, height):
    return int(width * height * 3 /2)