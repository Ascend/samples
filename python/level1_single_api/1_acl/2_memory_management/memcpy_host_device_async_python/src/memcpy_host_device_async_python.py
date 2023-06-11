import argparse
from time import sleep
import logging
import numpy as np
import acl


ACL_SUCCESS = 0
ACL_DDR_MEM = 0
ACL_HBM_MEM = 1
ACL_MEMCPY_HOST_TO_HOST = 0
ACL_MEMCPY_HOST_TO_DEVICE = 1
ACL_MEMCPY_DEVICE_TO_HOST = 2


logger = logging.getLogger('memcpy_host_device')
logger.setLevel(logging.DEBUG)
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
formatter = logging.Formatter('[%(asctime)s][%(name)s][%(levelname)s]>>> %(message)s')
ch.setFormatter(formatter)
logger.addHandler(ch)



def check_ret(message, ret):
    if ret != ACL_SUCCESS:
        logger.error(" %s failed ret = %s", message, ret)
        sleep(5)
        return True
    return False


def copy_data_as_numpy(message, data, size, policy):
    np_data = np.ones(size, dtype = np.byte)
    if "bytes_to_ptr" in dir(acl.util):
        bytes_data = np_data.tobytes()
        np_data_ptr = acl.util.bytes_to_ptr(bytes_data)
    else:
        np_data_ptr = acl.util.numpy_to_ptr(np_data)

    ret = acl.rt.memcpy(np_data_ptr, size, data, size, policy)
    check_ret("acl.rt.memcpy", ret)
    if "bytes_to_ptr" in dir(acl.util):
        np_data = np.frombuffer(bytes_data, dtype = np_data.dtype).reshape(np_data.shape)
    logger.info("%s is %s", message, np_data)
    return False


def search_memory(cycles_time, release_cycle_time):
    free, total, ret = acl.rt.get_mem_info(ACL_DDR_MEM)
    if check_ret("acl.rt.get_mem_info", ret):
        return True
    logger.info("At number_of_cycles = %s, release_cycle = %s, DDR free memory:%s Byte, DDR total memory:%s Byte.", \
                cycles_time, release_cycle_time, free, total)
    free, total, ret = acl.rt.get_mem_info(ACL_HBM_MEM)
    if check_ret("acl.rt.get_mem_info", ret):
        return True
    logger.info("At number_of_cycles = %s, release_cycle = %s, HBM free memory:%s Byte, HBM total memory:%s Byte.", \
                cycles_time, release_cycle_time, free, total)
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--release_cycle", type=int, default=2, help="Specify the release period")
    parser.add_argument("--number_of_cycles", type=int, default=1, help="The number of overall process cycles")
    parser.add_argument("--device_id", type=int, default=0, help="The ID of device currently in use")
    parser.add_argument("--memory_size", type=int, default=10485760, help="Single request memory size, \
    The units are Byte")
    args = parser.parse_args()
    logger.info("Using params are as follows.\n  device id : %s\n  release cycle : %s\n"
                "  number of cycles : %s\n  memory size : %s Bytes\n", \
                args.device_id, args.release_cycle, args.number_of_cycles, args.memory_size)


    ret = acl.init()
    if ret != ACL_SUCCESS:
        logger.error("acl.init failed ret = %s", ret)
        exit(1)
    ret = acl.rt.set_device(args.device_id)
    if ret != ACL_SUCCESS:
        logger.error("acl.rt.set_device failed ret = %s", ret)
        exit(1)
    context, ret = acl.rt.create_context(args.device_id)
    if ret != ACL_SUCCESS:
        logger.error("acl.rt.create_context failed ret = %s", ret)
        exit(1)
    stream, ret = acl.rt.create_stream()
    if ret != ACL_SUCCESS:
        logger.error("acl.rt.create_stream failed ret = %s", ret)
        exit(1)
    stream_, ret = acl.rt.create_stream()
    if ret != ACL_SUCCESS:
        logger.error("acl.rt.create_stream failed ret = %s", ret)
        exit(1)
    if search_memory(0, 0):
        exit(1)

    cycles_time = 0
    while (args.number_of_cycles - cycles_time != 0):
        dev_mem_list = []
        release_cycle_time = 0
        while args.release_cycle - release_cycle_time != 0:
            host_mem_list = []
            host_buffer, ret = acl.rt.malloc_host(args.memory_size)
            if check_ret("acl.rt.malloc_host", ret):
                continue
            host_mem_list.append(host_buffer)
            ret = acl.rt.memset_async(host_buffer, args.memory_size, 7, args.memory_size, stream)
            if check_ret("acl.rt.memset_async", ret):
                continue
            device_buffer, ret = acl.media.dvpp_malloc(args.memory_size)
            if check_ret("acl.media.dvpp_malloc", ret):
                continue
            dev_mem_list.append(device_buffer)
            ret = acl.rt.synchronize_stream(stream)
            if check_ret("acl.rt.synchronize_stream", ret):
                continue
            if copy_data_as_numpy("At first memset host data", host_buffer, args.memory_size, \
                                  ACL_MEMCPY_HOST_TO_HOST):
                continue
            ret = acl.rt.memcpy_async(device_buffer, args.memory_size, host_buffer, args.memory_size, \
                                      ACL_MEMCPY_HOST_TO_DEVICE, stream)
            if check_ret("acl.rt.memcpy_async", ret):
                continue
            host_buffer_, ret = acl.rt.malloc_host(args.memory_size)
            if check_ret("acl.rt.malloc_host", ret):
                continue
            if search_memory(cycles_time, release_cycle_time):
                exit(1)
            ret = acl.rt.synchronize_stream(stream)
            if check_ret("acl.rt.synchronize_stream", ret):
                continue
            if copy_data_as_numpy("At memcpy device data", host_buffer, args.memory_size, \
                                  ACL_MEMCPY_DEVICE_TO_HOST):
                continue
            ret = acl.rt.memcpy_async(host_buffer_, args.memory_size, device_buffer, args.memory_size, \
                                      ACL_MEMCPY_DEVICE_TO_HOST, stream)
            if check_ret("acl.rt.memcpy_async", ret):
                continue
            for host_buffer_1 in host_mem_list:
                ret = acl.rt.free_host(host_buffer_1)
                if check_ret("acl.rt.free_host", ret):
                    continue
            ret = acl.rt.synchronize_stream(stream)
            if check_ret("acl.rt.synchronize_stream", ret):
                continue
            ret = acl.rt.free_host(host_buffer_)
            if check_ret("acl.rt.free_host", ret):
                continue
            release_cycle_time += 1
            sleep(1)
        for device_buffer in dev_mem_list:
            ret = acl.media.dvpp_free(device_buffer)
            if check_ret("acl.media.dvpp_free", ret):
                continue
        cycles_time += 1
        sleep(1)
        if search_memory(cycles_time, release_cycle_time):
            exit(1)

    ret = acl.rt.destroy_context(context)
    if ret != ACL_SUCCESS:
        logger.error("acl.rt.destroy_context ret = %s", ret)
        exit(1)
    ret = acl.rt.reset_device(args.device_id)
    if ret != ACL_SUCCESS:
        logger.error("acl.rt.reset_device ret = %s", ret)
        exit(1)
    ret = acl.finalize()
    if ret != ACL_SUCCESS:
        logger.error("acl.finalize failed ret = %s", ret)
        exit(1)
    print("run success!")

if __name__ == '__main__':
    main()