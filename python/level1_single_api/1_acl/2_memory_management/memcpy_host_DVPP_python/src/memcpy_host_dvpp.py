import argparse
from time import sleep
import logging
import numpy as np
import acl

# 全局参数定义
ACL_SUCCESS = 0
ACL_DDR_MEM = 0
ACL_HBM_MEM = 1
ACL_MEMCPY_HOST_TO_HOST = 0
ACL_MEMCPY_HOST_TO_DEVICE = 1
ACL_MEMCPY_DEVICE_TO_HOST = 2

# 打屏日志设置
logger = logging.getLogger('memcpy_host_device')
logger.setLevel(logging.DEBUG)
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
formatter = logging.Formatter('[%(asctime)s][%(name)s][%(levelname)s]>>> %(message)s')
ch.setFormatter(formatter)
logger.addHandler(ch)


# 运行结果检查函数
def check_ret(message, ret):
    if ret != ACL_SUCCESS:
        logger.error("{} failed ret = {}".format(message, ret))
        sleep(5)
        return True
    return False


# 将malloc的内存拷贝到numpy数组中以便查看或使用
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
    logger.info("{} is {}".format(message, np_data))
    return False


# Device内存查询函数
def search_memory(cycles_time, release_cycle_time):
    free, total, ret = acl.rt.get_mem_info(ACL_DDR_MEM)
    if check_ret("acl.rt.get_mem_info", ret):
        return True
    logger.info("At number_of_cycles = {}, release_cycle = {}, DDR free memory:{} Byte, DDR total memory:{} Byte."
            .format(cycles_time, release_cycle_time, free, total))
    free, total, ret = acl.rt.get_mem_info(ACL_HBM_MEM)
    if check_ret("acl.rt.get_mem_info", ret):
        return True
    logger.info("At number_of_cycles = {}, release_cycle = {}, HBM free memory:{} Byte, HBM total memory:{} Byte."
            .format(cycles_time, release_cycle_time, free, total))
    return False


def main():
    # 获取入参
    parser = argparse.ArgumentParser()
    parser.add_argument("--release_cycle", type=int, default=-1, help="Specify the release period")
    parser.add_argument("--number_of_cycles", type=int, default=1, help="The number of overall process cycles")
    parser.add_argument("--device_id", type=int, default=0, help="The ID of device currently in use")
    parser.add_argument("--memory_size", type=int, default=10485760, help="Single request memory size, \
    The units are Byte")
    parser.add_argument("--memory_reuse", action='store_true', help="Whether to send back the host")
    args = parser.parse_args()
    logger.info("Using params are as follows.\n  device id : {}\n  release cycle : {}\n"
          "  number of cycles : {}\n  memory size : {} Bytes\n  memory_reuse : {}\n"
          .format(args.device_id, args.release_cycle, args.number_of_cycles, args.memory_size, args.memory_reuse))

    # acl资源初始化
    ret = acl.init()
    if ret != ACL_SUCCESS:
        logger.error("acl.init failed ret = {}".format(ret))
        exit(1)
    ret = acl.rt.set_device(args.device_id)
    if ret != ACL_SUCCESS:
        logger.error("acl.rt.set_device failed ret = {}".format(ret))
        exit(1)
    context, ret = acl.rt.create_context(args.device_id)
    if ret != ACL_SUCCESS:
        logger.error("acl.rt.create_context failed ret = {}".format(ret))
        exit(1)

    # 内存使用前，首次查询Device内存
    if search_memory(0, 0):
        exit(1)

    # 根据循环次数设置循环
    cycles_time = 0
    while (args.number_of_cycles - cycles_time != 0):
        # 设置内存list，方便后续释放
        host_mem_list = []
        dev_mem_list = []
        # 如果内存复用，则需要在释放周期循环外申请内存
        if args.memory_reuse:
            host_buffer, ret = acl.rt.malloc_host(args.memory_size)
            if check_ret("acl.rt.malloc_host", ret):
                continue
            device_buffer, ret = acl.media.dvpp_malloc(args.memory_size)
            if check_ret("acl.media.dvpp_malloc", ret):
                continue
            host_mem_list.append(host_buffer)
            dev_mem_list.append(device_buffer)
        # 根据释放周期设置循环
        release_cycle_time = 0
        while (args.release_cycle - release_cycle_time != 0):
            # 如果内存不复用，则每次都重新申请内存
            if not args.memory_reuse:
                host_buffer, ret = acl.rt.malloc_host(args.memory_size)
                if check_ret("acl.rt.malloc_host", ret):
                    continue
                device_buffer, ret = acl.media.dvpp_malloc(args.memory_size)
                if check_ret("acl.media.dvpp_malloc", ret):
                    continue
                host_mem_list.append(host_buffer)
                dev_mem_list.append(device_buffer)
            # 初始化内存，构建全7的数据
            ret = acl.rt.memset(host_buffer, args.memory_size, 7, args.memory_size)
            if check_ret("acl.rt.memset", ret):
                continue
            # 查看初始化后的内存
            if copy_data_as_numpy("At first memset host data", host_buffer, args.memory_size, \
            ACL_MEMCPY_HOST_TO_HOST):
                continue
            #内存拷贝H2D
            ret = acl.rt.memcpy(device_buffer, args.memory_size, host_buffer, args.memory_size, \
            ACL_MEMCPY_HOST_TO_DEVICE)
            if check_ret("acl.rt.memcpy", ret):
                continue
            # 查看device上的数据，需要在函数中将Deivce内存拷贝到Host上的numpy数组，所以传ACL_MEMCPY_DEVICE_TO_HOST
            if copy_data_as_numpy("At memcpy device data", host_buffer, args.memory_size, \
            ACL_MEMCPY_DEVICE_TO_HOST):
                continue
            # 查询循环周期时的Device内存，为方便查看，增加sleep函数
            release_cycle_time += 1
            sleep(1)
            if search_memory(cycles_time, release_cycle_time):
                exit(1)
        # 一个释放周期循环完毕后，进行内存释放
        for host_buffer in host_mem_list:
            ret = acl.rt.free_host(host_buffer)
            if check_ret("acl.rt.free_host", ret):
                continue
        for device_buffer in dev_mem_list:
            ret = acl.media.dvpp_free(device_buffer)
            if check_ret("acl.media.dvpp_free", ret):
                continue
        # 调用内存查询接口，查询divice内存
        cycles_time += 1
        sleep(1)
        if search_memory(cycles_time, release_cycle_time):
            exit(1)

    # 释放acl资源
    ret = acl.rt.destroy_context(context)
    if ret != ACL_SUCCESS:
        logger.error("acl.rt.destroy_context ret = {}".format(ret))
        exit(1)
    ret = acl.rt.reset_device(args.device_id)
    if ret != ACL_SUCCESS:
        logger.error("acl.rt.reset_device ret = {}".format(ret))
        exit(1)
    ret = acl.finalize()
    if ret != ACL_SUCCESS:
        logger.error("acl.finalize failed ret = {}".format(ret))
        exit(1)
    print("run success!")

if __name__ == '__main__':
    main()