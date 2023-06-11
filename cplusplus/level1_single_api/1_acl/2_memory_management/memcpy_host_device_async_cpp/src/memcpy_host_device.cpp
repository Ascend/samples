/**
 *  Copyright [2021] Huawei Technologies Co., Ltd
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License. 
 */
#include "memcpy_host_device.h"
#include "acl/acl_rt.h"
using namespace std;
const int32_t argcNum = 4;
// 检查入参
void memcpyh2d_usage(char *sPrgNm)
{
    aclError aclRet = aclrtGetRunMode(&g_run_mode);
    if (aclRet == ACL_SUCCESS) {
        if (g_run_mode == ACL_HOST) {
            SAMPLE_PRT(" Running in Host!\n");
        } else if (g_run_mode == ACL_DEVICE) {
            SAMPLE_PRT(" Running in Device!\n");
        } else {
            SAMPLE_PRT(" Running in Invalid platform! runMode:%u\n", g_run_mode);
            return;
        }
    } else {
        SAMPLE_PRT(" Get run mode fail! acl ret:%#x\n", aclRet);
        return;
    }

    SAMPLE_PRT("\n/*********************************************************/\n");
    SAMPLE_PRT("Usage :\n");
    SAMPLE_PRT("\t example: ./main --release_cycle -1\
    --number_of_cycles 1 --device_id 0 --memory_size 10485760\
    --write_back_host 0 --memory_reuse 1\n");
    SAMPLE_PRT("\t\n");
    SAMPLE_PRT("\t  --release_cycle: Specify the release period.\n");
    SAMPLE_PRT("\t  --number_of_cycles: The number of overall process cycles.\n");
    SAMPLE_PRT("\t  --device_id: The ID of device currently in use.\n");
    SAMPLE_PRT("\t  --memory_size: Single request memory size, The units are Byte\n");
    SAMPLE_PRT("\t  --memory_reuse: Whether to reuse memory\n");
    SAMPLE_PRT("\t  --write_back_host: Whether to send back the host\n");
    SAMPLE_PRT("\n/*********************************************************/\n\n");
}

// 获取参数
int32_t get_option(int32_t argc, char **argv)
{
    int32_t c = 0;
    int32_t option_index = 0;
    int32_t ret = 0;
    struct option long_options[] =
    {
        {"release_cycle",       1, nullptr, 'r'},
        {"number_of_cycles",    1, nullptr, 'n'},
        {"device_id",           1, nullptr, 'd'},
        {"memory_size",         1, nullptr, 's'},
        {nullptr,               0, nullptr, 0}
    };

    while (1) {
        option_index = 0;
        c = getopt_long(argc, argv, "r:n:d:s:", long_options, &option_index);
        if (c == -1) {
            break;
        }
        switch (c) {
            case 'r':
                g_release_cycle = atoi(optarg);
                break;
            case 'n':
                g_number_of_cycles = atoi(optarg);
                break;
            case 'd':
                g_device_id = atoi(optarg);
                break;
            case 's':
                g_memory_size = atoi(optarg);
                break;
            default:
                SAMPLE_PRT("unsupport option!\n");
                break;
        }
    }

    aclError aclRet = aclrtGetRunMode(&g_run_mode);
    if (aclRet == ACL_SUCCESS) {
        if (g_run_mode == ACL_HOST) {
            SAMPLE_PRT(" Running in Host!\n");
        } else if (g_run_mode == ACL_DEVICE) {
            SAMPLE_PRT(" Running in Device!\n");
        } else {
            SAMPLE_PRT(" Running in Invalid platform! runMode:%u\n", g_run_mode);
            return FAILED;
        }
    } else {
        SAMPLE_PRT(" Get run mode fail! acl ret:%#x\n", aclRet);
        return FAILED;
    }
    SAMPLE_PRT("\n/*********************************************************/\n");
    SAMPLE_PRT("\nUsing params are as follows.\n");
    SAMPLE_PRT("g_release_cycle: %u \n", g_release_cycle);
    SAMPLE_PRT("g_number_of_cycles: %u \n", g_number_of_cycles);
    SAMPLE_PRT("g_device_id: %u \n", g_device_id);
    SAMPLE_PRT("g_memory_size: %zu \n", g_memory_size);
    return SUCCESS;
}

int32_t setup_acl_device()
{
    aclError aclRet = aclInit(nullptr);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("aclInit fail with %d.\n", aclRet);
        return aclRet;
    }
    SAMPLE_PRT("aclInit succ.\n");

    aclRet = aclrtSetDevice(g_device_id);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("aclrtSetDevice %u fail with %d.\n", g_device_id, aclRet);
        aclFinalize();
        return aclRet;
    }
    SAMPLE_PRT("aclrtSetDevice(%u) succ.\n", g_device_id);

    aclRet = aclrtCreateContext(&g_context, g_device_id);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("acl create context failed with %d.\n", aclRet);
        aclrtResetDevice(g_device_id);
        aclFinalize();
        return aclRet;
    }
    SAMPLE_PRT("create context success\n");

    aclRet = aclrtGetCurrentContext(&g_context);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("get current context failed\n");
        aclrtDestroyContext(g_context);
        g_context = nullptr;
        aclrtResetDevice(g_device_id);
        aclFinalize();
        return aclRet;
    }
    SAMPLE_PRT("get current context success\n");
    aclRet = aclrtCreateStream(&stream);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("create stream is failed\n");
        aclrtDestroyStream(stream);
        stream = nullptr;
        aclrtDestroyContext(g_context);
        g_context = nullptr;
        aclrtResetDevice(g_device_id);
        aclFinalize();
        return aclRet;
    }
    return SUCCESS;
}

void print_data(string message, void *data, size_t size, aclrtMemcpyKind kind)
{
    void *host_data = nullptr;
    aclError aclRet = aclrtMallocHost(&host_data, size);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("aclrtMallocHost failed\n");
        return;
    }
    aclRet = aclrtMemcpy(host_data, size, data, size, kind);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("aclrtMemcpy failed\n");
        return;
    }
    SAMPLE_PRT("\n%s is %x\n", message.c_str(), *(unsigned char*)host_data);
    return;
}

int32_t search_memory(uint32_t cycles_time, uint32_t release_cycle_time)
{
    size_t free = 0;
    size_t total = 0;
    aclError aclRet = aclrtGetMemInfo(ACL_DDR_MEM, &free, &total);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("aclrtGetMemInfo failed\n");
        return aclRet;
    }
    SAMPLE_PRT("At number_of_cycles = %u, release_cycle = %u, DDR free memory:%zu Byte,   \
               DDR total memory:%zu Byte.\n", cycles_time, release_cycle_time, free, total);
    aclRet = aclrtGetMemInfo(ACL_HBM_MEM, &free, &total);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("aclrtGetMemInfo failed\n");
        return aclRet;
    }
    SAMPLE_PRT("At number_of_cycles = %u, release_cycle = %u, HBM free memory:%zu Byte,   \
               HBM total memory:%zu Byte.\n", cycles_time, release_cycle_time, free, total);
    return SUCCESS;
}

void destroy_acl_device()
{
    if (g_context) {
        aclrtDestroyContext(g_context);
        g_context = nullptr;
        aclrtResetDevice(g_device_id);
        aclFinalize();
    }
}

int32_t main(int32_t argc, char *argv[])
{
    int ret = SUCCESS;
    // 检查参数个数
    if (argc < argcNum) {
        SAMPLE_PRT("\nInput parameter's num:%d is not enough!\n", argc);
        memcpyh2d_usage(argv[0]);
        return FAILED;
    }
    // 获取入参
    ret = get_option(argc, &(*argv));
    if (ret != SUCCESS) {
        SAMPLE_PRT("get_option failed!\n");
        return FAILED;
    }
    // acl资源初始化
    ret = setup_acl_device();
    if (ret != SUCCESS) {
        SAMPLE_PRT("Setup Device failed! ret code:%#x\n", ret);
        return FAILED;
    }
    // 内存使用前，首次查询Device内存
    ret = search_memory(0, 0);
    if (ret != SUCCESS) {
        SAMPLE_PRT("search_memory failed! ret code:%#x\n", ret);
        return FAILED;
    }
    // 根据循环次数设置循环
    uint32_t cycles_time = 0;
    aclError aclRet = ACL_SUCCESS;
    aclError aclRet_ = ACL_SUCCESS;
    while (g_number_of_cycles - cycles_time != 0)
    {
        // 设置内存list，方便后续释放
        void* host_buffer = nullptr;
        void* device_buffer = nullptr;
        vector<void*> dev_mem_list;

        // 根据释放周期设置循环
        uint32_t release_cycle_time = 0;
        while (g_release_cycle - release_cycle_time != 0)
        {
            vector<void*> host_mem_list;
            // 申请host内存
            aclRet = aclrtMallocHost(&host_buffer, g_memory_size);
            if (aclRet != ACL_SUCCESS) {
                SAMPLE_PRT("aclrtMallocHost failed\n");
                continue;
            }
            host_mem_list.push_back(host_buffer);
            // 初始化内存，构建全7的数据，只是下发任务到stream并没有执行
            aclRet = aclrtMemsetAsync (host_buffer, g_memory_size, 7, g_memory_size, stream);
            if (aclRet != ACL_SUCCESS) {
                SAMPLE_PRT("aclrtMemsetAsync failed\n");
                continue;
            }
            // 申请device内存
            aclRet_ = aclrtMalloc(&device_buffer, g_memory_size, ACL_MEM_MALLOC_HUGE_FIRST);
            if (aclRet_ != ACL_SUCCESS) {
                SAMPLE_PRT("aclrtMalloc failed\n");
                continue;
            }
            dev_mem_list.push_back(device_buffer);

			// 执行stream初始化内存任务
            aclRet = aclrtSynchronizeStream(stream);
            if (aclRet != ACL_SUCCESS) {
                SAMPLE_PRT("aclrtSynchronizeStream failed\n");
                continue;
            }
            // 查看初始化后的内存
            print_data("At first memsetAsync host data", host_buffer, g_memory_size, ACL_MEMCPY_HOST_TO_HOST);
            // 内存拷贝H2D, 只是下发任务到stream并没有执行
            aclRet = aclrtMemcpyAsync(device_buffer, g_memory_size, host_buffer, g_memory_size,
                                      ACL_MEMCPY_HOST_TO_DEVICE, stream);
            if (aclRet != ACL_SUCCESS) {
                SAMPLE_PRT("aclrtMemcpyAsync failed\n");
                continue;
            }
            // 申请回传所需的Host内存
            void* write_back_buffer = nullptr;
            aclRet_ = aclrtMallocHost(&write_back_buffer, g_memory_size);
            if (aclRet_ != ACL_SUCCESS) {
                SAMPLE_PRT("aclrtMallocHost failed\n");
                continue;
            }
            ret = search_memory(cycles_time, release_cycle_time);
            if (ret != SUCCESS) {
                SAMPLE_PRT("search_memory failed! ret code:%#x\n", ret);
                continue;
            }
			// 执行stream内存拷贝H2D任务
            aclRet = aclrtSynchronizeStream(stream);
            if (aclRet != ACL_SUCCESS) {
                SAMPLE_PRT("aclrtSynchronizeStream failed\n");
                continue;
            }
            // 查看device上的数据
            print_data("At memcpy device data", device_buffer, g_memory_size, ACL_MEMCPY_DEVICE_TO_HOST);
            // 内存拷贝D2H，只是下发任务到stream并没有执行
            aclRet_ = aclrtMemcpyAsync(write_back_buffer, g_memory_size, device_buffer,
                                       g_memory_size, ACL_MEMCPY_DEVICE_TO_HOST, stream);
            if (aclRet_ != ACL_SUCCESS) {
                SAMPLE_PRT("aclrtMemcpyAsync failed\n");
                continue;
            }

            // host内存释放
            for (auto host_buffer : host_mem_list) {
                aclError aclRet = aclrtFreeHost(host_buffer);
                if (aclRet != ACL_SUCCESS) {
                    SAMPLE_PRT("aclrtFreeHost failed\n");
                    continue;
                }
            }

            // 执行stream内存拷贝D2H任务
            aclRet = aclrtSynchronizeStream(stream);
            if (aclRet != ACL_SUCCESS) {
                SAMPLE_PRT("aclrtSynchronizeStream failed\n");
                continue;
            }
            // 查看回传后的内存
            print_data("At write back memcpy host data", write_back_buffer, g_memory_size, ACL_MEMCPY_HOST_TO_HOST);
            // 回传所申请的Host内存单独释放
            aclError aclRet = aclrtFreeHost(write_back_buffer);
            if (aclRet != ACL_SUCCESS) {
                SAMPLE_PRT("aclrtFreeHost failed\n");
                continue;
            }
            release_cycle_time += 1;
            ret = search_memory(cycles_time, release_cycle_time);
            if (ret != SUCCESS) {
                SAMPLE_PRT("search_memory failed! ret code:%#x\n", ret);
                continue;
            }
        }

        for (auto device_buffer : dev_mem_list) {
            aclError aclRet = aclrtFree(device_buffer);
            if (aclRet != ACL_SUCCESS) {
                SAMPLE_PRT("aclrtFree failed\n");
                continue;
            }
        }
        // 调用内存查询接口，查询divice内存
        cycles_time += 1;
        ret = search_memory(cycles_time, release_cycle_time);
        if (ret != SUCCESS) {
            SAMPLE_PRT("search_memory failed! ret code:%#x\n", ret);
            return FAILED;
        }
    }
    // 释放acl资源
    destroy_acl_device();
    SAMPLE_PRT("run success!\n");
    return SUCCESS;
}