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

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/time.h>
#include <fcntl.h>
#include <cerrno>
#include <pthread.h>
#include <cmath>
#include <unistd.h>
#include <csignal>
#include "common.h"
#include <vector>

using namespace std;

int32_t main(int32_t argc, char *argv[])
{
    int32_t s32Ret = HI_SUCCESS;
    pthread_t jpegdSendThreadTid[VDEC_MAX_CHN_NUM] = {0};
    pthread_t jpegdGetThreadTid[VDEC_MAX_CHN_NUM] = {0};

    signal(SIGINT, jpegd_handle_signal);
    signal(SIGTERM, jpegd_handle_signal);

    //获取输入参数
    s32Ret = set_param();
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("set_param failed!\n");
        return HI_FAILURE;
    }

    //管理资源申请
    s32Ret = setup_acl_device();
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("Setup Device failed! ret code:%#x\n", s32Ret);
        return HI_FAILURE;
    }

    //初始化himpi
    s32Ret = hi_mpi_sys_init();
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_sys_init failed!\n");
        return HI_FAILURE;
    }

    //调用himpi接口创建通道并在启动解码之前，使用解码器开始接收用户发送的码流。
    s32Ret = jpegd_create();
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("start JPEGD fail for %#x!\n", s32Ret);
        int32_t exitRet = destory_resource_jpegd_mod();
        if (exitRet != HI_SUCCESS) {
            SAMPLE_PRT("exit JPEGD fail for %#x!\n", exitRet);
        }
        return s32Ret;
    }

    //开始发送码流
    s32Ret = jpegd_start_send_stream(&jpegdSendThreadTid[0]);
    if (s32Ret != 0) {
        jpegd_stop_send_stream();
    } else {
        s32Ret = jpegd_start_get_pic(&jpegdGetThreadTid[0]);
        if (s32Ret != 0) {
            jpegd_stop_send_stream();
            jpegd_stop_get_pic();
        }
    }

    //等待线程结束
    jpegd_cmd_ctrl(&jpegdSendThreadTid[0], &jpegdGetThreadTid[0]);

    // 打印各通道fps
    jpegd_show_decode_state();

    //去初始化
    s32Ret = destory_resource_jpegd_mod();
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("exit JPEGD fail for %#x!\n", s32Ret);
    }

    return s32Ret;
}

