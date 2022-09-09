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
#include "sample_comm.h"
#include <vector>

using namespace std;

/******************************************************************************
* function    : main()
* Description : pngd ddk sample
******************************************************************************/
int32_t main(int32_t argc, char *argv[])
{
    int32_t ret;
    pthread_t pngdSendThreadTid[PNGD_MAX_CHN_NUM] = {0};
    pthread_t pngdGetThreadTid[PNGD_MAX_CHN_NUM] = {0};

    if (argc < 4) {
        SAMPLE_PRT("\nInput parameter's num:%d is not enough!\n", argc);
        pngd_usage(argv[0]);
        return HI_FAILURE;
    }

    ret = get_option(argc, &(*argv));
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("get_option failed!\n");
        return HI_FAILURE;
    }

    ret = check_option();
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("check_option failed!\n");
        return HI_FAILURE;
    }

    print_arguement();

    ret = setup_acl_device();
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("Setup Device failed! ret code:%#x\n", ret);
        return HI_FAILURE;
    }

    ret = hi_mpi_sys_init();
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_sys_init failed!\n");
        return HI_FAILURE;
    }

    ret = pngd_create();
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("start PNGD fail for %#x!\n", ret);
        int32_t exit_ret = deinit_pngd_mod();
        if (exit_ret != HI_SUCCESS) {
            SAMPLE_PRT("exit PNGD fail for %#x!\n", exit_ret);
        }
        return ret;
    }

    pngd_init_start_time();

    ret = pngd_start_send_stream(&pngdSendThreadTid[0]);
    if (ret != 0) {
        pngd_stop_send_stream();
    } else {
        ret = pngd_start_get_pic(&pngdGetThreadTid[0]);
        if (ret != 0) {
            pngd_stop_send_stream();
            pngd_stop_get_pic();
        }
    }

    pngd_cmd_ctrl(&pngdSendThreadTid[0], &pngdGetThreadTid[0]);

    pngd_show_decode_state();

    ret = deinit_pngd_mod();
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("exit PNGD fail for %#x!\n", ret);
    }

    return ret;
}

