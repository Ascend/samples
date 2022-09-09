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

#include <cstdint>
#include <getopt.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <signal.h>
#include <vector>

#include "Vdec.h"

int32_t main(int32_t argc, char *argv[])
{
    int32_t ret = HI_SUCCESS;

    // Parse input parameters
    ret = get_option(argc, &(*argv));
    if (ret != HI_SUCCESS) {
        printf("[%s][%d] get_option Fail \n", __FUNCTION__, __LINE__);
        return 0;
    }

    // Check input parameters
    ret = check_option();
    if (ret != HI_SUCCESS) {
        printf("[%s][%d] check_option Fail \n", __FUNCTION__, __LINE__);
        return 0;
    }

    // Print input parameters
    print_parameter();

    // Dvpp system init
    ret = hi_dvpp_init();
    if (ret != HI_SUCCESS) {
        printf("[%s][%d] hi_dvpp_init failed!\n", __FUNCTION__, __LINE__);
        return 0;
    }

    // Create video decoder
    ret = vdec_create();
    if (ret != HI_SUCCESS) {
        printf("[%s][%d] VdecStart failed!\n", __FUNCTION__, __LINE__);
        // Destroy video decoder
        vdec_destroy();
        // Dvpp system exit
        hi_dvpp_deinit();
        return 0;
    }
    // Lock init
    init_outbuffer_lock();

    // Record start time
    get_start_time();

    // Create threads for sending stream
    ret = create_send_stream_thread();
    if (ret != 0) {
        // If create thread fail, stop all send stream thread
        stop_send_stream_thread();
    } else {
        // Create threads for getting result
        ret = create_get_pic_thread();
        if (ret != 0) {
            // If create thread fail, stop all get pic thread
            stop_get_pic_thread();
        }
    }

    // Wait decoding is complete.
    wait_vdec_end();
    // Print performance data
    show_decode_performance();
    // Destroy init
    destroy_outbuffer_lock();
    // Destroy video decoder
    vdec_destroy();
    // Dvpp system exit
    hi_dvpp_deinit();
    return 0;
}