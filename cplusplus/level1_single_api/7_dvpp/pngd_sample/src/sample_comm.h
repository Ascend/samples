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

#ifndef __SAMPLE_COMM_H__
#define __SAMPLE_COMM_H__
#include <cstdint>
#include <pthread.h>

#include "hi_dvpp.h"

/*******************************************************
    macro define
*******************************************************/
const uint32_t FILE_NAME_LEN = 128;

const uint64_t MULTIPLE_S_TO_US        = 1000000;
const uint32_t PNGD_DECODED_QUEUE_NUM = 10;

#define SAMPLE_PRT(fmt...)   \
    do { \
        printf("[%s]-%d: ", __FUNCTION__, __LINE__); \
        printf(fmt); \
    } while (0)

#define CHECK_NULL_PTR(ptr) \
    do { \
        if(NULL == ptr) \
        { \
            SAMPLE_PRT(" NULL pointer\n"); \
            return HI_FAILURE; \
        } \
    } while (0)

#define CHECK_CHN_RET(express, Chn, name) \
    do { \
        int32_t Ret; \
        Ret = express; \
        if (HI_SUCCESS != Ret) \
        { \
            SAMPLE_PRT(" %s chn %u failed with %#x!\n", name, Chn, Ret); \
            fflush(stdout); \
            return Ret; \
        } \
    } while (0)

/*******************************************************
    structure define
*******************************************************/
typedef enum hiTHREAD_CONTRL_E {
    THREAD_CTRL_INIT,
    THREAD_CTRL_START,
    THREAD_CTRL_PAUSE,
    THREAD_CTRL_STOP,
} THREAD_CONTRL_E;

typedef struct hiPNGD_THREAD_PARAM_S {
    char cFileName[FILE_NAME_LEN];
    bool bWholeDirDecode;
    uint8_t *perfStreamBuf;
    int32_t s32ChnId;
    int32_t s32StreamMode;
    int32_t s32MilliSec;
    int32_t s32IntervalTime;
    int32_t s32CircleSend;

    volatile uint32_t u32SendSucc;
    volatile uint32_t u32PicInPngd;
    volatile uint32_t u32Decoded;
    volatile uint32_t u32DecodeFail;
    volatile uint32_t u32DecodeSucc;

    uint64_t u64PtsInit;
    hi_pixel_format enPixFormat;
    THREAD_CONTRL_E enSendThreadCtrl;
    THREAD_CONTRL_E enGetThreadCtrl;
} PNGD_THREAD_PARAM_S;

/*******************************************************
    function announce
*******************************************************/
void pngd_init_start_time();
void pngd_usage(char *sPrgNm);
int32_t get_option(int32_t argc, char **argv);
int32_t check_option();
void print_arguement();

void pngd_cmd_ctrl(pthread_t *pJpegdSendTid, pthread_t *pJpegdGetTid);
int32_t pngd_start_send_stream(pthread_t *pJpegdSendTid);
void pngd_stop_send_stream();
int32_t pngd_start_get_pic(pthread_t *pJpegdGetTid);
void pngd_stop_get_pic();
int32_t pngd_create();
int32_t pngd_destroy();
void pngd_show_decode_state();
int32_t setup_acl_device();
int32_t deinit_pngd_mod();

#endif /* End of #ifndef __SAMPLE_COMMON_H__ */
