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

#ifndef SAMPLE_COMM_H
#define SAMPLE_COMM_H
#include "sample_api.h"

#ifdef HMEV_PLATFORM_SDK

#define MAX_ENC_CHANNEL_NUM 256
#define CALC_YUV420_SIZE 3 / 2
#define US_PER_SEC 1000000
#define CALC_K 1024
#define DEFAULT_FRAME_RATE 30
#define FIX_QP 30
#define INPUT_FRAME_LEN 5

#define FRAME_SIZE_360P  (640 * 360)
#define FRAME_SIZE_720P  (1280 * 720)
#define FRAME_SIZE_1080P (1920 * 1080)
#define FRAME_SIZE_3K    (2880 * 1856)
#define FRAME_SIZE_4K    (3840 * 2160)

// save codec data flag
extern bool g_save_codec_out_data;
extern int32_t g_tag_prt_level;

enum PrintLevel {
    VERBOSE = 0,
    DEBUG = 1,
    INFO = 2,
    WARN = 3,
    ERROR = 4,
};

#define HMEV_GET_TIMESTAMP_US(val)                                        \
    do                                                                    \
    {                                                                     \
        struct timeval stTimeVal;                                         \
        gettimeofday(&stTimeVal, NULL);                                   \
        val = (hi_u64)stTimeVal.tv_sec * 1000000 + stTimeVal.tv_usec;     \
    } while (0)

#define HMEV_HISDK_CHECK_RET_EXPRESS(express, name) \
    do \
    {  \
        int32_t checkRet; \
        checkRet = express; \
        if (HMEV_SUCCESS != checkRet) \
        { \
            printf("\033[0;31m%s failed at %s: LINE: %d with %#x!\033[0;39m\n", \
                name, __FUNCTION__, __LINE__, checkRet); \
            return checkRet; \
        } \
    } while (0)

#define HMEV_HISDK_CHECK_RET(express, ret, name)                                                                    \
    do                                                                                                              \
    {                                                                                                               \
        if (true == (express))                                                                                 \
        {                                                                                                           \
            printf("\033[0;31m%s failed at %s: LINE: %d with %#x!\033[0;39m\n", name, __FUNCTION__, __LINE__, ret); \
            return ret;                                                                                             \
        }                                                                                                           \
    } while (0)

#define HMEV_HISDK_PRT(level, fmt...)                                                 \
    do                                                                                \
    {                                                                                 \
        if (level >= g_tag_prt_level)                                                 \
        {                                                                             \
            printf("level\\%d [%s]-%d: ", level, __FUNCTION__, __LINE__); \
            printf(fmt);                                                              \
            printf("\n");                                                             \
        }                                                                             \
    } while (0)

#define CHECK_NULL_PTR(ptr)                                                    \
    do                                                                         \
    {                                                                          \
        if (NULL == ptr)                                                       \
        {                                                                      \
            printf("func:%s,line:%d, NULL pointer\n", __FUNCTION__, __LINE__); \
            return HMEV_FAILURE;                                               \
        }                                                                      \
    } while (0)

#define HMEV_GET_SYSTIME_US(u64USec)                                   \
    do                                                                 \
    {                                                                  \
        struct timespec spec;                                          \
        clock_gettime(CLOCK_MONOTONIC, &spec);                         \
        u64USec = (hi_u64)spec.tv_sec * 1000000 + spec.tv_nsec / 1000; \
    } while (0)

#define _T_HMEV_HISDK(val) #val

#define HMEV_HISDK_CHECK_PTR_RET(x, lRet)                        \
    {                                                               \
        if (NULL == (x))                                            \
        {                                                           \
            HMEV_HISDK_PRT(ERROR, "%s is NULL!", _T_HMEV_HISDK(x)); \
            return (lRet);                                          \
        }                                                           \
        else                                                        \
        {                                                           \
        }                                                           \
    }

#define HMEV_HISDK_CHECK_PTR_RET_NONE(x)                         \
    {                                                               \
        if (NULL == (x))                                            \
        {                                                           \
            HMEV_HISDK_PRT(ERROR, "%s is NULL!", _T_HMEV_HISDK(x)); \
            return;                                                 \
        }                                                           \
        else                                                        \
        {                                                           \
        }                                                           \
    }

typedef struct {
    pthread_t sendThread;
    hi_bool threadStart;
    hi_venc_chn chnId;
    hi_s32 chnlCnt;
    FrameSize frameSize;
    hi_s32 frameNum;
    std::string fileName;
    hi_s32 intervalTime;
    hi_bool circleSend;
} VencSendParams;

#endif

#endif // SAMPLE_COMM_H
