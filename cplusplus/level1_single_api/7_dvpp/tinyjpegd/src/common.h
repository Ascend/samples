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

typedef struct hiVDEC_THREAD_PARAM_S {
    uint8_t *perfStreamBuf;
    int32_t s32ChnId;
    int32_t s32StreamMode;
    int32_t s32MilliSec;
    int32_t s32MinBufSize;
    int32_t s32IntervalTime;
    uint32_t u32StmLenInJpegd; // JPEGD未解码码流长度,用于反压帧率
    uint32_t u32PicInJpegd;    // JPEGD已解码未取出图片数,用于反压帧率
    uint64_t u64PtsInit;
    uint64_t u64PtsIncrease;
    hi_payload_type type;
    hi_pixel_format enPixFormat;
    THREAD_CONTRL_E enSendThreadCtrl;
    THREAD_CONTRL_E enGetThreadCtrl;
} VDEC_THREAD_PARAM_S;

typedef struct hiSAMPLE_VDEC_VIDEO_ATTR {
    uint32_t           ref_frame_num;
    hi_video_dec_mode  dec_mode;
    hi_data_bit_width  bit_width;
} SAMPLE_VDEC_VIDEO_ATTR;

typedef struct hiSAMPLE_VDEC_PICTURE_ATTR {
    uint32_t        alpha;
    hi_pixel_format pixel_format;
} SAMPLE_VDEC_PICTURE_ATTR;

typedef struct hiSAMPLE_VDEC_ATTR {
    hi_payload_type   type;
    hi_vdec_send_mode mode;
    uint32_t width;
    uint32_t height;
    uint32_t frame_buf_cnt;
    uint32_t display_frame_num;
    union {
        SAMPLE_VDEC_VIDEO_ATTR video_attr;      /* structure with video ( h265/h264) */
        SAMPLE_VDEC_PICTURE_ATTR picture_attr; /* structure with picture (jpeg/mjpeg) */
    };
} SAMPLE_VDEC_ATTR;

/*******************************************************
    function announce
*******************************************************/
void jpegd_handle_signal(int32_t signo);
int32_t set_param();

void jpegd_cmd_ctrl(pthread_t *pJpegdSendTid, pthread_t *pJpegdGetTid);
int32_t jpegd_start_send_stream(pthread_t *pJpegdSendTid);
void jpegd_stop_send_stream();
int32_t jpegd_start_get_pic(pthread_t *pJpegdGetTid);
void jpegd_stop_get_pic();
int32_t jpegd_create();
int32_t jpegd_destroy();
void jpegd_show_decode_state();
int32_t setup_acl_device();
void destroy_acl_device();
int32_t destory_resource_jpegd_mod();

#endif /* End of #ifndef __SAMPLE_COMMON_H__ */
