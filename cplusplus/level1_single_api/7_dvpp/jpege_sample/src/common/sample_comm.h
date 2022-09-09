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

#include <pthread.h>
#include "hi_dvpp.h"
#include "acl.h"
#include "acl_rt.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

/*******************************************************
    macro define
*******************************************************/
#define HI_MAXINUM_LIMIT            100000
#define MAX_ALIGN                   1024
#define SEG_CMP_LENGTH              256
#define FILE_NAME_LEN               128
#define MAX_JPEGE_CHN               256
#define USER_DATA_LEN               128
#define VENC_MAX_CHN_NUM            256
#define MAX_MBUF_DATA_LEN           (4*1024*1024)
#define JPEGE_ALIGN                 128

#define DIV_UP(x, a)     ( ((x) + ((a) - 1) ) / a )
#define ALIGN_DOWN(x, a) ( ( (x) / (a)) * (a) )

#define CHECK_CHN_RET(express, chn, name)\
    do{\
        int32_t ret;\
        ret = express;\
        if (HI_SUCCESS != ret)\
        {\
            printf("\033[0;31m%s chn %d failed at %s: LINE: %d with %#x!\033[0;39m\n",\
                name, chn, __FUNCTION__, __LINE__, ret);\
            fflush(stdout);\
            return ret;\
        }\
    }while(0)

#define CHECK_RET(express, name)\
    do{\
        int32_t ret;\
        ret = express;\
        if (ret != HI_SUCCESS)\
        {\
            printf("\033[0;31m%s failed at %s: LINE: %d with %#x!\033[0;39m\n", name, __FUNCTION__, __LINE__, ret);\
            return ret;\
        }\
    }while(0)

#define PAUSE()  do {\
        printf("---------------press Enter to exit!---------------\n");\
        getchar();\
    } while (0)

#define SAMPLE_PRT(fmt...)   \
    do {\
        printf("[%s]-%d: ", __FUNCTION__, __LINE__);\
        printf(fmt);\
    }while(0)

#define CHECK_NULL_PTR(ptr)\
    do{\
        if(ptr == NULL)\
        {\
            printf("func:%s,line:%d, NULL pointer\n",__FUNCTION__,__LINE__);\
            return HI_FAILURE;\
        }\
    }while(0)

#define SAMPLE_GET_TIMESTAMP_US(val)                                        \
    do                                                                    \
    {                                                                     \
        struct timeval timeVal;                                         \
        gettimeofday(&timeVal, NULL);                                   \
        val = (hi_u64)timeVal.tv_sec * 1000000 + timeVal.tv_usec;     \
    } while (0)

/*******************************************************
    structure define
*******************************************************/
typedef struct {
    hi_bool  threadStart;
    hi_venc_chn vencChn[MAX_JPEGE_CHN]; // chnl_id in single-thread multi-channel mode
    hi_venc_chn chnlId; // chnl_id in single-thread and single-channel mode
    int32_t   chnlCnt;
    int32_t   supportPerformance;
    uint32_t   orSave;
} HiSampleJpegeGetStreamPara;

typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t align;
    hi_pixel_format pixelFormat;
    hi_data_bit_width bitWidth;
    hi_compress_mode cmpMode;
    hi_char inputFileName[64];
} HiSampleVencSendFrameInput;

typedef struct {
    hi_bool threadStart;
    hi_venc_chn vencChn;
    int32_t  chnlCnt;
    int32_t  supportPerformance;
    HiSampleVencSendFrameInput inputPara;
} HiSampleJpegeSendFramePara;

typedef struct {
    uint32_t vbSize;

    uint32_t headStride;
    uint32_t headSize;
    uint32_t headYSize;

    uint32_t mainStride;
    uint32_t mainSize;
    uint32_t mainYSize;

    uint32_t exStride;
    uint32_t exYSize;
} HiSampleCalConfig;

typedef struct {
    uint32_t totalLen;
    uint32_t dataLen;
    void *dataBlock;
    void *data;
    char userData[USER_DATA_LEN];
} HiDvppBuf;
/*******************************************************
    function announce
*******************************************************/
void* jpege_snap_send_frame(void* p);
void* jpege_snap_send_frame_dc_performance(void* p);
int32_t jpege_snap_start(hi_venc_chn vencChn, hi_video_size* size, hi_bool supportDc, uint32_t level);
void* jpege_snap_process_epoll(void* p);
void* jpege_snap_process(void* p);
int32_t jpege_snap_stop(hi_venc_chn vencChn[], int32_t cnt);
int32_t jpege_start_sync_enc(hi_venc_chn VencChn[], int32_t s32Cnt, int32_t per, HiSampleVencSendFrameInput* inputPara);
int32_t jpege_start_get_stream(hi_venc_chn vencChn[], int32_t cnt, int32_t per, uint32_t save);
int32_t jpege_start_send_frame(hi_venc_chn VencChn[], int32_t s32Cnt, int32_t per,
    HiSampleVencSendFrameInput* inputPara);
int32_t jpege_stop_get_stream(int32_t cnt);
int32_t jpege_stop_send_frame(int32_t s32Cnt);
void get_pic_buffer_config(uint32_t inputWidth, uint32_t inputHeight, hi_pixel_format pixelFormat,
    hi_data_bit_width inputBitWidth, hi_compress_mode cmpMode, uint32_t align, HiSampleCalConfig *calConfig);
uint32_t get_pic_buffer_size(uint32_t inputWidth, uint32_t inputHeight, hi_pixel_format pixelFormat,
    hi_data_bit_width inputBitWidth, hi_compress_mode cmpMode, uint32_t align);
int32_t wait_encoder_complete(uint32_t chnNumStart, int32_t chnNum);
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

#endif /* End of #ifndef __SAMPLE_COMMON_H__ */
