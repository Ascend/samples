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

#include "sample_api.h"
#include "sample_debug.h"
#include "sample_encoder_manage.h"
#include <getopt.h>

#ifdef HMEV_PLATFORM_SDK

int32_t g_tag_prt_level = VERBOSE;
bool g_save_codec_out_data = true;
int32_t g_codec_type = 0;
int32_t g_chn_num = 0;
int32_t g_img_width = 0;
int32_t g_img_height = 0;
int32_t g_img_stride = 0;
int32_t g_high_priority = 0;
int32_t g_pixel_format = 0;
char g_input_file[500] = "";
char g_output_file[500] = "";
int32_t g_bit_rate = 0;
int32_t g_frame_rate = 30;
int32_t g_max_qp = 51;
int32_t g_min_qp = 20;
int32_t g_max_iqp = 51;
int32_t g_min_iqp = 20;
int32_t g_profile = 0;
int32_t g_perf_test = 0;
int32_t g_perf_frame_num = 300;
int32_t g_start_chnl = 0; // specify the start channel id for current process in multi-process test
int32_t g_frame_gop = 65536;
bool g_read_at_once = false;

hi_u64 g_start_send_time[MAX_ENC_CHANNEL_NUM] = {0};
hi_u64 g_end_get_time[MAX_ENC_CHANNEL_NUM] = {0};
static hi_u64 g_send_frame_cnt[MAX_ENC_CHANNEL_NUM] = {0};
static hi_u64 g_get_stream_cnt[MAX_ENC_CHANNEL_NUM] = {0};
static FILE* g_save_file[MAX_ENC_CHANNEL_NUM];
RunMode g_run_mode = MODE_DEVICE;

std::map<hi_u64, hi_void*> g_mbuf_map;
pthread_mutex_t g_mbuf_mutex;

aclrtContext g_context = NULL;

// save the output stream
hi_s32 venc_save_stream(FILE* fp, VencOutStream* outStream, uint64_t packCount)
{
    hi_s32 i;
    HMEV_HISDK_PRT(DEBUG, "save packs cnt %lu", packCount);
    for (i = 0; i < packCount; i++) {
        fwrite(outStream[i].pcData, outStream[i].dataLen, 1, fp);
        fflush(fp);
    }

    return HI_SUCCESS;
}

// read the input YUV file
int32_t venc_read_yuv_file(FILE* fpYuv, hi_video_frame_info* videoFrameInfo, hi_s64* fileOffset)
{
    CHECK_NULL_PTR(fpYuv);
    CHECK_NULL_PTR(videoFrameInfo);
    hi_s64 usedBytes = *fileOffset;
    hi_s64 readLen = 0;
    hi_u32 h;
    char* bufVirtY = NULL;
    char* bufVirtC = NULL;
    char* memContent = NULL;
    hi_char* hostAddr = NULL;
    aclError aclRet = ACL_SUCCESS;
    hi_u32 bufSize = videoFrameInfo->v_frame.width_stride[0] * videoFrameInfo->v_frame.height * CALC_YUV420_SIZE;

    if (g_read_at_once == true) {
        bufSize = videoFrameInfo->v_frame.width * videoFrameInfo->v_frame.height * CALC_YUV420_SIZE;
    }

    if (g_img_stride == 0 || g_img_stride < g_img_width) {
        g_img_stride = g_img_width;
    }

    HMEV_HISDK_PRT(DEBUG, "pstVideoFrame.v_frame width %u, height %u, width_stride %u",
        videoFrameInfo->v_frame.width, videoFrameInfo->v_frame.height, videoFrameInfo->v_frame.width_stride[0]);

    if (g_run_mode == MODE_HOST) {
        aclRet = aclrtMallocHost((void**)&hostAddr, bufSize);
        if (aclRet != ACL_SUCCESS) {
            HMEV_HISDK_PRT(ERROR, "Malloc host memory %d fail with %d.\n", bufSize, aclRet);
            return HMEV_FAILURE;
        }
        bufVirtY = hostAddr;
    } else {
        bufVirtY = (hi_char*)videoFrameInfo->v_frame.virt_addr[0];
    }
    bufVirtC = bufVirtY + (videoFrameInfo->v_frame.width_stride[0]) * (videoFrameInfo->v_frame.height);

    if (g_read_at_once == true) {
        memContent = bufVirtY;
        fseek(fpYuv, usedBytes, SEEK_SET);
        readLen = fread(memContent, 1, bufSize, fpYuv);
        if (readLen < 1) {
            HMEV_HISDK_PRT(INFO, "fill buffer s64ReadLen %lld", readLen);
            fseek(fpYuv, usedBytes, SEEK_SET);
            *fileOffset = usedBytes;
            if (g_run_mode == MODE_HOST) {
                aclrtFreeHost(hostAddr);
            }
            return HMEV_FAILURE;
        }
        usedBytes = usedBytes + bufSize;
    } else {
        // y
        for (h = 0; h < videoFrameInfo->v_frame.height; h++) {
            memContent = bufVirtY + h * videoFrameInfo->v_frame.width_stride[0];
            fseek(fpYuv, usedBytes, SEEK_SET);
            readLen = fread(memContent, 1, videoFrameInfo->v_frame.width, fpYuv);
            if (readLen < 1) {
                HMEV_HISDK_PRT(INFO, "fill buffer y data h %u, s64ReadLen %lld", h, readLen);
                fseek(fpYuv, usedBytes, SEEK_SET);
                *fileOffset = usedBytes;
                if (g_run_mode == MODE_HOST) {
                    aclrtFreeHost(hostAddr);
                }
                return HMEV_FAILURE;
            }
            usedBytes = usedBytes + g_img_stride;
        }

        fseek(fpYuv, usedBytes, SEEK_SET);
        for (h = 0; h < videoFrameInfo->v_frame.height / 2; h++) { // uv 1/2 down-sampling
            memContent = bufVirtC + h * videoFrameInfo->v_frame.width_stride[0];
            fseek(fpYuv, usedBytes, SEEK_SET);
            readLen = fread(memContent, 1, videoFrameInfo->v_frame.width, fpYuv);
            if (readLen < 1) {
                HMEV_HISDK_PRT(INFO, "fill buffer c data h %u, s64ReadLen %lld", h, readLen);
                fseek(fpYuv, usedBytes, SEEK_SET);
                *fileOffset = usedBytes;
                if (g_run_mode == MODE_HOST) {
                    aclrtFreeHost(hostAddr);
                }
                return HMEV_FAILURE;
            }
            usedBytes = usedBytes + g_img_stride;
        }
    }

    if (g_run_mode == MODE_HOST) {
        aclRet = aclrtMemcpy((void*)(videoFrameInfo->v_frame.virt_addr[0]), bufSize,
            hostAddr, bufSize, ACL_MEMCPY_HOST_TO_DEVICE);
        if (aclRet != ACL_SUCCESS) {
            HMEV_HISDK_PRT(ERROR, "Copy host memcpy to device fail with %d.\n", aclRet);
        }
        aclRet = aclrtFreeHost(hostAddr);
        if (aclRet != ACL_SUCCESS) {
            HMEV_HISDK_PRT(ERROR, "aclrtFreeHost fail with %d.\n", aclRet);
        }
    }

    *fileOffset = usedBytes;
    return HMEV_SUCCESS;
}

// output encoded frame rate statistics
int32_t show_encode_state()
{
    for (uint32_t i = 0; i < MAX_ENC_CHANNEL_NUM; ++i) {
        hi_u64 diffTime = g_end_get_time[i] - g_start_send_time[i];
        if (diffTime == 0) {
            continue;
        }
        HMEV_HISDK_PRT(INFO, "--------------------------------------------------------------------------------------");
        float actualFrameRate = ((float)g_send_frame_cnt[i] * US_PER_SEC) / diffTime;
        HMEV_HISDK_PRT(INFO,
            "chnId %u, actualFrameRate %.1f, g_send_frame_cnt %llu, g_get_stream_cnt %llu, diffTime %llu",
            i, actualFrameRate, g_send_frame_cnt[i], g_get_stream_cnt[i], diffTime);
    }
    return HMEV_SUCCESS;
}

// wait for encoding to complete
void venc_wait_finish(IHWCODEC_HANDLE* encHandle, VencSendParams* vencThreadParam)
{
    hi_s32 i = 0;
    hi_s32 ret = 0;
    hi_bool canExit = HI_TRUE;
    while (1) {
        canExit = HI_TRUE;
        for (i = 0; i < g_chn_num; ++i) {
            if (vencThreadParam[i].threadStart != HI_FALSE ||
                g_send_frame_cnt[i + g_start_chnl] != g_get_stream_cnt[i + g_start_chnl]) {
                canExit = HI_FALSE;
                HMEV_HISDK_PRT(INFO, "encode uncomplete,wait 10s");
                sleep(10); // encode uncomplete,wait 10s
                break;
            }
        }

        if (canExit == HI_TRUE) {
            HMEV_HISDK_PRT(INFO, "encode complete,ready to exit");
            show_encode_state();
            sleep(2); // wait 2s to exit
            for (i = 0; i < g_chn_num; ++i) {
                if (vencThreadParam[i].sendThread) {
                    vencThreadParam[i].threadStart = HI_FALSE;
                    pthread_join(vencThreadParam[i].sendThread, HI_NULL);
                    HMEV_HISDK_PRT(INFO, "send thread die");
                    vencThreadParam[i].sendThread = 0;
                    venc_mng_delete(encHandle[i]);
                }
            }
            break;
        }
    }
    return;
}

// callback function, will be called when the encoding is complete in epoll_wait
void venc_stream_out(uint32_t channelId, void* buffer)
{
    uint32_t i = 0;
    int32_t ret = 0;
    hi_venc_stream* vencStream = (hi_venc_stream*)buffer;
    HMEV_HISDK_CHECK_PTR_RET_NONE(vencStream);

    ++g_get_stream_cnt[channelId];
    HMEV_GET_TIMESTAMP_US(g_end_get_time[channelId]);

    if (!g_save_codec_out_data) {
        return;
    }

    VencOutStream* pstTargetStream = new VencOutStream[vencStream->pack_cnt];
    HMEV_HISDK_CHECK_PTR_RET_NONE(pstTargetStream);
    for (i = 0; i < vencStream->pack_cnt; i++) {
        pstTargetStream[i].dataLen = vencStream->pack[i].len - vencStream->pack[i].offset;
        pstTargetStream[i].pcData = new char[pstTargetStream[i].dataLen];
        HMEV_HISDK_CHECK_PTR_RET_NONE(pstTargetStream[i].pcData);

        if (g_run_mode == MODE_HOST) {
            ret = aclrtMemcpy(pstTargetStream[i].pcData, pstTargetStream[i].dataLen,
                              vencStream->pack[i].addr + vencStream->pack[i].offset,
                              pstTargetStream[i].dataLen, ACL_MEMCPY_DEVICE_TO_HOST);
        } else {
            memcpy(pstTargetStream[i].pcData,
                   vencStream->pack[i].addr + vencStream->pack[i].offset,
                   pstTargetStream[i].dataLen);
        }
        if (ret != HI_SUCCESS) {
            HMEV_HISDK_PRT(ERROR, "memcpy i:%u fail ret:%d pcData:%p,dataLen:%lu,addr:%p,offset:%d",
                i, ret, pstTargetStream[i].pcData, pstTargetStream[i].dataLen,
                vencStream->pack[i].addr, vencStream->pack[i].offset);
            return;
        }
    }

    HMEV_HISDK_PRT(DEBUG, "start SaveStream");
    if (g_save_file[channelId] == NULL) {
        HMEV_HISDK_PRT(ERROR, "g_save_file is null!!!");
        return;
    }
    ret = venc_save_stream(g_save_file[channelId], pstTargetStream, vencStream->pack_cnt);
    if (ret != HI_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "venc_save_stream failed with %#x!\n", ret);
        return;
    }

    if (pstTargetStream != NULL) {
        for (i = 0; i < vencStream->pack_cnt; i++) {
            delete[] pstTargetStream[i].pcData;
            pstTargetStream[i].pcData = NULL;
        }
        delete[] pstTargetStream;
        pstTargetStream = NULL;
    }
}

// send thread processing function
void* venc_send_proc(void* args)
{
    hi_s32 ret;
    uint32_t i;
    hi_video_frame_info* videoFrameInfo = NULL;
    VencSendParams* sendFramePara;

    hi_u32 align;
    hi_u32 width;
    hi_u32 height;
    hi_pixel_format pixelFormat;
    hi_data_bit_width bitWidth;
    hi_compress_mode cmpMode;
    FILE* fpYuv = NULL;
    hi_s64 yuvFileReadOff = 0;
    hi_s64 curTime = 0;
    hi_s64 lastTime = 0;
    hi_char acThreadName[32] = {0};

    if (g_run_mode == MODE_HOST) {
        aclError aclRet = aclrtSetCurrentContext(g_context);
        if (aclRet != ACL_SUCCESS) {
            HMEV_HISDK_PRT(ERROR, "set current context failed:%d", aclRet);
            return NULL;
        }
    }

    HMEV_HISDK_PRT(DEBUG, "venc_send_proc");
    snprintf(acThreadName, sizeof(acThreadName), "HmevVencSendFrame");
    prctl(PR_SET_NAME, (unsigned long)acThreadName);

    sendFramePara = (VencSendParams*)args;
    EncoderHandle* encHandle = venc_mng_get_handle(sendFramePara->chnId);
    std::shared_ptr<HmevEncoder> enc(encHandle->encoder);
    if (enc == NULL) {
        sendFramePara->threadStart = HI_FALSE;
        HMEV_HISDK_PRT(ERROR, "enc is null");
        return NULL;
    }

    fpYuv = fopen(sendFramePara->fileName.c_str(), "rb");
    if (fpYuv == NULL) {
        sendFramePara->threadStart = HI_FALSE;
        HMEV_HISDK_PRT(ERROR, "can't open file %s in send stream thread!\n", sendFramePara->fileName.c_str());
        return NULL;
    }
    fflush(stdout);
    HMEV_HISDK_PRT(DEBUG, "cYuvFile %s", sendFramePara->fileName.c_str());

    width = sendFramePara->frameSize.width;
    height = sendFramePara->frameSize.height;
    pixelFormat = (hi_pixel_format)g_pixel_format;
    bitWidth = HI_DATA_BIT_WIDTH_8;
    cmpMode = HI_COMPRESS_MODE_NONE;
    align = DEFAULT_ALIGN;

    HMEV_GET_SYSTIME_US(curTime);
    while (HI_TRUE == sendFramePara->threadStart) {
        if (!g_perf_test || videoFrameInfo == NULL) {
            ret = enc->dequeue_input_buffer(width, height, pixelFormat, bitWidth, cmpMode, align, &videoFrameInfo);
            if (ret != HMEV_SUCCESS) {
                HMEV_HISDK_PRT(DEBUG, "dequeue_input_buffer fail");
                usleep(2000); // sleep 2000 us
                continue;
            }
            HMEV_HISDK_PRT(DEBUG, "s64YuvFileReadOff %lld", yuvFileReadOff);
            ret = venc_read_yuv_file(fpYuv, videoFrameInfo, &yuvFileReadOff);
            if (ret != HMEV_SUCCESS) {
                enc->cancel_frame(videoFrameInfo);
                sendFramePara->threadStart = HI_FALSE;
                HMEV_HISDK_PRT(INFO, "read yuv file end, offset %lld", yuvFileReadOff);
                // check whether the YUV file length is complete
                hi_u32 checkLen = yuvFileReadOff % (g_img_stride * height * CALC_YUV420_SIZE);
                if (checkLen > 0) {
                    HMEV_HISDK_PRT(WARN, "yuv file is not complete,please check it!");
                }
                break;
            }
        }

        videoFrameInfo->v_frame.time_ref = (g_send_frame_cnt[sendFramePara->chnId] * 2); // 2:timeref must be even

        EncoderHandle* encHandle = venc_mng_get_handle(sendFramePara->chnId);
        ret = venc_mng_process_buffer((IHWCODEC_HANDLE)encHandle, (void*)videoFrameInfo);
        if (ret != HMEV_SUCCESS) {
            HMEV_HISDK_PRT(ERROR, "Chn[%d] hi_mpi_venc_send_frame failed, s32Ret:0x%x\n", sendFramePara->chnId, ret);
            enc->cancel_frame(videoFrameInfo);
            sendFramePara->threadStart = HI_FALSE;
            break;
        }

        lastTime = curTime;
        g_send_frame_cnt[sendFramePara->chnId]++;

        if (g_perf_test && g_send_frame_cnt[sendFramePara->chnId] == g_perf_frame_num) {
            sendFramePara->threadStart = HI_FALSE;
            HMEV_HISDK_PRT(INFO, "perf test break");
            break;
        }
        // control framerate
        while (1) {
            HMEV_GET_SYSTIME_US(curTime);
            if (curTime >= (lastTime + sendFramePara->intervalTime)) {
                break;
            } else {
                usleep(500); // sleep 500 us
            }
        }
    }

    if (fpYuv != NULL) {
        fflush(stdout);
        if (fpYuv != NULL) {
            fclose(fpYuv);
        }
        fpYuv = NULL;
    }
    return NULL;
}

inline void init_h264_params(VencParam* encParam, uint32_t chnId)
{
    encParam->codecType = VENC_CODEC_TYPE_H264;
    encParam->encCallback.encDateOutProcess = venc_stream_out;
    encParam->rcMode = HMEV_RC_VBR;
    encParam->frameNum = -1;
    encParam->fixQp = FIX_QP;
    encParam->inputFrameLen = INPUT_FRAME_LEN;
    encParam->profile = g_profile; // h.264 -- 0:Base 1:Main 2:High

    encParam->maxQp = g_max_qp;
    encParam->minQp = g_min_qp;
    encParam->maxIQp = g_max_iqp;
    encParam->minIQp = g_min_iqp;
    encParam->frameSize.width = g_img_width;
    encParam->frameSize.height = g_img_height;
    encParam->bitRate = g_bit_rate;
    encParam->frameRate = g_frame_rate;
    encParam->frameGop = g_frame_gop;

    HMEV_HISDK_PRT(INFO, "H264 width %u, height %u, bitRate %u, pQP[%u, %u], iQP[%u, %u]", encParam->frameSize.width,
        encParam->frameSize.height, encParam->bitRate, encParam->minQp, encParam->maxQp,
        encParam->minIQp, encParam->maxIQp);
}

inline void init_h265_params(VencParam* encParam, uint32_t chnId)
{
    encParam->codecType = VENC_CODEC_TYPE_H265;
    encParam->encCallback.encDateOutProcess = venc_stream_out;
    encParam->rcMode = HMEV_RC_VBR;
    encParam->frameNum = -1;
    encParam->fixQp = FIX_QP;
    encParam->inputFrameLen = INPUT_FRAME_LEN;
    encParam->profile = g_profile; // h.265 -- 1

    encParam->maxQp = g_max_qp;
    encParam->minQp = g_min_qp;
    encParam->maxIQp = g_max_iqp;
    encParam->minIQp = g_min_iqp;
    encParam->frameSize.width = g_img_width;
    encParam->frameSize.height = g_img_height;
    encParam->bitRate = g_bit_rate;
    encParam->frameRate = g_frame_rate;
    encParam->frameGop = g_frame_gop;

    HMEV_HISDK_PRT(INFO, "H265 width %u, height %u, bitRate %u, pQP[%u, %u], iQP[%u, %u]", encParam->frameSize.width,
        encParam->frameSize.height, encParam->bitRate, encParam->minQp, encParam->maxQp,
        encParam->minIQp, encParam->maxIQp);
}

// initialization, channel creat before encoding
int32_t venc_do_init(IHWCODEC_HANDLE* encHandle, VencParam* encParam)
{
    if (g_codec_type == VENC_CODEC_TYPE_H264) {
        for (uint32_t i = 0; i < g_chn_num; ++i) {
            init_h264_params(&encParam[i], i);
        }
    } else if (g_codec_type == VENC_CODEC_TYPE_H265) {
        for (uint32_t i = 0; i < g_chn_num; ++i) {
            init_h265_params(&encParam[i], i);
        }
    } else {
        HMEV_HISDK_PRT(ERROR, "not support codec type %d", g_codec_type);
        return HMEV_FAILURE;
    }

    for (uint32_t i = 0; i < g_chn_num; ++i) {
        encParam[i].channelId = i + g_start_chnl;
        encParam[i].highPriority = g_high_priority;
        int32_t ret = venc_mng_create(&encHandle[i], &encParam[i]);
        HMEV_HISDK_CHECK_RET_EXPRESS(ret != HMEV_SUCCESS, "venc_mng_create fail!");
        HMEV_HISDK_PRT(INFO, "enc handle %p", encHandle[i]);
    }
    return HMEV_SUCCESS;
}

int32_t start_encode()
{
    hi_s32 ret = 0;
    HMEV_HISDK_PRT(INFO, "config g_chn_num = %d", g_chn_num);
    if (g_chn_num < 1) {
        HMEV_HISDK_PRT(ERROR, "invalid chnNum");
        return HMEV_FAILURE;
    }

    IHWCODEC_HANDLE encHandle[g_chn_num];
    VencParam encParam[g_chn_num];
    ret = venc_do_init(encHandle, encParam);
    if (ret != HI_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "venc_do_init failed");
        return HMEV_FAILURE;
    }

    char fileName[g_chn_num][500];
    VencSendParams yuvSendFramePara[g_chn_num];

    for (uint32_t i = 0; i < g_chn_num; ++i) {
        if (g_save_codec_out_data) {
            snprintf(fileName[i], sizeof(fileName[i]), g_output_file, i + g_start_chnl);
            HMEV_HISDK_PRT(DEBUG, "open file %s", fileName[i]);
            g_save_file[i + g_start_chnl] = fopen(fileName[i], "wb");
            if (!g_save_file[i + g_start_chnl]) {
                HMEV_HISDK_PRT(ERROR, "open file[%s] failed!\n", fileName[i]);
                return HMEV_FAILURE;
            }
        }
        yuvSendFramePara[i].fileName = g_input_file;
        yuvSendFramePara[i].threadStart = HI_TRUE;
        yuvSendFramePara[i].chnlCnt = g_chn_num;
        yuvSendFramePara[i].frameNum = encParam[i].frameNum;
        yuvSendFramePara[i].frameSize = encParam[i].frameSize;
        yuvSendFramePara[i].intervalTime = US_PER_SEC / encParam[i].frameRate;
        yuvSendFramePara[i].circleSend = HI_FALSE;
        yuvSendFramePara[i].chnId = i + g_start_chnl;
        ret = pthread_create(&(yuvSendFramePara[i].sendThread), 0, venc_send_proc, &yuvSendFramePara[i]);
        if (ret != HI_SUCCESS) {
            HMEV_HISDK_PRT(ERROR, "chnl:%u pthread_create failed ret:%d", i + g_start_chnl, ret);
            yuvSendFramePara[i].threadStart = HI_FALSE;
        }
    }

    venc_wait_finish(encHandle, yuvSendFramePara);

    for (uint32_t i = 0; i < g_chn_num; ++i) {
        if (g_save_file[i + g_start_chnl] != NULL) {
            HMEV_HISDK_PRT(DEBUG, "close file[%s]", fileName[i]);
            fclose(g_save_file[i + g_start_chnl]);
            g_save_file[i + g_start_chnl] = NULL;
        }
    }

    return HMEV_SUCCESS;
}

int32_t venc_sys_init()
{
    hi_s32 ret = 0;
    aclError aclRet = aclInit(NULL);
    if (aclRet != ACL_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "aclInit fail with %d.\n", aclRet);
        return HI_FAILURE;
    }

    aclRet = aclrtSetDevice(0);
    if (aclRet != ACL_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "aclrtSetDevice(0) fail with %d.\n", aclRet);
        return HI_FAILURE;
    }

    aclRet = aclrtCreateContext(&g_context, 0);
    if (aclRet != ACL_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "acl create context failed with %d.", aclRet);
        return HI_FAILURE;
    }

    aclRet = aclrtGetCurrentContext(&g_context);
    if (aclRet != ACL_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "get current context failed with %d", aclRet);
        return HI_FAILURE;
    }

    ret = hi_mpi_sys_init();
    if (ret != HI_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "hi_mpi_sys_init failed ret:%x", ret);
        return HI_FAILURE;
    }

    g_mbuf_map.clear();
    pthread_mutex_init(&g_mbuf_mutex, NULL);

    return HI_SUCCESS;
}

int32_t venc_sys_exit()
{
    hi_s32 ret = 0;
    aclError aclRet = ACL_SUCCESS;
    for (auto iter = g_mbuf_map.begin(); iter != g_mbuf_map.end(); ++iter) {
        hi_void* buffer = iter->second;
        ret = hi_mpi_dvpp_free(buffer);
        if (ret != HI_SUCCESS) {
            HMEV_HISDK_PRT(ERROR, "hi_mpi_dvpp_free %p fail\n", buffer);
        }
    }
    g_mbuf_map.clear();
    pthread_mutex_destroy(&g_mbuf_mutex);

    ret = hi_mpi_sys_exit();
    if (ret != HI_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "hi_mpi_sys_exit failed ret:%x", ret);
    }

    aclRet = aclrtDestroyContext(g_context);
    if (aclRet != ACL_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "destroy context failed with %d.", aclRet);
    }
    aclRet = aclrtResetDevice(0);
    if (aclRet != ACL_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "reset device(0) fail with %d.\n", aclRet);
    }
    aclRet = aclFinalize();
    if (aclRet != ACL_SUCCESS) {
        HMEV_HISDK_PRT(ERROR, "finalize acl failed with %d.\n", aclRet);
    }

    return HI_SUCCESS;
}

// command line processing function, parsing user input parameters
void venc_get_option(int argc, char** argv)
{
    while (1) {
        int optionIndex = 0;
        struct option longOptions[] = {
            {"ImgWidth", 1, 0, 'w'},
            {"ImgHeight", 1, 0, 'h'},
            {"ImgStride", 1, 0, 'r'},
            {"ChnNum", 1, 0, 'n'},
            {"CodecType", 1, 0, 'c'}, // 0:264 1:265 2:jpeg
            {"BitRate", 1, 0, 'b'},
            {"FrameRate", 1, 0, 'f'},
            {"HighPriority", 1, 0, 'H'}, // 0：nomal 1：high priority
            {"InputFileName", 1, 0, 'i'},
            {"OutputFileName", 1, 0, 'o'}, // use format:xxx_%d.265 %d for different channels
            {"PixelFormat", 1, 0, 'p'}, // 1:NV12 2:NV21
            {"OneStreamBuffer", 1, 0, 'O'}, // 0:multi packs 1:single pack
            {"Profile", 1, 0, 'l'}, // profile level H.264[0,2] H.265:0
            {"PerfTest", 1, 0, 'P'}, // 0:function test 1:performance test
            {"PerfFrameNum", 1, 0, 'm'}, // performance test input frame numbers
            {"SaveOutData", 1, 0, 's'}, // 0:do not save stream 1:save stream
            {"IFrameGop", 1, 0, 'g'}, // I frame interval[1, 65536‬]，set 1 for all I frame，65536 as default‬
            {"StartChnlId", 1, 0, 'd'}, // specify the start channel id for current process in multi-process test
            {0, 0, 0, 0},
        };
        int c = getopt_long(argc, argv, "w:h:r:n:c:b:f:H:i:o:p:O:l:P:m:s:g:d:", longOptions, &optionIndex);
        if (c == -1) {
            break;
        }
        switch (c) {
            case 'w':
                g_img_width = atoi(optarg);
                break;
            case 'h':
                g_img_height  = atoi(optarg);
                break;
            case 'r':
                g_img_stride  = atoi(optarg);
                break;
            case 'H':
                g_high_priority = atoi(optarg);
                break;
            case 'n':
                g_chn_num = atoi(optarg);
                break;
            case 'c':
                g_codec_type = atoi(optarg);
                break;
            case 'b':
                g_bit_rate  = atoi(optarg);
                break;
            case 'f':
                g_frame_rate  = atoi(optarg);
                break;
            case 'i':
                strcpy(g_input_file, optarg);
                break;
            case 'o':
                strcpy(g_output_file, optarg);
                break;
            case 'p':
                g_pixel_format  = atoi(optarg);
                break;
            case 'l':
                g_profile  = atoi(optarg);
                break;
            case 'P':
                g_perf_test = atoi(optarg);
                break;
            case 'm':
                g_perf_frame_num = atoi(optarg);
                break;
            case 's':
                g_save_codec_out_data = atoi(optarg);
                break;
            case 'g':
                g_frame_gop = atoi(optarg);
                break;
            case 'd':
                g_start_chnl = atoi(optarg);
                break;
            default:
                break;
        }
    }
    return;
}

hi_void venc_print_usage()
{
    HMEV_HISDK_PRT(INFO, "*********************************************************");
    HMEV_HISDK_PRT(INFO, "Usage Example:");
    HMEV_HISDK_PRT(INFO, "./venc_demo -w 1920 -h 1080 -n 8 -c 1 -i ./input.yuv -o ./output_%%d.265 -p 1 ");
    HMEV_HISDK_PRT(INFO, "Option List:");
    HMEV_HISDK_PRT(INFO, "--InputFileName(-i): input yuv name");
    HMEV_HISDK_PRT(INFO, "--OutputFileName(-o): output stream name,use %%d in tail for output different channel");
    HMEV_HISDK_PRT(INFO, "--ImgWidth(-w): input image width");
    HMEV_HISDK_PRT(INFO, "--ImgHeight(-h): input image height");
    HMEV_HISDK_PRT(INFO, "--ImgStride(-r): input image width_stride");
    HMEV_HISDK_PRT(INFO, "--ChnNum(-n): encode channel number");
    HMEV_HISDK_PRT(INFO, "--CodecType(-c): codec type 0:H.264 1:H.265");
    HMEV_HISDK_PRT(INFO, "--ChnNum(-n): encode channel number");
    HMEV_HISDK_PRT(INFO, "--PixelFormat(-p): input yuv format 1:NV12 2:NV21");
    HMEV_HISDK_PRT(INFO, "--HighPriority(-H): channel priority 0:normal 1:high");
    HMEV_HISDK_PRT(INFO, "--Profile(-l): encode profile H.264[0,2] H.265:0");
    HMEV_HISDK_PRT(INFO, "--IFrameGop(-g): I frame interval[1,65536]");
    HMEV_HISDK_PRT(INFO, "--StartChnlId(-d): start channel id of this process");
    HMEV_HISDK_PRT(INFO, "--PerfTest(-P): test type 0:function test 1:performance test");
    HMEV_HISDK_PRT(INFO, "--PerfFrameNum(-m): performance test frame number");
    HMEV_HISDK_PRT(INFO, "--SaveOutData(-s): save output stream 0:do not save 1:save ");
    HMEV_HISDK_PRT(INFO, "*********************************************************");
}

// check the input parameters and configure the bitrate adaptively
int32_t venc_check_option()
{
    if (g_chn_num < 1 || g_chn_num > MAX_ENC_CHANNEL_NUM) {
        HMEV_HISDK_PRT(ERROR, "g_chn_num:%d is not in[1,%d]", g_chn_num, MAX_ENC_CHANNEL_NUM);
        return HI_FAILURE;
    }
    if (g_start_chnl < 0 || g_start_chnl > (MAX_ENC_CHANNEL_NUM - 1)) {
        HMEV_HISDK_PRT(ERROR, "g_start_chnl:%d is not in[0,%d]", g_start_chnl, MAX_ENC_CHANNEL_NUM - 1);
        return HI_FAILURE;
    }
    if ((g_start_chnl + g_chn_num) < 1 || (g_start_chnl + g_chn_num) > MAX_ENC_CHANNEL_NUM) {
        HMEV_HISDK_PRT(ERROR, "g_start_chnl:%d + g_chn_num:%d is not in[1,%d]",
            g_start_chnl, g_chn_num, MAX_ENC_CHANNEL_NUM);
        return HI_FAILURE;
    }
    if (g_codec_type < 0 || g_codec_type > 1) {
        HMEV_HISDK_PRT(ERROR, "g_codec_type:%d is not H264(0) or H265(1)", g_codec_type);
        return HI_FAILURE;
    }
    if (g_input_file == NULL) {
        HMEV_HISDK_PRT(ERROR, "g_input_file == NULL");
        return HI_FAILURE;
    }
    if (g_output_file == NULL) {
        HMEV_HISDK_PRT(ERROR, "g_output_file == NULL");
        return HI_FAILURE;
    }

    // if user does not set bitrate, calculate the appropriate encode bitrate according to the resolution&framerate
    if (g_bit_rate == 0) {
        int32_t frameSize = g_img_width * g_img_height;
        if (frameSize <= FRAME_SIZE_360P) {
            g_bit_rate = CALC_K * 1 + 1 * CALC_K * g_frame_rate / DEFAULT_FRAME_RATE;
        } else if (frameSize <= FRAME_SIZE_720P) {
            g_bit_rate = CALC_K * 2 + 1 * CALC_K * g_frame_rate / DEFAULT_FRAME_RATE; // 2:use to calculate bitrate
        } else if (frameSize <= FRAME_SIZE_1080P) {
            g_bit_rate = CALC_K * 2 + 2 * CALC_K * g_frame_rate / DEFAULT_FRAME_RATE; // 2:use to calculate bitrate
        } else if (frameSize <= FRAME_SIZE_3K) {
            g_bit_rate = CALC_K * 3 + 3 * CALC_K * g_frame_rate / DEFAULT_FRAME_RATE; // 3:use to calculate bitrate
        } else if (frameSize <= FRAME_SIZE_4K) {
            g_bit_rate = CALC_K * 5 + 5 * CALC_K * g_frame_rate / DEFAULT_FRAME_RATE; // 5:use to calculate bitrate
        } else {
            g_bit_rate = CALC_K * 10 + 5 * CALC_K * g_frame_rate / DEFAULT_FRAME_RATE; // 5,10:use to calculate bitrate
        }
    }

    if (g_img_stride == g_img_width) {
        g_read_at_once = true;
    }

    HMEV_HISDK_PRT(INFO,
        "g_codec_type:%d g_chn_num:%d g_img_width:%d g_img_height:%d g_img_stride:%d g_pixel_format:%d",
        g_codec_type, g_chn_num, g_img_width, g_img_height, g_img_stride, g_pixel_format);
    HMEV_HISDK_PRT(INFO,
        "g_bit_rate:%d g_frame_rate:%d g_profile:%d g_input_file:%s g_output_file:%s",
        g_bit_rate, g_frame_rate, g_profile, g_input_file, g_output_file);
    HMEV_HISDK_PRT(INFO,
        "g_frame_gop:%d g_start_chnl:%d g_high_priority:%d g_perf_test:%d g_perf_frame_num:%d g_save_codec_out_data:%d",
        g_frame_gop, g_start_chnl, g_high_priority, g_perf_test, g_perf_frame_num, g_save_codec_out_data);
    return HMEV_SUCCESS;
}

void venc_get_run_mode()
{
    aclrtRunMode runMode;
    aclError ret = aclrtGetRunMode(&runMode);
    if (runMode == ACL_HOST) {
        HMEV_HISDK_PRT(INFO, "Running on Host");
        g_run_mode = MODE_HOST;
    } else {
        HMEV_HISDK_PRT(INFO, "Running on devcie");
        g_run_mode = MODE_DEVICE;
    }
}

int main(int argc, char* argv[])
{
    if (argc < 2) { // 2:at least 2 args
        HMEV_HISDK_PRT(ERROR, "Invaild input!  For examples:\n");
        venc_print_usage();
        return HI_FAILURE;
    }
    venc_get_option(argc, &(*argv));
    HMEV_HISDK_CHECK_RET_EXPRESS(venc_check_option() != HI_SUCCESS, "check option fail");
    HMEV_HISDK_CHECK_RET_EXPRESS(venc_sys_init() != HI_SUCCESS, "init sys fail");
    venc_get_run_mode();
    start_encode();
    venc_sys_exit();
    return HMEV_SUCCESS;
}

#endif
