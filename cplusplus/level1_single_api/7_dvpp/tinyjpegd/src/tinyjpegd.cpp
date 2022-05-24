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

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <sstream>
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
#include <sys/prctl.h>
#include <dirent.h>
#include <getopt.h>
#include "common.h"
#include <vector>
#include "acl.h"
#include "acl_rt.h"

using namespace std;

const uint32_t JPEGD_ALIGN_W = 64;
const uint32_t JPEGD_ALIGN_H = 16;
const uint64_t MULTIPLE_S_TO_US        = 1000000;
const uint32_t JPEGD_SLEEP_INTERVAL_TIME   = 5000;
//const uint32_t JPEGD_DECODED_QUEUE_NUM = 10;

uint32_t g_width        = 3840;
uint32_t g_height       = 2160;
uint32_t g_out_width     = 1920;
uint32_t g_out_height    = 1080;
uint32_t g_write_file    = 1;
char g_input_file_name[128]  = "infile";
aclrtContext g_context = nullptr;
aclrtRunMode g_run_mode = ACL_DEVICE;

vector<void *> g_out_buffer_pool[VDEC_MAX_CHN_NUM];
pthread_mutex_t g_out_buffer_pool_lock[VDEC_MAX_CHN_NUM];

static uint64_t g_end_get_time[VDEC_MAX_CHN_NUM] = {0};
static uint64_t g_get_frame_cnt[VDEC_MAX_CHN_NUM] = {0};
static uint64_t g_start_send_time[VDEC_MAX_CHN_NUM] = {0};
static VDEC_THREAD_PARAM_S g_jpegd_thread_param[VDEC_MAX_CHN_NUM];

static inline void jpegd_get_time_stamp_us(uint64_t *val)
{
    struct timeval stTimeVal;
    gettimeofday(&stTimeVal, nullptr);
    *val = (uint64_t)stTimeVal.tv_sec * MULTIPLE_S_TO_US + stTimeVal.tv_usec;
}

void jpegd_handle_signal(int32_t signo)
{
    if (SIGINT == signo || SIGTSTP == signo || SIGTERM == signo) {
        g_jpegd_thread_param[0].enSendThreadCtrl = THREAD_CTRL_STOP;
        g_jpegd_thread_param[0].enGetThreadCtrl  = THREAD_CTRL_STOP;
        SAMPLE_PRT("\033[0;31mprogram exit abnormally!\033[0;39m\n");
    }
}

int32_t set_param()
{
    //g_width及g_height需要满足需要满足[32, 8192]
    g_width = 1920;
    g_height = 1080;
    strcpy(g_input_file_name, "../data/dvpp_jpegd_decode_1920x1080.jpg");
    g_write_file = 1;

    aclError aclRet = aclrtGetRunMode(&g_run_mode);
    if (aclRet == ACL_SUCCESS) {
        if (g_run_mode == ACL_HOST) {
            SAMPLE_PRT(" Running in Host!\n");
        } else if (g_run_mode == ACL_DEVICE) {
            SAMPLE_PRT(" Running in Device!\n");
        } else {
            SAMPLE_PRT(" Running in Invalid platform! runMode:%u\n", g_run_mode);
            return HI_FAILURE;
        }
    } else {
        SAMPLE_PRT(" Get run mode fail! acl ret:%#x\n", aclRet);
        return HI_FAILURE;
    }

    return HI_SUCCESS;
}

// 保存JPEGD解码后的YUV_SemiPlanar格式像素内容
static void jpegd_save_yuv_file_linear_8bit_semiplanar(const std::string &fileName, hi_video_frame *pVBuf)
{
    uint8_t *yMapPtr       = nullptr;
    uint8_t *cMapPtr       = nullptr;
    uint8_t *memContentPtr = nullptr;
    uint32_t u32IdxW       = 0;
    uint32_t u32IdxH       = 0;
    uint32_t u32Size       = 0;
    uint32_t u32Ysize      = 0;
    uint32_t u32AlignW     = 0;
    uint32_t u32AlignH     = 0;
    uint32_t u32UvWidth    = 0;
    uint32_t u32UvHeight   = 0;
    hi_pixel_format pixel_format = pVBuf->pixel_format;

    u32Ysize  = (pVBuf->width_stride[0] * pVBuf->height_stride[0]);
    u32AlignW = pVBuf->width;
    u32AlignH = pVBuf->height;

    u32UvWidth  = u32AlignW;
    u32Size     = u32Ysize * 3 / 2;
    u32UvHeight = u32AlignH / 2;

    std::vector<uint8_t> tmpHostBuffer;
    if (g_run_mode == ACL_HOST) {
        tmpHostBuffer.resize(u32Size);

        int ret = aclrtMemcpy(&tmpHostBuffer[0], u32Size,
                              (void *)pVBuf->virt_addr[0], u32Size,
                              ACL_MEMCPY_DEVICE_TO_HOST);
        if (ret) {
            SAMPLE_PRT("aclrtMemcpy %d\n",ret);
            return;
        }

        yMapPtr = static_cast<uint8_t *>(&tmpHostBuffer[0]);
    } else {
        yMapPtr = (uint8_t *)pVBuf->virt_addr[0];
    }

    if (yMapPtr == nullptr) {
        SAMPLE_PRT("yMapPtr is NULL\n");
        return;
    }

    cMapPtr = yMapPtr + u32Ysize;

    FILE *fp = fopen(fileName.c_str(), "wb");
    if (fp == nullptr) {
        SAMPLE_PRT(" can't open file %s in get picture thread!\n", fileName.c_str());
        return;
    }
    SAMPLE_PRT(" saving file:%s in semi planar format\n", fileName.c_str());

    for (u32IdxH = 0; u32IdxH < u32AlignH; u32IdxH++) {
        memContentPtr = yMapPtr + u32IdxH * pVBuf->width_stride[0];
        fwrite(memContentPtr, u32AlignW, 1, fp);
    }

    uint8_t *tmpBuffPtr = static_cast<uint8_t *>(malloc(pVBuf->width_stride[1]));
    for (u32IdxH = 0; u32IdxH < u32UvHeight; u32IdxH++) {
        if (pixel_format != HI_PIXEL_FORMAT_YUV_400) {
            memContentPtr = cMapPtr + u32IdxH * pVBuf->width_stride[1];
            fwrite(memContentPtr, u32UvWidth, 1, fp);
        } else {
            for (u32IdxW = 0; u32IdxW < u32UvWidth; u32IdxW++) {
                tmpBuffPtr[u32IdxW] = 0x80;
            }
            fwrite(tmpBuffPtr, u32UvWidth, 1, fp);
        }
    }
    free(tmpBuffPtr);
    tmpBuffPtr = nullptr;

    fsync(fileno(fp));
    fclose(fp);
    SAMPLE_PRT(" save %s done!\n", fileName.c_str());

    return;
}

// 保存JPEGD解码后的像素内容
void jpegd_save_file(int32_t chn, int32_t picCnt, hi_video_frame stVFrame)
{
    std::ostringstream oFileName;
    oFileName << "chn" << chn << "_" << picCnt << "_outfile" << ".yuv";
    std::string strFileName = oFileName.str();

    SAMPLE_PRT("[chn:%d] pixel_format = %d\n", chn, stVFrame.pixel_format);
    SAMPLE_PRT("[chn:%d] width = %u\n", chn, stVFrame.width);
    SAMPLE_PRT("[chn:%d] height = %u\n", chn, stVFrame.height);
    SAMPLE_PRT("[chn:%d] header_stride[0] = %u\n", chn, stVFrame.header_stride[0]);
    SAMPLE_PRT("[chn:%d] header_stride[1] = %u\n", chn, stVFrame.header_stride[1]);
    SAMPLE_PRT("[chn:%d] width_stride[0] = %u\n", chn, stVFrame.width_stride[0]);
    SAMPLE_PRT("[chn:%d] width_stride[1] = %u\n", chn, stVFrame.width_stride[1]);
    SAMPLE_PRT("[chn:%d] height_stride[0] = %u\n", chn, stVFrame.height_stride[0]);
    SAMPLE_PRT("[chn:%d] height_stride[1] = %u\n", chn, stVFrame.height_stride[1]);
    SAMPLE_PRT("[chn:%d] virt_addr[0] = 0x%lx\n", chn, (uint64_t)(uintptr_t)stVFrame.virt_addr[0]);
    SAMPLE_PRT("[chn:%d] ComPressMode[%d] VideoFormat %d\n", chn, stVFrame.compress_mode,
                   stVFrame.video_format);
    jpegd_save_yuv_file_linear_8bit_semiplanar(strFileName, &stVFrame);
}

// 常规解码发送线程
void *jpegd_send_stream(void *pArgs)
{
    FILE     *fpStrm         = nullptr;
    char     *pu8Buf         = nullptr;
    void     *outBuffer      = nullptr;
    int32_t  s32ReadLen      = 0;
    int32_t  fileSize        = 0;
    int32_t  s32Ret          = 0;
    uint32_t outBufferSize   = 0;
    uint32_t bufAllocCount   = 0;
    uint32_t errCnt          = 0;
    hi_vdec_stream stStream{};
    hi_vdec_pic_info outPicInfo{};
    hi_img_info stImgInfo{};
    hi_vdec_chn_status stStatus{};
    VDEC_THREAD_PARAM_S *pstJpegdThreadParam = (VDEC_THREAD_PARAM_S *)pArgs;

    if (g_run_mode == ACL_HOST) {
        if (aclrtSetCurrentContext(g_context)) {
            SAMPLE_PRT("set context error\n");
            return (void *)(HI_FAILURE);
        }
    }

    std::ostringstream pthreadName;
    pthreadName << "JpegSend_" << pstJpegdThreadParam->s32ChnId;
    prctl(PR_SET_NAME, pthreadName.str().c_str(), 0, 0, 0);

    fpStrm = fopen(g_input_file_name, "rb");
    if (fpStrm == nullptr) {
        SAMPLE_PRT("[chn:%d] can't open file %s in send stream thread!\n",
                   pstJpegdThreadParam->s32ChnId, g_input_file_name);
        return (void *)(HI_FAILURE);
    }

    fseek(fpStrm, 0L, SEEK_END);
    fileSize = ftell(fpStrm);

    fflush(stdout);

    if (pstJpegdThreadParam->enPixFormat == HI_PIXEL_FORMAT_UNKNOWN) {
        outBufferSize = ALIGN_UP(g_width, JPEGD_ALIGN_W) * ALIGN_UP(g_height, JPEGD_ALIGN_H) * 3;
    } else {
        hi_pic_buf_attr buf_attr{g_width, g_height, 0,
                                 HI_DATA_BIT_WIDTH_8, pstJpegdThreadParam->enPixFormat, HI_COMPRESS_MODE_NONE};
        outBufferSize = hi_vdec_get_pic_buf_size(HI_PT_JPEG, &buf_attr);
    }
    SAMPLE_PRT("[chn:%d] outBufferSize:%u. pixFormat:%d stream file:%s, filesize:%d Start!\n",
               pstJpegdThreadParam->s32ChnId, outBufferSize, pstJpegdThreadParam->enPixFormat,
               g_input_file_name, fileSize);

    jpegd_get_time_stamp_us(&g_start_send_time[pstJpegdThreadParam->s32ChnId]);

    // 开始发码流
    while (1) {
        if (pstJpegdThreadParam->enSendThreadCtrl == THREAD_CTRL_STOP) {
            break;
        } else if (pstJpegdThreadParam->enSendThreadCtrl == THREAD_CTRL_PAUSE) {
            sleep(1);
            continue;
        }

        std::vector<uint8_t*> fileData;

        s32Ret = hi_mpi_dvpp_malloc(0, (void**)&pu8Buf, fileSize);
        if ((s32Ret != 0) || (pu8Buf == nullptr)) {
            SAMPLE_PRT("[chn:%d] can't alloc %d in send stream thread!\n",
                       pstJpegdThreadParam->s32ChnId, fileSize);
            fclose(fpStrm);
            return (void *)(HI_FAILURE);
        }

        fseek(fpStrm, 0L, SEEK_SET);
        if (g_run_mode == ACL_HOST) {
            fileData.resize(fileSize);
            s32ReadLen = fread(&fileData[0], 1, fileSize, fpStrm);
        } else {
            s32ReadLen = fread(pu8Buf, 1, fileSize, fpStrm);
        }
        if (s32ReadLen == 0) {
            hi_mpi_dvpp_free(pu8Buf);
            SAMPLE_PRT(" read size is 0!\n");
            break;
        }

        if (g_run_mode == ACL_HOST) {
            auto aclRet = aclrtMemcpy(pu8Buf, s32ReadLen, &fileData[0], s32ReadLen, ACL_MEMCPY_HOST_TO_DEVICE);
            if (aclRet != ACL_SUCCESS) {
                hi_mpi_dvpp_free(pu8Buf);
                fclose(fpStrm);
                SAMPLE_PRT("Copy host memcpy to device fail with %d.\n", aclRet);
                return (void *)(HI_FAILURE);
            }

            // Host端 hi_mpi_dvpp_get_image_info 使用host侧的内存
            stStream.addr  = (uint8_t *)&fileData[0];
        }

        stStream.pts       = pstJpegdThreadParam->u64PtsInit;
        stStream.len       = s32ReadLen;
        stStream.end_of_frame  = HI_TRUE;
        stStream.end_of_stream = HI_FALSE;
        stStream.need_display  = HI_TRUE;

        if (g_run_mode == ACL_HOST) {
            s32Ret = hi_mpi_dvpp_get_image_info(HI_PT_JPEG, &stStream, &stImgInfo);
            if (s32Ret != HI_SUCCESS) {
                SAMPLE_PRT("[chn:%d] hi_mpi_dvpp_get_image_info faild!\n",
                           pstJpegdThreadParam->s32ChnId);
                hi_mpi_dvpp_free((void*)pu8Buf);
                break;
            }
        }
        // hi_mpi_vdec_send_stream 接口使用Device侧的内存
        stStream.addr = (uint8_t *)pu8Buf;

        s32Ret = hi_mpi_dvpp_malloc(0, &outBuffer, outBufferSize);
        if (s32Ret != 0) {
            SAMPLE_PRT("[chn:%d] hi_mpi_dvpp_malloc %u Failed.\n",
                       pstJpegdThreadParam->s32ChnId, outBufferSize);
            hi_mpi_dvpp_free((void*)pu8Buf);
            break;
        }
        bufAllocCount++;

        // Add limit to avoid memory exhausted
        do {
            s32Ret = hi_mpi_vdec_query_status(pstJpegdThreadParam->s32ChnId, &stStatus);
            if ((stStatus.left_decoded_frames > pstJpegdThreadParam->u32PicInJpegd) ||
                (stStatus.left_stream_bytes > pstJpegdThreadParam->u32StmLenInJpegd)) {
                SAMPLE_PRT("[chn:%d] JPEGD decode is full. undecode stream len:%u, decoded pic:%u\n",
                    pstJpegdThreadParam->s32ChnId, stStatus.left_stream_bytes, stStatus.left_decoded_frames);

                if (pstJpegdThreadParam->enSendThreadCtrl == THREAD_CTRL_STOP) {
                    break; // break do...while
                }

                usleep(JPEGD_SLEEP_INTERVAL_TIME);
            }
        } while ((stStatus.left_decoded_frames > pstJpegdThreadParam->u32PicInJpegd) ||
                 (stStatus.left_stream_bytes > pstJpegdThreadParam->u32StmLenInJpegd));
        if (pstJpegdThreadParam->enSendThreadCtrl == THREAD_CTRL_STOP) {
            s32Ret = hi_mpi_dvpp_free((void*)pu8Buf);
            if (s32Ret != 0) {
                SAMPLE_PRT("[chn:%d] Free stream failed\n", pstJpegdThreadParam->s32ChnId);
            }
            s32Ret = hi_mpi_dvpp_free((void *)outBuffer);
            if (s32Ret != 0) {
                SAMPLE_PRT("[chn:%d] Free outBuffer failed\n", pstJpegdThreadParam->s32ChnId);
            }
            break; // break while(1)
        }

        outPicInfo.vir_addr    = (uint64_t)outBuffer;
        outPicInfo.buffer_size = outBufferSize;
        outPicInfo.width      = g_width;
        outPicInfo.height     = g_height;
        outPicInfo.pixel_format = pstJpegdThreadParam->enPixFormat;

        do {
            s32Ret = hi_mpi_vdec_send_stream(pstJpegdThreadParam->s32ChnId,
                                             &stStream, &outPicInfo, pstJpegdThreadParam->s32MilliSec);
            if (s32Ret != HI_SUCCESS) {
                errCnt++;
                if (errCnt > 100) {
                    break; // break do...while
                }

                usleep(pstJpegdThreadParam->s32IntervalTime);
            }
        } while ((s32Ret != HI_SUCCESS) && (pstJpegdThreadParam->enSendThreadCtrl == THREAD_CTRL_START));
        if ((s32Ret != HI_SUCCESS) || (pstJpegdThreadParam->enSendThreadCtrl == THREAD_CTRL_STOP)) {
            if (errCnt != 0) {
                SAMPLE_PRT("[chn:%d] SendStream [%u]failed %u times! addr:%#lx, len:%u, outBuffer:0x%lx, size:%u\n",
                    pstJpegdThreadParam->s32ChnId, bufAllocCount, errCnt,
                    (uint64_t)(uintptr_t)stStream.addr, stStream.len,
                    (uint64_t)(uintptr_t)outBuffer, outBufferSize);
            }
            s32Ret = hi_mpi_dvpp_free((void*)pu8Buf);
            if (s32Ret != 0) {
                SAMPLE_PRT("[chn:%d] Free stream failed\n", pstJpegdThreadParam->s32ChnId);
            }
            s32Ret = hi_mpi_dvpp_free((void *)outBuffer);
            if (s32Ret != 0) {
                SAMPLE_PRT("[chn:%d] Free outBuffer failed\n", pstJpegdThreadParam->s32ChnId);
            }

            break; // break while(1)
        }

        errCnt = 0;
        SAMPLE_PRT("[chn:%d] hi_mpi_vdec_send_stream Send[%u] ok! addr:%#lx, len:%u, outBuffer:0x%lx, size:%u\n",
                   pstJpegdThreadParam->s32ChnId, bufAllocCount,
                   (uint64_t)(uintptr_t)stStream.addr, stStream.len,
                   (uint64_t)(uintptr_t)outBuffer, outBufferSize);
        break;

    } // while (1)

    // 发送结束码
    stStream.end_of_stream = HI_TRUE;
    stStream.addr = NULL;
    stStream.len = 0;
    stStream.end_of_frame = HI_FALSE;
    stStream.end_of_stream = HI_TRUE;
    outPicInfo.vir_addr = 0;
    outPicInfo.buffer_size = 0;
    SAMPLE_PRT("[chn:%d] hi_mpi_vdec_send_stream Send eos\n", pstJpegdThreadParam->s32ChnId);
    hi_mpi_vdec_send_stream(pstJpegdThreadParam->s32ChnId, &stStream, &outPicInfo, -1);

    fflush(stdout);
    fclose(fpStrm);

    SAMPLE_PRT("[chn:%d] send steam thread return\n", pstJpegdThreadParam->s32ChnId);
    return (void *)HI_SUCCESS;
}

void jpegd_cmd_ctrl(pthread_t *pJpegdSendTid, pthread_t *pJpegdGetTid)
{
    int32_t s32Ret;
    uint64_t tmpCurtime;

    uint64_t curTime = 0;
    uint64_t lastTime = 0;
    struct timeval currentTime;
    hi_vdec_chn_status stStatus;


    if (pJpegdSendTid[0] != 0) {
        s32Ret = pthread_join(pJpegdSendTid[0], NULL);
        pJpegdSendTid[0] = 0;
    }
    g_jpegd_thread_param[0].enSendThreadCtrl = THREAD_CTRL_STOP;

    while (1) {
        s32Ret = hi_mpi_vdec_query_status(g_jpegd_thread_param[0].s32ChnId, &stStatus);
        if (s32Ret != HI_SUCCESS) {
            SAMPLE_PRT("[chn:%u] hi_mpi_vdec_query_status fail!!!\n", 0);
            g_jpegd_thread_param[0].enGetThreadCtrl = THREAD_CTRL_STOP;
            break;
        }

        // jump out if no decoding data
        if ((stStatus.left_stream_bytes == 0) &&
            (stStatus.left_decoded_frames == 0)) {
            if (g_jpegd_thread_param[0].enGetThreadCtrl == THREAD_CTRL_START) {
                g_jpegd_thread_param[0].enGetThreadCtrl = THREAD_CTRL_STOP;
                break;
            }
        }

        // jump out when user input CTRL+C or CTRL+Z
        if (g_jpegd_thread_param[0].enGetThreadCtrl == THREAD_CTRL_STOP) {
            break;
        }

        jpegd_get_time_stamp_us(&curTime);
        if (curTime - lastTime > 5000000) {
            SAMPLE_PRT("[chn:%u] Waiting status LeftStreamBytes:%u. LeftPics:%u\n",
                        0, stStatus.left_stream_bytes, stStatus.left_decoded_frames);
            lastTime = curTime;
        }

        usleep(1000);
    }

    if (pJpegdGetTid[0] != 0) {
        pthread_join(pJpegdGetTid[0], nullptr);
        pJpegdGetTid[0] = 0;
    }

    return;
}

int32_t jpegd_start_send_stream(pthread_t *pJpegdSendTid)
{
    int32_t ret = -1;
    pJpegdSendTid[0] = 0;
    g_jpegd_thread_param[0].enSendThreadCtrl = THREAD_CTRL_START;
        
    ret = pthread_create(&pJpegdSendTid[0], 0,
                            jpegd_send_stream,
                            (void *)&g_jpegd_thread_param[0]);
    if (ret != 0) {
        SAMPLE_PRT("[chn:%u] create send pic thread Fail, ret = %d \n", 0, ret);
        pJpegdSendTid[0] = 0;
        g_jpegd_thread_param[0].enSendThreadCtrl = THREAD_CTRL_STOP;
        return ret;
    }

    return 0;
}

void jpegd_stop_send_stream()
{
    SAMPLE_PRT(" JPEGD stop send stream threads\n");
    g_jpegd_thread_param[0].enSendThreadCtrl = THREAD_CTRL_STOP;
}

void jpegd_show_decode_state()
{
    uint64_t u64DiffTime = g_end_get_time[0] - g_start_send_time[0];
    if (u64DiffTime == 0) {
        return;
    }
    SAMPLE_PRT("\033[0;33m --------------------------------------------------------------------------\033[0;39m\n");
    double actualFrameRate = ((double)g_get_frame_cnt[0] * MULTIPLE_S_TO_US) / u64DiffTime;
    SAMPLE_PRT("\033[0;33m chnId %u, actualFrameRate %.1f, SendFrameCnt %lu, DiffTime %lu \033[0;39m\n",
                0, actualFrameRate, g_get_frame_cnt[0], u64DiffTime);
    SAMPLE_PRT("\033[0;33m --------------------------------------------------------------------------\033[0;39m\n");

    return;
}

// JPEGD解码接收线程
void *jpegd_get_pic(void *pArgs)
{
    uint32_t bufFreeCount = 0;
    int32_t s32Ret        = 0;
    int32_t s32Cnt        = 0;
    int32_t s32DecFailCnt = 0;
    hi_vdec_chn_attr     stAttr{};
    hi_video_frame_info  stVFrame{};
    hi_vdec_stream       stStream{};
    hi_vdec_supplement_info stSupplement{};
    VDEC_THREAD_PARAM_S *pstJpegdThreadParam = (VDEC_THREAD_PARAM_S *)pArgs;

    if (g_run_mode == ACL_HOST) {
        if (aclrtSetCurrentContext(g_context)) {
            SAMPLE_PRT("set context error\n");
            return (void *)(HI_FAILURE);
        }
    }

    std::ostringstream pthreadName;
    pthreadName << "JpegGetPic_" << pstJpegdThreadParam->s32ChnId;
    prctl(PR_SET_NAME, pthreadName.str().c_str(), 0, 0, 0);

    s32Ret = hi_mpi_vdec_get_chn_attr(pstJpegdThreadParam->s32ChnId, &stAttr);
    if (HI_SUCCESS != s32Ret) {
        SAMPLE_PRT("[chn:%d] get chn attr fail for %#x!\n", pstJpegdThreadParam->s32ChnId, s32Ret);
        return (void *)(HI_FAILURE);
    }

    SAMPLE_PRT("[chn:%d] jpegd_get_pic Start\n", pstJpegdThreadParam->s32ChnId);
    while (1) {
        if (pstJpegdThreadParam->enGetThreadCtrl == THREAD_CTRL_STOP) {
            SAMPLE_PRT("[chn:%d] jpegd_get_pic break out\n", pstJpegdThreadParam->s32ChnId);
            break;
        }

        s32Ret = hi_mpi_vdec_get_frame(pstJpegdThreadParam->s32ChnId,
                                       &stVFrame, &stSupplement, &stStream, pstJpegdThreadParam->s32MilliSec);
        if (s32Ret == HI_SUCCESS) {
            jpegd_get_time_stamp_us(&g_end_get_time[pstJpegdThreadParam->s32ChnId]);
            // 统计解码结果
            if (stVFrame.v_frame.frame_flag == 0) {
                SAMPLE_PRT("[chn:%d] Yes, GetFrame Success [%d].\n", pstJpegdThreadParam->s32ChnId, s32Cnt);
                s32Cnt++;

                if (g_write_file > 0) {
                    jpegd_save_file(pstJpegdThreadParam->s32ChnId, s32Cnt, stVFrame.v_frame);
                }
            } else {
                SAMPLE_PRT("[chn:%d] Yes, GetFrame Fail(Decoder Fail) [%d]. flg:%d\n",
                           pstJpegdThreadParam->s32ChnId, s32DecFailCnt, stVFrame.v_frame.frame_flag);
                s32DecFailCnt++;
            }
            g_get_frame_cnt[pstJpegdThreadParam->s32ChnId] = s32Cnt + s32DecFailCnt;

            // 释放码流内存
            if (stStream.addr != nullptr) {
                SAMPLE_PRT("[chn:%d] Free stream buf: %#lx, len:%u\n", pstJpegdThreadParam->s32ChnId,
                           (uint64_t)(uintptr_t)stStream.addr, stStream.len);
                hi_mpi_dvpp_free((void*)stStream.addr);
                stStream.addr = nullptr;
            }

            // 释放输出像素内存
            bufFreeCount++;
            SAMPLE_PRT("[chn:%d] BuffFree count = %u [%p].\n", pstJpegdThreadParam->s32ChnId,
                       bufFreeCount, (void *)stVFrame.v_frame.virt_addr[0]);
            s32Ret = hi_mpi_dvpp_free((void *)stVFrame.v_frame.virt_addr[0]);
            if (s32Ret != HI_SUCCESS) {
                SAMPLE_PRT("[chn:%d] hi_mpi_dvpp_free[%p] failed!\n",
                           pstJpegdThreadParam->s32ChnId, (void *)stVFrame.v_frame.virt_addr[0]);
            }

            s32Ret = hi_mpi_vdec_release_frame(pstJpegdThreadParam->s32ChnId, &stVFrame);
            if (s32Ret != HI_SUCCESS) {
                SAMPLE_PRT("[chn:%d] hi_mpi_vdec_release_frame fail for s32Ret=0x%x!\n",
                           pstJpegdThreadParam->s32ChnId, s32Ret);
            }
        } else {
            usleep(300);
        }
    }

    SAMPLE_PRT("[chn:%d] Get thread end\n", pstJpegdThreadParam->s32ChnId);
    return (void *)HI_SUCCESS;
}

// 启动接收线程,开始接收图片
int32_t jpegd_start_get_pic(pthread_t *pJpegdGetTid)
{
    int32_t ret = -1;
    pJpegdGetTid[0] = 0;
    g_jpegd_thread_param[0].enGetThreadCtrl = THREAD_CTRL_START;
    ret = pthread_create(&pJpegdGetTid[0], 0,
                        jpegd_get_pic, (void *)&g_jpegd_thread_param[0]);
    if (ret != 0) {
        SAMPLE_PRT(" chn %u, create get pic thread Fail, ret = %d \n", 0, ret);
        pJpegdGetTid[0] = 0;
        g_jpegd_thread_param[0].enGetThreadCtrl = THREAD_CTRL_STOP;
        return ret;
    }

    return 0;
}

void jpegd_stop_get_pic()
{
    SAMPLE_PRT(" JPEGD stop get pic threads\n");
    g_jpegd_thread_param[0].enGetThreadCtrl = THREAD_CTRL_STOP;
}

int32_t jpegd_create()
{
    int32_t  s32Ret = HI_SUCCESS;
    SAMPLE_VDEC_ATTR astSampleVdec[VDEC_MAX_CHN_NUM];
    hi_vdec_chn_attr stChnAttr[VDEC_MAX_CHN_NUM];
    hi_vdec_chn_param stChnParam;

    astSampleVdec[0].type                             = HI_PT_JPEG;
    astSampleVdec[0].width                            = ALIGN_UP(g_width, JPEGD_ALIGN_W);
    astSampleVdec[0].height                           = ALIGN_UP(g_height, JPEGD_ALIGN_H);
    astSampleVdec[0].mode                             = HI_VDEC_SEND_MODE_FRAME;
    astSampleVdec[0].picture_attr.pixel_format = HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    astSampleVdec[0].picture_attr.alpha        = 255;
    astSampleVdec[0].display_frame_num                = 0;
    astSampleVdec[0].frame_buf_cnt = astSampleVdec[0].display_frame_num + 1;

    pthread_mutex_init(&g_out_buffer_pool_lock[0], NULL);

    stChnAttr[0].type = astSampleVdec[0].type;
    stChnAttr[0].mode = astSampleVdec[0].mode;
    stChnAttr[0].pic_width  = astSampleVdec[0].width;
    stChnAttr[0].pic_height = astSampleVdec[0].height;
    stChnAttr[0].stream_buf_size = astSampleVdec[0].width *
                                        astSampleVdec[0].height;

    stChnAttr[0].frame_buf_cnt  = 0;
    stChnAttr[0].frame_buf_size = 0;

    CHECK_CHN_RET(hi_mpi_vdec_create_chn(0, &stChnAttr[0]), 0, "hi_mpi_vdec_create_chn");

    CHECK_CHN_RET(hi_mpi_vdec_get_chn_param(0, &stChnParam), 0, "hi_mpi_vdec_get_chn_param");
    stChnParam.pic_param.pixel_format = astSampleVdec[0].picture_attr.pixel_format;
    stChnParam.pic_param.alpha      = astSampleVdec[0].picture_attr.alpha;
    stChnParam.display_frame_num = astSampleVdec[0].display_frame_num;
    CHECK_CHN_RET(hi_mpi_vdec_set_chn_param(0, &stChnParam), 0, "hi_mpi_vdec_set_chn_param");

    CHECK_CHN_RET(hi_mpi_vdec_start_recv_stream(0), 0, "hi_mpi_vdec_start_recv_stream");

    g_jpegd_thread_param[0].type = astSampleVdec[0].type;
    g_jpegd_thread_param[0].s32StreamMode = astSampleVdec[0].mode;
    g_jpegd_thread_param[0].s32ChnId = 0;
    g_jpegd_thread_param[0].s32IntervalTime = 1000;
    g_jpegd_thread_param[0].u64PtsInit = 0;
    g_jpegd_thread_param[0].u64PtsIncrease = 0;
    g_jpegd_thread_param[0].enSendThreadCtrl = THREAD_CTRL_INIT;
    g_jpegd_thread_param[0].enGetThreadCtrl  = THREAD_CTRL_INIT;
    g_jpegd_thread_param[0].perfStreamBuf = nullptr;
    g_jpegd_thread_param[0].s32MilliSec = 1000;
    g_jpegd_thread_param[0].enPixFormat = HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    g_jpegd_thread_param[0].s32MinBufSize = astSampleVdec[0].width * astSampleVdec[0].height;
    g_jpegd_thread_param[0].u32StmLenInJpegd = ALIGN_UP(astSampleVdec[0].width, JPEGD_ALIGN_W) *
                                                    ALIGN_UP(astSampleVdec[0].height, JPEGD_ALIGN_H) * 4;
    g_jpegd_thread_param[0].u32PicInJpegd    = 1; //JPEGD_DECODED_QUEUE_NUM;

    return HI_SUCCESS;
}

int32_t jpegd_destroy()
{
    CHECK_CHN_RET(hi_mpi_vdec_stop_recv_stream(0), 0, "hi_mpi_vdec_stop_recv_stream");
    CHECK_CHN_RET(hi_mpi_vdec_destroy_chn(0), 0, "hi_mpi_vdec_destroy_chn");

    // Release JPEG stream mem
    if (g_jpegd_thread_param[0].perfStreamBuf != nullptr) {
        hi_mpi_dvpp_free((void*)g_jpegd_thread_param[0].perfStreamBuf);
        g_jpegd_thread_param[0].perfStreamBuf = nullptr;
    }

    // Release YUV picture mem
    while (g_out_buffer_pool[0].empty() == false) {
        (void)pthread_mutex_lock(&g_out_buffer_pool_lock[0]);
        hi_mpi_dvpp_free(g_out_buffer_pool[0].back());
        g_out_buffer_pool[0].pop_back();
        (void)pthread_mutex_unlock(&g_out_buffer_pool_lock[0]);
    }

    pthread_mutex_destroy(&g_out_buffer_pool_lock[0]);

    return HI_SUCCESS;
}

int32_t setup_acl_device()
{
    if (g_run_mode != ACL_HOST) {
        return HI_SUCCESS;
    }

    aclError aclRet = aclInit(nullptr);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("aclInit fail with %d.\n", aclRet);
        return aclRet;
    }
    SAMPLE_PRT("aclInit succ.\n");

    aclRet = aclrtSetDevice(0);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("aclrtSetDevice(0) fail with %d.\n", aclRet);
        aclFinalize();
        return aclRet;
    }
    SAMPLE_PRT("aclrtSetDevice(0) succ.\n");

    aclRet = aclrtCreateContext(&g_context, 0);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("acl create context failed with %d.\n", aclRet);
        aclrtResetDevice(0);
        aclFinalize();
        return aclRet;
    }
    SAMPLE_PRT("create context success\n");

    aclRet = aclrtGetCurrentContext(&g_context);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("get current context failed\n");
        aclrtDestroyContext(g_context);
        g_context = nullptr;
        aclrtResetDevice(0);
        aclFinalize();
        return aclRet;
    }
    SAMPLE_PRT("get current context success\n");

    return HI_SUCCESS;
}

void destroy_acl_device()
{
    if (g_run_mode != ACL_HOST) {
        return;
    }

    if (g_context) {
        aclrtDestroyContext(g_context);
        g_context = nullptr;
        aclrtResetDevice(0);
        aclFinalize();
    }
}

int32_t destory_resource_jpegd_mod()
{
    int32_t s32Ret = HI_SUCCESS;

    s32Ret = jpegd_destroy();
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("jpegd_destroy failed. ret code:%#x\n", s32Ret);
    }

    s32Ret = hi_mpi_sys_exit();
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_sys_exit failed. ret code:%#x\n", s32Ret);
    }

    destroy_acl_device();

    return s32Ret;
}