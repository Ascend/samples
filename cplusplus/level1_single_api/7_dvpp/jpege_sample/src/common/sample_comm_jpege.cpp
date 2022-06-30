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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/time.h>
#include <sys/select.h>
#include <fcntl.h>
#include <sys/errno.h>
#include <pthread.h>
#include <math.h>
#include <unistd.h>
#include <signal.h>
#include <sys/prctl.h>
#include <map>
#include <set>
#include <vector>
#include "acl.h"
#include "sample_comm.h"

static pthread_t g_jpege_pid[MAX_JPEGE_CHN];
static pthread_t g_jpege_get_stream_epoll_pid;
static pthread_t g_jpege_get_stream_pid[MAX_JPEGE_CHN];
static pthread_t g_jpege_send_frame_pid[MAX_JPEGE_CHN];
static HiSampleJpegeSendFramePara g_jpege_send_frame_para[MAX_JPEGE_CHN];
static hi_u64 g_jpege_send_frame_cnt[VENC_MAX_CHN_NUM] = {0};
static hi_u64 g_encode_stream_cnt[VENC_MAX_CHN_NUM] = {0};
static hi_u64 g_start_send_time_array[VENC_MAX_CHN_NUM] = {0};
static hi_u64 g_end_get_time[VENC_MAX_CHN_NUM] = {0};
static hi_u64 g_last_get_time[VENC_MAX_CHN_NUM] = {0};
static hi_u64 g_start_send_time;
static HiSampleJpegeGetStreamPara g_jpege_para;
static HiSampleJpegeGetStreamPara g_jpege_get_stream_para[MAX_JPEGE_CHN];
static int32_t g_jpege_epoll_fd = -1;

extern uint32_t g_per_count;
extern uint32_t g_mem_count;
// Notify user to obtain streams, 0:Single-thread and multi-channel, Non-zero:Single-thread single-channel
extern uint32_t g_select_one_thread;
extern uint32_t g_chn_num_start;
extern uint32_t g_chn_num;
extern char g_output_file_name[500];
extern uint32_t g_set_outfile;
extern uint32_t g_save;
extern uint32_t g_is_syn_enc;
extern uint32_t g_frameCount;
extern uint32_t g_isZeroCopy;
extern aclrtContext g_context;
extern aclrtRunMode g_run_mode;

/*
* function:    delay_exec(uint64_t execTime, int delaySeconds)
* Description: This Parameter Is Used To Set The Delay For Concurrent Threads
* input:       execTime:Start Time, delaySeconds:Extension time(s)
* output:      none
* return:      none
* others:      none
*/
void delay_exec(uint64_t execTime, int delaySeconds)
{
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);
    long tmpCurTime = currentTime.tv_sec * 1000000 + currentTime.tv_usec;

    // 延时10s后再调用createDvppApi
    long nextExcTime = execTime + delaySeconds * 1000000;

    while (tmpCurTime < nextExcTime) {
        usleep(500);
        gettimeofday(&currentTime, NULL);
        tmpCurTime = currentTime.tv_sec * 1000000 + currentTime.tv_usec;
    }
    SAMPLE_PRT("tmpCurTime = %ld ms!\n", tmpCurTime / 1000);
    return;
}

/*
* function:    get_pic_buffer_config(uint32_t inputWidth, uint32_t inputHeight, hi_pixel_format pixelFormat,
*                  hi_data_bit_width inputBitWidth, hi_compress_mode cmpMode, uint32_t align, HiSampleCalConfig *calConfig)
* Description: Obtains The Configuration Of input Buffer Of Input Picture To Be Encoded
* input:       inputWidth:input picture width, inputHeight:input picture height, pixelFormat: input picture format
*              inputBitWidth:input picture bitwidth, cmpMode:Compression Mode,
               align:alignment parameters, calConfig:parameters to be calculated
* output:      none
* return:      none
* others:      none
*/
void get_pic_buffer_config(uint32_t inputWidth, uint32_t inputHeight, hi_pixel_format pixelFormat,
    hi_data_bit_width inputBitWidth, hi_compress_mode cmpMode, uint32_t align, HiSampleCalConfig *calConfig)
{
    uint32_t bitWidth = 0;
    uint32_t headStride = 0;
    uint32_t vbSize = 0; // recalculate, the others for default value
    uint32_t headSize = 0;
    uint32_t alignHeight;
    uint32_t mainStride = 0; // recalculate
    uint32_t mainSize = 0; // recalculate
    uint32_t exStride = 0;
    uint32_t exYSize = 0;
    uint32_t headYSize = 0;
    uint32_t ySize = 0; // recalculate

    if ((inputWidth > HI_MAXINUM_LIMIT) || (inputHeight > HI_MAXINUM_LIMIT)) {
        calConfig->vbSize = 0;
    }

    // align: 0(Auto Align), Others(Align Size)
    if (align == 0) {
        align = DEFAULT_ALIGN;
    } else if (align > MAX_ALIGN) {
        align = MAX_ALIGN;
    } else {
        align = (ALIGN_UP(align, DEFAULT_ALIGN));
    }

    switch (inputBitWidth) {
        case HI_DATA_BIT_WIDTH_8: {
            bitWidth = 8;
            break;
        }
        case HI_DATA_BIT_WIDTH_10: {
            bitWidth = 10;
            break;
        }
        case HI_DATA_BIT_WIDTH_12: {
            bitWidth = 12;
            break;
        }
        case HI_DATA_BIT_WIDTH_14: {
            bitWidth = 14;
            break;
        }
        case HI_DATA_BIT_WIDTH_16: {
            bitWidth = 16;
            break;
        }
        default:
        {
            bitWidth = 0;
            break;
        }
    }

    alignHeight = ALIGN_UP(inputHeight, 2);

    if (cmpMode == HI_COMPRESS_MODE_NONE) {
        mainStride = ALIGN_UP((inputWidth * bitWidth + 7) >> 3, align);
        if ((pixelFormat >= HI_PIXEL_FORMAT_YUYV_PACKED_422) && (pixelFormat <= HI_PIXEL_FORMAT_VYUY_PACKED_422)) {
            mainStride = ALIGN_UP(((inputWidth * bitWidth + 7) >> 3) * 2, align); // width的2倍后对齐
        }
        ySize = mainStride * alignHeight;

        if ((pixelFormat == HI_PIXEL_FORMAT_YVU_SEMIPLANAR_420) ||
            (pixelFormat == HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420)) {
            mainSize = (mainStride * alignHeight * 3) >> 1;
        } else if ((pixelFormat == HI_PIXEL_FORMAT_YVU_SEMIPLANAR_422) ||
                   (pixelFormat == HI_PIXEL_FORMAT_YUV_SEMIPLANAR_422)) {
            mainSize = mainStride * alignHeight * 2;
        } else if (pixelFormat == HI_PIXEL_FORMAT_YUV_400) {
            mainSize = mainStride * alignHeight;
        } else if ((pixelFormat >= HI_PIXEL_FORMAT_YUYV_PACKED_422) &&
                   (pixelFormat <= HI_PIXEL_FORMAT_VYUY_PACKED_422)) {
            mainSize = mainStride * alignHeight;
        } else {
            mainSize = mainStride * alignHeight * 3;
        }
        vbSize = mainSize;
    }

    calConfig->vbSize = vbSize;
    calConfig->headYSize = headYSize;
    calConfig->headSize = headSize;
    calConfig->headStride = headStride;
    calConfig->mainStride = mainStride;
    calConfig->mainYSize = ySize;
    calConfig->mainSize = mainSize;
    calConfig->exStride = exStride;
    calConfig->exYSize = exYSize;

    return;
}

/*
* function:    get_pic_buffer_size(uint32_t inputWidth, uint32_t inputHeight, hi_pixel_format pixelFormat,
*                               hi_data_bit_width inputBitWidth, hi_compress_mode cmpMode, uint32_t align)
* Description: get input picture buffer size
* input:       inputWidth: input width, inputHeight: input height pixelFormat: input format,
*              inputBitWidth: input bit width, cmpMode: compress mdoe，align: align parameter
* output:      buffer size
* return:      vbSize
* others:      无
*/
uint32_t get_pic_buffer_size(uint32_t inputWidth, uint32_t inputHeight, hi_pixel_format pixelFormat,
    hi_data_bit_width inputBitWidth, hi_compress_mode cmpMode, uint32_t align)
{
    HiSampleCalConfig calConfig;

    get_pic_buffer_config(inputWidth, inputHeight, pixelFormat, inputBitWidth, cmpMode, align, &calConfig);

    return calConfig.vbSize;
}

/*
* function:    alloc_input_buffer(uint32_t inputWidth, uint32_t inputHeight, hi_pixel_format pixelFormat,
*                  hi_data_bit_width inputBitWidth, hi_compress_mode cmpMode, uint32_t align, hi_video_frame_info *inputFrame)
* Description: Alloc Input Buffer
* input:       inputWidth:input picture width, inputHeight:input picture height, pixelFormat:input picture format
*              inputBitWidth:bit width of the input image, cmpMode:Compression Mode,
               align:Alignment Parameters,inputFrame:input stream buffer
* output:      Whether the input memory is successfully applied
* return:      HI_SUCCESS:success, HI_FAILURE:fail
* others:      none
*/
int32_t alloc_input_buffer(uint32_t inputWidth, uint32_t inputHeight, hi_pixel_format pixelFormat,
    hi_data_bit_width inputBitWidth, hi_compress_mode cmpMode, uint32_t align, hi_video_frame_info *inputFrame)
{
    HiSampleCalConfig calConfig;
    CHECK_NULL_PTR(inputFrame);
    memset(inputFrame, 0, sizeof(hi_video_frame_info));

    get_pic_buffer_config(inputWidth, inputHeight, pixelFormat, inputBitWidth, cmpMode, align, &calConfig);

    void* inputBuf = NULL;
    int32_t ret = hi_mpi_dvpp_malloc(0, &inputBuf, calConfig.vbSize);
    if (inputBuf == NULL) {
        SAMPLE_PRT("hi_mpi_dvpp_malloc fail size = %u\n", calConfig.vbSize);
        return HI_FAILURE;
    }

    inputFrame->pool_id = 0;
    inputFrame->mod_id = HI_ID_VGS;
    inputFrame->v_frame.width = inputWidth;
    inputFrame->v_frame.height = inputHeight;
    inputFrame->v_frame.dynamic_range = HI_DYNAMIC_RANGE_SDR8; // Dynamic Range
    inputFrame->v_frame.compress_mode = cmpMode; // Compression Mode
    inputFrame->v_frame.pixel_format = pixelFormat; // Pixel format
    inputFrame->v_frame.video_format = HI_VIDEO_FORMAT_LINEAR; // Video format
    inputFrame->v_frame.field = HI_VIDEO_FIELD_FRAME; // Frame Or Field Mode
    inputFrame->v_frame.color_gamut = HI_COLOR_GAMUT_BT709; // Gamut range
    inputFrame->v_frame.header_stride[0] = calConfig.headStride; // Image compression head span
    inputFrame->v_frame.header_stride[1] = calConfig.headStride;
    // Image data span
    inputFrame->v_frame.width_stride[0] = calConfig.mainStride;
    inputFrame->v_frame.width_stride[1] = calConfig.mainStride;

    inputFrame->v_frame.header_virt_addr[0] = inputBuf; // Compression header virtual address

    inputFrame->v_frame.header_virt_addr[1] =
        (hi_void*)((uintptr_t)inputFrame->v_frame.header_virt_addr[0] + calConfig.headYSize);
    // virtual address
    inputFrame->v_frame.virt_addr[0] =
        (hi_void*)((uintptr_t)inputFrame->v_frame.header_virt_addr[0] + calConfig.headSize);
    inputFrame->v_frame.virt_addr[1] =
        (hi_void*)((uintptr_t)inputFrame->v_frame.virt_addr[0] + calConfig.mainYSize);

    return HI_SUCCESS;
}

/*
* function:    read_yuv_file(FILE *yuv, hi_video_frame_info *videoFrame,
*                          hi_pixel_format format, int64_t *fileOffset, hi_venc_chn chn, int32_t count)
* Description: Read The Input YUV File
* input:       yuv: enter the file path and file name, videoFrame：input buffer,
*              format:input picture format, fileOffset: read file offset, chn:chnl_id,
               count:number of input image frames
* output:      Whether the input YUV file is read successfully
* return:      HI_SUCCESS:success, HI_FAILURE:fail
* others:      none
*/
int32_t read_yuv_file(FILE *yuv, hi_video_frame_info *videoFrame,
    hi_pixel_format format, hi_s64 *fileOffset, hi_venc_chn chn, int32_t count)
{
    CHECK_NULL_PTR(yuv);
    CHECK_NULL_PTR(videoFrame);

    hi_s64 usedBytes = *fileOffset;
    hi_s64 readLen = 0;
    unsigned int w = 0;
    unsigned int h = 0;
    char *yBufVir = nullptr;
    char *cBufVir = nullptr;
    char *memContent = nullptr;
    hi_char *userPageAddr[2] = {nullptr};
    hi_char* hostAddr = NULL;
    uint32_t bufSize = 0;

    if ((format == HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420) || (format == HI_PIXEL_FORMAT_YVU_SEMIPLANAR_420)) {
        bufSize = videoFrame->v_frame.width_stride[0] * videoFrame->v_frame.height * 3 / 2;
    } else {
        bufSize = videoFrame->v_frame.width_stride[0] * videoFrame->v_frame.height;
    }

    if (g_run_mode == ACL_HOST) {
        auto aclRet = aclrtMallocHost((void **)&hostAddr, bufSize);
        if (aclRet != ACL_SUCCESS) {
            SAMPLE_PRT("Malloc host memory %u fail with %d.\n", bufSize, aclRet);
            return HI_FAILURE;
        }
        userPageAddr[0] = (hi_char *)hostAddr;
    } else {
        userPageAddr[0] = (hi_char *)videoFrame->v_frame.virt_addr[0];
    }
    if (HI_NULL == userPageAddr[0]) {
        SAMPLE_PRT("userPageAddr[0] is NULL.\n");
        return HI_FAILURE;
    }

    if ((format == HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420) || (format == HI_PIXEL_FORMAT_YVU_SEMIPLANAR_420)) {
        yBufVir = userPageAddr[0];
        cBufVir = yBufVir + (videoFrame->v_frame.width_stride[0]) * (videoFrame->v_frame.height);
        // y data
        for (h = 0; h < videoFrame->v_frame.height; h++) {
            memContent = yBufVir + h * videoFrame->v_frame.width_stride[0];
            fseek(yuv, usedBytes, SEEK_SET);
            readLen = fread(memContent, 1, videoFrame->v_frame.width, yuv);
            if (readLen < 1) {
                SAMPLE_PRT("fill buffer y data h %u, readLen %lld \n", h, readLen);
                fseek(yuv, usedBytes, SEEK_SET);
                if (g_run_mode == ACL_HOST) {
                    aclrtFreeHost(hostAddr);
                }
                return HI_FAILURE;
            }
            usedBytes = usedBytes + readLen;
        }

        fseek(yuv, usedBytes, SEEK_SET);
        for (h = 0; h < videoFrame->v_frame.height / 2; h++) {
            memContent = cBufVir + h * videoFrame->v_frame.width_stride[0];
            fseek(yuv, usedBytes, SEEK_SET);
            readLen = fread(memContent, 1, videoFrame->v_frame.width, yuv);
            if (readLen < 1) {
                SAMPLE_PRT("fill buffer c data h %u, readLen %lld", h, readLen);
                fseek(yuv, usedBytes, SEEK_SET);
                if (g_run_mode == ACL_HOST) {
                    aclrtFreeHost(hostAddr);
                }
                return HI_FAILURE;
            }
            usedBytes = usedBytes + readLen;
        }
    } else {
        yBufVir = userPageAddr[0];
        for (h = 0; h < videoFrame->v_frame.height; h++) {
            memContent = yBufVir + h * videoFrame->v_frame.width_stride[0];
            fseek(yuv, usedBytes, SEEK_SET);
            readLen = fread(memContent, 1, videoFrame->v_frame.width * 2, yuv);
            if (readLen < 1) {
                SAMPLE_PRT("fill buffer yuv data h %u, readLen %lld \n", h, readLen);
                fseek(yuv, usedBytes, SEEK_SET);
                if (g_run_mode == ACL_HOST) {
                    aclrtFreeHost(hostAddr);
                }
                return HI_FAILURE;
            }
            usedBytes = usedBytes + readLen;
        }
    }

    if (g_run_mode == ACL_HOST) {
        aclError aclRet = aclrtMemcpy((void*)(videoFrame->v_frame.virt_addr[0]), bufSize,
            hostAddr, bufSize, ACL_MEMCPY_HOST_TO_DEVICE);
        if (aclRet != ACL_SUCCESS) {
            SAMPLE_PRT("Copy host memcpy to device fail with %d.\n", aclRet);
        }
        aclRet = aclrtFreeHost(hostAddr);
        if (aclRet != ACL_SUCCESS) {
            SAMPLE_PRT("aclrtFreeHost fail with %d.\n", aclRet);
        }
    }

    *fileOffset = usedBytes;
    return HI_SUCCESS;
}

/*
* function:    jpege_save_stream(FILE* fd, hi_venc_stream* stream)
* Description: Save Output Streams
* input:       fd:file handle, stream:input stream
* output:      Whether the output stream is saved successfully
* return:      HI_SUCCESS:Success, HI_FAILURE:fail
* others:      none
*/
int32_t jpege_save_stream(FILE* fd, hi_venc_stream* stream)
{
    for (int32_t i = 0; i < stream->pack_cnt; i++) {
        if ((stream->pack[i].addr == NULL) || (stream->pack[i].len == 0)) {
            SAMPLE_PRT("jpeg encode error!\n");
            return HI_FAILURE;
        }
        if (g_run_mode == ACL_HOST) {
            uint32_t dataLen = stream->pack[i].len - stream->pack[i].offset;
            char* pcData = new char[dataLen];
            auto aclRet = aclrtMemcpy(pcData, dataLen, stream->pack[i].addr + stream->pack[i].offset,
                                    dataLen, ACL_MEMCPY_DEVICE_TO_HOST);
            if (aclRet != ACL_SUCCESS) {
                SAMPLE_PRT("aclrtMemcpy i:%d fail %d pcData:%lx,dataLen:%u,addr:%lx,offset:%d ",
                    i, aclRet, (uint64_t)pcData, dataLen, (uint64_t)stream->pack[i].addr, stream->pack[i].offset);
                delete[] pcData;
                pcData = NULL;
                return HI_FAILURE;
            }

            fwrite(pcData, dataLen, 1, fd);
            fflush(fd);
            delete[] pcData;
            pcData = NULL;
        } else {
            fwrite(stream->pack[i].addr + stream->pack[i].offset,
               stream->pack[i].len - stream->pack[i].offset, 1, fd);
            fflush(fd);
        }
    }

    return HI_SUCCESS;
}

int32_t jpege_save_stream_to_file(hi_venc_chn vencChn, int32_t chnImgCnt, hi_venc_stream* stream)
{
    int32_t ret = HI_SUCCESS;
    char originalFile[FILE_NAME_LEN] = {0};
    FILE* pFile = nullptr;

    if (g_set_outfile != 0) {
        snprintf(originalFile, FILE_NAME_LEN, "%s_%d_%d", g_output_file_name, vencChn, chnImgCnt);
    } else {
        snprintf(originalFile, FILE_NAME_LEN, "snap_chnl%d_no%d.jpg", vencChn, chnImgCnt);
    }

    pFile = fopen(originalFile, "wb");
    if (pFile == NULL) {
        SAMPLE_PRT("open output file err\n");
        return HI_FAILURE;
    }

    ret = jpege_save_stream(pFile, stream);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("save snap picture failed!\n");
        fclose(pFile);
        pFile = NULL;
        return HI_FAILURE;
    }

    fclose(pFile);
    pFile = NULL;
    return ret;
}

int32_t alloc_output_buffer(hi_video_frame_info *frame, void** outputBuf, hi_u32 *outputBufSize)
{
    hi_s32 ret = HI_SUCCESS;
    // get the size of output buffer
    hi_venc_jpeg_param stParamJpeg;
    hi_u32 out_buffer_size = 0;
    hi_u32 i;
    stParamJpeg.qfactor = 100;
    for (i = 0; i < HI_VENC_JPEG_QT_COEF_NUM; i++) {
        stParamJpeg.y_qt[i] = 255;
        stParamJpeg.cb_qt[i] = 255;
        stParamJpeg.cr_qt[i] = 255;
    }
    ret = hi_mpi_venc_get_jpege_predicted_size(frame, &stParamJpeg, &out_buffer_size);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_venc_get_jpege_predicted_size err 0x%x\n", ret);
        return ret;
    }

    // malloc output buffer
    ret = hi_mpi_dvpp_malloc(0, outputBuf, out_buffer_size);
    if ((ret != HI_SUCCESS) || (*outputBuf == nullptr)) {
        SAMPLE_PRT("hi_mpi_dvpp_malloc fail size = %d, ret = %d\n", out_buffer_size, ret);
        return HI_FAILURE;
    }
    *outputBufSize = out_buffer_size;

    return HI_SUCCESS;
}

int32_t start_sync_encode(hi_venc_chn vencChn, hi_video_frame_info *videoFrame, int32_t supportPerformance)
{
    int32_t ret = HI_SUCCESS;
    int i = 0;
    hi_venc_stream stream;
    hi_u64 getStreamTime, elapseTime;
    double sumTime = 0;
    stream.pack = (hi_venc_pack*)malloc(sizeof(hi_venc_pack));
    if (stream.pack == NULL) {
        SAMPLE_PRT("malloc memory failed!\n");
        return HI_FAILURE;
    }

    void* outputBuf = nullptr;
    hi_u32 outputBufSize = 0;
    if (g_isZeroCopy) {
        ret = alloc_output_buffer(videoFrame, &outputBuf, &outputBufSize);
        if (ret != HI_SUCCESS) {
            free(stream.pack);
            stream.pack = NULL;
            return HI_FAILURE;
        }
        SAMPLE_PRT("output buffer addr:0x%p, size:%u\n", outputBuf, outputBufSize);
    }
    for (i = 0; i < g_per_count; ++i) {
        // send frame
        if (g_isZeroCopy) {
            hi_img_stream out_stream;
            out_stream.addr = (hi_u8 *)outputBuf;
            out_stream.len = outputBufSize;
            SAMPLE_GET_TIMESTAMP_US(videoFrame->v_frame.pts);
            ret = hi_mpi_venc_send_jpege_frame(vencChn, videoFrame, &out_stream, 10000);
        } else {
            SAMPLE_GET_TIMESTAMP_US(videoFrame->v_frame.pts);
            ret = hi_mpi_venc_send_frame(vencChn, videoFrame, 10000);
        }
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("send frame failed, ret:0x%x\n", ret);
            break;
        }
        g_jpege_send_frame_cnt[vencChn]++;

        // get stream
        stream.pack_cnt = 1;
        ret = hi_mpi_venc_get_stream(vencChn, &stream, -1);
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("hi_mpi_venc_get_stream failed with %#x!\n", ret);
            break;
        }
        SAMPLE_GET_TIMESTAMP_US(getStreamTime);
        elapseTime = getStreamTime - videoFrame->v_frame.pts;
        sumTime = sumTime + (double)elapseTime;
        SAMPLE_PRT("Jpege chn: %d getStream %llu encTime: %llu\n",
            vencChn, g_jpege_send_frame_cnt[vencChn], elapseTime);
        fflush(stdout);

        if (g_save) {
            ret = jpege_save_stream_to_file(vencChn, g_jpege_send_frame_cnt[vencChn], &stream);
            if (ret != HI_SUCCESS) {
                SAMPLE_PRT("jpege_save_stream_to_file failed with %#x!\n", ret);
                break;
            }
        }
        // release stream
        ret = hi_mpi_venc_release_stream(vencChn, &stream);
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("hi_mpi_venc_release_stream failed with %#x!\n", ret);
            break;
        }
    }

    // performance statistics
    if (supportPerformance && (i == g_per_count)) {
        double avgTime = sumTime / g_per_count;
        double fps = 1000000 / avgTime;
        SAMPLE_PRT("Jpege chn: %d actualFrameRate %.4lf,averageTime %.4lf us,encodeFrame %llu\n",
            vencChn, fps, avgTime, g_jpege_send_frame_cnt[vencChn]);
        fflush(stdout);
    }

    if (g_isZeroCopy) {
        // free output buffer
        ret = hi_mpi_dvpp_free(outputBuf);
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("hi_mpi_dvpp_free failed, ret = %d!\n", ret);
        }
    }

    free(stream.pack);
    stream.pack = NULL;

    return HI_SUCCESS;
}

void sync_encode(FILE *fpYuv, HiSampleJpegeSendFramePara* jpegeSendPara)
{
    int32_t ret = HI_SUCCESS;
    hi_venc_chn vencChn = jpegeSendPara->vencChn;
    hi_video_frame_info *videoFrame = nullptr;

    uint32_t width = jpegeSendPara->inputPara.width;
    uint32_t height = jpegeSendPara->inputPara.height;
    uint32_t align = jpegeSendPara->inputPara.align;
    hi_pixel_format pixelFormat = jpegeSendPara->inputPara.pixelFormat;
    hi_data_bit_width bitWidth = jpegeSendPara->inputPara.bitWidth;
    hi_compress_mode cmpMode = jpegeSendPara->inputPara.cmpMode;
    hi_video_frame_info* tmp = nullptr;

    int32_t count = 0;
    hi_s64 yuvFileReadOff = 0;

    while (jpegeSendPara->threadStart == HI_TRUE) {
        tmp = (hi_video_frame_info *)malloc(sizeof(hi_video_frame_info));
        if (tmp == NULL) {
            SAMPLE_PRT("chn %d malloc input buffer failed \n", vencChn);
            jpegeSendPara->threadStart = HI_FALSE;
            break;
        }

        // 2. alloc input buffer
        ret = alloc_input_buffer(width, height, pixelFormat, bitWidth, cmpMode, align, tmp);
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("chn %d alloc_input_buffer failed \n", vencChn);
            if (tmp != NULL) {
                free(tmp);
                tmp = NULL;
                SAMPLE_PRT("free hi_video_frame_info \n");
            }
            jpegeSendPara->threadStart = HI_FALSE;
            break;
        }

        do {
            // 3. read input file
            videoFrame = tmp;
            ret = read_yuv_file(fpYuv, videoFrame, pixelFormat, &yuvFileReadOff, vencChn, count);
            if (ret != HI_SUCCESS) {
                SAMPLE_PRT("read yuv file fail, offset %lld \n", yuvFileReadOff);
                break;
            }
            count++;

            // 4. start_encode
            ret = start_sync_encode(vencChn, videoFrame, jpegeSendPara->supportPerformance);
            if (ret != HI_SUCCESS) {
                SAMPLE_PRT("send frame fail \n");
                break;
            }
        } while(false);

        if (videoFrame->v_frame.header_virt_addr[0] != nullptr) {
            hi_mpi_dvpp_free((void*)(videoFrame->v_frame.header_virt_addr[0]));
            SAMPLE_PRT("hi_mpi_dvpp_free input buffer \n");
        }

        if (tmp != NULL) {
            free(tmp);
            tmp = NULL;
            SAMPLE_PRT("free hi_video_frame_info \n");
        }

        jpegeSendPara->threadStart = HI_FALSE;
    }
    fflush(stdout);
}

void* jpege_sync_enc(void *p)
{
    hi_char inputFileName[64];
    FILE *fpYuv = NULL;
    HiSampleJpegeSendFramePara* jpegeSendPara = nullptr;

    aclError acl_ret = aclrtSetCurrentContext(g_context);
    if (acl_ret != ACL_SUCCESS) {
        SAMPLE_PRT("set current context failed");
        return NULL;
    }

    if (p == nullptr) {
        SAMPLE_PRT("input param is nullptr");
        return NULL;
    }

    jpegeSendPara = (HiSampleJpegeSendFramePara*) p;

    // open file fp
    snprintf(inputFileName, sizeof(inputFileName), "%s", jpegeSendPara->inputPara.inputFileName);

    fpYuv = fopen(inputFileName, "rb");
    if (fpYuv == NULL) {
        SAMPLE_PRT("can't open file %s in send stream thread!\n", inputFileName);
        return NULL;
    }

    sync_encode(fpYuv, jpegeSendPara);

    if (fpYuv != NULL) {
        fclose(fpYuv);
        fpYuv = NULL;
    }

    return NULL;
}

/*
* function:    jpege_start_sync_enc(hi_venc_chn vencChn[], int32_t cnt, int32_t per, HiSampleVencSendFrameInput* inputPara)
* Description: Create A Thread for jpeg encode in sychronize mode
* input:       vencChn:chnl_id, cnt:number of frames, per:whether performance test or not,
               inputPara:Input file parameters
* output:      Whether the thread for sending input data is successfully created
* return:      HI_SUCCESS:success, HI_FAILURE:fail
* others:      none
*/
int32_t jpege_start_sync_enc(hi_venc_chn vencChn[], int32_t cnt, int32_t per, HiSampleVencSendFrameInput* inputPara)
{
    int32_t i = 0;
    int32_t j = 0;
    int32_t ret = HI_SUCCESS;
    for (i = 0; i < cnt; ++i) {
        g_jpege_send_frame_para[i].threadStart = HI_TRUE;
        g_jpege_send_frame_para[i].chnlCnt = cnt;
        g_jpege_send_frame_para[i].inputPara = *inputPara;
        g_jpege_send_frame_para[i].vencChn = vencChn[i];
        g_jpege_send_frame_para[i].supportPerformance = per;

        ret = pthread_create(&g_jpege_pid[i], NULL, jpege_sync_enc, (void*)&g_jpege_send_frame_para[i]);
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("thread create failed %d\n", ret);
            ret = HI_FAILURE;
            break;
        }
    }
    for (j = 0; j < i; ++j) {
        pthread_join(g_jpege_pid[j], NULL);
    }
    SAMPLE_PRT("test finished\n");
    return ret;
}

/*
* function:    jpege_start_send_frame(hi_venc_chn vencChn[], int32_t cnt, int32_t per, HiSampleVencSendFrameInput* inputPara)
* Description: Create A Thread for Sending Input Data
* input:       vencChn:chnl_id, cnt:number of frames, per:whether performance test or not,
               inputPara:Input file parameters
* output:      Whether the thread for sending input data is successfully created
* return:      HI_SUCCESS:success, HI_FAILURE:fail
* others:      none
*/
int32_t jpege_start_send_frame(hi_venc_chn vencChn[], int32_t cnt, int32_t per, HiSampleVencSendFrameInput* inputPara)
{
    int32_t ret = HI_SUCCESS;
    uint32_t i = 0;
    SAMPLE_GET_TIMESTAMP_US(g_start_send_time);
    SAMPLE_PRT("g_start_send_time = %lld ms!\n", g_start_send_time / 1000);
    for(i = 0; i < cnt; i++) {
        g_jpege_send_frame_para[i].threadStart = HI_TRUE;
        g_jpege_send_frame_para[i].chnlCnt = cnt;
        g_jpege_send_frame_para[i].inputPara = *inputPara;
        g_jpege_send_frame_para[i].vencChn = vencChn[i];
        g_jpege_send_frame_para[i].supportPerformance = per;
        // The performance test uses the method of applying for all memory before encoding
        if (g_jpege_send_frame_para[i].supportPerformance) {
            ret = pthread_create(&g_jpege_send_frame_pid[i], 0, jpege_snap_send_frame_dc_performance,
                (void*)&g_jpege_send_frame_para[i]);
            if (ret != HI_SUCCESS) {
                SAMPLE_PRT("thread %u create failed !\n", i);
                g_jpege_send_frame_para[i].threadStart = HI_FALSE;
                return HI_FAILURE;
            }
        } else {
            ret = pthread_create(&g_jpege_send_frame_pid[i], 0, jpege_snap_send_frame,
                (void*)&g_jpege_send_frame_para[i]);
            if (ret != HI_SUCCESS) {
                SAMPLE_PRT("thread %u create failed !\n", i);
                g_jpege_send_frame_para[i].threadStart = HI_FALSE;
                return HI_FAILURE;
            }
        }
    }
    return ret;
}

int32_t send_one_frame(hi_venc_chn vencChn, hi_video_frame_info *videoFrame)
{
    int32_t ret = HI_SUCCESS;
    if (g_isZeroCopy) {
        void* outputBuf = nullptr;
        hi_u32 outputBufSize = 0;
        ret = alloc_output_buffer(videoFrame, &outputBuf, &outputBufSize);
        if (ret != HI_SUCCESS) {
            return HI_FAILURE;
        }
        SAMPLE_PRT("output buffer addr:0x%p, size:%u\n", outputBuf, outputBufSize);

        hi_img_stream out_stream;
        out_stream.addr = (hi_u8 *)outputBuf;
        out_stream.len = outputBufSize;

        if (g_start_send_time_array[vencChn] == 0) {
            SAMPLE_GET_TIMESTAMP_US(g_start_send_time_array[vencChn]);
        }
        SAMPLE_GET_TIMESTAMP_US(videoFrame->v_frame.pts);
        ret = hi_mpi_venc_send_jpege_frame(vencChn, videoFrame, &out_stream, 10000);
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("hi_mpi_venc_send_jpege_frame failed, ret:0x%x\n", ret);
            // free output buffer
            (void)hi_mpi_dvpp_free(outputBuf);
            return HI_FAILURE;
        }
    } else {
        if (g_start_send_time_array[vencChn] == 0) {
            SAMPLE_GET_TIMESTAMP_US(g_start_send_time_array[vencChn]);
        }
        SAMPLE_GET_TIMESTAMP_US(videoFrame->v_frame.pts);
        ret = hi_mpi_venc_send_frame(vencChn, videoFrame, 10000);
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("hi_mpi_venc_send_frame failed, ret:0x%x\n", ret);
            return HI_FAILURE;
        }
    }
    g_jpege_send_frame_cnt[vencChn]++;
    return HI_SUCCESS;
}

int32_t start_async_send_frame(hi_venc_chn vencChn, hi_video_frame_info *videoFrame, int32_t supportPerformance)
{
    int32_t ret = HI_SUCCESS;
    int32_t i = 0;
    if (supportPerformance) { // performance statistic
        for (i = 0; i < g_per_count; ++i) {
            ret = send_one_frame(vencChn, videoFrame);
            if (ret != HI_SUCCESS) {
                break;
            }
        }
    } else {
        ret = send_one_frame(vencChn, videoFrame);
    }

    return ret;
}

void async_send_frame(FILE *fpYuv, HiSampleJpegeSendFramePara* jpegeSendPara)
{
    int32_t ret = HI_SUCCESS;
    hi_venc_chn vencChn = jpegeSendPara->vencChn;
    hi_video_frame_info *videoFrame = nullptr;

    uint32_t width = jpegeSendPara->inputPara.width;
    uint32_t height = jpegeSendPara->inputPara.height;
    uint32_t align = jpegeSendPara->inputPara.align;
    hi_pixel_format pixelFormat = jpegeSendPara->inputPara.pixelFormat;
    hi_data_bit_width bitWidth = jpegeSendPara->inputPara.bitWidth;
    hi_compress_mode cmpMode = jpegeSendPara->inputPara.cmpMode;
    hi_video_frame_info* tmp = nullptr;

    int32_t count = 0;
    hi_s64 yuvFileReadOff = 0;

    while (jpegeSendPara->threadStart == HI_TRUE) {
        // 2. alloc input buffer
        tmp = (hi_video_frame_info *)malloc(sizeof(hi_video_frame_info));
        if (tmp == NULL) {
            SAMPLE_PRT("malloc input buffer failed \n");
            jpegeSendPara->threadStart = HI_FALSE;
            break;
        }
        ret = alloc_input_buffer(width, height, pixelFormat, bitWidth, cmpMode, align, tmp);
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("alloc_input_buffer failed \n");
            if (tmp != NULL) {
                free(tmp);
                tmp = NULL;
                SAMPLE_PRT("free hi_video_frame_info \n");
            }
            jpegeSendPara->threadStart = HI_FALSE;
            break;
        }

        bool isAbnormal = false;
        do {
            // 3. read file
            videoFrame = tmp;
            ret = read_yuv_file(fpYuv, videoFrame, pixelFormat, &yuvFileReadOff, vencChn, count);
            if (ret != HI_SUCCESS) {
                SAMPLE_PRT("read yuv file fail, offset %lld \n", yuvFileReadOff);
                isAbnormal = true;
                break;
            }

            count++;

            // 4. send image
            ret = start_async_send_frame(vencChn, videoFrame, jpegeSendPara->supportPerformance);
            if (ret != HI_SUCCESS) {
                SAMPLE_PRT("async send frame fail \n");
                isAbnormal = true;
                break;
            }
        } while(false);

        // free input buffer
        if (isAbnormal && (videoFrame->v_frame.header_virt_addr[0] != nullptr)) {
            hi_mpi_dvpp_free((void*)(videoFrame->v_frame.header_virt_addr[0]));
            SAMPLE_PRT("hi_mpi_dvpp_free input buffer \n");
        }

        if (tmp != NULL) {
            free(tmp);
            tmp = NULL;
            SAMPLE_PRT("free hi_video_frame_info \n");
        }
        if (jpegeSendPara->supportPerformance || (count == g_frameCount) || isAbnormal) {
            jpegeSendPara->threadStart = HI_FALSE;
        } else {
            sleep(3); // 3s is set to prevent the buffer from being full because the data is transmitted too fast.
        }
    }
    fflush(stdout);
}

/*
* function:    jpege_snap_send_frame(void* p)
* Description: Start Send Input Data
* input:       p:address of the function
* output:      none
* return:      none
* others:      none
*/
void* jpege_snap_send_frame(void* p)
{
    hi_char inputFileName[64];
    FILE *fpYuv = nullptr;
    HiSampleJpegeSendFramePara* jpegeSendPara = nullptr;

    aclError acl_ret = aclrtSetCurrentContext(g_context);
    if (acl_ret != ACL_SUCCESS) {
        SAMPLE_PRT("set current context failed");
        return NULL;
    }

    if (p == nullptr) {
        SAMPLE_PRT("input param is nullptr");
        return NULL;
    }

    jpegeSendPara = (HiSampleJpegeSendFramePara*) p;

    snprintf(inputFileName, sizeof(inputFileName), "%s", jpegeSendPara->inputPara.inputFileName);

    fpYuv = fopen(inputFileName, "rb");
    if (fpYuv == NULL) {
        jpegeSendPara->threadStart = HI_FALSE;
        SAMPLE_PRT("can't open file in send stream thread!\n");
        return NULL;
    }

    async_send_frame(fpYuv, jpegeSendPara);

    if (fpYuv != NULL) {
        fclose(fpYuv);
        fpYuv = NULL;
    }

    return NULL;
}

int32_t alloc_input_buffer_for_dc_performance(FILE *fpYuv, HiSampleJpegeSendFramePara* jpegeSendPara,
    hi_video_frame_info **tmp, hi_video_frame_info **videoFrame)
{
    int32_t ret = HI_SUCCESS;
    hi_venc_chn vencChn = jpegeSendPara->vencChn;

    uint32_t width = jpegeSendPara->inputPara.width;
    uint32_t height = jpegeSendPara->inputPara.height;
    uint32_t align = jpegeSendPara->inputPara.align;
    hi_pixel_format pixelFormat = jpegeSendPara->inputPara.pixelFormat;
    hi_data_bit_width bitWidth = jpegeSendPara->inputPara.bitWidth;
    hi_compress_mode cmpMode = jpegeSendPara->inputPara.cmpMode;

    int32_t count = 0;
    hi_s64 yuvFileReadOff = 0;
    int m = 0;

    for (m = 0; m < g_mem_count; ++m) {
        // 2. alloc input Buffer
        tmp[m] = (hi_video_frame_info *)malloc(sizeof(hi_video_frame_info));
        if (tmp[m] == NULL) {
            SAMPLE_PRT("malloc input buffer failed \n");
            break;
        }
        ret = alloc_input_buffer(width, height, pixelFormat, bitWidth, cmpMode, align, tmp[m]);
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("alloc_input_buffer failed \n");
            free(tmp[m]);
            tmp[m] = NULL;
            break;
        }

        // 3. read input file
        videoFrame[m] = tmp[m];
        ret = read_yuv_file(fpYuv, videoFrame[m], pixelFormat, &yuvFileReadOff, vencChn, count);
        yuvFileReadOff = 0;
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("read yuv file fail, offset %lld \n", yuvFileReadOff);
            free(tmp[m]);
            tmp[m] = NULL;

            // release input buffer
            ret = hi_mpi_dvpp_free((void*)(videoFrame[m]->v_frame.header_virt_addr[0]));
            if (ret != HI_SUCCESS) {
                SAMPLE_PRT("hi_mpi_dvpp_free failed!\n");
            }
            break;
        }
    }

    fflush(stdout);

    if (m == g_mem_count) {
        return HI_SUCCESS;
    }

    for (int i = 0; i < m; ++i) {
        if (tmp[i] != NULL) {
            free(tmp[m]);
            tmp[m] = NULL;
        }
        if (videoFrame[i]->v_frame.header_virt_addr[0] != NULL) {
            ret = hi_mpi_dvpp_free((void*)(videoFrame[i]->v_frame.header_virt_addr[0]));
            if (ret != HI_SUCCESS) {
                SAMPLE_PRT("hi_mpi_dvpp_free failed!\n");
            }
        }
    }
    return HI_FAILURE;
}

void send_frame_for_dc_performance(FILE *fpYuv, HiSampleJpegeSendFramePara* jpegeSendPara)
{
    int32_t ret = HI_SUCCESS;
    hi_venc_chn vencChn = jpegeSendPara->vencChn;
    int32_t delay = 0;
    hi_video_frame_info *videoFrame[g_mem_count];
    hi_video_frame_info *tmp[g_mem_count];

    ret = alloc_input_buffer_for_dc_performance(fpYuv, jpegeSendPara, tmp, videoFrame);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("alloc input buffer failed!\n");
        return;
    }

    delay = 30; // 单位：s
    delay_exec(g_start_send_time, delay); // 所有线程等待并发
    SAMPLE_PRT("chn %d start send!\n", vencChn);

    bool isAbnormal = false;
    while (jpegeSendPara->threadStart == HI_TRUE) {
        isAbnormal = false;
        // 4. 发送图像
        for (int j = 0; j < g_mem_count; ++j){
            ret = start_async_send_frame(vencChn, videoFrame[j], jpegeSendPara->supportPerformance);
            if (ret != HI_SUCCESS) {
                isAbnormal = true;
                break;
            }
        }
        jpegeSendPara->threadStart = HI_FALSE;
    }

    fflush(stdout);
    for (int n = 0; n < g_mem_count; ++n) {
        if (tmp[n] != NULL) {
            free(tmp[n]);
            tmp[n] = NULL;
        }

        if (isAbnormal && (videoFrame[n]->v_frame.header_virt_addr[0] != NULL)) {
            ret = hi_mpi_dvpp_free((void*)(videoFrame[n]->v_frame.header_virt_addr[0]));
            if (ret != HI_SUCCESS) {
                SAMPLE_PRT("hi_mpi_dvpp_free failed!\n");
            }
        }
    }
}

/*
* function:    jpege_snap_send_frame_dc_performance(void* p)
* Description: Start sending input data(Used for DC performance test)
* input:       p:start address of the function
* output:      none
* return:      none
* others:      none
*/
void* jpege_snap_send_frame_dc_performance(void* p)
{
    int32_t ret = HI_SUCCESS;
    hi_char inputFileName[64];
    FILE *fpYuv = NULL;
    HiSampleJpegeSendFramePara* jpegeSendPara = nullptr;

    aclError acl_ret = aclrtSetCurrentContext(g_context);
    if (acl_ret != ACL_SUCCESS) {
        SAMPLE_PRT("set current context failed");
        return NULL;
    }

    if (p == nullptr) {
        SAMPLE_PRT("input param is nullptr");
        return NULL;
    }

    jpegeSendPara = (HiSampleJpegeSendFramePara*)p;

    // 1. open the input file handle
    snprintf(inputFileName, sizeof(inputFileName), "%s", jpegeSendPara->inputPara.inputFileName);

    fpYuv = fopen(inputFileName, "rb");
    if (fpYuv == NULL) {
        SAMPLE_PRT("can't open file in send stream thread!\n");
        return NULL;
    }

    send_frame_for_dc_performance(fpYuv, jpegeSendPara);

    if (fpYuv != NULL) {
        fclose(fpYuv);
        fpYuv = NULL;
    }

    return NULL;
}

uint32_t get_out_ring_buf_size(uint32_t width, uint32_t height)
{
    uint32_t bufSize = 0;
    if (g_isZeroCopy == 0) {
        uint32_t align_up_width = ALIGN_UP(width, 16);
        uint32_t align_up_height = ALIGN_UP(height, 16);
        if (width > 4096) {
            bufSize = ALIGN_UP(align_up_width * align_up_height * 4, 64); // 4 buffer blocks for huge input
        } else {
            bufSize = ALIGN_UP(align_up_width * align_up_height * 5, 64); // 5:number of buffer blocks, 64-aligned
        }
        bufSize = (bufSize < 384000) ? 384000 : bufSize; // 384000 = 320 * 240 * 5
    }
    return bufSize;
}

/*
* function:    jpege_snap_start(hi_venc_chn vencChn, hi_video_size* size, hi_bool supportDc, uint32_t level)
* Description: 创建通道并接收输入图像
* input:       vencChn:通道号，size:通道大小, supportDc:是否仅支持dc, level: 编码质量
* output:      创建通道并接收输入图像是否成功
* return:      HI_SUCCESS: 成功, 错误码: 失败
* others:      无
*/
int32_t jpege_snap_start(hi_venc_chn vencChn, hi_video_size* size, hi_bool supportDc, uint32_t level)
{
    int32_t ret = HI_SUCCESS;
    uint32_t bufSize = 0;
    hi_venc_start_param recvParam;
    hi_venc_chn_attr vencChnAttr;

    // 1: create channel
    bufSize = get_out_ring_buf_size(size->width, size->height);
    SAMPLE_PRT("vencChn = %d, bufSize = %u !\n", vencChn, bufSize);

    vencChnAttr.venc_attr.type = HI_PT_JPEG;
    vencChnAttr.venc_attr.profile = 0;
    vencChnAttr.venc_attr.max_pic_width = size->width;
    vencChnAttr.venc_attr.max_pic_height = size->height;
    vencChnAttr.venc_attr.pic_width = size->width;
    vencChnAttr.venc_attr.pic_height = size->height;
    vencChnAttr.venc_attr.buf_size = bufSize; // aligned to 64 bytes
    vencChnAttr.venc_attr.is_by_frame = HI_TRUE; // get stream mode is field mode or frame mode
    vencChnAttr.venc_attr.jpeg_attr.dcf_en = supportDc;
    vencChnAttr.venc_attr.jpeg_attr.recv_mode = HI_VENC_PIC_RECV_SINGLE;
    vencChnAttr.venc_attr.jpeg_attr.mpf_cfg.large_thumbnail_num = 0;

    ret = hi_mpi_venc_create_chn(vencChn, &vencChnAttr);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_venc_create_chn [%d] faild with %#x!\n", vencChn, ret);
        return ret;
    }

    // 2: start receive frame
    recvParam.recv_pic_num = -1; // unspecified frame count
    ret = hi_mpi_venc_start_chn(vencChn, &recvParam);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_venc_start_chn faild with%#x in chnl %d!\n", ret, vencChn);
        return ret;
    }

    // 3. set encode parameter
    hi_venc_jpeg_param stParamJpeg;
    ret = hi_mpi_venc_get_jpeg_param(vencChn, &stParamJpeg);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_venc_get_jpeg_param err 0x%x\n", ret);
        return ret;
    }
    stParamJpeg.qfactor = level;
    ret = hi_mpi_venc_set_jpeg_param(vencChn, &stParamJpeg);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_venc_set_jpeg_param err 0x%x\n", ret);
        return ret;
    }

    return HI_SUCCESS;
}

/*
* function:    jpege_snap_stop(hi_venc_chn vencChn[], int32_t cnt)
* Description: Stop receiving pictures
* input:       vencChn:chnl_id, cnt:number of frames
* output:      Whether the video receiving is stopped successfully
* return:      HI_SUCCESS: success, HI_FAILURE: fail
* others:      none
*/
int32_t jpege_snap_stop(hi_venc_chn vencChn[], int32_t cnt)
{
    int32_t ret = HI_SUCCESS;

    for (int32_t i = 0; i < cnt; ++i) {
        ret = hi_mpi_venc_stop_chn(vencChn[i]);
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("hi_mpi_venc_stop_chn vencChn[%d] failed with %#x!\n", vencChn[i], ret);
            return HI_FAILURE;
        }
        ret = hi_mpi_venc_destroy_chn(vencChn[i]);
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("hi_mpi_venc_destroy_chn vencChn[%d] failed with %#x!\n", vencChn[i], ret);
            return HI_FAILURE;
        }
    }
    return HI_SUCCESS;
}

/*
* function:    jpege_start_get_stream(hi_venc_chn vencChn[], int32_t cnt, int32_t per, uint32_t save)
* Description: Create a Thread for Obtaining Output Streams
* input:       vencChn:chnl_id, cnt:number of frames, per:whether performance test or not,
               save: whether save output stream or not
* output:      Whether the thread for obtaining output streams is created successfully
* return:      HI_SUCCESS: success, Less than 0: fail
* others:      none
*/
int32_t jpege_start_get_stream(hi_venc_chn vencChn[], int32_t cnt, int32_t per, uint32_t save)
{
    uint32_t i = 0;
    int32_t ret = HI_SUCCESS;

    if (g_select_one_thread) {
        for(i = 0; i < cnt; i++) {
            // chnld_id of single-thread single-channel
            g_jpege_get_stream_para[i].chnlId = vencChn[i];
            // thread start, when the thread is destoryed, setting HI_FALSE
            g_jpege_get_stream_para[i].threadStart = HI_TRUE;
            g_jpege_get_stream_para[i].chnlCnt = 1; // Number of channels
            g_jpege_get_stream_para[i].orSave = save; // Whether to save the output result
            g_jpege_get_stream_para[i].supportPerformance = per; // Is current performance statistics
            SAMPLE_PRT("start pthread_create get stream thread!\n");
            ret = pthread_create(&g_jpege_get_stream_pid[i], 0, jpege_snap_process, (void*)&g_jpege_get_stream_para[i]);
            if (ret != HI_SUCCESS) {
                SAMPLE_PRT("thread %u create failed !\n", i);
                g_jpege_get_stream_para[i].threadStart = HI_FALSE;
                return HI_FAILURE;
            }
        }
    } else {
        // use epoll, single thread for multi-channel
        ret = hi_mpi_sys_create_epoll(10, &g_jpege_epoll_fd);
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("create epoll failed !\n");
            return HI_FAILURE;
        }
        // thread start, when the thread is destoryed, setting HI_FALSE
        g_jpege_para.threadStart = HI_TRUE;
        g_jpege_para.chnlCnt = cnt; // Number of channels
        g_jpege_para.orSave = save; // Whether to save the output result
        g_jpege_para.supportPerformance = per; // Is current performance statistics
        for (i = 0; i < cnt; i++) {
            g_jpege_para.vencChn[i] = vencChn[i];
            int32_t fd = hi_mpi_venc_get_fd(vencChn[i]);
            hi_dvpp_epoll_event event;
            event.events = HI_DVPP_EPOLL_IN;
            event.data = (void *)(unsigned long)(vencChn[i]);
            hi_mpi_sys_ctl_epoll(g_jpege_epoll_fd, HI_DVPP_EPOLL_CTL_ADD, fd, &event);
        }
        SAMPLE_PRT("start pthread_create get stream thread!\n");
        ret = pthread_create(&g_jpege_get_stream_epoll_pid, 0, jpege_snap_process_epoll, (void*)&g_jpege_para);
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("thread create failed !\n");
            g_jpege_para.threadStart = HI_FALSE;
            return HI_FAILURE;
        }
    }

    return ret;
}

/*
* function:    jpege_snap_process_epoll(void* p)
* Description: Notify User To Obtain The Output Stream
* input:       p:start address of the function
* output:      none
* return:      none
* others:      none
*/
void* jpege_snap_process_epoll(void* p)
{
    struct timeval timeOutVal;
    fd_set readFd;
    int32_t chnImgCnt[MAX_JPEGE_CHN] = {0};
    hi_venc_chn vencChn;
    hi_venc_chn_status status;
    hi_venc_stream stream;
    HiSampleJpegeGetStreamPara* jpegePara = nullptr;
    int32_t ret = HI_SUCCESS;
    uint32_t i = 0;
    int32_t chnTotal = 0;
    uint64_t getStreamTime = 0;
    std::vector<std::set<uint64_t>> inputAddr;
    hi_dvpp_epoll_event events[1024];
    int32_t eventCount = 0;

    aclError acl_ret = aclrtSetCurrentContext(g_context);
    if (acl_ret != ACL_SUCCESS) {
        SAMPLE_PRT("set current context failed");
        return NULL;
    }

    for (i = 0; i < MAX_JPEGE_CHN; ++i) {
        std::set<uint64_t> temp;
        inputAddr.push_back(temp);
    }

    jpegePara = (HiSampleJpegeGetStreamPara*)p;
    chnTotal = jpegePara->chnlCnt;
    SAMPLE_PRT("one thread get multi-channel stream\n");

    // 3: notify user to obtain the output stream
    while (jpegePara->threadStart == HI_TRUE) { // When the thread does not receive a stop signal
        ret = hi_mpi_sys_wait_epoll(g_jpege_epoll_fd, events, 1024, 1000, &eventCount);
        if (ret) {
            SAMPLE_PRT("wait epoll fail\n");
            break;
        }
        SAMPLE_PRT("epoll wait event count: %d\n", eventCount);
        for (i = 0; i < eventCount; i++) {
            hi_dvpp_epoll_event *ev = &events[i];
            vencChn = (hi_venc_chn)(unsigned long)ev->data;

            ret = hi_mpi_venc_query_status(vencChn, &status);
            if (ret != HI_SUCCESS) {
                SAMPLE_PRT("hi_mpi_venc_query_status failed with %#x!\n", ret);
                break;
            }

            if (status.cur_packs == 0) {
                SAMPLE_PRT("NOTE: Current  frame is NULL!\n");
                return NULL;
            }

            stream.pack = (hi_venc_pack*)malloc(sizeof(hi_venc_pack) * status.cur_packs);
            if (stream.pack == NULL) {
                SAMPLE_PRT("malloc memory failed!\n");
                return NULL;
            }
            stream.pack_cnt = status.cur_packs;
            SAMPLE_GET_TIMESTAMP_US(getStreamTime);
            ret = hi_mpi_venc_get_stream(vencChn, &stream, -1);
            if (ret != HI_SUCCESS) {
                SAMPLE_PRT("hi_mpi_venc_get_stream failed with %#x!\n", ret);
                hi_mpi_dvpp_free((void*)(stream.pack[0].input_addr));
                free(stream.pack);
                stream.pack = NULL;
                return NULL;
            }

            SAMPLE_GET_TIMESTAMP_US(g_end_get_time[vencChn]); // update Time

            if (jpegePara->orSave) {
                ret = jpege_save_stream_to_file(vencChn, chnImgCnt[vencChn]++, &stream);
                if (ret != HI_SUCCESS) {
                    SAMPLE_PRT("save snap picture failed!\n");
                    hi_mpi_dvpp_free((void*)(stream.pack[0].input_addr));
                    free(stream.pack);
                    stream.pack = NULL;
                    return NULL;
                }
            }
            g_encode_stream_cnt[vencChn]++; // Number of frames received on the current channel +1

            if (jpegePara->supportPerformance) {
                inputAddr[vencChn].insert(stream.pack[0].input_addr);
            } else {
                ret = hi_mpi_dvpp_free((void*)(stream.pack[0].input_addr));
                if (ret != HI_SUCCESS) {
                    SAMPLE_PRT("hi_mpi_dvpp_free failed!\n");
                    free(stream.pack);
                    stream.pack = NULL;
                    return NULL;
                }
            }
            // Releases the output buffer
            ret = hi_mpi_venc_release_stream(vencChn, &stream);
            if (ret != HI_SUCCESS) {
                SAMPLE_PRT("hi_mpi_venc_release_stream failed with %#x!\n", ret);
                free(stream.pack);
                stream.pack = NULL;
                return NULL;
            }

            if (g_isZeroCopy) {
                // free output buffer
                ret = hi_mpi_dvpp_free(stream.pack[0].addr);
                if (ret != HI_SUCCESS) {
                    SAMPLE_PRT("hi_mpi_dvpp_free failed, ret = %d!\n", ret);
                }
            }

            free(stream.pack);
            stream.pack = NULL;
        }
    }

    // Release input buffer
    for (i = 0; i < chnTotal; ++i) {
        for (std::set<uint64_t>::iterator it = inputAddr[jpegePara->vencChn[i]].begin();
            it != inputAddr[jpegePara->vencChn[i]].end(); ++it) {
            ret = hi_mpi_dvpp_free((void*)(*it));
            if (ret != HI_SUCCESS) {
                SAMPLE_PRT("hi_mpi_dvpp_free failed!\n");
            }
        }
    }
}

/*
* function:    jpege_snap_process(void* p)
* Description: Notify User To Obtain The Output Stream
* input:       p:start address of the function
* output:      无
* return:      无
* others:      无
*/
void* jpege_snap_process(void* p)
{
    struct timeval timeOutVal;
    fd_set readFd;
    int32_t chnImgCnt = 0;
    hi_venc_chn vencChn;
    hi_venc_chn_status status;
    hi_venc_stream stream;
    HiSampleJpegeGetStreamPara* jpegePara = nullptr;
    int32_t ret = HI_SUCCESS;
    uint32_t i = 0;
    int32_t chnTotal = 0;
    hi_u64 getStreamTime = 0;
    char str[128];
    std::vector<std::set<hi_u64>> inputAddr;

    aclError acl_ret = aclrtSetCurrentContext(g_context);
    if (acl_ret != ACL_SUCCESS) {
        SAMPLE_PRT("set current context failed");
        return NULL;
    }

    for (i = 0; i < MAX_JPEGE_CHN; ++i) {
        std::set<hi_u64> temp;
        inputAddr.push_back(temp);
    }

    jpegePara = (HiSampleJpegeGetStreamPara*)p;
    vencChn = jpegePara->chnlId;
    chnTotal = jpegePara->chnlCnt;

    SAMPLE_PRT("one thread one-chnl get stream, chnl_id:%d\n", vencChn);

    // 3: 通知用户取走输出码流
    while (jpegePara->threadStart == HI_TRUE) { // 当线程没有接受到停止信号

        ret = hi_mpi_venc_query_status(vencChn, &status);
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("hi_mpi_venc_query_status failed with %#x!\n", ret);
        }

        stream.pack = (hi_venc_pack*)malloc(sizeof(hi_venc_pack));
        if (stream.pack == NULL) {
            SAMPLE_PRT("malloc memory failed!\n");
            return NULL;
        }
        stream.pack_cnt = 1;
        SAMPLE_GET_TIMESTAMP_US(getStreamTime);
        ret = hi_mpi_venc_get_stream(vencChn, &stream, -1);
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("hi_mpi_venc_get_stream failed with %#x!\n", ret);
            free(stream.pack);
            stream.pack = NULL;
            return NULL;
        }

        g_encode_stream_cnt[vencChn]++; // 当前通道接收帧数+1
        SAMPLE_GET_TIMESTAMP_US(g_end_get_time[vencChn]); // 更新时间
        SAMPLE_PRT("channelId:%d, m_getFrame:%llu, cost:%lld encTime:%u\n",
        vencChn, g_encode_stream_cnt[vencChn], g_end_get_time[vencChn] - getStreamTime,
        (uint32_t)(g_end_get_time[vencChn] - stream.pack[0].pts));

        if (jpegePara->orSave) {
            ret = jpege_save_stream_to_file(vencChn, chnImgCnt++, &stream);
            if (ret != HI_SUCCESS) {
                hi_mpi_dvpp_free((void*)(stream.pack[0].input_addr));
                free(stream.pack);
                stream.pack = NULL;
                return NULL;
            }
        }
        if (jpegePara->supportPerformance) {
            inputAddr[vencChn].insert(stream.pack[0].input_addr);
        } else {
            ret = hi_mpi_dvpp_free((void*)(stream.pack[0].input_addr));
            if (ret != HI_SUCCESS) {
                SAMPLE_PRT("hi_mpi_dvpp_free failed!\n");
                return NULL;
            }
        }
        // release stream
        ret = hi_mpi_venc_release_stream(vencChn, &stream);
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("hi_mpi_venc_release_stream failed with %#x!\n", ret);
            free(stream.pack);
            stream.pack = NULL;
            return NULL;
        }

        if (g_isZeroCopy) {
            // free output buffer
            ret = hi_mpi_dvpp_free(stream.pack[0].addr);
            if (ret != HI_SUCCESS) {
                SAMPLE_PRT("hi_mpi_dvpp_free failed, ret = %d!\n", ret);
            }
        }

        free(stream.pack);
        stream.pack = NULL;

        while (1) {
            if (g_jpege_send_frame_cnt[vencChn] > g_encode_stream_cnt[vencChn] ||
                jpegePara->threadStart == HI_FALSE) {
                break;
            } else {
                usleep(500);
            }
        }
    }

    // 释放输入内存
    for (i = 0; i < chnTotal; ++i) {
        for (std::set<hi_u64>::iterator it = inputAddr[vencChn].begin(); it != inputAddr[vencChn].end(); ++it) {
            ret = hi_mpi_dvpp_free((void*)(*it));
            if (ret != HI_SUCCESS) {
                SAMPLE_PRT("hi_mpi_dvpp_free failed!\n");
            }
        }
    }
    return NULL;
}

/*
* function:    jpege_stop_get_stream(int32_t cnt)
* Description: 停止获取输出码流销毁线程
* input:       cnt:通道数
* output:      停止获取输出码流销毁线程成功
* return:      HI_SUCCESS：成功
* others:      无
*/
int32_t jpege_stop_get_stream(int32_t cnt)
{
    int32_t i = 0;

    if (g_select_one_thread == 1) {
        for (i = 0; i < cnt; i++) {
            if (g_jpege_get_stream_para[i].threadStart == HI_TRUE) {
                g_jpege_get_stream_para[i].threadStart = HI_FALSE;
                pthread_join(g_jpege_get_stream_pid[i], 0);
            }
        }
    } else {
        if (g_jpege_para.threadStart == HI_TRUE) {
            g_jpege_para.threadStart = HI_FALSE;
            pthread_join(g_jpege_get_stream_epoll_pid, 0);
        }
        if (g_jpege_epoll_fd != -1) {
            for (i = g_chn_num_start; i < cnt + g_chn_num_start; i++) {
                int32_t fd = hi_mpi_venc_get_fd(i);
                hi_mpi_sys_ctl_epoll(g_jpege_epoll_fd, HI_DVPP_EPOLL_CTL_DEL, fd, NULL);
            }
            int32_t ret = hi_mpi_sys_close_epoll(g_jpege_epoll_fd);
            if (ret) {
                SAMPLE_PRT("destory epoll fail\n");
                return HI_FAILURE;
            }
        }
    }
    return HI_SUCCESS;
}
/*
* function:    jpege_stop_send_frame(int32_t cnt)
* Description: 停止发送输入数据并销毁线程
* input:       cnt:通道数
* output:      停止发送输入数据并销毁线程成功
* return:      HI_SUCCESS：成功
* others:      无
*/
int32_t jpege_stop_send_frame(int32_t cnt)
{
    for (uint32_t i = 0; i < cnt; i++) {
        if (g_jpege_send_frame_para[i].threadStart == HI_TRUE) {
            g_jpege_send_frame_para[i].threadStart = HI_FALSE;
            pthread_join(g_jpege_send_frame_pid[i], 0);
        }
    }
    return HI_SUCCESS;
}

/*
* function:    wait_encoder_complete(uint32_t chnNumStart, int32_t chnNum)
* Description: 等待编码完成
* input:       chnNumStart:起始通道号, chnNum:通道数
* output:      无
* return:      无
* others:      无
*/
int32_t wait_encoder_complete(uint32_t chnNumStart, int32_t chnNum)
{
    int32_t i = 0;
    hi_bool canExit = HI_TRUE;
    hi_u64 diffTime = 0;
    double actualFrameRate = 0;

    while (1) {
        canExit = HI_TRUE;
        for (i = 0; i < chnNum; ++i) {
            if ((g_jpege_send_frame_para[i].threadStart != HI_FALSE) ||
                (g_jpege_send_frame_cnt[i + chnNumStart] != g_encode_stream_cnt[i + chnNumStart])) {
                canExit = HI_FALSE;
                SAMPLE_PRT("chn[%u] send %llu, encode %llu \n",
                    i + chnNumStart, g_jpege_send_frame_cnt[i + chnNumStart], g_encode_stream_cnt[i + chnNumStart]);
                SAMPLE_PRT("encode uncomplete,wait 5s \n");
                sleep(5);
                break;
            }
        }
        if (canExit == HI_TRUE) {
            SAMPLE_PRT("encode complete! \n");
            // 打印性能信息
            for (i = 0; i < chnNum; ++i) {
                diffTime = g_end_get_time[i + chnNumStart] - g_start_send_time_array[i + chnNumStart];
                if (diffTime == 0) {
                    continue;
                }
                actualFrameRate = ((double)g_jpege_send_frame_cnt[i + chnNumStart] * 1000000) / diffTime;
                SAMPLE_PRT("-----------------------------JPEGE Performance statistics-----------------------------\n");
                SAMPLE_PRT("chnId %u, actualFrameRate %.1f, sendFrame %llu, encodeFrame %llu, diffTime %llu\n",
                    i + chnNumStart, actualFrameRate, g_jpege_send_frame_cnt[i + chnNumStart],
                    g_encode_stream_cnt[i + chnNumStart], diffTime);
            }
            break;
        }

    }

    return HI_SUCCESS;
}