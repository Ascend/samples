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

#include "Vdec.h"

#include <cstdint>
#include <getopt.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/prctl.h>
#include <vector>
#include <unistd.h>

#include "acl.h"
#include "acl_rt.h"
#include "hi_dvpp.h"

const int32_t MAX_NAME_LENGTH = 500;
char g_input_file_name[MAX_NAME_LENGTH] = "infile"; // Input stream file name
char g_output_file_name[MAX_NAME_LENGTH] = "outfile"; // Output file name
uint32_t g_in_width = 3840; // Input stream width
uint32_t g_in_height = 2160; // Input stream height
uint32_t g_in_format = 0; // Input stream format, 0: H264, 1: H265
uint32_t g_in_bitwidth = 8; // Input stream bit width, 8 or 10

uint32_t g_out_width = 0; // Output image width, supports resize, set 0 means no resize
uint32_t g_out_height = 0; // Output image height, supports resize, set 0 means no resize
uint32_t g_out_format = 0; // Output image format, supports format conversion, 
                           // 0: YUV420sp, 1: YVU420sp, 2: RGB888, 3: BGR888
uint32_t g_out_width_stride = 4096; // Output memory width stride, 16-aligned
uint32_t g_out_height_stride = 4096; // Output memory height stride, YUV420sp/YVU420sp need 2-aligned

uint32_t g_is_write_file = 1; // Whether write the result to a file or not, 0: no write to a file, 1: write to a file
uint32_t g_chn_num = 1; // Video decoder channel number [1,96]
uint32_t g_ref_frame_num = 8; // Number of reference frames [0, 16]
uint32_t g_display_frame_num = 2; // Number of display frames [0, 16]
uint32_t g_output_order = 0; // Video decoder output mode, 0: Display sequence, 1: Decoding sequence
uint32_t g_send_times = 1; // Stream send times, set 0 means send stream forever
uint32_t g_send_interval = 0; // Stream send interval(us)
uint32_t g_delay_time = 1; // Stream send delay time(s)
uint32_t g_alloc_num = 20; // Number of out buffer
uint32_t g_start_chn_num = 0; // Video decoder channel start number
uint32_t g_render = 0; // Extract frame interval, set 0 means every frame need post process
uint32_t g_exit = 0; // Force Exit Flag

uint32_t g_chan_create_state[VDEC_MAX_CHN_NUM] = {0}; // Video decoder channel state, 0: not created, 1: created
uint64_t g_start_time = 0; // Time of create channel

std::vector<void*> g_out_buffer_pool[VDEC_MAX_CHN_NUM]; // Out buffer pool
pthread_mutex_t g_out_buffer_pool_lock[VDEC_MAX_CHN_NUM]; // Lock of out buffer pool

uint32_t g_send_exit_state[VDEC_MAX_CHN_NUM] = {0}; // State of send thread
uint32_t g_get_exit_state[VDEC_MAX_CHN_NUM] = {0}; // State of get thread

pthread_t g_vdec_send_thread[VDEC_MAX_CHN_NUM] = {0};
pthread_t g_vdec_get_thread[VDEC_MAX_CHN_NUM] = {0};
uint32_t g_send_thread_id[VDEC_MAX_CHN_NUM] = {0};
uint32_t g_get_thread_id[VDEC_MAX_CHN_NUM] = {0};

uint8_t* g_frame_addr[VDEC_MAX_CHN_NUM][9999]; // Frame address
uint64_t g_frame_len[VDEC_MAX_CHN_NUM][9999]; // Frame size

uint64_t g_vdec_start_time[VDEC_MAX_CHN_NUM] = {0}; // Video decoder start time
uint64_t g_vdec_end_time[VDEC_MAX_CHN_NUM] = {0}; // Video decoder end time
uint64_t g_vdec_get_frame_cnt[VDEC_MAX_CHN_NUM] = {0}; // Number of decode success

aclrtRunMode g_run_mode = ACL_HOST;
aclrtContext g_context = NULL;
// Get current time(us)
void get_current_time_us(uint64_t& timeUs)
{
    struct timeval curTime;
    gettimeofday(&curTime, NULL);
    // 1s = 1000000 us
    timeUs = (uint64_t)curTime.tv_sec * 1000000 + curTime.tv_usec;
}

// YUV data write to a file
void save_yuv_file(FILE* const fd, hi_video_frame frame, uint32_t chanId)
{
    uint8_t* addr = (uint8_t*)frame.virt_addr[0];
    uint32_t imageSize = frame.width * frame.height * 3 / 2; // Size = width * height * 3 / 2
    int32_t ret = HI_SUCCESS;
    uint8_t* outImageBuf = nullptr;
    uint32_t outWidthStride = frame.width_stride[0];
    uint32_t outHeightStride = frame.height_stride[0];

    printf("[%s][%d] Chn %u, addr = %lx \n", __FUNCTION__, __LINE__, chanId, (uint64_t)(uintptr_t)frame.virt_addr[0]);
    printf("[%s][%d] Chn %u, Width = %u \n", __FUNCTION__, __LINE__, chanId, frame.width);
    printf("[%s][%d] Chn %u, Height = %u \n", __FUNCTION__, __LINE__, chanId, frame.height);
    printf("[%s][%d] Chn %u, PixelFormat = %d \n", __FUNCTION__, __LINE__, chanId, frame.pixel_format);
    printf("[%s][%d] Chn %u, OutWidthStride = %u \n", __FUNCTION__, __LINE__, chanId, frame.width_stride[0]);
    printf("[%s][%d] Chn %u, OutHeightStride = %u \n", __FUNCTION__, __LINE__, chanId, frame.height_stride[1]);

    if (g_run_mode == ACL_HOST) {
        // malloc host memory
        ret = aclrtMallocHost((void **)&outImageBuf, imageSize);
        if (ret != ACL_SUCCESS) {
            printf("[%s][%d] Chn %u malloc host memory %u failed, error code = %d.\n",
                __FUNCTION__, __LINE__, chanId, imageSize, ret);
            return;
        }
    }

    if ((frame.width == outWidthStride) && (frame.height == outHeightStride)) {
        if (g_run_mode == ACL_HOST) {
            // copy device data to host
            ret = aclrtMemcpy(outImageBuf, imageSize, addr, imageSize, ACL_MEMCPY_DEVICE_TO_HOST);
            if (ret != ACL_SUCCESS) {
                printf("[%s][%d] Chn %u Copy aclrtMemcpy %u from device to host failed, error code = %d.\n",
                    __FUNCTION__, __LINE__, chanId, imageSize, ret);
                return;
            }

            fwrite(outImageBuf, 1, imageSize, fd);
            aclrtFreeHost(outImageBuf);
        } else {
            fwrite(addr, imageSize, 1, fd);
        }
    } else {
        if (g_run_mode == ACL_HOST) {
            if (outImageBuf == NULL) {
                return;
            }
            // Copy valid Y data
            for (uint32_t i = 0; i < frame.height; i++) {
                ret = aclrtMemcpy(outImageBuf + i * frame.width, frame.width, addr + i * outWidthStride,
                    frame.width, ACL_MEMCPY_DEVICE_TO_HOST);
                if (ret != ACL_SUCCESS) {
                    printf("[%s][%d] Chn %u Copy aclrtMemcpy %u from device to host failed, error code = %d.\n",
                        __FUNCTION__, __LINE__, chanId, imageSize, ret);
                    aclrtFreeHost(outImageBuf);
                    return;
                }
            }
            // Copy valid UV data
            for (uint32_t i = 0; i < frame.height / 2; i++) {
                ret = aclrtMemcpy(outImageBuf + i * frame.width + frame.width * frame.height, frame.width,
                    addr + i * outWidthStride + outWidthStride * outHeightStride, frame.width,
                    ACL_MEMCPY_DEVICE_TO_HOST);
                if (ret != ACL_SUCCESS) {
                    printf("[%s][%d] Chn %u Copy aclrtMemcpy %u from device to host failed, error code = %d.\n",
                        __FUNCTION__, __LINE__, chanId, imageSize, ret);
                    aclrtFreeHost(outImageBuf);
                    return;
                }
            }

            fwrite(outImageBuf, 1, imageSize, fd);
            aclrtFreeHost(outImageBuf);
        } else {
            // Crop the invalid data, then write the valid data to a file
            uint8_t* outImageBuf = (uint8_t*)malloc(imageSize);
            if (outImageBuf == NULL) {
                printf("[%s][%d] Chn %u Malloc Fail \n", __FUNCTION__, __LINE__, chanId);
                return;
            }
            // Copy valid Y data
            for (uint32_t i = 0; i < frame.height; i++) {
                memcpy(outImageBuf + i * frame.width, addr + i * outWidthStride, frame.width);
            }
            // Copy valid UV data
            for (uint32_t i = 0; i < frame.height / 2; i++) {
                memcpy(outImageBuf + i * frame.width + frame.width * frame.height,
                    addr + i * outWidthStride + outWidthStride * outHeightStride, frame.width);
            }

            fwrite(outImageBuf, 1, imageSize, fd);
            free(outImageBuf);
        }
    }
    return;
}

// RGB data write to a file
void save_rgb_file(FILE* const fd, hi_video_frame frame, uint32_t chanId)
{
    uint8_t* addr = (uint8_t*)frame.virt_addr[0];
    uint32_t imageSize = frame.width * frame.height * 3; // Size = width * height * 3
    int32_t ret = HI_SUCCESS;
    uint8_t* outImageBuf = nullptr;
    uint32_t outWidthStride = frame.width_stride[0];
    uint32_t outHeightStride = frame.height_stride[0];

    printf("[%s][%d] Chn %u, addr = %lx \n", __FUNCTION__, __LINE__, chanId, (uint64_t)(uintptr_t)frame.virt_addr[0]);
    printf("[%s][%d] Chn %u, Width = %d \n", __FUNCTION__, __LINE__, chanId, frame.width);
    printf("[%s][%d] Chn %u, Height = %d \n", __FUNCTION__, __LINE__, chanId, frame.height);
    printf("[%s][%d] Chn %u, PixelFormat = %d \n", __FUNCTION__, __LINE__, chanId, frame.pixel_format);
    printf("[%s][%d] Chn %u, OutWidthStride = %d \n", __FUNCTION__, __LINE__, chanId, frame.width_stride[0]);
    printf("[%s][%d] Chn %u, OutHeightStride = %d \n", __FUNCTION__, __LINE__, chanId, frame.height_stride[0]);

    if (g_run_mode == ACL_HOST) {
        // malloc host memory
        ret = aclrtMallocHost((void **)&outImageBuf, imageSize);
        if (ret != ACL_SUCCESS) {
            printf("[%s][%d] Chn %u malloc host memory %u failed, error code = %d.\n",
                __FUNCTION__, __LINE__, chanId, imageSize, ret);
            return;
        }
    }

    // RGB888:width * 3 = stride
    if (((frame.width * 3) == outWidthStride) && (frame.height == outHeightStride)) {
        if (g_run_mode == ACL_HOST) {
            // copy device data to host
            ret = aclrtMemcpy(outImageBuf, imageSize, addr, imageSize, ACL_MEMCPY_DEVICE_TO_HOST);
            if (ret != ACL_SUCCESS) {
                printf("[%s][%d] Chn %u Copy aclrtMemcpy %u from device to host failed, error code = %d.\n",
                    __FUNCTION__, __LINE__, chanId, imageSize, ret);
                return;
            }

            fwrite(outImageBuf, 1, imageSize, fd);
            aclrtFreeHost(outImageBuf);
        } else {
            fwrite(addr, imageSize, 1, fd);
        }
    } else {
        if (g_run_mode == ACL_HOST) {
            // Crop the invalid data, then write the valid data to a file
            // Copy valid data
            for (uint32_t i = 0; i < frame.height; i++) {
                ret = aclrtMemcpy(outImageBuf + i * frame.width * 3, frame.width * 3, addr + i * outWidthStride,
                    frame.width * 3, ACL_MEMCPY_DEVICE_TO_HOST);
                if (ret != HI_SUCCESS) {
                    printf("[%s][%d] Chn %u Copy aclrtMemcpy %u from device to host failed, error code = %d.\n",
                        __FUNCTION__, __LINE__, chanId, imageSize, ret);
                    aclrtFreeHost(outImageBuf);
                    return;
                }
            }
            fwrite(outImageBuf, 1, imageSize, fd);
            aclrtFreeHost(outImageBuf);
        } else {
            // Crop the invalid data, then write the valid data to a file
            uint8_t* outImageBuf = (uint8_t*)malloc(imageSize);
            if (outImageBuf == NULL) {
                printf("[%s][%d] Chn %u Malloc Fail \n", __FUNCTION__, __LINE__, chanId);
                return;
            }
            // Copy valid data
            for (uint32_t i = 0; i < frame.height; i++) {
                memcpy(outImageBuf + i * frame.width * 3, addr + i * outWidthStride, frame.width * 3);
            }
            fwrite(outImageBuf, 1, imageSize, fd);
            free(outImageBuf);
        }
    }
    return;
}

// Create video decoder channel, channel number is [g_start_chn_num, g_start_chn_num + g_chn_num)
int32_t vdec_create()
{
    int32_t ret = HI_SUCCESS;
    hi_vdec_chn_attr chnAttr[VDEC_MAX_CHN_NUM]{};
    hi_data_bit_width bitWidth = HI_DATA_BIT_WIDTH_8;
    if (g_in_bitwidth == 10) { // 10Bit stream
        bitWidth = HI_DATA_BIT_WIDTH_10;
    }

    for (uint32_t i = g_start_chn_num; i < g_start_chn_num + g_chn_num; i++) {
        if (g_in_format == 0) {
            chnAttr[i].type = HI_PT_H264; // Input stream is H264
        } else if (g_in_format == 1) {
            chnAttr[i].type = HI_PT_H265; // Input stream is H265
        }
        // Configure channel attribute
        chnAttr[i].mode = HI_VDEC_SEND_MODE_FRAME; // Only support frame mode
        chnAttr[i].pic_width = g_in_width;
        chnAttr[i].pic_height = g_in_height;
        // Stream buffer size, Recommended value is width * height * 3 / 2
        chnAttr[i].stream_buf_size = g_in_width * g_in_height * 3 / 2;
        chnAttr[i].frame_buf_cnt = g_ref_frame_num + g_display_frame_num + 1;

        hi_pic_buf_attr buf_attr{g_in_width, g_in_height, 0,
                                 bitWidth, HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420, HI_COMPRESS_MODE_NONE};
        chnAttr[i].frame_buf_size = hi_vdec_get_pic_buf_size(chnAttr[i].type, &buf_attr);

        // Configure video decoder channel attribute
        chnAttr[i].video_attr.ref_frame_num = g_ref_frame_num;
        chnAttr[i].video_attr.temporal_mvp_en = HI_TRUE;
        chnAttr[i].video_attr.tmv_buf_size = hi_vdec_get_tmv_buf_size(chnAttr[i].type, g_in_width, g_in_height);

        ret = hi_mpi_vdec_create_chn(i, &chnAttr[i]);
        if (ret != HI_SUCCESS) {
            printf("[%s][%d] Chn %u, hi_mpi_vdec_create_chn Fail, ret = %x \n", __FUNCTION__, __LINE__, i, ret);
            return ret;
        }
        g_chan_create_state[i] = 1;

        hi_vdec_chn_param chnParam;
        // Get channel parameter
        ret = hi_mpi_vdec_get_chn_param(i, &chnParam);
        if (ret != HI_SUCCESS) {
            printf("[%s][%d] Chn %u, hi_mpi_vdec_get_chn_param Fail, ret = %x \n", __FUNCTION__, __LINE__, i, ret);
            return ret;
        }
        chnParam.video_param.dec_mode = HI_VIDEO_DEC_MODE_IPB;
        chnParam.video_param.compress_mode = HI_COMPRESS_MODE_HFBC;
        chnParam.video_param.video_format = HI_VIDEO_FORMAT_TILE_64x16;
        chnParam.display_frame_num = g_display_frame_num;
        if (g_output_order == 0) {
            chnParam.video_param.out_order = HI_VIDEO_OUT_ORDER_DISPLAY; // Display sequence
        } else {
            chnParam.video_param.out_order = HI_VIDEO_OUT_ORDER_DEC; // Decoding sequence
        }

        // Set channel parameter
        ret = hi_mpi_vdec_set_chn_param(i, &chnParam);
        if (ret != HI_SUCCESS) {
            printf("[%s][%d] Chn %u, hi_mpi_vdec_set_chn_param Fail, ret = %x \n", __FUNCTION__, __LINE__, i, ret);
            return ret;
        }

        // Decoder channel start receive stream
        ret = hi_mpi_vdec_start_recv_stream(i);
        if (ret != HI_SUCCESS) {
            printf("[%s][%d] Chn %u, hi_mpi_vdec_start_recv_stream Fail, ret = %x \n", __FUNCTION__, __LINE__, i, ret);
            return ret;
        }
    }
    return ret;
}

void wait_vdec_end()
{
    int32_t ret = HI_SUCCESS;
    hi_vdec_chn_status status;

    for (uint32_t i = g_start_chn_num; i < g_start_chn_num + g_chn_num; i++) {
        if (g_vdec_send_thread[i] != 0) {
            // Wait send thread exit
            ret = pthread_join(g_vdec_send_thread[i], NULL);
        }
        g_vdec_send_thread[i] = 0;

        // Wait channel decode over
        while (g_exit == 0) {
            ret = hi_mpi_vdec_query_status(i, &status);
            if (ret != HI_SUCCESS) {
                printf("[%s][%d] Chn %u, hi_mpi_vdec_query_status Fail, ret = %x \n", __FUNCTION__, __LINE__, i, ret);
                break;
            }
            if (((status.left_stream_bytes == 0) && (status.left_decoded_frames == 0)) || (g_get_exit_state[i] == 1)) {
                break;
            }
            // 10000us
            usleep(10000);
        }
    }
    // 1000000us
    usleep(1000000);

    // Notify get thread exit
    for (uint32_t i = g_start_chn_num; i < g_start_chn_num + g_chn_num; i++) {
        g_get_exit_state[i] = 1;
    }

    for (uint32_t i = g_start_chn_num; i < g_start_chn_num + g_chn_num; i++) {
        if (g_vdec_get_thread[i] != 0) {
            // Wait get thread exit
            ret = pthread_join(g_vdec_get_thread[i], NULL);
        }
        g_vdec_get_thread[i] = 0;
    }
}

void vdec_destroy()
{
    int32_t ret = HI_SUCCESS;
    for (uint32_t i = g_start_chn_num; i < g_start_chn_num + g_chn_num; i++) {
        if (g_chan_create_state[i] == 1) {
            // Decoder channel stop receive stream
            ret = hi_mpi_vdec_stop_recv_stream(i);
            if (ret != HI_SUCCESS) {
                printf("[%s][%d] Chn %u, hi_mpi_vdec_stop_recv_stream Fail, ret = %x \n",
                     __FUNCTION__, __LINE__, i, ret);
            }
            // Destroy channel
            ret = hi_mpi_vdec_destroy_chn(i);
            if (ret != HI_SUCCESS) {
                printf("[%s][%d] Chn %u, hi_mpi_vdec_destroy_chn Fail, ret = %x \n", __FUNCTION__, __LINE__, i, ret);
            }
            release_outbuffer(i);
        }
    }
}

int32_t create_send_stream_thread()
{
    int32_t ret;
    for (uint32_t i = g_start_chn_num; i < g_start_chn_num + g_chn_num; i++) {
        g_vdec_send_thread[i] = 0;
        g_send_thread_id[i] = i;
        // Create send thread
        ret = pthread_create(&g_vdec_send_thread[i], 0, send_stream, (void*)&g_send_thread_id[i]);
        if (ret != 0) {
            printf("[%s][%d] Chn %u, create send stream thread Fail, ret = %d \n", __FUNCTION__, __LINE__, i, ret);
            g_vdec_send_thread[i] = 0;
            return ret;
        }
    }

    return ret;
}

int32_t create_get_pic_thread()
{
    int32_t ret;
    for (uint32_t i = g_start_chn_num; i < g_start_chn_num + g_chn_num; i++) {
        g_vdec_get_thread[i] = 0;
        g_get_thread_id[i] = i;
        // Create get thread
        ret = pthread_create(&g_vdec_get_thread[i], 0, get_pic, (void*)&g_get_thread_id[i]);
        if (ret != 0) {
            printf("[%s][%d] Chn %u, create get pic thread Fail, ret = %d \n", __FUNCTION__, __LINE__, i, ret);
            g_vdec_get_thread[i] = 0;
            return ret;
        }
    }

    return ret;
}

void stop_send_stream_thread()
{
    for (uint32_t i = g_start_chn_num; i < g_start_chn_num + g_chn_num; i++) {
        // Set thread state to 1, then thread will end
        g_send_exit_state[i] = 1;
    }
    // Set exit to 1, then wait_vdec_end loop will end
    g_exit = 1;
}

void stop_get_pic_thread()
{
    for (uint32_t i = g_start_chn_num; i < g_start_chn_num + g_chn_num; i++) {
        // Set thread state to 1, then send stream and get pic thread will end
        g_get_exit_state[i] = 1;
        g_send_exit_state[i] = 1;
    }
    // Set exit to 1, then wait_vdec_end loop will end
    g_exit = 1;
}

void* send_stream(void* const chanNum)
{
    prctl(PR_SET_NAME, "VdecSendStream", 0, 0, 0);
    uint32_t chanId = *(uint32_t*)chanNum;

    aclError aclRet = aclrtSetCurrentContext(g_context);
    if (aclRet != ACL_SUCCESS) {
        printf("[%s][%d] Chn %u set current context failed, error code = %d",
            __FUNCTION__, __LINE__, chanId, aclRet);
        return (void*)(HI_FAILURE);
    }

    // Open input stream file
    FILE* fpInputFile = NULL;
    fpInputFile = fopen(g_input_file_name, "rb");
    if (fpInputFile == NULL) {
        printf("[%s][%d] Chn %u Can't open file %s \n", __FUNCTION__, __LINE__, chanId, g_input_file_name);
        return (void*)(HI_FAILURE);
    }

    // Calculate input stream file size
    uint32_t fileSize = 0;
    fseek(fpInputFile, 0L, SEEK_END);
    fileSize = ftell(fpInputFile);
    fseek(fpInputFile, 0L, SEEK_SET);

    // Alloc buffer for all input stream file
    uint8_t* inputFileBuf = NULL;
    inputFileBuf = (uint8_t*)malloc(fileSize);
    if (inputFileBuf == NULL) {
        fclose(fpInputFile);
        printf("[%s][%d] Chn %u Malloc InputFile Buffer Fail \n", __FUNCTION__, __LINE__, chanId);
        return (void*)(HI_FAILURE);
    }

    // Read input stream file
    uint32_t readLen = 0;
    readLen = fread(inputFileBuf, 1, fileSize, fpInputFile);
    if (readLen != fileSize) {
        fclose(fpInputFile);
        free(inputFileBuf);
        printf("[%s][%d] Chn %u Read InputFile Fail \n", __FUNCTION__, __LINE__, chanId);
        return (void*)(HI_FAILURE);
    }

    uint8_t* dataDev = HI_NULL;
    int32_t ret = HI_SUCCESS;

    if (g_run_mode == ACL_HOST) {
        // alloc device inbuffer mem
        ret = hi_mpi_dvpp_malloc(0, (void **)&dataDev, fileSize);
        if (ret != 0) {
            fclose(fpInputFile);
            free(inputFileBuf);
            printf("[%s][%d] Chn %u Malloc device memory %u failed.\n",
                __FUNCTION__, __LINE__, chanId, fileSize);
            return (hi_void *)(HI_FAILURE);
        }

        // copy host to device
        ret = aclrtMemcpy(dataDev, fileSize, inputFileBuf, fileSize, ACL_MEMCPY_HOST_TO_DEVICE);
        if (ret != ACL_SUCCESS) {
            fclose(fpInputFile);
            free(inputFileBuf);
            hi_mpi_dvpp_free(dataDev);
            printf("[%s][%d] Chn %u Copy host memcpy to device failed, error code = %d.\n",
                __FUNCTION__, __LINE__, chanId, ret);
            return (hi_void *)(HI_FAILURE);
        }
    }
    uint32_t frameCount = 0;
    hi_payload_type type = (g_in_format == 0) ? HI_PT_H264 : HI_PT_H265;
    // Cutting stream
    get_every_frame(chanId, inputFileBuf, &frameCount, fileSize, type, dataDev);

    // Using out buffer pool in order to prevent system oom
    void* outBuffer = NULL;
    uint32_t outBufferSize = 0;
    if ((g_out_format == 0) || (g_out_format == 1)) {
        // Out format is YUV420sp or YVU420sp
        outBufferSize = g_out_width_stride * g_out_height_stride * 3 / 2;
    } else if ((g_out_format == 2) || (g_out_format == 3)) {
        // Out format is RGB888 or BGR888
        outBufferSize = g_out_width_stride * g_out_height_stride;
    }

    // Alloc out buffer
    for (uint32_t i = 0; i < g_alloc_num; i++) {
        ret = hi_mpi_dvpp_malloc(0, &outBuffer, outBufferSize); // Alloc Vdec out buffer must use hi_mpi_dvpp_malloc
        if (ret != HI_SUCCESS) {
            fclose(fpInputFile);
            free(inputFileBuf);
            if (g_run_mode == ACL_HOST) {
                hi_mpi_dvpp_free(dataDev);
            }
            printf("[%s][%d] Chn %u hi_mpi_dvpp_malloc failed.\n",
                __FUNCTION__, __LINE__, chanId);
            return (void*)(HI_FAILURE);
        }
        // Put buffer to pool
        g_out_buffer_pool[chanId].push_back(outBuffer);
    }
    // Delay g_delay_time seconds
    delay_exec(g_start_time, g_delay_time);
    // Get video decoder start time
    get_current_time_us(g_vdec_start_time[chanId]);

    // Start send stream
    hi_vdec_stream stream;
    hi_vdec_pic_info outPicInfo;
    uint32_t readCount = 0;
    hi_pixel_format outFormat = HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    uint64_t currentSendTime = 0;
    uint64_t lastSendTime = 0;
    uint32_t circleTimes = 0;
    switch (g_out_format) {
        case 0:
            outFormat = HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420;
            break;
        case 1:
            outFormat = HI_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
            break;
        case 2:
            outFormat = HI_PIXEL_FORMAT_RGB_888;
            break;
        case 3:
            outFormat = HI_PIXEL_FORMAT_BGR_888;
            break;
        default:
            break;
    }
    int32_t timeOut = 1000;
    get_current_time_us(currentSendTime);
    while (1) {
        if (g_send_exit_state[chanId] == 1) {
            break;
        }
        stream.pts = currentSendTime + g_send_interval;
        stream.addr = g_frame_addr[chanId][readCount]; // Configure input stream address
        stream.len = g_frame_len[chanId][readCount]; // Configure input stream size
        stream.end_of_frame = HI_TRUE; // Configure flage of frame end
        stream.end_of_stream = HI_FALSE; // Configure flage of stream end

        outPicInfo.width = g_out_width; // Output image width, supports resize, set 0 means no resize
        outPicInfo.height = g_out_height; // Output image height, supports resize, set 0 means no resize
        outPicInfo.width_stride = g_out_width_stride; // Output memory width stride
        outPicInfo.height_stride = g_out_height_stride; // Output memory height stride
        outPicInfo.pixel_format = outFormat; // Configure output format

        if ((g_render == 0) || (readCount % g_render == 0)) {
            int32_t mallocCount = 0;
            int32_t tryTimes = 20000;
            // Get out buffer from pool
            while (g_send_exit_state[chanId] == 0) {
                mallocCount++;
                (void)pthread_mutex_lock(&g_out_buffer_pool_lock[chanId]);
                if (g_out_buffer_pool[chanId].empty() == false) {
                    outBuffer = g_out_buffer_pool[chanId].back();
                    g_out_buffer_pool[chanId].pop_back();
                    (void)pthread_mutex_unlock(&g_out_buffer_pool_lock[chanId]);
                    break;
                } else {
                    (void)pthread_mutex_unlock(&g_out_buffer_pool_lock[chanId]);
                    usleep(1000); // 1000us
                }

                if (mallocCount >= tryTimes) {
                    printf("[%s][%d] Chn %u DvppMalloc From Pool Fail, Try again\n", __FUNCTION__, __LINE__, chanId);
                    mallocCount = 0;
                }
            }

            stream.need_display = HI_TRUE;
            outPicInfo.vir_addr = (uint64_t)outBuffer;
            outPicInfo.buffer_size = outBufferSize;
        } else { // if this frame is set no need post process, no need configure out buffer
            stream.need_display = HI_FALSE;
            outPicInfo.vir_addr = 0;
            outPicInfo.buffer_size = 0;
        }

        readCount = (readCount + 1) % frameCount;
        // Finish sending stream one times
        if (readCount == 0) {
            circleTimes++;
        }

        // Stream send interval
        get_current_time_us(currentSendTime);
        if ((currentSendTime - lastSendTime) < g_send_interval) {
            usleep(g_send_interval - (currentSendTime - lastSendTime));
        }
        get_current_time_us(lastSendTime);

        do {
            // Send one frame data
            ret = hi_mpi_vdec_send_stream(chanId, &stream, &outPicInfo, timeOut);
        } while (ret == HI_ERR_VDEC_BUF_FULL); // Try again

        if (ret != HI_SUCCESS) {           
            printf("[%s][%d] Chn %u hi_mpi_vdec_send_stream Fail, Error Code = %x \n",
                __FUNCTION__, __LINE__, chanId, ret);
            break;
        } else {
            if ((g_send_times != 0) && (circleTimes >= g_send_times)) {
                break;
            }
        }
    }

    // Send stream end flage
    stream.addr = NULL;
    stream.len = 0;
    stream.end_of_frame = HI_FALSE;
    stream.end_of_stream = HI_TRUE; // Stream end flage
    outPicInfo.vir_addr = 0;
    outPicInfo.buffer_size = 0;
    hi_mpi_vdec_send_stream(chanId, &stream, &outPicInfo, -1);

    fclose(fpInputFile);
    free(inputFileBuf);
    if (g_run_mode == ACL_HOST) {
        hi_mpi_dvpp_free(dataDev);
    }
    printf("[%s][%d] Chn %u send_stream Thread Exit \n", __FUNCTION__, __LINE__, chanId);
    return (hi_void *)HI_SUCCESS;
}

void* get_pic(void* const chanNum)
{
    prctl(PR_SET_NAME, "VdecGetPic", 0, 0, 0);
    uint32_t chanId = *(uint32_t*)chanNum;

    aclError aclRet = aclrtSetCurrentContext(g_context);
    if (aclRet != ACL_SUCCESS) {
        printf("[%s][%d] Chn %u set current context failed, error code = %d",
            __FUNCTION__, __LINE__, chanId, aclRet);
        g_get_exit_state[chanId] = 1;
        return (void*)(HI_FAILURE);
    }

    int32_t ret = HI_SUCCESS;
    hi_video_frame_info frame;
    hi_vdec_stream stream;
    int32_t decResult = 0; // Decode result
    void* outputBuffer = NULL;
    int32_t successCnt = 0;
    int32_t failCnt = 0;
    int32_t timeOut = 1000;
    int32_t writeFileCnt = 1;
    hi_vdec_supplement_info stSupplement{};

    while (1) {
        if (g_get_exit_state[chanId] == 1) {
            break;
        }
        ret = hi_mpi_vdec_get_frame(chanId, &frame, &stSupplement, &stream, timeOut);
        if (ret == HI_SUCCESS) {
            // Flush decode end time
            get_current_time_us(g_vdec_end_time[chanId]);
            outputBuffer = (void*)frame.v_frame.virt_addr[0];
            decResult = frame.v_frame.frame_flag;
            if (decResult == 0) { // 0: Decode success
                successCnt++;
                printf("[%s][%d] Chn %u GetFrame Success, Decode Success[%d] \n",
                    __FUNCTION__, __LINE__, chanId, successCnt);
            } else if (decResult == 1) { // 1: Decode fail
                failCnt++;
                printf("[%s][%d] Chn %u GetFrame Success, Decode Fail[%d] \n",
                    __FUNCTION__, __LINE__, chanId, failCnt);
            } else if (decResult == 2) { // 2:This result is returned for the second field of
                                         // the interlaced field stream, which is normal.
                printf("[%s][%d] Chn %u GetFrame Success, No Picture \n", __FUNCTION__, __LINE__, chanId);
            } else if (decResult == 3) { // 3: Reference frame number set error
                failCnt++;
                printf("[%s][%d] Chn %u GetFrame Success, RefFrame Num Error[%d] \n",
                    __FUNCTION__, __LINE__, chanId, failCnt);
            } else if (decResult == 4) { // 4: Reference frame size set error
                failCnt++;
                printf("[%s][%d] Chn %u GetFrame Success, RefFrame Size Error[%d] \n",
                    __FUNCTION__, __LINE__, chanId, failCnt);
            }
            // Flush decode count
            g_vdec_get_frame_cnt[chanId] = successCnt + failCnt;

            // Decode result write to a file
            if ((g_is_write_file == 1) && (decResult == 0) &&
                (outputBuffer != NULL) && (stream.need_display == HI_TRUE)) {
                FILE* fp = NULL;
                char saveFileName[256];
                // Configure file name
                ret = snprintf(saveFileName, sizeof(saveFileName), "%s_chn%u_%d",
                    g_output_file_name, chanId, writeFileCnt);
                if (ret <= 0) {
                    printf("[%s][%d] Chn %u, snprintf_s fail \n", __FUNCTION__, __LINE__, chanId);
                    g_get_exit_state[chanId] = 1;
                    return (void*)(HI_FAILURE);
                }

                fp = fopen(saveFileName, "wb");
                if (fp == NULL) {
                    printf("[%s][%d] Chn %u Can't Open File %s \n", __FUNCTION__, __LINE__, chanId, saveFileName);
                } else {
                    if ((frame.v_frame.pixel_format == HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420) ||
                        (frame.v_frame.pixel_format == HI_PIXEL_FORMAT_YVU_SEMIPLANAR_420)) {
                        // Write YUV data
                        save_yuv_file(fp, frame.v_frame, chanId);
                    } else if ((frame.v_frame.pixel_format == HI_PIXEL_FORMAT_RGB_888) ||
                               (frame.v_frame.pixel_format == HI_PIXEL_FORMAT_BGR_888)) {
                        // Write RGB data
                        save_rgb_file(fp, frame.v_frame, chanId);
                    }
                    fclose(fp);
                    writeFileCnt++;
                }
            }
            if (outputBuffer != NULL) {
                // Put out buffer to pool
                (void)pthread_mutex_lock(&g_out_buffer_pool_lock[chanId]);
                g_out_buffer_pool[chanId].push_back(outputBuffer);
                (void)pthread_mutex_unlock(&g_out_buffer_pool_lock[chanId]);
            }
            // Release Frame
            ret = hi_mpi_vdec_release_frame(chanId, &frame);

            if (ret != HI_SUCCESS) {
                printf("[%s][%d] Chn %u hi_mpi_vdec_release_frame Fail, Error Code = %x \n",
                    __FUNCTION__, __LINE__, chanId, ret);
            }
        } else {
            // 500us
            usleep(500);
        }
    }
    printf("[%s][%d] Chn %u get_pic Thread Exit \n", __FUNCTION__, __LINE__, chanId);
    return (void*)HI_SUCCESS;
}

// Cutting stream to frame
void get_every_frame(int32_t chanId, uint8_t* const inputFileBuf, uint32_t* const frameCount, uint32_t fileSize,
    hi_payload_type type, uint8_t* dataDev)
{
    int32_t i = 0;
    int32_t usedBytes = 0;
    int32_t readLen = 0;
    uint32_t count = 0;
    uint8_t* bufPointer = NULL;
    bool isFindStart = false;
    bool isFindEnd = false;

    while (1) {
        isFindStart = false;
        isFindEnd = false;

        bufPointer = inputFileBuf + usedBytes;
        readLen = fileSize - usedBytes;
        if (readLen <= 0) {
            break;
        }
// H264
        if (type == HI_PT_H264) {
            for (i = 0; i < readLen - 8; i++) {
                int32_t tmp = bufPointer[i + 3] & 0x1F;
                // Find 00 00 01
                if ((bufPointer[i] == 0) && (bufPointer[i + 1] == 0) && (bufPointer[i + 2] == 1) &&
                    (((tmp == 0x5 || tmp == 0x1) && ((bufPointer[i + 4] & 0x80) == 0x80)) ||
                    (tmp == 20 && (bufPointer[i + 7] & 0x80) == 0x80))) {
                    isFindStart = true;
                    i += 8;
                    break;
                }
            }

            for (; i < readLen - 8; i++) {
                int32_t tmp = bufPointer[i + 3] & 0x1F;
                // Find 00 00 01
                if ((bufPointer[i] == 0) && (bufPointer[i + 1] == 0) && (bufPointer[i + 2] == 1) &&
                    ((tmp == 15) || (tmp == 7) || (tmp == 8) || (tmp == 6) ||
                    ((tmp == 5 || tmp == 1) && ((bufPointer[i + 4] & 0x80) == 0x80)) ||
                    (tmp == 20 && (bufPointer[i + 7] & 0x80) == 0x80))) {
                    isFindEnd = true;
                    break;
                }
            }

            if (i > 0) {
                readLen = i;
            }

            if (isFindStart == false) {
                printf("Chn %d can not find H264 start code!readLen %d, usedBytes %d.!\n", chanId, readLen, usedBytes);
            }
            if (isFindEnd == false) {
                readLen = i + 8;
            }
        } else if (type == HI_PT_H265) {
// H265
            bool isNewPic = false;

            for (i = 0; i < readLen - 6; i++) {
                uint32_t tmp = (bufPointer[i + 3] & 0x7E) >> 1;
                // Find 00 00 01
                if ((bufPointer[i + 0] == 0) && (bufPointer[i + 1] == 0) && (bufPointer[i + 2] == 1) &&
                    (tmp >= 0 && tmp <= 21) && ((bufPointer[i + 5] & 0x80) == 0x80)) {
                    isNewPic = true;
                }

                if (isNewPic == true) {
                    isFindStart = true;
                    i += 6;
                    break;
                }
            }

            for (; i < readLen - 6; i++) {
                uint32_t tmp = (bufPointer[i + 3] & 0x7E) >> 1;
                // Find 00 00 01
                isNewPic = ((bufPointer[i + 0] == 0) && (bufPointer[i + 1] == 0) && (bufPointer[i + 2] == 1) &&
                    ((tmp == 32) || (tmp == 33) || (tmp == 34) || (tmp == 39) || (tmp == 40) ||
                    ((tmp >= 0 && tmp <= 21) && (bufPointer[i + 5] & 0x80) == 0x80)));

                if (isNewPic == true) {
                    isFindEnd = true;
                    break;
                }
            }
            if (i > 0) {
                readLen = i;
            }

            if (isFindStart == false) {
                printf("Chn %d can not find H265 start code!readLen %d, usedBytes %d.!\n", chanId, readLen, usedBytes);
            }

            if (isFindEnd == false) {
                readLen = i + 6;
            }
        }

        if (g_run_mode == ACL_HOST) {
            g_frame_addr[chanId][count] = (bufPointer - inputFileBuf) + dataDev; // Record frame address
        } else {
            g_frame_addr[chanId][count] = bufPointer; // Record frame address
        }
        g_frame_len[chanId][count] = readLen; // Record frame size
        count++;
        usedBytes = usedBytes + readLen;
    }
    // Frame count
    *frameCount = count;
}
// Delay some time
void delay_exec(uint64_t execTime, int32_t seconds)
{
    struct timeval currentTime;
    uint64_t tmpCurtime = 0;
    uint64_t secondToUs = 1000000;

    gettimeofday(&currentTime, NULL);

    tmpCurtime = currentTime.tv_sec * secondToUs + currentTime.tv_usec;

    uint64_t nextExecTime = execTime + seconds * secondToUs;

    while (tmpCurtime < nextExecTime) {
        // 500us
        usleep(500);
        gettimeofday(&currentTime, NULL);
        tmpCurtime = currentTime.tv_sec * secondToUs + currentTime.tv_usec;
    }
    return;
}

// Decode performance
void show_decode_performance()
{
    double averFrameRate = 0;
    uint32_t chnNum = 0;
    uint64_t secondToUs = 1000000;
    for (uint32_t i = g_start_chn_num; i < g_start_chn_num + g_chn_num; i++) {
        // Caculate decode cost time
        uint64_t diffTime = g_vdec_end_time[i] - g_vdec_start_time[i];
        if (diffTime == 0) {
            continue;
        }
        printf("\033[0;33m --------------------------------------------------------------------------\033[0;39m\n");
        // Caculate frame rate(fps)
        double actualFrameRate = ((double)g_vdec_get_frame_cnt[i] * secondToUs) / diffTime;

        printf("\033[0;33m chnId %u, actualFrameRate %.1f, SendFrameCnt %lu, DiffTime %lu \033[0;39m\n",
            i, actualFrameRate, g_vdec_get_frame_cnt[i], diffTime);
        printf("\033[0;33m --------------------------------------------------------------------------\033[0;39m\n");

        averFrameRate += actualFrameRate;
        chnNum++;
    }
    // Caculate average frame rate(fps)
    averFrameRate = averFrameRate / chnNum;
    printf("\033[0;33m ChanNum = %u, Average FrameRate = %.1f fps \033[0;39m\n", chnNum, averFrameRate);
    return;
}


// Parse input parameters
int32_t get_option(int32_t argc, char **argv)
{
    int32_t ret = HI_SUCCESS;

    while (1) {
        int32_t optionIndex = 0;
        option longOptions[] = {
            {"img_width", 1, 0, 'a'},
            {"img_height", 1, 0, 'b'},
            {"in_image_file", 1, 0, 'c'},
            {"in_format", 1, 0, 'd'},
            {"in_bitwidth", 1, 0, 'e'},
            {"out_width", 1, 0, 'f'},
            {"out_height", 1, 0, 'g'},
            {"out_image_file", 1, 0, 'h'},
            {"out_format", 1, 0, 'i'},
            {"width_stride", 1, 0, 'j'},
            {"height_stride", 1, 0, 'k'},
            {"write_file", 1, 0, 'l'},
            {"chn_num", 1, 0, 'm'},
            {"ref_frame_num", 1, 0, 'n'},
            {"dis_frame_num", 1, 0, 'o'},
            {"output_order", 1, 0, 'p'},
            {"send_times", 1, 0, 'q'},
            {"send_interval", 1, 0, 'r'},
            {"delay_time", 1, 0, 's'},
            {"alloc_num", 1, 0, 't'},
            {"start_chn", 1, 0, 'w'},
            {"render", 1, 0, 'v'}
        };

        int32_t parameter = getopt_long(argc, argv, "a:b:c:d:e:f:g:h:i:j:k:l:m:n:o:p:q:r:s:t:w:v:",
            longOptions, &optionIndex);
        if (parameter == -1) {
            break;
        }

        switch (parameter) {
            case 'a':
                g_in_width = atoi(optarg);
                break;
            case 'b':
                g_in_height = atoi(optarg);
                break;
            case 'c':
                strcpy(g_input_file_name, optarg);
                break;
            case 'd':
                g_in_format = atoi(optarg);
                break;
            case 'e':
                g_in_bitwidth = atoi(optarg);
                break;
            case 'f':
                g_out_width = atoi(optarg);
                break;
            case 'g':
                g_out_height = atoi(optarg);
                break;
            case 'h':
                strcpy(g_output_file_name, optarg);
                break;
            case 'i':
                g_out_format = atoi(optarg);
                break;
            case 'j':
                g_out_width_stride = atoi(optarg);
                break;
            case 'k':
                g_out_height_stride = atoi(optarg);
                break;
            case 'l':
                g_is_write_file = atoi(optarg);
                break;
            case 'm':
                g_chn_num = atoi(optarg);
                break;
            case 'n':
                g_ref_frame_num = atoi(optarg);
                break;
            case 'o':
                g_display_frame_num = atoi(optarg);
                break;
            case 'p':
                g_output_order = atoi(optarg);
                break;
            case 'q':
                g_send_times = atoi(optarg);
                break;
            case 'r':
                g_send_interval = atoi(optarg);
                break;
            case 's':
                g_delay_time = atoi(optarg);
                break;
            case 't':
                g_alloc_num = atoi(optarg);
                break;
            case 'w':
                g_start_chn_num = atoi(optarg);
                break;
            case 'v':
                g_render = atoi(optarg);
                break;
            default:
                printf("this is default!\n");
                break;
        }
        if (ret != HI_SUCCESS) {
            return ret;
        }
    }
    return ret;
}

// Check input parameters
int32_t check_option()
{
    // Check input stream width[128, 4096]
    if ((g_in_width > 4096) || (g_in_width < 128)) {
        printf("[%s][%d] input file width is invalid, width = %u \n", __FUNCTION__, __LINE__, g_in_width);
        return HI_FAILURE;
    }
    // Check input stream height[128, 4096]
    if ((g_in_height > 4096) || (g_in_height < 128)) {
        printf("[%s][%d] input file height is invalid, height = %u \n", __FUNCTION__, __LINE__, g_in_height);
        return HI_FAILURE;
    }
    // Check input stream format, valid value is 0 or 1
    if ((g_in_format != 0) && (g_in_format != 1)) {
        printf("[%s][%d] input format is invalid, format = %u \n", __FUNCTION__, __LINE__, g_in_format);
        return HI_FAILURE;
    }
    // Check input stream bit width, valid value is 8 or 10
    if ((g_in_bitwidth != 8) && (g_in_bitwidth != 10)) {
        printf("[%s][%d] input bitwidth is invalid, bitwidth = %u \n", __FUNCTION__, __LINE__, g_in_bitwidth);
        return HI_FAILURE;
    }
    // Check output image width[10, 4096]
    if ((g_out_width != 0) && ((g_out_width > 4096) || (g_out_width < 10))) {
        printf("[%s][%d] output width is invalid, width = %u \n", __FUNCTION__, __LINE__, g_out_width);
        return HI_FAILURE;
    }
    // Check output image height[6, 4096]
    if ((g_out_height != 0) && ((g_out_height > 4096) || (g_out_height < 6))) {
        printf("[%s][%d] output height is invalid, height = %u \n", __FUNCTION__, __LINE__, g_out_height);
        return HI_FAILURE;
    }
    // Check output image format, greater than 3 is invalid
    if (g_out_format > 3) {
        printf("[%s][%d] output format is invalid, format = %u \n", __FUNCTION__, __LINE__, g_out_format);
        return HI_FAILURE;
    }
    // Check output memory width stride, 16-aligned
    if ((g_out_width_stride % 16 != 0) || (g_out_width_stride < g_out_width) || (g_out_width_stride < 32)) {
        printf("[%s][%d] output width stride is invalid, width stride = %u \n",
            __FUNCTION__, __LINE__, g_out_width_stride);
        return HI_FAILURE;
    }
    // 2:RGB888, 3:BGR888
    if (((g_out_format == 2) || (g_out_format == 3)) && (g_out_width_stride < g_out_width * 3)) {
        printf("[%s][%d] output width stride is invalid, width stride = %u \n",
            __FUNCTION__, __LINE__, g_out_width_stride);
        return HI_FAILURE;
    }
    // Check output memory height stride, YUV420SP or YVU420SP need 2-aligned
    if (((g_out_height_stride % 2 != 0) && ((g_out_format == 0) || (g_out_format == 1))) ||
        (g_out_height_stride < g_out_height)) {
        printf("[%s][%d] output height stride is invalid, height stride = %u \n",
            __FUNCTION__, __LINE__, g_out_height_stride);
        return HI_FAILURE;
    }
    // Check write file flag, greater than 1 is invalid
    if (g_is_write_file > 1) {
        printf("[%s][%d] write file parameter is invalid, isWriteFile = %u \n",
            __FUNCTION__, __LINE__, g_is_write_file);
        return HI_FAILURE;
    }
    // Check channel number[1, 96]
    if ((g_chn_num > 96) || (g_chn_num < 1)) {
        printf("[%s][%d] chan num is invalid, chan num = %u \n", __FUNCTION__, __LINE__, g_chn_num);
        return HI_FAILURE;
    }
    // Check reference frame number[0, 16]
    if (g_ref_frame_num > 16) {
        printf("[%s][%d] RefFrame num is invalid, RefFrame num = %u \n", __FUNCTION__, __LINE__, g_ref_frame_num);
        return HI_FAILURE;
    }
    // Check display frame number[0, 16]
    if (g_display_frame_num > 16) {
        printf("[%s][%d] DisplayFrame num is invalid, DisplayFrame num = %u \n",
            __FUNCTION__, __LINE__, g_display_frame_num);
        return HI_FAILURE;
    }
    // Check video decoder output mode, greater than 2 is invalid
    if (g_output_order > 2) {
        printf("[%s][%d] output order is invalid, output order = %u \n", __FUNCTION__, __LINE__, g_output_order);
        return HI_FAILURE;
    }
    // Check channel start number
    if (g_start_chn_num + g_chn_num >= VDEC_MAX_CHN_NUM) {
        printf("[%s][%d] start chan num is invalid, start chan num = %u \n", __FUNCTION__, __LINE__, g_start_chn_num);
        return HI_FAILURE;
    }

    return HI_SUCCESS;
}

// Print input parameters
void print_parameter()
{
    printf("\n/************************Vdec Parameter********************/\n");
    printf("InputFileName: %s \n", g_input_file_name);
    printf("OutputFileName: %s \n", g_output_file_name);
    printf("Width: %u \n", g_in_width);
    printf("Height: %u \n", g_in_height);
    printf("InFormat: %u \n", g_in_format);
    printf("Bitwidth: %u \n", g_in_bitwidth);
    printf("OutWidth: %u \n", g_out_width);
    printf("OutHeight: %u \n", g_out_height);
    printf("OutFormat: %u \n", g_out_format);
    printf("OutWidthStride: %u \n", g_out_width_stride);
    printf("OutHeightStride: %u \n", g_out_height_stride);
    printf("ChnNum: %u \n", g_chn_num);
    printf("RefFrameNum: %u \n", g_ref_frame_num);
    printf("DisplayFrameNum: %u \n", g_display_frame_num);
    printf("OutPutOrder: %u \n", g_output_order);
    printf("SendTimes: %u \n", g_send_times);
    printf("SendInterval: %u \n", g_send_interval);
    printf("DelayTime: %u \n", g_delay_time);
    printf("AllocNum: %u \n", g_alloc_num);
    printf("StartChnNum: %u \n", g_start_chn_num);
    printf("Render: %u \n", g_render);
    printf("/**********************************************************/\n");
}

// Exception exit processing function
void vdec_handle_signal(int32_t signo)
{
    for (int32_t i = g_start_chn_num; i < g_start_chn_num + g_chn_num; i++) {
        g_send_exit_state[i] = 1;
    }
    g_exit = 1;
    printf("Program Exit Abnormally!\n");
}

void init_outbuffer_lock()
{
    // Lock init
    for (uint32_t i = g_start_chn_num; i < g_start_chn_num + g_chn_num; i++) {
        pthread_mutex_init(&g_out_buffer_pool_lock[i], NULL);
    }
}

void destroy_outbuffer_lock()
{
    // Lock init
    for (uint32_t i = g_start_chn_num; i < g_start_chn_num + g_chn_num; i++) {
        pthread_mutex_destroy(&g_out_buffer_pool_lock[i]);
    }
}

void release_outbuffer(uint32_t chanId)
{
    void* outBuffer = NULL;
    while (g_out_buffer_pool[chanId].empty() == false) {
        outBuffer = g_out_buffer_pool[chanId].back();
        g_out_buffer_pool[chanId].pop_back();
        hi_mpi_dvpp_free(outBuffer);
    }
}
// Record start time
void get_start_time()
{
    struct timeval currentTime;
    gettimeofday(&currentTime, NULL);
    // 1s = 1000000us
    g_start_time = currentTime.tv_sec * 1000000 + currentTime.tv_usec;
}

int32_t hi_dvpp_init()
{
    aclError aclRet = aclInit(NULL);
    if (aclRet != ACL_SUCCESS) {
        printf("[%s][%d] aclInit failed, error code = %d.\n", __FUNCTION__, __LINE__, aclRet);
        return HI_FAILURE;
    }
    printf("[%s][%d] aclInit Success.\n", __FUNCTION__, __LINE__);

    aclRet = aclrtSetDevice(0);
    if (aclRet != ACL_SUCCESS) {
        printf("[%s][%d] aclrtSetDevice 0 failed, error code = %d.\n", __FUNCTION__, __LINE__, aclRet);
        aclFinalize();
        return HI_FAILURE;
    }
    printf("[%s][%d] aclrtSetDevice 0 Success.\n", __FUNCTION__, __LINE__);

    aclRet = aclrtCreateContext(&g_context, 0);
    if (aclRet != ACL_SUCCESS) {
        printf("[%s][%d] aclrtCreateContext failed, error code = %d.\n", __FUNCTION__, __LINE__, aclRet);
        aclrtResetDevice(0);
        aclFinalize();
        return HI_FAILURE;
    }
    printf("[%s][%d] aclrtCreateContext Success\n", __FUNCTION__, __LINE__);

    int32_t ret = hi_mpi_sys_init();
    if (ret != HI_SUCCESS) {
        printf("[%s][%d] hi_mpi_sys_init failed, error code = %x\n", __FUNCTION__, __LINE__, ret);
        aclrtDestroyContext(g_context);
        aclrtResetDevice(0);
        aclFinalize();
        return HI_FAILURE;
    }

    aclRet = aclrtGetRunMode(&g_run_mode);
    if (aclRet != HI_SUCCESS) {
        printf("[%s][%d] aclrtGetRunMode failed, error code = %d\n", __FUNCTION__, __LINE__, aclRet);
        hi_mpi_sys_exit();
        aclrtDestroyContext(g_context);
        aclrtResetDevice(0);
        aclFinalize();
        return HI_FAILURE;
    }

    printf("[%s][%d] Dvpp system init success\n", __FUNCTION__, __LINE__);
    return HI_SUCCESS;
}

void hi_dvpp_deinit()
{
    int32_t ret = hi_mpi_sys_exit();
    if (ret != HI_SUCCESS) {
        printf("[%s][%d] hi_mpi_sys_exit failed, error code = %x.\n", __FUNCTION__, __LINE__, ret);
    }

    aclError aclRet = aclrtDestroyContext(g_context);
    if (aclRet != ACL_SUCCESS) {
        printf("[%s][%d] aclrtDestroyContext failed, error code = %d.\n", __FUNCTION__, __LINE__, aclRet);
    }

    aclRet = aclrtResetDevice(0);
    if (aclRet != ACL_SUCCESS) {
        printf("[%s][%d] aclrtResetDevice 0 failed, error code = %d.\n", __FUNCTION__, __LINE__, aclRet);
    }

    aclRet = aclFinalize();
    if (aclRet != ACL_SUCCESS) {
        printf("[%s][%d] aclFinalize failed, error code = %d.\n", __FUNCTION__, __LINE__, aclRet);
    }

    printf("[%s][%d] Dvpp system exit success\n", __FUNCTION__, __LINE__);
}