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
#include <string.h>
#include <sys/time.h>
#include <sys/prctl.h>
#include <vector>
#include <unistd.h>

#include "acl.h"
#include "acl_rt.h"
#include "hi_dvpp.h"

// vdec配置参数
const char *VDEC_FILE_NAME = "dvpp_vdec_h264_1frame_bp_51_1920x1080.h264";
const uint32_t STREAM_WIDTH = 1920;   // 输入码流宽
const uint32_t STREAM_HEIGHT = 1080;  // 输入码流高
const uint32_t STREAM_WIDTH_STRIDE = 1920;
const uint32_t STREAM_HEIGHT_STRIDE = 1080;
const uint32_t DEFAULT_DISPALY_FRAME_NUM = 2;
const uint32_t DEFAULT_REF_FRAME_NUM = 8;
const hi_payload_type VIDEO_TYPE = HI_PT_H264;
const uint32_t SEND_INTERVAL = 0;  // Stream send interval(us)
const int32_t OUT_BUFFER_CNT = 10;

// vpc配置参数
const int32_t VPC_CROP_X = 0;
const int32_t VPC_CROP_Y = 0;
const int32_t VPC_CROP_WIDTH = 192;
const int32_t VPC_CROP_HEIGHT = 100;
const int32_t VPC_OUT_WIDTH = 128;
const int32_t VPC_OUT_HEIGHT = 128;
const char* VPC_OUT_FILE_NAME = "vpc_output.rgb";

const uint32_t FILE_NAME_LEN = 500;
const uint32_t MAX_MULTI_COUNT = 256;

aclrtRunMode g_run_mode = ACL_HOST;
aclrtContext g_context = HI_NULL;
// 根据实际需要创建线程
pthread_t g_vdec_send_thread;
pthread_t g_vdec_get_thread;
uint32_t g_send_exit_state;
uint32_t g_get_exit_state;

std::vector<void *> g_out_buffer_pool;   // Out buffer pool
pthread_mutex_t g_out_buffer_pool_lock;  // Lock of out buffer pool

int32_t hi_dvpp_init()
{
    aclError aclRet = aclInit(HI_NULL);
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

int32_t read_stream_file(const char *fileName, uint8_t *&data)
{
    // Open input stream file
    FILE *fpInputFile = HI_NULL;
    fpInputFile = fopen(fileName, "rb");
    if (fpInputFile == HI_NULL) {
        printf("[%s][%d] Can't open file %s \n", __FUNCTION__, __LINE__, fileName);
        return -1;
    }

    // Calculate input stream file size
    int32_t fileSize = 0;
    fseek(fpInputFile, 0L, SEEK_END);
    fileSize = ftell(fpInputFile);
    fseek(fpInputFile, 0L, SEEK_SET);

    // Alloc buffer for all input stream file
    uint8_t *inputFileBuf = HI_NULL;
    inputFileBuf = (uint8_t *)malloc(fileSize);
    if (inputFileBuf == HI_NULL) {
        fclose(fpInputFile);
        printf("[%s][%d] Malloc InputFile Buffer Fail \n", __FUNCTION__, __LINE__);
        return -1;
    }

    // Read input stream file
    uint32_t readLen = 0;
    readLen = fread(inputFileBuf, 1, fileSize, fpInputFile);
    if (readLen != fileSize) {
        fclose(fpInputFile);
        free(inputFileBuf);
        printf("[%s][%d] Read InputFile Fail \n", __FUNCTION__, __LINE__);
        return -1;
    }
    fclose(fpInputFile);
    data = inputFileBuf;
    return fileSize;
}

// Cutting stream to frame, return frame size
int32_t get_one_frame(const uint8_t *inputFileBuf, uint32_t fileSize, int offset, hi_payload_type type)
{
    int32_t i = 0;
    int32_t readLen = 0;
    const uint8_t *bufPointer = HI_NULL;
    bool isFindStart = false;
    bool isFindEnd = false;

    if (offset >= fileSize) {
        return -1;
    }

    bufPointer = inputFileBuf + offset;
    readLen = fileSize - offset;
    if (type == HI_PT_H264) {
        for (i = 0; i < readLen - 8; i++) { // 8 bytes per time
            int32_t tmp = bufPointer[i + 3] & 0x1F; // start with the 3rd byte
            // Find 00 00 01
            if ((bufPointer[i] == 0) && (bufPointer[i + 1] == 0) && (bufPointer[i + 2] == 1) && // 1,2: search 00 00 01
                (((tmp == 0x5 || tmp == 0x1) && ((bufPointer[i + 4] & 0x80) == 0x80)) || // 4: search
                    (tmp == 20 && (bufPointer[i + 7] & 0x80) == 0x80))) { // 20, 7: search
                isFindStart = true;
                i += 8; // 8 bytes per time
                break;
            }
        }

        for (; i < readLen - 8; i++) { // 8 bytes per time
            int32_t tmp = bufPointer[i + 3] & 0x1F; // start with the 3rd byte
            // Find 00 00 01
            if ((bufPointer[i] == 0) && (bufPointer[i + 1] == 0) && (bufPointer[i + 2] == 1) && // 1,2: search 00 00 01
                ((tmp == 15) || (tmp == 7) || (tmp == 8) || (tmp == 6) || // 15,7,8,6 : start code
                    ((tmp == 5 || tmp == 1) && ((bufPointer[i + 4] & 0x80) == 0x80)) || // 5,1,4: search 00 00 01
                    (tmp == 20 && (bufPointer[i + 7] & 0x80) == 0x80))) { // 20,7: search
                isFindEnd = true;
                break;
            }
        }

        if (i > 0) {
            readLen = i;
        }
        if (isFindEnd == false) {
            readLen = i + 8; // 8 bytes per time
        }
    } else if (type == HI_PT_H265) {
        bool isNewPic = false;

        for (i = 0; i < readLen - 6; i++) { // 6 bytes per time
            uint32_t tmp = (bufPointer[i + 3] & 0x7E) >> 1; // start with the 3rd byte
            // Find 00 00 01
            if ((bufPointer[i + 0] == 0) && (bufPointer[i + 1] == 0) && (bufPointer[i + 2] == 1) && // 2,1: search
                (tmp >= 0 && tmp <= 21) && ((bufPointer[i + 5] & 0x80) == 0x80)) { // 21, 5
                isNewPic = true;
            }

            if (isNewPic == true) {
                isFindStart = true;
                i += 6; // 6 bytes per time
                break;
            }
        }

        for (; i < readLen - 6; i++) { // 6 bytes per time
            uint32_t tmp = (bufPointer[i + 3] & 0x7E) >> 1; // start with the 3rd byte
            // Find 00 00 01
            isNewPic = ((bufPointer[i + 0] == 0) && (bufPointer[i + 1] == 0) && (bufPointer[i + 2] == 1) && // 1,2: search
                        ((tmp == 32) || (tmp == 33) || (tmp == 34) || (tmp == 39) || (tmp == 40) || // 32,33,34,39,40: start code
                            ((tmp >= 0 && tmp <= 21) && (bufPointer[i + 5] & 0x80) == 0x80))); // 21,5: search

            if (isNewPic == true) {
                isFindEnd = true;
                break;
            }
        }
        if (i > 0) {
            readLen = i;
        }
        if (isFindEnd == false) {
            readLen = i + 6; // 6 bytes per time
        }
    }
    return readLen;
}

int32_t vdec_outbuf_alloc(void)
{
    // Using out buffer pool in order to prevent system oom
    void *outBuffer = HI_NULL;
    uint32_t outBufferSize = 0;
    outBufferSize = STREAM_WIDTH_STRIDE * STREAM_HEIGHT_STRIDE * 3 / 2; // *3/2: YUV420

    // Alloc out buffer
    int32_t ret =
        hi_mpi_dvpp_malloc(0, &outBuffer, outBufferSize);  // Alloc Vdec out buffer must use hi_mpi_dvpp_malloc
    if (ret != HI_SUCCESS) {
        printf("[%s][%d] hi_mpi_dvpp_malloc failed.\n", __FUNCTION__, __LINE__);
        return HI_FAILURE;
    }
    // Put buffer to pool
    g_out_buffer_pool.push_back(outBuffer);
    pthread_mutex_init(&g_out_buffer_pool_lock, HI_NULL);
    return HI_SUCCESS;
}

int32_t vdec_outbuf_free(void)
{
    while (!g_out_buffer_pool.empty()) {
        void *outBuffer = g_out_buffer_pool.back();
        g_out_buffer_pool.pop_back();
        hi_mpi_dvpp_free(outBuffer);
    }
    pthread_mutex_destroy(&g_out_buffer_pool_lock);
}

void get_current_time_us(uint64_t &timeUs)
{
    struct timeval curTime;
    gettimeofday(&curTime, HI_NULL);
    // 1s = 1000000 us
    timeUs = static_cast<uint64_t>(curTime.tv_sec) * 1000000 + curTime.tv_usec;
}

uint8_t *vdec_prepare_dev_inbuf(uint8_t *inputFileBuf, int32_t fileSize)
{
    uint8_t *dataDev = HI_NULL;
    // alloc device inbuffer mem
    int32_t ret = hi_mpi_dvpp_malloc(0, (void **)&dataDev, fileSize);
    if (ret != 0) {
        printf("[%s][%d] Malloc device memory %u failed.\n", __FUNCTION__, __LINE__, fileSize);
        return HI_NULL;
    }

    // copy host to device
    ret = aclrtMemcpy(dataDev, fileSize, inputFileBuf, fileSize, ACL_MEMCPY_HOST_TO_DEVICE);
    if (ret != ACL_SUCCESS) {
        hi_mpi_dvpp_free(dataDev);
        printf("[%s][%d] Copy host memcpy to device failed, error code = %d.\n", __FUNCTION__, __LINE__, ret);
        return HI_NULL;
    }
    return dataDev;
}

void *vdec_outbuf_pop(void)
{
    int32_t tryCnt = 0;
    void *outBuffer = HI_NULL;
    do {
        (void)pthread_mutex_lock(&g_out_buffer_pool_lock);
        if (g_out_buffer_pool.empty() == false) {
            outBuffer = g_out_buffer_pool.back();
            g_out_buffer_pool.pop_back();
            (void)pthread_mutex_unlock(&g_out_buffer_pool_lock);
            break;
        } else {
            (void)pthread_mutex_unlock(&g_out_buffer_pool_lock);
            usleep(1000);  // 1000us
        }
    } while (++tryCnt <= 10); // try 10 times
    if (tryCnt > 10) { // try 10 times
        printf("get outbuf error!\n");
    }
    return outBuffer;
}

void vdec_reset_chn(uint32_t chanId)
{
    int32_t ret = HI_SUCCESS;
    hi_vdec_chn_status status{};

    ret = hi_mpi_vdec_stop_recv_stream(chanId);
    if (ret != HI_SUCCESS) {
        printf("[%s][%d] Chn %u, hi_mpi_vdec_stop_recv_stream Fail, ret = %x \n", __FUNCTION__, __LINE__, chanId, ret);
        return;
    }
    // reset channel
    ret = hi_mpi_vdec_reset_chn(chanId);
    if (ret != HI_SUCCESS) {
        printf("[%s][%d] Chn %u, hi_mpi_vdec_reset_chn Fail, ret = %x \n", __FUNCTION__, __LINE__, chanId, ret);
        return;
    }

    ret = hi_mpi_vdec_start_recv_stream(chanId);
    if (ret != HI_SUCCESS) {
        printf("[%s][%d] Chn %u, hi_mpi_vdec_start_recv_stream Fail, ret = %x \n", __FUNCTION__, __LINE__, chanId, ret);
        return;
    }
    printf("[%s][%d] Chn %u, reset chn success \n", __FUNCTION__, __LINE__, chanId);
    return;
}

void *send_stream(void *args)
{
    prctl(PR_SET_NAME, "VdecSendStream", 0, 0, 0);
    int32_t chanId = 0;

    aclError aclRet = aclrtSetCurrentContext(g_context);
    if (aclRet != ACL_SUCCESS) {
        printf("[%s][%d] Chn %u set current context failed, error code = %d", __FUNCTION__, __LINE__, chanId, aclRet);
        return (void *)(HI_FAILURE);
    }
    uint8_t *inputFileBuf = HI_NULL;
    int32_t fileSize = read_stream_file(VDEC_FILE_NAME, inputFileBuf);
    if (fileSize < 0) {
        printf("[%s][%d] Chn %u read file %s failed\n", __FUNCTION__, __LINE__, chanId, VDEC_FILE_NAME);
        return (void *)(HI_FAILURE);
    }
    uint8_t *dataDev = HI_NULL;
    if (g_run_mode == ACL_HOST) {
        dataDev = vdec_prepare_dev_inbuf(inputFileBuf, fileSize);
        if (dataDev == HI_NULL) {
            return (void *)(HI_FAILURE);
        }
    }

    // Delay g_delay_time seconds
    usleep(5000); // delay 5000us

    // Start send stream
    hi_vdec_stream stream{};
    hi_vdec_pic_info outPicInfo;
    uint32_t readCount = 0;
    uint64_t currentSendTime, lastSendTime;
    get_current_time_us(currentSendTime);
    int32_t ret = HI_SUCCESS;
    int offset = 0;
    int32_t frameSize = 0;
    int32_t sendOneFrameCnt = 0;
    while (1) {
        if (g_send_exit_state == 1) {
            break;
        }
        offset += frameSize;
        frameSize = get_one_frame(inputFileBuf, fileSize, offset, VIDEO_TYPE);
        if (frameSize <= 0) {
            printf("[%s][%d] Chn %u no new frame!\n", __FUNCTION__, __LINE__, chanId);
            break;
        }
        printf("[%s][%d] get new frame, size %d\n", __FUNCTION__, __LINE__, frameSize);
        stream.pts = currentSendTime + SEND_INTERVAL;
        stream.end_of_frame = HI_TRUE;    // Configure flage of frame end
        stream.end_of_stream = HI_FALSE;  // Configure flage of stream end
        stream.need_display = HI_TRUE;
        stream.addr =
            (g_run_mode == ACL_HOST ? (dataDev + offset) : (inputFileBuf + offset));  // Configure input stream address
        stream.len = frameSize;                                                       // Configure input stream size

        hi_module_type mod_type = HI_MOD_VDEC;
        hi_img_align_info align_info{};
        hi_img_base_info img_base_info = {
            .width = STREAM_WIDTH, .height = STREAM_HEIGHT, .pixel_format = HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420};
        ret = hi_mpi_sys_get_image_align_info(&mod_type, 1, &img_base_info, &align_info);
        if (ret != HI_SUCCESS) {
            printf("[%s][%d] hi_mpi_sys_get_image_align_info error code = %d", __FUNCTION__, __LINE__, ret);
        }
        outPicInfo.width = STREAM_WIDTH;                // Output image width, supports resize, set 0 means no resize
        outPicInfo.height = STREAM_HEIGHT;              // Output image height, supports resize, set 0 means no resize
        outPicInfo.width_stride = align_info.width_stride;  // Output memory width stride
        outPicInfo.height_stride = align_info.height_stride;               // Output memory height stride
        outPicInfo.pixel_format = HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420;  // Configure output format
        outPicInfo.vir_addr = (uint64_t)vdec_outbuf_pop();
        outPicInfo.buffer_size = align_info.img_buf_size;
        sendOneFrameCnt = 0;
        do {
            // Send one frame data
            ret = hi_mpi_vdec_send_stream(chanId, &stream, &outPicInfo, 1000); // 1000ms
        } while ((ret == HI_ERR_VDEC_BUF_FULL) && (sendOneFrameCnt++ < 30));  // less than 30, Try again
        if (ret != HI_SUCCESS) {
            printf("[%s][%d] Chn %u hi_mpi_vdec_send_stream Fail, Error Code = 0x%x, reset channel!\n",
                __FUNCTION__,
                __LINE__,
                chanId,
                ret);
            vdec_reset_chn(chanId);
            break;
        }
        // Stream send interval
        get_current_time_us(currentSendTime);
        if ((currentSendTime - lastSendTime) < SEND_INTERVAL) {
            usleep(SEND_INTERVAL - (currentSendTime - lastSendTime));
        }
        get_current_time_us(lastSendTime);
    }
    // send EOS
    stream.addr = HI_NULL;
    stream.len = 0;
    stream.end_of_frame = HI_FALSE;
    stream.end_of_stream = HI_TRUE;  // Stream end flage
    outPicInfo.vir_addr = 0;
    outPicInfo.buffer_size = 0;
    ret = hi_mpi_vdec_send_stream(chanId, &stream, &outPicInfo, -1);
    if (ret != HI_SUCCESS) {  // if send stream timeout 30 times, end the decode process
        printf("[%s][%d] Chn %u hi_mpi_vdec_send_stream Fail, Error Code = %x, reset channel!\n",
            __FUNCTION__,
            __LINE__,
            chanId,
            ret);
        vdec_reset_chn(chanId);
    }
    free(inputFileBuf);
    if (g_run_mode == ACL_HOST) {
        hi_mpi_dvpp_free(dataDev);
    }
    printf("[%s][%d] Chn %u send_stream Thread Exit \n", __FUNCTION__, __LINE__, chanId);
    return (hi_void *)HI_SUCCESS;
}

void check_dec_result(int32_t chanId, int32_t decResult)
{
    int32_t successCnt = 0;
    int32_t failCnt = 0;
    if (decResult == 0) {  // 0: Decode success
        successCnt++;
        printf("[%s][%d] Chn %u GetFrame Success, Decode Success[%d] \n", __FUNCTION__, __LINE__, chanId, successCnt);
    } else if (decResult == 1) {  // 1: Decode fail
        failCnt++;
        printf("[%s][%d] Chn %u GetFrame Success, Decode Fail[%d] \n", __FUNCTION__, __LINE__, chanId, failCnt);
    } else if (decResult == 2) {  // 2:This result is returned for the second field of
                                  // the interlaced field stream, which is normal.
        printf("[%s][%d] Chn %u GetFrame Success, No Picture \n", __FUNCTION__, __LINE__, chanId);
    } else if (decResult == 3) {  // 3: Reference frame number set error
        failCnt++;
        printf("[%s][%d] Chn %u GetFrame Success, RefFrame Num Error[%d] \n", __FUNCTION__, __LINE__, chanId, failCnt);
    } else if (decResult == 4) {  // 4: Reference frame size set error
        failCnt++;
        printf("[%s][%d] Chn %u GetFrame Success, RefFrame Size Error[%d] \n", __FUNCTION__, __LINE__, chanId, failCnt);
    }
}
int32_t handle_vpc_output_data(hi_vpc_pic_info &outputPic, uint32_t dstBufferSize, const char outputFileName[])
{
    // copy output width and height configuration
    hi_vpc_pic_info outputPicHost = outputPic;
    FILE *dstFp = HI_NULL;
    size_t numWrite = 0;
    int32_t ret = HI_SUCCESS;
    if (g_run_mode == ACL_HOST) {
        // we cannot access memory on device, so we malloc memory of the same size at host and copy them
        int32_t ret = aclrtMallocHost(&outputPicHost.picture_address, outputPic.picture_buffer_size);
        if (ret != ACL_SUCCESS) {
            printf("[%s][%d] malloc output buffer in host failed, ret = %d!\n", __FUNCTION__, __LINE__, ret);
            return HI_FAILURE;
        }
        ret = aclrtMemcpy(outputPicHost.picture_address,
            outputPic.picture_buffer_size,
            outputPic.picture_address,
            outputPic.picture_buffer_size,
            ACL_MEMCPY_DEVICE_TO_HOST);
        if (ret != ACL_SUCCESS) {
            printf("[%s][%d] Copy memcpy to host failed, ret = %d.\n", __FUNCTION__, __LINE__, ret);
            goto OUT0;
        }
    }

    dstFp = fopen(outputFileName, "wb");
    if (dstFp == HI_NULL) {
        printf("[%s][%d] fopen %s failed!\n", __FUNCTION__, __LINE__, outputFileName);
        ret = HI_FAILURE;
        goto OUT0;
    }
    numWrite = fwrite(outputPicHost.picture_address, 1, dstBufferSize, dstFp);
    if (numWrite < dstBufferSize) {
        printf("[%s][%d] write output data failed, numWrite = %zu, data size = %u\n",
            __FUNCTION__,
            __LINE__,
            numWrite,
            dstBufferSize);
        ret = HI_FAILURE;
        goto OUT1;
    }
    fflush(dstFp);
OUT1:
    fclose(dstFp);
OUT0:
    if (g_run_mode == ACL_HOST) {
        aclrtFreeHost(outputPicHost.picture_address);
        outputPicHost.picture_address = HI_NULL;
    }
    dstFp = HI_NULL;
    return ret;
}

void memset_buffer(hi_vpc_pic_info &picInfo)
{
    if (g_run_mode == ACL_HOST) {
        int32_t ret = aclrtMemset(picInfo.picture_address, picInfo.picture_buffer_size, 0, picInfo.picture_buffer_size);
        if (ret != HI_SUCCESS) {
            printf("[%s][%d] memset_s failed, ret = %d!\n", __FUNCTION__, __LINE__, ret);
        }
    } else {
        memset(picInfo.picture_address, 0, picInfo.picture_buffer_size);
    }
}

int32_t handle_vpc_process(hi_video_frame &frame)
{
    hi_vpc_chn chnId = 0;
    // configure input picture
    hi_vpc_pic_info inputPic;
    inputPic.picture_width = frame.width;
    inputPic.picture_height = frame.height;
    inputPic.picture_width_stride = frame.width_stride[0];
    inputPic.picture_height_stride = frame.height_stride[0];
    inputPic.picture_format = frame.pixel_format;
    inputPic.picture_address = frame.virt_addr[0];
    inputPic.picture_buffer_size = frame.width_stride[0] * frame.height_stride[0] * 3 / 2; // YUV420: *3/2

    // malloc and configure output buffer
    hi_vpc_pic_info outputPic;
    hi_module_type mod_type = HI_MOD_VPC;
    hi_img_align_info align_info{};
    hi_img_base_info img_base_info = {
        .width = VPC_OUT_WIDTH, .height = VPC_OUT_HEIGHT, .pixel_format = HI_PIXEL_FORMAT_RGB_888};
    hi_s32 ret = hi_mpi_sys_get_image_align_info(&mod_type, 1, &img_base_info, &align_info);
    if (ret != HI_SUCCESS) {
        printf("[%s][%d] hi_mpi_sys_get_image_align_info error code = %d", __FUNCTION__, __LINE__, ret);
    }
    outputPic.picture_width = img_base_info.width;
    outputPic.picture_height = img_base_info.height;
    outputPic.picture_format = img_base_info.pixel_format;
    outputPic.picture_width_stride = align_info.width_stride;
    outputPic.picture_height_stride = align_info.height_stride;
    outputPic.picture_buffer_size = align_info.img_buf_size;
    // malloc output buffer
    ret = hi_mpi_dvpp_malloc(0, &outputPic.picture_address, outputPic.picture_buffer_size);
    if (ret != HI_SUCCESS) {
        printf("[%s][%d] dvpp alloc failed, error code = %d", __FUNCTION__, __LINE__, ret);
        return HI_FAILURE;
    }
    memset_buffer(outputPic);

    hi_vpc_crop_resize_region cropResizeRegionInfo;
    // This array can be configured by what you want to crop and resize. All regions are the same here
    cropResizeRegionInfo.dest_pic_info = outputPic;
    cropResizeRegionInfo.crop_region.left_offset = VPC_CROP_X;
    cropResizeRegionInfo.crop_region.top_offset = VPC_CROP_Y;
    cropResizeRegionInfo.crop_region.crop_width = VPC_CROP_WIDTH;
    cropResizeRegionInfo.crop_region.crop_height = VPC_CROP_HEIGHT;
    cropResizeRegionInfo.resize_info.resize_width = VPC_OUT_WIDTH;
    cropResizeRegionInfo.resize_info.resize_height = VPC_OUT_HEIGHT;
    cropResizeRegionInfo.resize_info.interpolation = 1;

    // start to call hi_mpi_vpc_crop_resize interface
    uint32_t taskID = 0;
    ret = hi_mpi_vpc_crop_resize(chnId, &inputPic, &cropResizeRegionInfo, 1, &taskID, -1);
    if (ret != HI_SUCCESS) {
        printf("[%s][%d] hi_mpi_vpc_crop_resize failed, ret = %#x!\n", __FUNCTION__, __LINE__, ret);
        hi_mpi_dvpp_free(cropResizeRegionInfo.dest_pic_info.picture_address);
        cropResizeRegionInfo.dest_pic_info.picture_address = HI_NULL;
        return HI_FAILURE;
    }

    // asign the taskID that you get from the last interface to get the process result
    uint32_t taskIDResult = taskID;
    ret = hi_mpi_vpc_get_process_result(chnId, taskIDResult, -1);
    if (ret != HI_SUCCESS) {
        printf("[%s][%d] hi_mpi_vpc_get_process_result failed, ret = %#x!\n", __FUNCTION__, __LINE__, ret);
        hi_mpi_dvpp_free(cropResizeRegionInfo.dest_pic_info.picture_address);
        cropResizeRegionInfo.dest_pic_info.picture_address = HI_NULL;
        return HI_FAILURE;
    }

    // write the first picture in the cropResizeRegionInfos array
    ret = handle_vpc_output_data(cropResizeRegionInfo.dest_pic_info, outputPic.picture_buffer_size, VPC_OUT_FILE_NAME);
    if (ret != HI_SUCCESS) {
        printf("[%s][%d] handle_vpc_output_data failed!\n", __FUNCTION__, __LINE__);
    }

    hi_mpi_dvpp_free(cropResizeRegionInfo.dest_pic_info.picture_address);
    cropResizeRegionInfo.dest_pic_info.picture_address = HI_NULL;

    return ret;
}

void *get_pic_work_thread(void *args)
{
    prctl(PR_SET_NAME, "VdecGetPic", 0, 0, 0);
    uint32_t chanId = 0;

    aclError aclRet = aclrtSetCurrentContext(g_context);
    if (aclRet != ACL_SUCCESS) {
        printf("[%s][%d] Chn %u set current context failed, error code = %d", __FUNCTION__, __LINE__, chanId, aclRet);
        g_get_exit_state = 1;
        return (void *)(HI_FAILURE);
    }

    int32_t ret = HI_SUCCESS;
    hi_video_frame_info frame;
    hi_vdec_stream stream;
    int32_t timeOut = 1000;
    hi_vdec_supplement_info stSupplement{};

    while (1) {
        if (g_get_exit_state == 1) {
            break;
        }
        ret = hi_mpi_vdec_get_frame(chanId, &frame, &stSupplement, &stream, timeOut);
        if (ret == HI_SUCCESS) {
            // Flush decode end time
            void *outputBuffer = (void *)frame.v_frame.virt_addr[0];
            int32_t decResult = frame.v_frame.frame_flag;
            check_dec_result(chanId, decResult);
            // Decode result write to a file
            if ((decResult == 0) && (outputBuffer != HI_NULL) && (stream.need_display == HI_TRUE)) {
                // vpc crop resize csc
                handle_vpc_process(frame.v_frame);
            }
            if (outputBuffer != HI_NULL) {
                // Put out buffer to pool
                (void)pthread_mutex_lock(&g_out_buffer_pool_lock);
                g_out_buffer_pool.push_back(outputBuffer);
                (void)pthread_mutex_unlock(&g_out_buffer_pool_lock);
            }
            // Release Frame
            ret = hi_mpi_vdec_release_frame(chanId, &frame);
            if (ret != HI_SUCCESS) {
                printf("[%s][%d] Chn %u hi_mpi_vdec_release_frame Fail, Error Code = %x \n",
                    __FUNCTION__,
                    __LINE__,
                    chanId,
                    ret);
            }
        } else {
            // 500us
            usleep(500);
        }
    }
    printf("[%s][%d] Chn %u get_pic Thread Exit \n", __FUNCTION__, __LINE__, chanId);
    return (void *)HI_SUCCESS;
}

int32_t create_send_stream_thread()
{
    int32_t ret;

    // Create send thread
    ret = pthread_create(&g_vdec_send_thread, HI_NULL, send_stream, HI_NULL);
    if (ret != 0) {
        printf("[%s][%d] create send stream thread Fail, ret = %d \n", __FUNCTION__, __LINE__, ret);
        g_vdec_send_thread = 0;
        return ret;
    }

    return ret;
}

int32_t create_get_pic_thread()
{
    int32_t ret;

    // Create get thread
    ret = pthread_create(&g_vdec_get_thread, HI_NULL, get_pic_work_thread, HI_NULL);
    if (ret != 0) {
        printf("[%s][%d] create get pic thread Fail, ret = %d \n", __FUNCTION__, __LINE__, ret);
        g_vdec_get_thread = 0;
        return ret;
    }

    return ret;
}

void stop_send_stream_thread()
{
    // Set thread state to 1, then thread will end
    g_send_exit_state = 1;
}

void stop_get_pic_thread()
{
    // Set thread state to 1, then send stream and get pic thread will end
    g_get_exit_state = 1;
    g_send_exit_state = 1;
}

int32_t vdec_create(void)
{
    int32_t ret = HI_SUCCESS;
    int32_t chan = 0;
    hi_vdec_chn_attr chnAttr{};
    hi_data_bit_width bitWidth = HI_DATA_BIT_WIDTH_8;

    chnAttr.type = VIDEO_TYPE;  // Input stream is H264
    // Configure channel attribute
    chnAttr.mode = HI_VDEC_SEND_MODE_FRAME;  // Only support frame mode
    chnAttr.pic_width = STREAM_WIDTH;
    chnAttr.pic_height = STREAM_HEIGHT;
    // Stream buffer size, Recommended value is width * height * 3 / 2
    chnAttr.stream_buf_size = STREAM_WIDTH * STREAM_HEIGHT * 3 / 2;
    chnAttr.frame_buf_cnt = DEFAULT_REF_FRAME_NUM + DEFAULT_DISPALY_FRAME_NUM + 1;

    hi_pic_buf_attr buf_attr{
        STREAM_WIDTH, STREAM_HEIGHT, 0, HI_DATA_BIT_WIDTH_8, HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420, HI_COMPRESS_MODE_NONE};
    chnAttr.frame_buf_size = hi_vdec_get_pic_buf_size(chnAttr.type, &buf_attr);

    // Configure video decoder channel attribute
    chnAttr.video_attr.ref_frame_num = DEFAULT_REF_FRAME_NUM;
    chnAttr.video_attr.temporal_mvp_en = HI_TRUE;
    chnAttr.video_attr.tmv_buf_size = hi_vdec_get_tmv_buf_size(chnAttr.type, STREAM_WIDTH, STREAM_HEIGHT);

    ret = hi_mpi_vdec_create_chn(chan, &chnAttr);
    if (ret != HI_SUCCESS) {
        printf("[%s][%d] chn %d hi_mpi_vdec_create_chn Fail, ret = %x \n", __FUNCTION__, __LINE__, chan, ret);
        return ret;
    }

    hi_vdec_chn_param chnParam;
    // Get channel parameter
    ret = hi_mpi_vdec_get_chn_param(0, &chnParam);
    if (ret != HI_SUCCESS) {
        printf("[%s][%d] Chn %u, hi_mpi_vdec_get_chn_param Fail, ret = %x \n", __FUNCTION__, __LINE__, chan, ret);
        return ret;
    }
    chnParam.video_param.dec_mode = HI_VIDEO_DEC_MODE_IPB;
    chnParam.video_param.compress_mode = HI_COMPRESS_MODE_HFBC;
    chnParam.video_param.video_format = HI_VIDEO_FORMAT_TILE_64x16;
    chnParam.display_frame_num = DEFAULT_DISPALY_FRAME_NUM;
    chnParam.video_param.out_order = HI_VIDEO_OUT_ORDER_DISPLAY;  // Display sequence

    // Set channel parameter
    ret = hi_mpi_vdec_set_chn_param(chan, &chnParam);
    if (ret != HI_SUCCESS) {
        printf("[%s][%d] Chn %u, hi_mpi_vdec_set_chn_param Fail, ret = %x \n", __FUNCTION__, __LINE__, chan, ret);
        return ret;
    }

    // Decoder channel start receive stream
    ret = hi_mpi_vdec_start_recv_stream(chan);
    if (ret != HI_SUCCESS) {
        printf("[%s][%d] Chn %u, hi_mpi_vdec_start_recv_stream Fail, ret = %x \n", __FUNCTION__, __LINE__, chan, ret);
        return ret;
    }

    return ret;
}

void vdec_destroy()
{
    int32_t ret = HI_SUCCESS;
    int32_t chan = 0;
    // Decoder channel stop receive stream
    ret = hi_mpi_vdec_stop_recv_stream(chan);
    if (ret != HI_SUCCESS) {
        printf("[%s][%d] Chn %u, hi_mpi_vdec_stop_recv_stream Fail, ret = %x \n", __FUNCTION__, __LINE__, chan, ret);
    }
    // Destroy channel
    ret = hi_mpi_vdec_destroy_chn(chan);
    if (ret != HI_SUCCESS) {
        printf("[%s][%d] Chn %u, hi_mpi_vdec_destroy_chn Fail, ret = %x \n", __FUNCTION__, __LINE__, chan, ret);
    }
}

int32_t vpc_create()
{
    // create vpc channel
    hi_vpc_chn_attr stChnAttr{};
    int32_t s32Ret = hi_mpi_vpc_create_chn(0, &stChnAttr);
    if (s32Ret != HI_SUCCESS) {
        printf("hi_mpi_vpc_create_chn failed, ret = %#x\n", s32Ret);
        return s32Ret;
    }
    return s32Ret;
}

int32_t vpc_destroy()
{
    return hi_mpi_vpc_destroy_chn(0);
}

void wait_vdec_end()
{
    int32_t ret = HI_SUCCESS;
    int32_t waitTimes;
    int32_t sleepTime = 10000;  // 10000us
    hi_vdec_chn_status status{};
    hi_vdec_chn_status pre_status{};
    int32_t chan = 0;

    if (g_vdec_send_thread != 0) {
        // Wait send thread exit
        ret = pthread_join(g_vdec_send_thread, HI_NULL);
    }
    g_vdec_send_thread = 0;

    waitTimes = 0;
    // Wait channel decode over
    while (1) {
        ret = hi_mpi_vdec_query_status(chan, &status);
        if (ret != HI_SUCCESS) {
            printf("[%s][%d] Chn %u, hi_mpi_vdec_query_status Fail, ret = %x \n", __FUNCTION__, __LINE__, chan, ret);
            break;
        }
        if (((status.left_stream_bytes == 0) && (status.left_decoded_frames == 0)) || (g_get_exit_state == 1)) {
            break;
        }
        if (status.left_decoded_frames == pre_status.left_decoded_frames) {
            waitTimes += sleepTime;
        } else {
            waitTimes = 0;
        }
        pre_status = status;
        // 10000us
        usleep(sleepTime);

        if (waitTimes >= 5000000) {  // 5000000 us
            vdec_reset_chn(chan);
            break;
        }
    }

    // 1000000us
    usleep(1000000);

    // Notify get thread exit
    g_get_exit_state = 1;
    if (g_vdec_get_thread != 0) {
        // Wait get thread exit
        pthread_join(g_vdec_get_thread, HI_NULL);
    }
    g_vdec_get_thread = 0;
}

int32_t main(int32_t argc, char *argv[])
{
    int32_t ret = HI_SUCCESS;

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
        vdec_destroy();
        hi_dvpp_deinit();
        return 0;
    }
    // create vpc
    if (vpc_create() != HI_SUCCESS) {
        vdec_destroy();
        hi_dvpp_deinit();
        return 0;
    }
    vdec_outbuf_alloc();
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
    // Destroy video decoder
    vdec_destroy();
    // destroy vpc
    vpc_destroy();
    vdec_outbuf_free();
    // Dvpp system exit
    hi_dvpp_deinit();
    return 0;
}