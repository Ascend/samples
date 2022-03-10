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
#include "sample_comm.h"
#include <vector>
#include "acl.h"
#include "acl_rt.h"

using namespace std;

const uint32_t FILE_NAME_LEN = 128;
const uint32_t JPEGD_ALIGN_W = 64;
const uint32_t JPEGD_ALIGN_H = 16;

const uint64_t MULTIPLE_S_TO_US        = 1000000;
const uint32_t JPEGD_DECODED_QUEUE_NUM = 10;

bool g_whole_dir_decode   = HI_FALSE;
int32_t g_send_circle    = 0;
int32_t g_wait_time      = -1;
uint32_t g_width        = 3840;
uint32_t g_height       = 2160;
uint32_t g_out_width     = 1920;
uint32_t g_out_height    = 1080;
uint32_t g_chn_num       = 1;
uint32_t g_bit_width     = 8;
uint32_t g_write_file    = 1;
uint32_t g_start_channel = 0;
uint32_t g_delay_time    = 5;
uint32_t g_performance  = 0;
uint64_t g_start_time    = 0;

char g_input_file_name[FILE_NAME_LEN]  = "infile";
char g_output_file_name[FILE_NAME_LEN] = "outfile";
char g_compatible_dir_name[FILE_NAME_LEN]  = "./";

aclrtContext g_context = nullptr;
aclrtRunMode g_run_mode = ACL_DEVICE;
hi_vdec_send_mode g_video_mode = HI_VDEC_SEND_MODE_FRAME;
hi_compress_mode g_compress_mode = HI_COMPRESS_MODE_NONE;
hi_pixel_format g_pixel_format = HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420;

vector<void *> g_out_buffer_pool[VDEC_MAX_CHN_NUM];
pthread_mutex_t g_out_buffer_pool_lock[VDEC_MAX_CHN_NUM];

static uint64_t g_end_get_time[VDEC_MAX_CHN_NUM] = {0};
static uint64_t g_get_frame_cnt[VDEC_MAX_CHN_NUM] = {0};
static uint64_t g_start_send_time[VDEC_MAX_CHN_NUM] = {0};
static VDEC_THREAD_PARAM_S g_jpegd_thread_param[VDEC_MAX_CHN_NUM];

const uint32_t JPEGD_PERFOR_MODE_BUF_NUM   = 6;
const uint32_t JPEGD_PERFOR_MODE_QUERY_CNT = 20000;
const uint32_t JPEGD_SLEEP_INTERVAL_TIME   = 5000;

static int32_t save_file_type(hi_pixel_format pixelFormat,
                            hi_compress_mode compressMode,
                            char *typeName,
                            uint32_t typeNameLen)
{
    int32_t ret;
    if (compressMode == HI_COMPRESS_MODE_HFBC) {
        ret = snprintf(typeName, typeNameLen - 1, "hfbc");
    } else if (pixelFormat == HI_PIXEL_FORMAT_RGB_888) {
        ret = snprintf(typeName, typeNameLen - 1, "rgb888");
    } else if (pixelFormat == HI_PIXEL_FORMAT_BGR_888) {
        ret = snprintf(typeName, typeNameLen - 1, "bgr888");
    } else if (pixelFormat == HI_PIXEL_FORMAT_ARGB_8888) {
        ret = snprintf(typeName, typeNameLen - 1, "argb8888");
    } else if (pixelFormat == HI_PIXEL_FORMAT_ABGR_8888) {
        ret = snprintf(typeName, typeNameLen - 1, "abgr8888");
    } else if ((pixelFormat >= HI_PIXEL_FORMAT_YUV_400) && (pixelFormat < HI_PIXEL_FORMAT_BUTT)) {
        ret = snprintf(typeName, typeNameLen - 1, "yuv");
    } else {
        SAMPLE_PRT("pixelFormat type err\n");
        return HI_FAILURE;
    }

    return ret;
}

static inline void jpegd_get_time_stamp_us(uint64_t *val)
{
    struct timeval stTimeVal;
    gettimeofday(&stTimeVal, nullptr);
    *val = (uint64_t)stTimeVal.tv_sec * MULTIPLE_S_TO_US + stTimeVal.tv_usec;
}

void jpegd_handle_signal(int32_t signo)
{
    if (SIGINT == signo || SIGTSTP == signo || SIGTERM == signo) {
        for (uint32_t idxChn = g_start_channel; idxChn < g_start_channel + g_chn_num; idxChn++) {
            g_jpegd_thread_param[idxChn].enSendThreadCtrl = THREAD_CTRL_STOP;
            g_jpegd_thread_param[idxChn].enGetThreadCtrl  = THREAD_CTRL_STOP;
        }
        SAMPLE_PRT("\033[0;31mprogram exit abnormally!\033[0;39m\n");
    }
}

void jpegd_usage(char *sPrgNm)
{
    aclError aclRet = aclrtGetRunMode(&g_run_mode);
    if (aclRet == ACL_SUCCESS) {
        if (g_run_mode == ACL_HOST) {
            SAMPLE_PRT(" Running in Host!\n");
        } else if (g_run_mode == ACL_DEVICE) {
            SAMPLE_PRT(" Running in Device!\n");
        } else {
            SAMPLE_PRT(" Running in Invalid platform! runMode:%u\n", g_run_mode);
            return;
        }
    } else {
        SAMPLE_PRT(" Get run mode fail! acl ret:%#x\n", aclRet);
        return;
    }

    SAMPLE_PRT("\n/*********************************************************/\n");
    SAMPLE_PRT("Usage :\n");
    SAMPLE_PRT("\t:example: ./jpegd_demo --in_image_file 420_1920x1080.jpg\
 --img_width 1920 --img_height 1080 --pixel_mode 2\
 --chn_num 1 --out_image_file 420_1920x1080.yuv --send_circle 100\n");
    SAMPLE_PRT("\t\n");
    SAMPLE_PRT("\t in_image_file: input image name.\n");
    SAMPLE_PRT("\t img_width: input image width.\n");
    SAMPLE_PRT("\t img_height: input image height.\n");
    SAMPLE_PRT("\t in_bitwidth:\n");
    SAMPLE_PRT("\t\t  8: HI_DATA_BIT_WIDTH_8\n");
    SAMPLE_PRT("\t\t 10: HI_DATA_BIT_WIDTH_10\n");
    SAMPLE_PRT("\t chn_num: vdec channel num.\n");
    SAMPLE_PRT("\t out_width: output width.\n");
    SAMPLE_PRT("\t out_heigth: output height.\n");
    SAMPLE_PRT("\t video_mode: send video by stream or frame.\n");
    SAMPLE_PRT("\t pixel_mode: yuv pixel format.\n");
    SAMPLE_PRT("\t compress_mode: compress mode none/tile/line.\n");
    SAMPLE_PRT("\t send_circle: send stream circle times.\n");
    SAMPLE_PRT("\t              100 means decode 100 times.\n");
    SAMPLE_PRT("\t               -1 means decode forever until Ctrl+C or power off.\n");
    SAMPLE_PRT("\t write_file: 0 won't write out image file\n");
    SAMPLE_PRT("\t             1 write out image file. It's defalut value\n");
    SAMPLE_PRT("\t out_image_file: output image file name.\n");
    SAMPLE_PRT("\t start_chn: start chn for decode.\n");
    SAMPLE_PRT("\t delay_time: start decode after input seconds.\n");
    SAMPLE_PRT("\t performance_mode: mode to test decode fps.\n");
    SAMPLE_PRT("\t wait_time: max wait input seconds,\n");
    SAMPLE_PRT("\t            JpegdDemo would exit after input seconds if it still running.\n");
    SAMPLE_PRT("\t whole_dir: compatible mode decode dir path. JPEGD would decode all jpeg to test compatibility\n");
    SAMPLE_PRT("\t If you want to check whether pics in folder can be decoded, run like:\n");
    SAMPLE_PRT("\t ./jpegd_demo --whole_dir ./ImageNetRaw/ --write_file 1\n");
    SAMPLE_PRT("\t If you want to get JPEGD fps, run like:\n");
    SAMPLE_PRT("\t ./jpegd_demo --performance_mode 1 --in_image_file 420_1920x1080.jpg\
 --img_width 1920 --img_height 1080 --pixel_mode 2 --chn_num 18 --send_circle 1000 --delay_time 20\n");
    SAMPLE_PRT("/*********************************************************/\n\n");
}

int32_t get_option(int32_t argc, char **argv)
{
    int32_t c = 0;
    int32_t option_index = 0;
    int32_t ret = 0;
    struct option long_options[] =
    {
        {"img_width",        1, nullptr, 'w'},
        {"img_height",       1, nullptr, 'h'},
        {"in_bitwidth",      1, nullptr, 'b'},
        {"chn_num",          1, nullptr, 'c'},
        {"out_width",        1, nullptr, 'd'},
        {"out_height",       1, nullptr, 'g'},
        {"video_mode",       1, nullptr, 'e'},
        {"send_circle",      1, nullptr, 's'},
        {"pixel_mode",       1, nullptr, 'p'},
        {"compress_mode",    1, nullptr, 'C'},
        {"dis_frame_num",    1, nullptr, 'D'},
        {"in_image_file",    1, nullptr, 'F'},
        {"out_image_file",   1, nullptr, 'O'},
        {"write_file",       1, nullptr, 'W'},
        {"whole_dir",        1, nullptr, 'H'},
        {"start_chn",        1, nullptr, 'S'},
        {"delay_time",       1, nullptr, 'Y'},
        {"wait_time",        1, nullptr, 'a'},
        {"performance_mode", 1, nullptr, 'P'},
        {nullptr,            0, nullptr, 0}
    };

    while (1) {
        option_index = 0;

        c = getopt_long(argc, argv, "w:h:b:c:d:g:e:s:p:C:D:F:O:W:H:S:Y:a:P:", long_options, &option_index);
        if (c == -1) {
            break;
        }
        switch (c) {
            case 'w':
                g_width = atoi(optarg);
                break;
            case 'h':
                g_height = atoi(optarg);
                break;
            case 'b':
                g_bit_width = atoi(optarg);
                break;
            case 'c':
                g_chn_num = atoi(optarg);
                break;
            case 'd':
                g_out_width = atoi(optarg);
                break;
            case 'g':
                g_out_height = atoi(optarg);
                break;
            case 'e':
                g_video_mode = (hi_vdec_send_mode)atoi(optarg);
                break;
            case 's':
                g_send_circle = (hi_bool)atoi(optarg);
                break;
            case 'p':
                g_pixel_format = (hi_pixel_format)atoi(optarg);
                break;
            case 'C':
                g_compress_mode = (hi_compress_mode)atoi(optarg);
                break;
            case 'F':
                strcpy(g_input_file_name, optarg);
                break;
            case 'O':
                strcpy(g_output_file_name, optarg);
                break;
            case 'W':
                g_write_file = atoi(optarg);
                break;
            case 'H':
                strcpy(g_compatible_dir_name, optarg);
                g_whole_dir_decode = HI_TRUE;
                break;
            case 'S':
                g_start_channel = atoi(optarg);
                break;
            case 'P':
                g_performance = atoi(optarg);
                break;
            case 'Y':
                g_delay_time = atoi(optarg);
                break;
            case 'a':
                g_wait_time = atoi(optarg);
                break;
            default:
                SAMPLE_PRT("unsupport option!\n");
                break;
        }
    }

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

int32_t check_option()
{
    // Check input stream width[32, 8192]
    if ((g_width > 8192) || (g_width < 32)) {
        SAMPLE_PRT("input file width is invalid, width = %u \n", g_width);
        return HI_FAILURE;
    }
    // Check input stream height[32, 8192]
    if ((g_height > 8192) || (g_height < 32)) {
        SAMPLE_PRT("input file height is invalid, height = %u \n", g_height);
        return HI_FAILURE;
    }
    // Check input stream format, valid value is 0 or 1
    if ((g_pixel_format != HI_PIXEL_FORMAT_YUV_400) &&
        (g_pixel_format != HI_PIXEL_FORMAT_UNKNOWN) &&
        (g_pixel_format != HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420) &&
        (g_pixel_format != HI_PIXEL_FORMAT_YVU_SEMIPLANAR_420) &&
        (g_pixel_format != HI_PIXEL_FORMAT_YUV_SEMIPLANAR_422) &&
        (g_pixel_format != HI_PIXEL_FORMAT_YVU_SEMIPLANAR_422) &&
        (g_pixel_format != HI_PIXEL_FORMAT_YUV_SEMIPLANAR_440) &&
        (g_pixel_format != HI_PIXEL_FORMAT_YVU_SEMIPLANAR_440) &&
        (g_pixel_format != HI_PIXEL_FORMAT_YUV_SEMIPLANAR_444) &&
        (g_pixel_format != HI_PIXEL_FORMAT_YVU_SEMIPLANAR_444)) {
        SAMPLE_PRT("format is invalid, format = %d \n", g_pixel_format);
        return HI_FAILURE;
    }
    // Check input stream bit width, valid value is 8 or 10
    if ((g_bit_width != 8) && (g_bit_width != 10)) {
        SAMPLE_PRT("input bitwidth is invalid, bitwidth = %u \n", g_bit_width);
        return HI_FAILURE;
    }
    // Check channel number[1, 96]
    if ((g_chn_num > 96) || (g_chn_num < 1)) {
        SAMPLE_PRT("chan num is invalid, chan num = %u \n", g_chn_num);
        return HI_FAILURE;
    }
    // Check channel start number
    if (g_start_channel + g_chn_num >= VDEC_MAX_CHN_NUM) {
        SAMPLE_PRT("total chan num is invalid,start chan = %u, chn num = %u \n", g_start_channel, g_chn_num);
        return HI_FAILURE;
    }

    return HI_SUCCESS;
}

void print_arguement()
{
    SAMPLE_PRT("\n/*********************************************************/\n");
    SAMPLE_PRT("g_input_file_name: %s \n", g_input_file_name);
    SAMPLE_PRT("g_output_file_name: %s \n", g_output_file_name);
    SAMPLE_PRT("g_width: %u \n", g_width);
    SAMPLE_PRT("g_height: %u \n", g_height);
    SAMPLE_PRT("g_out_height: %u \n", g_out_height);
    SAMPLE_PRT("g_out_width: %u \n", g_out_width);
    SAMPLE_PRT("g_chn_num: %u \n", g_chn_num);
    SAMPLE_PRT("g_bit_width: %u \n", g_bit_width);
    SAMPLE_PRT("g_video_mode: %d \n", g_video_mode);
    SAMPLE_PRT("g_send_circle: %d \n", g_send_circle);
    SAMPLE_PRT("g_pixel_format: %d \n", g_pixel_format);
    SAMPLE_PRT("g_compress_mode: %d \n", g_compress_mode);
    SAMPLE_PRT("g_whole_dir_decode: %d\n", g_whole_dir_decode);
    if (g_whole_dir_decode == HI_TRUE) {
        SAMPLE_PRT(" compatibleTestDirName: %s \n", g_compatible_dir_name);
    }
    SAMPLE_PRT("g_start_channel: %u \n", g_start_channel);
    SAMPLE_PRT("g_performance: %u \n", g_performance);
    SAMPLE_PRT("g_wait_time: %d \n", g_wait_time);
}

// 以启动通道前的时刻为基准等待delayseconds秒
// 性能测试专用,防止通道启动时间不一致导致通道间性能数据差异过大
void delay_exec(uint64_t execTime, int delaySeconds)
{
    struct timeval currentTime;
    uint64_t tmpCurtime = 0;

    gettimeofday(&currentTime, nullptr);
    tmpCurtime = currentTime.tv_sec * MULTIPLE_S_TO_US + currentTime.tv_usec;

    uint64_t nextexcTime = execTime + delaySeconds * MULTIPLE_S_TO_US;

    while (tmpCurtime < nextexcTime) {
        usleep(500);
        gettimeofday(&currentTime, nullptr);
        tmpCurtime = currentTime.tv_sec * MULTIPLE_S_TO_US + currentTime.tv_usec;
    }
    return;
}

void jpegd_init_start_time()
{
    struct timeval currentTime;

    gettimeofday(&currentTime, nullptr);
    g_start_time = currentTime.tv_sec * MULTIPLE_S_TO_US + currentTime.tv_usec;
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

    if ((pixel_format == HI_PIXEL_FORMAT_YVU_SEMIPLANAR_420) ||
        (pixel_format == HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420)) {
        u32UvWidth  = u32AlignW;
        u32Size     = u32Ysize * 3 / 2;
        u32UvHeight = u32AlignH / 2;
    } else if ((pixel_format == HI_PIXEL_FORMAT_YUV_SEMIPLANAR_422) ||
               (pixel_format == HI_PIXEL_FORMAT_YVU_SEMIPLANAR_422)) {
        u32UvWidth  = u32AlignW;
        u32Size     = u32Ysize * 2;
        u32UvHeight = u32AlignH;
    } else if ((pixel_format == HI_PIXEL_FORMAT_YUV_SEMIPLANAR_440) ||
               (pixel_format == HI_PIXEL_FORMAT_YVU_SEMIPLANAR_440)) {
        u32UvWidth  = u32AlignW * 2;
        u32Size     = u32Ysize * 2;
        u32UvHeight = u32AlignH / 2;
    } else if ((pixel_format == HI_PIXEL_FORMAT_YUV_SEMIPLANAR_444) ||
               (pixel_format == HI_PIXEL_FORMAT_YVU_SEMIPLANAR_444)) {
        u32UvWidth  = u32AlignW * 2;
        u32Size     = u32Ysize * 3;
        u32UvHeight = u32AlignH;
    } else if (pixel_format == HI_PIXEL_FORMAT_YUV_400) {
        u32UvWidth  = u32AlignW;
        u32Size     = u32Ysize;
        u32UvHeight = 0;
    } else {
        SAMPLE_PRT("This YUV format:%u is not support during save %s!\n", pixel_format, fileName.c_str());
        return;
    }

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

// 保存JPEGD解码后的RGB格式像素内容
void jpegd_save_rgb_file(const std::string &fileName, hi_video_frame* pVB)
{
    uint8_t *tmpPtr          = nullptr;
    uint32_t u32WidthInBytes = 0;
    uint32_t u32Stride       = 0;

    if ((pVB->pixel_format == HI_PIXEL_FORMAT_ARGB_8888) ||
        (pVB->pixel_format == HI_PIXEL_FORMAT_ABGR_8888)) {
        u32WidthInBytes = pVB->width * 4;
    } else if ((pVB->pixel_format == HI_PIXEL_FORMAT_RGB_888) ||
               (pVB->pixel_format == HI_PIXEL_FORMAT_BGR_888)) {
        u32WidthInBytes = pVB->width * 3;
    } else {
        SAMPLE_PRT("This RGB format:%u is not support during save %s!\n", pVB->pixel_format, fileName.c_str());
        return;
    }

    u32Stride = ALIGN_UP(u32WidthInBytes, JPEGD_ALIGN_H);

    SAMPLE_PRT("saving %s......RGB..%dx%d......\n", fileName.c_str(), pVB->width, pVB->height);

    FILE *fp = fopen(fileName.c_str(), "wb");
    if (fp == nullptr) {
        SAMPLE_PRT("can't open file %s in get picture thread!\n", fileName.c_str());
        return;
    }

    tmpPtr = (uint8_t *)pVB->virt_addr[0];
    for (uint32_t i = 0; i < pVB->height; i++, tmpPtr += u32Stride) {
        fwrite(tmpPtr, u32WidthInBytes, 1, fp);
    }

    fsync(fileno(fp));
    fclose(fp);
    SAMPLE_PRT(" save %s done!\n", fileName.c_str());

    return;
}

// 保存JPEGD解码后的像素内容
void jpegd_save_file(int32_t chn, int32_t picCnt, hi_video_frame stVFrame)
{
    char typeName[16] = {0};

    int32_t s32Ret = save_file_type(stVFrame.pixel_format, stVFrame.compress_mode,
                                  typeName, sizeof(typeName));
    if (s32Ret <= 0) {
        SAMPLE_PRT("Get file type failed!\n");
        return;
    }

    std::ostringstream oFileName;
    oFileName << "chn" << chn << "_" << picCnt << "_" << g_output_file_name << "." << typeName;
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

    if ((HI_PIXEL_FORMAT_RGB_888 <= stVFrame.pixel_format) &&
        (stVFrame.pixel_format <= HI_PIXEL_FORMAT_BGRA_8888)) {
        jpegd_save_rgb_file(strFileName, &stVFrame);
    } else if ((stVFrame.compress_mode == HI_COMPRESS_MODE_NONE) &&
               (stVFrame.video_format == HI_VIDEO_FORMAT_LINEAR)) {
        SAMPLE_PRT("[chn:%d] ComPressMode[%d] VideoFormat %d\n", chn, stVFrame.compress_mode,
                   stVFrame.video_format);
        jpegd_save_yuv_file_linear_8bit_semiplanar(strFileName, &stVFrame);
    }
}

hi_pixel_format jpegd_convert_image_format_to_pixel(hi_jpeg_raw_format rawFormat, hi_bool bVBeforeU)
{
    if (rawFormat == HI_JPEG_RAW_FORMAT_YUV444) {
        if (bVBeforeU) {
            return HI_PIXEL_FORMAT_YVU_SEMIPLANAR_444;
        } else {
            return HI_PIXEL_FORMAT_YUV_SEMIPLANAR_444;
        }
    } else if (rawFormat == HI_JPEG_RAW_FORMAT_YUV422) {
        if (bVBeforeU) {
            return HI_PIXEL_FORMAT_YVU_SEMIPLANAR_422;
        } else {
            return HI_PIXEL_FORMAT_YUV_SEMIPLANAR_422;
        }
    } else if (rawFormat == HI_JPEG_RAW_FORMAT_YUV420) {
        if (bVBeforeU) {
            return HI_PIXEL_FORMAT_YVU_SEMIPLANAR_420;
        } else {
            return HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420;
        }
    } else if (rawFormat == HI_JPEG_RAW_FORMAT_YUV440) {
        if (bVBeforeU) {
            return HI_PIXEL_FORMAT_YVU_SEMIPLANAR_440;
        } else {
            return HI_PIXEL_FORMAT_YUV_SEMIPLANAR_440;
        }
    } else if (rawFormat == HI_JPEG_RAW_FORMAT_YUV400) {
        return HI_PIXEL_FORMAT_YUV_400;
    }

    SAMPLE_PRT("Unsupport hi_jpeg_raw_format:(%u)!\n", rawFormat);
    return HI_PIXEL_FORMAT_BUTT;
}

// JPEGD兼容性测试线程函数:读取文件夹内所有.jpg文件并送入VDEC
void *jpegd_send_stream_compatible(void *pArgs)
{
    FILE    *fpStrm        = nullptr;
    int32_t s32Ret         = 0;
    int32_t fileSize       = 0;
    int32_t s32ReadLen     = 0;
    uint32_t idx           = 0;
    uint32_t bufAllocCount = 0;
    uint32_t errCnt        = 0;
    void *outBuffer        = nullptr;
    hi_vdec_stream stStream{};
    hi_vdec_pic_info outPicInfo{};
    hi_img_info stImgInfo{};
    hi_vdec_chn_status stStatus{};
    VDEC_THREAD_PARAM_S *pstJpegdThreadParam = (VDEC_THREAD_PARAM_S *)pArgs;
    DIR *currentDir = nullptr;
    struct dirent *dirp;

    if (g_run_mode == ACL_HOST) {
        if (aclrtSetCurrentContext(g_context)) {
            SAMPLE_PRT("set context error\n");
            return (void *)(HI_FAILURE);
        }
    }

    prctl(PR_SET_NAME, "JpegSendStreamComp", 0, 0, 0);

    if ((currentDir = opendir(g_compatible_dir_name)) == nullptr) {
        SAMPLE_PRT("[chn:%d] opendir error\n", pstJpegdThreadParam->s32ChnId);
        return (void *)(HI_FAILURE);
    }

    jpegd_get_time_stamp_us(&g_start_send_time[pstJpegdThreadParam->s32ChnId]);

    while (1) {
        if (pstJpegdThreadParam->enSendThreadCtrl == THREAD_CTRL_STOP) {
            break;
        } else if (pstJpegdThreadParam->enSendThreadCtrl == THREAD_CTRL_PAUSE) {
            sleep(1);
            continue;
        }

        while ((dirp = readdir(currentDir)) != nullptr) {
            if (pstJpegdThreadParam->enSendThreadCtrl == THREAD_CTRL_STOP) {
                break;
            } else if (pstJpegdThreadParam->enSendThreadCtrl == THREAD_CTRL_PAUSE) {
                sleep(1);
                continue;
            }

            if ((strcmp(dirp->d_name, ".") == 0) ||
                (strcmp(dirp->d_name, "..") == 0) ||
                (dirp->d_type == DT_DIR)) { continue;}

            for (idx = 0; idx < strlen(dirp->d_name); idx++) {
                if (dirp->d_name[idx] == '.') { break;}
            }
            if (idx == (strlen(dirp->d_name) - 4)) {
                if ((dirp->d_name[idx + 1] != 'j') ||
                    (dirp->d_name[idx + 2] != 'p') ||
                    (dirp->d_name[idx + 3] != 'g')) {
                        SAMPLE_PRT("[chn:%d] %s is not JPEG!\n", pstJpegdThreadParam->s32ChnId, dirp->d_name);
                        continue;
                    }
            } else if (idx == (strlen(dirp->d_name) - 5)) {
                if ((dirp->d_name[idx + 1] != 'J') ||
                    (dirp->d_name[idx + 2] != 'P') ||
                    (dirp->d_name[idx + 3] != 'E') ||
                    (dirp->d_name[idx + 4] != 'G')) {
                        SAMPLE_PRT("[chn:%d] %s is not JPEG!\n", pstJpegdThreadParam->s32ChnId, dirp->d_name);
                        continue;
                    }
            } else {
                SAMPLE_PRT("[chn:%d] %s is not JPEG!\n", pstJpegdThreadParam->s32ChnId, dirp->d_name);
                continue;
            }

            std::ostringstream jpegdFileFullName;
            jpegdFileFullName << g_compatible_dir_name << dirp->d_name;

            fpStrm = fopen(jpegdFileFullName.str().c_str(), "rb");
            if (fpStrm == nullptr) {
                SAMPLE_PRT("[chn:%d] can't open file %s in send stream thread!\n",
                           pstJpegdThreadParam->s32ChnId, jpegdFileFullName.str().c_str());
                return (void *)(HI_FAILURE);
            }

            fseek(fpStrm, 0L, SEEK_END);
            fileSize = ftell(fpStrm);
            fflush(stdout);
            fseek(fpStrm, 0L, SEEK_SET);

            std::vector<uint8_t> fileData;
            void *inBuffer = nullptr;
            if (g_run_mode == ACL_HOST) {
                fileData.resize(fileSize);
                s32ReadLen = fread(&fileData[0], 1, fileSize, fpStrm);
                if (s32ReadLen == 0) {
                    fclose(fpStrm);
                    continue;
                }

                s32Ret = hi_mpi_dvpp_malloc(0, &inBuffer, s32ReadLen);
                if (s32Ret) {
                    SAMPLE_PRT("hi_mpi_dvpp_malloc failed!\n");
                    continue;
                }

                auto aclRet = aclrtMemcpy(inBuffer, s32ReadLen, &fileData[0], s32ReadLen, ACL_MEMCPY_HOST_TO_DEVICE);
                if (aclRet != ACL_SUCCESS) {
                    hi_mpi_dvpp_free(inBuffer);
                    SAMPLE_PRT("Copy host memcpy to device fail with %d.\n", aclRet);
                    continue;
                }
            } else {
                s32Ret = hi_mpi_dvpp_malloc(0, (void**)&inBuffer, fileSize);
                if ((s32Ret != 0) || (inBuffer == NULL)) {
                    SAMPLE_PRT("[chn:%d] can't alloc %d in send thread!\n", pstJpegdThreadParam->s32ChnId, fileSize);
                    fclose(fpStrm);
                    continue;
                }

                SAMPLE_PRT("[chn:%d] [%s] stream addr:0x%lx size: %d\n", pstJpegdThreadParam->s32ChnId,
                           jpegdFileFullName.str().c_str(), (uint64_t)(uintptr_t)inBuffer, fileSize);
                s32ReadLen = fread(inBuffer, 1, fileSize, fpStrm);
                if (s32ReadLen == 0) {
                    hi_mpi_dvpp_free((void*)inBuffer);
                    fclose(fpStrm);
                    continue;
                }
            }
            fclose(fpStrm);

            stStream.pts       = pstJpegdThreadParam->u64PtsInit;
            if (g_run_mode == ACL_HOST) {
                stStream.addr  = (uint8_t *)&fileData[0];
            } else {
                stStream.addr  = (uint8_t *)inBuffer;
            }
            stStream.len       = s32ReadLen;
            stStream.end_of_frame  = HI_TRUE;
            stStream.end_of_stream = HI_FALSE;
            stStream.need_display  = HI_TRUE;

            s32Ret = hi_mpi_dvpp_get_image_info(HI_PT_JPEG, &stStream, &stImgInfo);
            if (s32Ret != HI_SUCCESS) {
                SAMPLE_PRT("[chn:%d] hi_mpi_dvpp_get_image_info failed! ret:%#x\n",
                           pstJpegdThreadParam->s32ChnId, s32Ret);
                hi_mpi_dvpp_free(inBuffer);
                continue;
            }

            stStream.addr     = (uint8_t *)inBuffer;
            outPicInfo.width  = stImgInfo.width;
            outPicInfo.height = stImgInfo.height;
            outPicInfo.width_stride  = stImgInfo.width_stride;
            outPicInfo.height_stride = stImgInfo.height_stride;
            outPicInfo.buffer_size   = stImgInfo.img_buf_size;
            outPicInfo.pixel_format  = HI_PIXEL_FORMAT_UNKNOWN;

            s32Ret = hi_mpi_dvpp_malloc(0, &outBuffer, outPicInfo.buffer_size);
            if (s32Ret != 0) {
                SAMPLE_PRT("hi_mpi_dvpp_malloc out buf Failed\n");
                hi_mpi_dvpp_free(inBuffer);
                continue;
            }
            bufAllocCount++;
            outPicInfo.vir_addr = (uint64_t)outBuffer;

            SAMPLE_PRT("[chn:%d] [%s] w:%u h:%u pixel:%u. idx:%u buffer:0x%lx size:%u\n",
                       pstJpegdThreadParam->s32ChnId, jpegdFileFullName.str().c_str(),
                       outPicInfo.width, outPicInfo.height,
                       outPicInfo.pixel_format, bufAllocCount,
                       (uint64_t)(uintptr_t)outBuffer, outPicInfo.buffer_size);

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
                s32Ret = hi_mpi_dvpp_free(inBuffer);
                if (s32Ret != 0) {
                    SAMPLE_PRT("[chn:%d] Free stream failed\n", pstJpegdThreadParam->s32ChnId);
                }
                s32Ret = hi_mpi_dvpp_free((void *)outBuffer);
                if (s32Ret != 0) {
                    SAMPLE_PRT("[chn:%d] Free outBuffer failed\n", pstJpegdThreadParam->s32ChnId);
                }
                break; // break while(1)
            }

            do {
                s32Ret = hi_mpi_vdec_send_stream(pstJpegdThreadParam->s32ChnId,
                                                 &stStream,
                                                 &outPicInfo,
                                                 pstJpegdThreadParam->s32MilliSec);
                if (s32Ret != HI_SUCCESS) {
                    errCnt++;
                    if (errCnt > 100) {
                        break; // break do...while
                    }
                }

                usleep(pstJpegdThreadParam->s32IntervalTime);
            } while ((s32Ret != HI_SUCCESS) && (pstJpegdThreadParam->enSendThreadCtrl == THREAD_CTRL_START));
            if ((s32Ret != HI_SUCCESS) || (pstJpegdThreadParam->enSendThreadCtrl == THREAD_CTRL_STOP)) {
                if (errCnt != 0) {
                    SAMPLE_PRT("[chn:%d] hi_mpi_vdec_send_stream Send [%s] failed for %u times! addr:%#lx, len:%u\n",
                        pstJpegdThreadParam->s32ChnId, jpegdFileFullName.str().c_str(), errCnt,
                        (uint64_t)(uintptr_t)stStream.addr, stStream.len);
                }

                s32Ret = hi_mpi_dvpp_free(inBuffer);
                if (s32Ret != 0) {
                    SAMPLE_PRT("hi_mpi_dvpp_free inBuffer failed\n");
                }
                s32Ret = hi_mpi_dvpp_free((void*)outBuffer);
                if (s32Ret != 0) {
                    SAMPLE_PRT("hi_mpi_dvpp_free outBuffer failed\n");
                }

                if (pstJpegdThreadParam->enSendThreadCtrl == THREAD_CTRL_STOP) {
                    break; // break while ((dirp = readdir(currentDir)) != nullptr)
                }
            }

            errCnt = 0;
            SAMPLE_PRT("[chn:%d] hi_mpi_vdec_send_stream Send [%u] Again ok! addr:%#lx, len:%u\n",
                pstJpegdThreadParam->s32ChnId, bufAllocCount,
                (uint64_t)(uintptr_t)stStream.addr, stStream.len);

            usleep(pstJpegdThreadParam->s32IntervalTime);
        } // while ((dirp = readdir(currentDir)) != nullptr))

        SAMPLE_PRT("[chn:%d] s32CircleSend:%d\n", pstJpegdThreadParam->s32ChnId, pstJpegdThreadParam->s32CircleSend);
        seekdir(currentDir, 0);

        if (pstJpegdThreadParam->enSendThreadCtrl == THREAD_CTRL_STOP) {
            break; // break while (1)
        }

        if ((pstJpegdThreadParam->s32CircleSend == 0) ||
            (pstJpegdThreadParam->s32CircleSend == 1)) {
            break;
        } else if (pstJpegdThreadParam->s32CircleSend > 0) {
            pstJpegdThreadParam->s32CircleSend--;
        }
    } // while (1)
    closedir(currentDir);

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

    SAMPLE_PRT("[chn:%d] send steam thread return\n", pstJpegdThreadParam->s32ChnId);

    return (void *)HI_SUCCESS;
}

// JPEGD性能测试线程:一张图片反复解码并计算fps
void *jpegd_send_stream_performance(void *pArgs)
{
    FILE *fpStrm           = nullptr;
    int32_t s32Ret         = 0;
    int32_t fileSize       = 0;
    int32_t s32ReadLen     = 0;
    uint32_t idxChn        = 0;
    uint32_t outBufferSize = 0;
    uint32_t mallocCount   = 0;
    uint32_t errCnt        = 0;
    void *outBuffer        = nullptr;
    hi_vdec_stream stStream;
    hi_vdec_pic_info outPicInfo;
    VDEC_THREAD_PARAM_S *pstJpegdThreadParam = (VDEC_THREAD_PARAM_S *)pArgs;

    if (g_run_mode == ACL_HOST) {
        if (aclrtSetCurrentContext(g_context)) {
            SAMPLE_PRT("set context error\n");
            return (void *)(HI_FAILURE);
        }
    }

    std::ostringstream pthreadName;
    pthreadName << "JpegSendPerf_" << pstJpegdThreadParam->s32ChnId;
    prctl(PR_SET_NAME, pthreadName.str().c_str(), 0, 0, 0);

    fpStrm = fopen(g_input_file_name, "rb");
    if (fpStrm == nullptr) {
        SAMPLE_PRT("[chn:%d] can't open file %s!\n",
                   pstJpegdThreadParam->s32ChnId, g_input_file_name);
        return (void *)(HI_FAILURE);
    }

    if (pstJpegdThreadParam->enPixFormat == HI_PIXEL_FORMAT_UNKNOWN) {
        outBufferSize = ALIGN_UP(g_width, JPEGD_ALIGN_W) * ALIGN_UP(g_height, JPEGD_ALIGN_H) * 3;
    } else {
        hi_pic_buf_attr buf_attr{g_width, g_height, 0,
                                 HI_DATA_BIT_WIDTH_8,
                                 pstJpegdThreadParam->enPixFormat, HI_COMPRESS_MODE_NONE};
        outBufferSize = hi_vdec_get_pic_buf_size(HI_PT_JPEG, &buf_attr);
    }

    fseek(fpStrm, 0L, SEEK_END);
    fileSize = ftell(fpStrm);

    s32Ret = hi_mpi_dvpp_malloc(0, (void**)&pstJpegdThreadParam->perfStreamBuf, fileSize);
    if ((s32Ret != 0) || (pstJpegdThreadParam->perfStreamBuf == nullptr)) {
        SAMPLE_PRT("[chn:%d] can't alloc %d in send stream thread!\n",
                   pstJpegdThreadParam->s32ChnId, fileSize);
        fclose(fpStrm);
        return (void *)(HI_FAILURE);
    }
    fflush(stdout);

    fseek(fpStrm, 0L, SEEK_SET);
    std::vector<uint8_t> fileData;

    if (g_run_mode == ACL_HOST) {
        fileData.resize(fileSize);
        s32ReadLen = fread(&fileData[0], 1, fileSize, fpStrm);
        auto aclRet = aclrtMemcpy(pstJpegdThreadParam->perfStreamBuf, s32ReadLen,
                                  &fileData[0], s32ReadLen,
                                  ACL_MEMCPY_HOST_TO_DEVICE);
        if (aclRet != ACL_SUCCESS) {
            hi_mpi_dvpp_free(pstJpegdThreadParam->perfStreamBuf);
            fflush(stdout);
            fclose(fpStrm);
            SAMPLE_PRT("Copy host memcpy to device fail with %d.\n", aclRet);
            return (void *)(HI_FAILURE);
        }
    } else {
        s32ReadLen = fread(pstJpegdThreadParam->perfStreamBuf, 1, fileSize, fpStrm);
    }
    fflush(stdout);
    fclose(fpStrm);

    for (idxChn = 0; idxChn < JPEGD_PERFOR_MODE_BUF_NUM;) {
        s32Ret = hi_mpi_dvpp_malloc(0, &outBuffer, outBufferSize);
        if ((s32Ret != 0) || (outBuffer == nullptr)) {
            SAMPLE_PRT("[chn:%d] DvppMalloc From Pool Failed!\n", pstJpegdThreadParam->s32ChnId);
        } else {
            idxChn++;
            g_out_buffer_pool[pstJpegdThreadParam->s32ChnId].push_back(outBuffer);
        }
    }

    // 等待g_delay_time秒之后发送码流,保证所有通道同一时间开始工作
    delay_exec(g_start_time, g_delay_time);

    // 开始发码流
    jpegd_get_time_stamp_us(&g_start_send_time[pstJpegdThreadParam->s32ChnId]);
    while (1) {
        if (pstJpegdThreadParam->enSendThreadCtrl == THREAD_CTRL_STOP) {
            break;
        } else if (pstJpegdThreadParam->enSendThreadCtrl == THREAD_CTRL_PAUSE) {
            sleep(1);
            continue;
        }

        stStream.pts       = pstJpegdThreadParam->u64PtsInit;
        stStream.addr      = pstJpegdThreadParam->perfStreamBuf;
        stStream.len       = s32ReadLen;
        stStream.end_of_frame  = HI_TRUE;
        stStream.end_of_stream = HI_FALSE;
        stStream.need_display  = HI_TRUE;

        for (mallocCount = 0; mallocCount < JPEGD_PERFOR_MODE_QUERY_CNT; mallocCount++) {
            (void)pthread_mutex_lock(&g_out_buffer_pool_lock[pstJpegdThreadParam->s32ChnId]);
            if (g_out_buffer_pool[pstJpegdThreadParam->s32ChnId].empty() == false) {
                outBuffer = g_out_buffer_pool[pstJpegdThreadParam->s32ChnId].back();
                g_out_buffer_pool[pstJpegdThreadParam->s32ChnId].pop_back();
                (void)pthread_mutex_unlock(&g_out_buffer_pool_lock[pstJpegdThreadParam->s32ChnId]);
                break;
            } else {
                (void)pthread_mutex_unlock(&g_out_buffer_pool_lock[pstJpegdThreadParam->s32ChnId]);
                usleep(1000);
            }

            if (pstJpegdThreadParam->enSendThreadCtrl == THREAD_CTRL_STOP) {
                break; // break for(;;)
            }
        }

        if (mallocCount >= JPEGD_PERFOR_MODE_QUERY_CNT) {
            SAMPLE_PRT("[chn:%d] DvppMalloc From Pool Failed.\n", pstJpegdThreadParam->s32ChnId);
            break;
        }

        outPicInfo.vir_addr     = (uint64_t)outBuffer;
        outPicInfo.buffer_size  = outBufferSize;
        outPicInfo.width        = g_width;
        outPicInfo.height       = g_height;
        outPicInfo.pixel_format = pstJpegdThreadParam->enPixFormat;

        do {
            s32Ret = hi_mpi_vdec_send_stream(pstJpegdThreadParam->s32ChnId,
                                             &stStream, &outPicInfo,
                                             pstJpegdThreadParam->s32MilliSec);
            if (s32Ret != HI_SUCCESS) {
                errCnt++;
                if (errCnt > 100) {
                    SAMPLE_PRT("[chn:%d] SendStream failed %u times! addr:%#lx, len:%u, outBuffer:0x%lx, size:%u\n",
                               pstJpegdThreadParam->s32ChnId, errCnt,
                               (uint64_t)(uintptr_t)stStream.addr, stStream.len,
                               (uint64_t)(uintptr_t)outBuffer, outBufferSize);
                    break; // break do...while
                }
            }

            usleep(pstJpegdThreadParam->s32IntervalTime);
        } while ((s32Ret != HI_SUCCESS) &&
                 (pstJpegdThreadParam->enSendThreadCtrl == THREAD_CTRL_START));
        if ((s32Ret != HI_SUCCESS) || (pstJpegdThreadParam->enSendThreadCtrl == THREAD_CTRL_STOP)) {
            break; // break while(1)
        }

        errCnt = 0;

        if ((pstJpegdThreadParam->s32CircleSend == 0) ||
            (pstJpegdThreadParam->s32CircleSend == 1)) {
            break;
        } else if (pstJpegdThreadParam->s32CircleSend > 0) {
            pstJpegdThreadParam->s32CircleSend--;
        }
    } // while (1)

    // 发送结束码
    stStream.end_of_stream = HI_TRUE;
    stStream.addr = NULL;
    stStream.len = 0;
    stStream.end_of_frame = HI_FALSE;
    stStream.end_of_stream    = HI_TRUE;
    outPicInfo.vir_addr    = 0;
    outPicInfo.buffer_size = 0;
    hi_mpi_vdec_send_stream(pstJpegdThreadParam->s32ChnId, &stStream, &outPicInfo, -1);

    return (void *)HI_SUCCESS;
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

        if ((pstJpegdThreadParam->s32CircleSend == 0) ||
            (pstJpegdThreadParam->s32CircleSend == 1)) {
            break;
        } else if (pstJpegdThreadParam->s32CircleSend > 0) {
            pstJpegdThreadParam->s32CircleSend--;
            usleep(pstJpegdThreadParam->s32IntervalTime);
        } else {
            usleep(pstJpegdThreadParam->s32IntervalTime);
        }
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
    uint64_t nextexcTime;
    uint64_t tmpCurtime;

    uint64_t curTime = 0;
    uint64_t lastTime = 0;
    struct timeval currentTime;
    hi_vdec_chn_status stStatus;

    if (g_wait_time > 0) {
        nextexcTime = g_start_time + static_cast<uint64_t>(g_wait_time * MULTIPLE_S_TO_US);
    }

    for (uint32_t idxChn = g_start_channel; idxChn < g_chn_num + g_start_channel; idxChn++) {
        if (pJpegdSendTid[idxChn] != 0) {
            s32Ret = pthread_join(pJpegdSendTid[idxChn], NULL);
            pJpegdSendTid[idxChn] = 0;
        }
        g_jpegd_thread_param[idxChn].enSendThreadCtrl = THREAD_CTRL_STOP;

        while (1) {
            s32Ret = hi_mpi_vdec_query_status(g_jpegd_thread_param[idxChn].s32ChnId, &stStatus);
            if (s32Ret != HI_SUCCESS) {
                SAMPLE_PRT("[chn:%u] hi_mpi_vdec_query_status fail!!!\n", idxChn);
                g_jpegd_thread_param[idxChn].enGetThreadCtrl = THREAD_CTRL_STOP;
                break;
            }

            // jump out if no decoding data
            if ((stStatus.left_stream_bytes == 0) &&
                (stStatus.left_decoded_frames == 0)) {
                if (g_jpegd_thread_param[idxChn].enGetThreadCtrl == THREAD_CTRL_START) {
                    g_jpegd_thread_param[idxChn].enGetThreadCtrl = THREAD_CTRL_STOP;
                    break;
                }
            }

            // jump out when user input CTRL+C or CTRL+Z
            if (g_jpegd_thread_param[idxChn].enGetThreadCtrl == THREAD_CTRL_STOP) {
                break;
            }

            // jump out when timeout
            if (g_wait_time > 0) {
                gettimeofday(&currentTime, nullptr);
                tmpCurtime = currentTime.tv_sec * MULTIPLE_S_TO_US + currentTime.tv_usec;
                if (tmpCurtime > nextexcTime) {
                    SAMPLE_PRT("[chn:%u] Running Time out. Try to exit!\n", idxChn);
                    g_jpegd_thread_param[idxChn].enGetThreadCtrl = THREAD_CTRL_STOP;
                    break;
                }
            }

            if (g_performance <= 0) {
                jpegd_get_time_stamp_us(&curTime);
                if (curTime - lastTime > 5000000) {
                    SAMPLE_PRT("[chn:%u] Waiting status LeftStreamBytes:%u. LeftPics:%u\n",
                               idxChn, stStatus.left_stream_bytes, stStatus.left_decoded_frames);
                    lastTime = curTime;
                }
            }

            usleep(1000);
        }
    }

    for (uint32_t idxChn = g_start_channel; idxChn < g_chn_num + g_start_channel; idxChn++) {
        if (pJpegdGetTid[idxChn] != 0) {
            pthread_join(pJpegdGetTid[idxChn], nullptr);
            pJpegdGetTid[idxChn] = 0;
        }
    }

    return;
}

int32_t jpegd_start_send_stream(pthread_t *pJpegdSendTid)
{
    int32_t ret = -1;

    for (uint32_t idxChn = g_start_channel; idxChn < g_chn_num + g_start_channel; idxChn++) {
        pJpegdSendTid[idxChn] = 0;
        g_jpegd_thread_param[idxChn].enSendThreadCtrl = THREAD_CTRL_START;
        if (g_jpegd_thread_param[idxChn].bWholeDirDecode == HI_TRUE) {
            ret = pthread_create(&pJpegdSendTid[idxChn], 0,
                                 jpegd_send_stream_compatible,
                                 (void *)&g_jpegd_thread_param[idxChn]);
            if (ret != 0) {
                SAMPLE_PRT("[chn:%u] create send pic thread Fail, ret = %d \n", idxChn, ret);
                pJpegdSendTid[idxChn] = 0;
                g_jpegd_thread_param[idxChn].enSendThreadCtrl = THREAD_CTRL_STOP;
                return ret;
            }
        } else if (g_performance > 0) {
            ret = pthread_create(&pJpegdSendTid[idxChn], 0,
                                 jpegd_send_stream_performance,
                                 (void *)&g_jpegd_thread_param[idxChn]);
            if (ret != 0) {
                SAMPLE_PRT("[chn:%u] create send pic thread Fail, ret = %d \n", idxChn, ret);
                pJpegdSendTid[idxChn] = 0;
                g_jpegd_thread_param[idxChn].enSendThreadCtrl = THREAD_CTRL_STOP;
                return ret;
            }
        } else {
            ret = pthread_create(&pJpegdSendTid[idxChn], 0,
                                 jpegd_send_stream,
                                 (void *)&g_jpegd_thread_param[idxChn]);
            if (ret != 0) {
                SAMPLE_PRT("[chn:%u] create send pic thread Fail, ret = %d \n", idxChn, ret);
                pJpegdSendTid[idxChn] = 0;
                g_jpegd_thread_param[idxChn].enSendThreadCtrl = THREAD_CTRL_STOP;
                return ret;
            }
        }
    }

    return 0;
}

void jpegd_stop_send_stream()
{
    SAMPLE_PRT(" JPEGD stop send stream threads\n");
    for (uint32_t idxChn = g_start_channel; idxChn < g_chn_num + g_start_channel; idxChn++) {
        g_jpegd_thread_param[idxChn].enSendThreadCtrl = THREAD_CTRL_STOP;
    }
}

void jpegd_show_decode_state()
{
    for (uint32_t idxChn = g_start_channel; idxChn < g_chn_num + g_start_channel; ++idxChn) {
        uint64_t u64DiffTime = g_end_get_time[idxChn] - g_start_send_time[idxChn];
        if (u64DiffTime == 0) {
            continue;
        }
        SAMPLE_PRT("\033[0;33m --------------------------------------------------------------------------\033[0;39m\n");
        double actualFrameRate = ((double)g_get_frame_cnt[idxChn] * MULTIPLE_S_TO_US) / u64DiffTime;
        SAMPLE_PRT("\033[0;33m chnId %u, actualFrameRate %.1f, SendFrameCnt %lu, DiffTime %lu \033[0;39m\n",
                   idxChn, actualFrameRate, g_get_frame_cnt[idxChn], u64DiffTime);
        SAMPLE_PRT("\033[0;33m --------------------------------------------------------------------------\033[0;39m\n");
    }

    return;
}

// JPEGD兼容性测试或正常解码接收线程
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

// JPEGD性能测试接收线程
void *jpegd_get_pic_performance(void *pArgs)
{
    int32_t s32Ret;
    int32_t s32Cnt = 0;
    int32_t s32DecFailCnt = 0;
    hi_vdec_chn_attr stAttr{};
    hi_vdec_stream pstStream{};
    hi_video_frame_info stVFrame{};
    hi_vdec_supplement_info stSupplement{};
    VDEC_THREAD_PARAM_S *pstJpegdThreadParam = (VDEC_THREAD_PARAM_S *)pArgs;

    if (g_run_mode == ACL_HOST) {
        if (aclrtSetCurrentContext(g_context)) {
            SAMPLE_PRT("set context error\n");
            return (void *)(HI_FAILURE);
        }
    }

    std::ostringstream pthreadName;
    pthreadName << "JpegGetPerf_" << pstJpegdThreadParam->s32ChnId;
    prctl(PR_SET_NAME, pthreadName.str().c_str(), 0, 0, 0);

    s32Ret = hi_mpi_vdec_get_chn_attr(pstJpegdThreadParam->s32ChnId, &stAttr);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("[chn:%d] get chn attr fail for %#x!\n", pstJpegdThreadParam->s32ChnId, s32Ret);
        return (void *)(HI_FAILURE);
    }

    while (1) {
        if (pstJpegdThreadParam->enGetThreadCtrl == THREAD_CTRL_STOP) {
            break;
        }

        s32Ret = hi_mpi_vdec_get_frame(pstJpegdThreadParam->s32ChnId,
                                       &stVFrame, &stSupplement, &pstStream, pstJpegdThreadParam->s32MilliSec);
        if (s32Ret == HI_SUCCESS) {
            jpegd_get_time_stamp_us(&g_end_get_time[pstJpegdThreadParam->s32ChnId]);

            // 统计解码结果
            if (stVFrame.v_frame.frame_flag == 0) {
                s32Cnt++;
            } else {
                SAMPLE_PRT("[chn:%d] GetFrame Fail, Decoder Fail\n", pstJpegdThreadParam->s32ChnId);
                s32DecFailCnt++;
            }

            g_get_frame_cnt[pstJpegdThreadParam->s32ChnId] = s32Cnt + s32DecFailCnt;

            // 输出像素内存放回队列,性能测试不反复申请内存
            (void)pthread_mutex_lock(&g_out_buffer_pool_lock[pstJpegdThreadParam->s32ChnId]);
            g_out_buffer_pool[pstJpegdThreadParam->s32ChnId].push_back((void *)stVFrame.v_frame.virt_addr[0]);
            (void)pthread_mutex_unlock(&g_out_buffer_pool_lock[pstJpegdThreadParam->s32ChnId]);

            s32Ret = hi_mpi_vdec_release_frame(pstJpegdThreadParam->s32ChnId, &stVFrame);
            if (s32Ret != HI_SUCCESS) {
                SAMPLE_PRT("[chn:%d] hi_mpi_vdec_release_frame fail for s32Ret=0x%x!\n",
                           pstJpegdThreadParam->s32ChnId, s32Ret);
            }
        } else {
            usleep(300);
        }
    }

    return (void *)HI_SUCCESS;
}

// 启动接收线程,开始接收图片
int32_t jpegd_start_get_pic(pthread_t *pJpegdGetTid)
{
    int32_t ret = -1;

    for (uint32_t idxChn = g_start_channel; idxChn < g_chn_num + g_start_channel; idxChn++) {
        pJpegdGetTid[idxChn] = 0;
        g_jpegd_thread_param[idxChn].enGetThreadCtrl = THREAD_CTRL_START;
        if (g_performance > 0) {
            ret = pthread_create(&pJpegdGetTid[idxChn], 0,
                                 jpegd_get_pic_performance, (void *)&g_jpegd_thread_param[idxChn]);
            if (ret != 0) {
                SAMPLE_PRT("[chn:%u] create get pic thread Fail, ret = %d \n", idxChn, ret);
                pJpegdGetTid[idxChn] = 0;
                g_jpegd_thread_param[idxChn].enGetThreadCtrl = THREAD_CTRL_STOP;
                return ret;
            }
        } else {
            ret = pthread_create(&pJpegdGetTid[idxChn], 0,
                                 jpegd_get_pic, (void *)&g_jpegd_thread_param[idxChn]);
            if (ret != 0) {
                SAMPLE_PRT(" chn %u, create get pic thread Fail, ret = %d \n", idxChn, ret);
                pJpegdGetTid[idxChn] = 0;
                g_jpegd_thread_param[idxChn].enGetThreadCtrl = THREAD_CTRL_STOP;
                return ret;
            }
        }
    }

    return 0;
}

void jpegd_stop_get_pic()
{
    SAMPLE_PRT(" JPEGD stop get pic threads\n");
    for (uint32_t idxChn = g_start_channel; idxChn < g_chn_num + g_start_channel; idxChn++) {
        g_jpegd_thread_param[idxChn].enGetThreadCtrl = THREAD_CTRL_STOP;
    }
}

int32_t jpegd_create()
{
    uint32_t idxChn = 0;
    int32_t  s32Ret = HI_SUCCESS;
    SAMPLE_VDEC_ATTR astSampleVdec[VDEC_MAX_CHN_NUM];
    hi_vdec_chn_attr stChnAttr[VDEC_MAX_CHN_NUM];
    hi_vdec_chn_param stChnParam;

    for (idxChn = g_start_channel; idxChn < g_chn_num + g_start_channel; idxChn++) {
        astSampleVdec[idxChn].type                             = HI_PT_JPEG;
        astSampleVdec[idxChn].width                            = ALIGN_UP(g_width, JPEGD_ALIGN_W);
        astSampleVdec[idxChn].height                           = ALIGN_UP(g_height, JPEGD_ALIGN_H);
        astSampleVdec[idxChn].mode                             = g_video_mode;
        astSampleVdec[idxChn].picture_attr.pixel_format = g_pixel_format;
        astSampleVdec[idxChn].picture_attr.alpha        = 255;
        astSampleVdec[idxChn].display_frame_num                = 0;
        astSampleVdec[idxChn].frame_buf_cnt = astSampleVdec[idxChn].display_frame_num + 1;

        pthread_mutex_init(&g_out_buffer_pool_lock[idxChn], NULL);
    }

    for (idxChn = g_start_channel; idxChn < g_chn_num + g_start_channel; idxChn++) {
        stChnAttr[idxChn].type = astSampleVdec[idxChn].type;
        stChnAttr[idxChn].mode = astSampleVdec[idxChn].mode;
        stChnAttr[idxChn].pic_width  = astSampleVdec[idxChn].width;
        stChnAttr[idxChn].pic_height = astSampleVdec[idxChn].height;
        stChnAttr[idxChn].stream_buf_size = astSampleVdec[idxChn].width *
                                            astSampleVdec[idxChn].height;

        stChnAttr[idxChn].frame_buf_cnt  = 0;
        stChnAttr[idxChn].frame_buf_size = 0;

        CHECK_CHN_RET(hi_mpi_vdec_create_chn(idxChn, &stChnAttr[idxChn]), idxChn, "hi_mpi_vdec_create_chn");

        CHECK_CHN_RET(hi_mpi_vdec_get_chn_param(idxChn, &stChnParam), idxChn, "hi_mpi_vdec_get_chn_param");
        stChnParam.pic_param.pixel_format = astSampleVdec[idxChn].picture_attr.pixel_format;
        stChnParam.pic_param.alpha      = astSampleVdec[idxChn].picture_attr.alpha;
        stChnParam.display_frame_num = astSampleVdec[idxChn].display_frame_num;
        CHECK_CHN_RET(hi_mpi_vdec_set_chn_param(idxChn, &stChnParam), idxChn, "hi_mpi_vdec_set_chn_param");

        CHECK_CHN_RET(hi_mpi_vdec_start_recv_stream(idxChn), idxChn, "hi_mpi_vdec_start_recv_stream");

        g_jpegd_thread_param[idxChn].type = astSampleVdec[idxChn].type;
        g_jpegd_thread_param[idxChn].s32StreamMode = astSampleVdec[idxChn].mode;
        g_jpegd_thread_param[idxChn].s32ChnId = idxChn;
        g_jpegd_thread_param[idxChn].s32IntervalTime = 1000;
        g_jpegd_thread_param[idxChn].u64PtsInit = 0;
        g_jpegd_thread_param[idxChn].u64PtsIncrease = 0;
        g_jpegd_thread_param[idxChn].enSendThreadCtrl = THREAD_CTRL_INIT;
        g_jpegd_thread_param[idxChn].enGetThreadCtrl  = THREAD_CTRL_INIT;
        g_jpegd_thread_param[idxChn].perfStreamBuf = nullptr;
        g_jpegd_thread_param[idxChn].s32CircleSend = g_send_circle;
        g_jpegd_thread_param[idxChn].bWholeDirDecode = g_whole_dir_decode;
        g_jpegd_thread_param[idxChn].s32MilliSec = 1000;
        g_jpegd_thread_param[idxChn].enPixFormat = g_pixel_format;
        g_jpegd_thread_param[idxChn].s32MinBufSize = astSampleVdec[idxChn].width * astSampleVdec[idxChn].height;
        g_jpegd_thread_param[idxChn].u32StmLenInJpegd = ALIGN_UP(astSampleVdec[idxChn].width, JPEGD_ALIGN_W) *
                                                      ALIGN_UP(astSampleVdec[idxChn].height, JPEGD_ALIGN_H) * 4;
        g_jpegd_thread_param[idxChn].u32PicInJpegd    = JPEGD_DECODED_QUEUE_NUM;
    }

    return HI_SUCCESS;
}

int32_t jpegd_destroy()
{
    for (uint32_t idxChn = g_start_channel; idxChn < g_chn_num + g_start_channel; idxChn++) {
        CHECK_CHN_RET(hi_mpi_vdec_stop_recv_stream(idxChn), idxChn, "hi_mpi_vdec_stop_recv_stream");
        CHECK_CHN_RET(hi_mpi_vdec_destroy_chn(idxChn), idxChn, "hi_mpi_vdec_destroy_chn");

        // Release JPEG stream mem
        if (g_jpegd_thread_param[idxChn].perfStreamBuf != nullptr) {
            hi_mpi_dvpp_free((void*)g_jpegd_thread_param[idxChn].perfStreamBuf);
            g_jpegd_thread_param[idxChn].perfStreamBuf = nullptr;
        }

        // Release YUV picture mem
        while (g_out_buffer_pool[idxChn].empty() == false) {
            (void)pthread_mutex_lock(&g_out_buffer_pool_lock[idxChn]);
            hi_mpi_dvpp_free(g_out_buffer_pool[idxChn].back());
            g_out_buffer_pool[idxChn].pop_back();
            (void)pthread_mutex_unlock(&g_out_buffer_pool_lock[idxChn]);
        }

        pthread_mutex_destroy(&g_out_buffer_pool_lock[idxChn]);
    }

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

int32_t deinit_jpegd_mod()
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