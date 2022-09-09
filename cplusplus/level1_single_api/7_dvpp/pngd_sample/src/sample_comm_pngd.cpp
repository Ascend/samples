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

bool g_whole_dir_decode  = HI_FALSE;
int32_t g_send_circle    = 0;
int32_t g_wait_time      = -1;
uint32_t g_width         = 3840;
uint32_t g_height        = 2160;
uint32_t g_chn_num       = 1;
uint32_t g_write_file    = 1;
uint32_t g_start_channel = 0;
uint32_t g_delay_time    = 5;
uint32_t g_performance   = 0;
uint32_t g_align         = 16;
uint64_t g_start_time    = 0;

char g_input_file_name[FILE_NAME_LEN]  = "infile";
char g_output_file_name[FILE_NAME_LEN] = "outfile";
char g_compatible_dir_name[FILE_NAME_LEN]  = "./";

aclrtContext g_context = nullptr;
aclrtRunMode g_run_mode = ACL_DEVICE;
hi_pixel_format g_pixel_format = HI_PIXEL_FORMAT_RGB_888;

vector<void *> g_out_buffer_pool[PNGD_MAX_CHN_NUM];
pthread_mutex_t g_out_buffer_pool_lock[PNGD_MAX_CHN_NUM];

static uint64_t g_end_get_time[PNGD_MAX_CHN_NUM] = {0};
static uint64_t g_start_send_time[PNGD_MAX_CHN_NUM] = {0};
static PNGD_THREAD_PARAM_S g_pngd_thread_param[PNGD_MAX_CHN_NUM]{};

const uint32_t PNGD_PERFOR_MODE_BUF_NUM   = 6;
const uint32_t PNGD_PERFOR_MODE_QUERY_CNT = 20000;

static void print_pngd_chn_status(int32_t chn)
{
    SAMPLE_PRT("---------------------------------------------------------------------------------------------------\n");
    SAMPLE_PRT("chn:%d, SendFrame:%u, DecodeFrames:%u, DecodeSucc:%u, DecodeFail:%u\n",
        chn,
        g_pngd_thread_param[chn].u32SendSucc,
        g_pngd_thread_param[chn].u32Decoded,
        g_pngd_thread_param[chn].u32DecodeSucc,
        g_pngd_thread_param[chn].u32DecodeFail);
    SAMPLE_PRT("---------------------------------------------------------------------------------------------------\n");
}

static int32_t save_file_type(hi_pixel_format pixel_format,
                              hi_compress_mode compress_mode,
                              char *type_name,
                              uint32_t type_name_len)
{
    int32_t ret;
    if (compress_mode == HI_COMPRESS_MODE_HFBC) {
        ret = snprintf(type_name, type_name_len - 1, "hfbc");
    } else if (pixel_format == HI_PIXEL_FORMAT_RGB_888) {
        ret = snprintf(type_name, type_name_len - 1, "rgb888");
    } else if (pixel_format == HI_PIXEL_FORMAT_BGR_888) {
        ret = snprintf(type_name, type_name_len - 1, "bgr888");
    } else if (pixel_format == HI_PIXEL_FORMAT_ARGB_8888) {
        ret = snprintf(type_name, type_name_len - 1, "argb8888");
    } else if (pixel_format == HI_PIXEL_FORMAT_RGBA_8888) {
        ret = snprintf(type_name, type_name_len - 1, "rgba8888");
    } else if (pixel_format == HI_PIXEL_FORMAT_ABGR_8888) {
        ret = snprintf(type_name, type_name_len - 1, "abgr8888");
    } else if ((pixel_format >= HI_PIXEL_FORMAT_YUV_400) && (pixel_format < HI_PIXEL_FORMAT_BUTT)) {
        ret = snprintf(type_name, type_name_len - 1, "yuv");
    } else {
        SAMPLE_PRT("pixel_format type err\n");
        return HI_FAILURE;
    }

    return ret;
}

static inline void pngd_get_time_stamp_us(uint64_t *val)
{
    struct timeval stTimeVal;
    gettimeofday(&stTimeVal, nullptr);
    *val = (uint64_t)stTimeVal.tv_sec * MULTIPLE_S_TO_US + stTimeVal.tv_usec;
}

void pngd_usage(char *sPrgNm)
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
    SAMPLE_PRT("\t:example: ./pngd_demo --in_image_file png_gray_32x32.png\
 --img_width 32 --img_height 32 --pixel_mode 12 --chn_num 1\
 --out_image_file png_gray_32x32.rgb --send_circle 100\n");
    SAMPLE_PRT("\t\n");
    SAMPLE_PRT("\t in_image_file: input image name.\n");
    SAMPLE_PRT("\t img_width: input image width.\n");
    SAMPLE_PRT("\t img_height: input image height.\n");
    SAMPLE_PRT("\t chn_num: pngd channel num.\n");
    SAMPLE_PRT("\t pixel_mode: yuv pixel format.\n");
    SAMPLE_PRT("\t align: output RGB width&height align param. 1, 16 or 128 valid\n");
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
    SAMPLE_PRT("\t            PngdDemo would exit after input seconds if it still running.\n");
    SAMPLE_PRT("\t whole_dir: compatible mode decode dir path. PNGD would decode all png to test compatibility\n");
    SAMPLE_PRT("\t If you want to check whether pics in folder can be decoded, run like:\n");
    SAMPLE_PRT("\t ./pngd_demo --whole_dir ./ImageNetRaw/ --write_file 1\n");
    SAMPLE_PRT("\t If you want to get PNGD fps, run like:\n");
    SAMPLE_PRT("\t ./pngd_demo --performance_mode 1 --in_image_file png_1920x1080.png\
 --img_width 1920 --img_height 1080 --pixel_mode 12 --chn_num 12 --send_circle 1000 --delay_time 20\n");
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
        {"chn_num",          1, nullptr, 'c'},
        {"send_circle",      1, nullptr, 's'},
        {"pixel_mode",       1, nullptr, 'p'},
        {"in_image_file",    1, nullptr, 'F'},
        {"out_image_file",   1, nullptr, 'O'},
        {"write_file",       1, nullptr, 'W'},
        {"whole_dir",        1, nullptr, 'H'},
        {"start_chn",        1, nullptr, 'S'},
        {"delay_time",       1, nullptr, 'Y'},
        {"wait_time",        1, nullptr, 'a'},
        {"performance_mode", 1, nullptr, 'P'},
        {"align",            1, nullptr, 'b'},
        {nullptr,            0, nullptr, 0}
    };

    while (1) {
        option_index = 0;

        c = getopt_long(argc, argv, "w:h:c:s:p:F:O:W:H:S:Y:a:P:b:", long_options, &option_index);
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
            case 'c':
                g_chn_num = atoi(optarg);
                break;
            case 's':
                g_send_circle = (hi_bool)atoi(optarg);
                break;
            case 'p':
                g_pixel_format = (hi_pixel_format)atoi(optarg);
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
            case 'b':
                g_align = atoi(optarg);
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
    // Check input stream width[32, 4096]
    if ((g_width > 4096) || (g_width < 32)) {
        SAMPLE_PRT("input file width is invalid, width = %u \n", g_width);
        return HI_FAILURE;
    }
    // Check input stream height[32, 4096]
    if ((g_height > 4096) || (g_height < 32)) {
        SAMPLE_PRT("input file height is invalid, height = %u \n", g_height);
        return HI_FAILURE;
    }
    // Check input stream format, valid value is 0 or 1
    if ((g_pixel_format != HI_PIXEL_FORMAT_UNKNOWN) &&
        (g_pixel_format != HI_PIXEL_FORMAT_RGB_888) &&
        (g_pixel_format != HI_PIXEL_FORMAT_RGBA_8888)) {
        SAMPLE_PRT("format is invalid, format = %d \n", g_pixel_format);
        return HI_FAILURE;
    }
    // Check channel number[1, PNGD_MAX_CHN_NUM]
    if ((g_chn_num > PNGD_MAX_CHN_NUM) || (g_chn_num < 1)) {
        SAMPLE_PRT("chan num is invalid, chan num = %u \n", g_chn_num);
        return HI_FAILURE;
    }
    // Check channel start number
    if (g_start_channel + g_chn_num > PNGD_MAX_CHN_NUM) {
        SAMPLE_PRT("total chan num is invalid,start chan = %u, chn num = %u \n", g_start_channel, g_chn_num);
        return HI_FAILURE;
    }
    // Check g_align is 16 or 128 or 1
    if ((g_align != 1) && (g_align != 16) && (g_align != 128)) {
        SAMPLE_PRT("align:%u isn't 1, 16 or 128\n", g_align);
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
    SAMPLE_PRT("g_chn_num: %u \n", g_chn_num);
    SAMPLE_PRT("g_send_circle: %d \n", g_send_circle);
    SAMPLE_PRT("g_pixel_format: %d \n", g_pixel_format);
    SAMPLE_PRT("g_whole_dir_decode: %d\n", g_whole_dir_decode);
    if (g_whole_dir_decode == HI_TRUE) {
        SAMPLE_PRT(" compatibleTestDirName: %s \n", g_compatible_dir_name);
    }
    SAMPLE_PRT("g_start_channel: %u \n", g_start_channel);
    SAMPLE_PRT("g_performance: %u \n", g_performance);
    SAMPLE_PRT("g_wait_time: %d \n", g_wait_time);
}

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

void pngd_init_start_time()
{
    struct timeval currentTime;

    gettimeofday(&currentTime, nullptr);
    g_start_time = currentTime.tv_sec * MULTIPLE_S_TO_US + currentTime.tv_usec;
}

void pngd_save_rgb_file(const std::string &fileName, hi_pic_info &picInfo)
{
    uint8_t *tmpPtr          = nullptr;
    uint32_t u32WidthInBytes = 0;

    if ((picInfo.picture_format == HI_PIXEL_FORMAT_ARGB_8888) ||
        (picInfo.picture_format == HI_PIXEL_FORMAT_ABGR_8888) ||
        (picInfo.picture_format == HI_PIXEL_FORMAT_RGBA_8888) ||
        (picInfo.picture_format == HI_PIXEL_FORMAT_BGRA_8888)) {
        u32WidthInBytes = picInfo.picture_width * 4;
    } else if ((picInfo.picture_format == HI_PIXEL_FORMAT_RGB_888) ||
               (picInfo.picture_format == HI_PIXEL_FORMAT_BGR_888)) {
        u32WidthInBytes = picInfo.picture_width * 3;
    } else {
        SAMPLE_PRT("This RGB format:%u is not support during save %s!\n", picInfo.picture_format, fileName.c_str());
        return;
    }

    SAMPLE_PRT("saving %s......RGB..%dx%d......\n", fileName.c_str(), picInfo.picture_width, picInfo.picture_height);
    std::vector<uint8_t> tmpHostBuffer;
    if (g_run_mode == ACL_HOST) {
        tmpHostBuffer.resize(picInfo.picture_buffer_size);

        int ret = aclrtMemcpy(&tmpHostBuffer[0], picInfo.picture_buffer_size,
                              (void *)picInfo.picture_address, picInfo.picture_buffer_size,
                              ACL_MEMCPY_DEVICE_TO_HOST);
        if (ret) {
            SAMPLE_PRT("aclrtMemcpy %d\n",ret);
            return;
        }

        tmpPtr = static_cast<uint8_t *>(&tmpHostBuffer[0]);
    } else {
        tmpPtr = (uint8_t *)picInfo.picture_address;
    }

    if (tmpPtr == nullptr) {
        SAMPLE_PRT("tmpPtr is NULL\n");
        return;
    }

    FILE *fp = fopen(fileName.c_str(), "wb");
    if (fp == nullptr) {
        SAMPLE_PRT("can't open file %s in get picture thread!\n", fileName.c_str());
        return;
    }

    for (uint32_t i = 0; i < picInfo.picture_height; i++, tmpPtr += picInfo.picture_width_stride) {
        fwrite(tmpPtr, u32WidthInBytes, 1, fp);
    }

    fsync(fileno(fp));
    fclose(fp);
    SAMPLE_PRT(" save %s done!\n", fileName.c_str());

    return;
}

void pngd_save_file(int32_t chn, int32_t picCnt, hi_pic_info &picInfo)
{
    char type_name[16] = {0};

    int32_t s32Ret = save_file_type(picInfo.picture_format, HI_COMPRESS_MODE_NONE,
                                    type_name, sizeof(type_name));
    if (s32Ret <= 0) {
        SAMPLE_PRT("Get file type failed!\n");
        return;
    }

    std::ostringstream oFileName;
    oFileName << "chn" << chn << "_" << picCnt << "_" << g_output_file_name << "." << type_name;
    std::string strFileName = oFileName.str();

    SAMPLE_PRT("[chn:%d] img_format = %d\n", chn, picInfo.picture_format);
    SAMPLE_PRT("[chn:%d] width = %u\n", chn, picInfo.picture_width);
    SAMPLE_PRT("[chn:%d] height = %u\n", chn, picInfo.picture_height);
    SAMPLE_PRT("[chn:%d] width_stride = %u\n", chn, picInfo.picture_width_stride);
    SAMPLE_PRT("[chn:%d] height_stride = %u\n", chn, picInfo.picture_height_stride);
    SAMPLE_PRT("[chn:%d] virt_size = %u\n", chn, picInfo.picture_buffer_size);
    SAMPLE_PRT("[chn:%d] virt_addr[0] = 0x%lx\n", chn, (uint64_t)(uintptr_t)picInfo.picture_address);

    if ((HI_PIXEL_FORMAT_RGB_888 <= picInfo.picture_format) &&
        (picInfo.picture_format <= HI_PIXEL_FORMAT_BGRA_8888)) {
        pngd_save_rgb_file(strFileName, picInfo);
    }
}

hi_pixel_format pngd_convert_image_format_to_pixel(hi_png_color_format rawFormat)
{
    if (rawFormat == HI_PNG_COLOR_FORMAT_GRAY) {
        return HI_PIXEL_FORMAT_RGB_888;
    } else if (rawFormat == HI_PNG_COLOR_FORMAT_RGB) {
        return HI_PIXEL_FORMAT_RGB_888;
    } else if (rawFormat == HI_PNG_COLOR_FORMAT_AGRAY) {
        return HI_PIXEL_FORMAT_RGBA_8888;
    } else if (rawFormat == HI_PNG_COLOR_FORMAT_ARGB) {
        return HI_PIXEL_FORMAT_RGBA_8888;
    }

    SAMPLE_PRT("Unsupport hi_png_color_format:(%u)!\n", rawFormat);
    return HI_PIXEL_FORMAT_BUTT;
}

void *pngd_send_stream_compatible(void *pArgs)
{
    FILE    *fpStrm        = nullptr;
    int32_t s32Ret         = 0;
    int32_t fileSize       = 0;
    int32_t s32ReadLen     = 0;
    uint32_t idx           = 0;
    uint32_t bufAllocCount = 0;
    uint32_t errCnt        = 0;
    void *outBuffer        = nullptr;
    hi_img_stream stStream{};
    hi_pic_info outPicInfo{};
    hi_img_info stImgInfo{};
    PNGD_THREAD_PARAM_S *pstPngdThreadParam = (PNGD_THREAD_PARAM_S *)pArgs;
    DIR *currentDir = nullptr;
    struct dirent *dirp;

    if (aclrtSetCurrentContext(g_context)) {
        SAMPLE_PRT("set context error\n");
        return (void *)(HI_FAILURE);
    }

    prctl(PR_SET_NAME, "PngSendStreamComp", 0, 0, 0);

    if ((currentDir = opendir(g_compatible_dir_name)) == nullptr) {
        SAMPLE_PRT("[chn:%d] opendir error\n", pstPngdThreadParam->s32ChnId);
        return (void *)(HI_FAILURE);
    }

    while (1) {
        if (pstPngdThreadParam->enSendThreadCtrl == THREAD_CTRL_STOP) {
            break;
        } else if (pstPngdThreadParam->enSendThreadCtrl == THREAD_CTRL_PAUSE) {
            sleep(1);
            continue;
        }

        while ((dirp = readdir(currentDir)) != nullptr) {
            if (pstPngdThreadParam->enSendThreadCtrl == THREAD_CTRL_STOP) {
                break;
            } else if (pstPngdThreadParam->enSendThreadCtrl == THREAD_CTRL_PAUSE) {
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
                if ((dirp->d_name[idx + 1] != 'p') ||
                    (dirp->d_name[idx + 2] != 'n') ||
                    (dirp->d_name[idx + 3] != 'g')) {
                        continue;
                    }
            } else {
                continue;
            }

            std::ostringstream pngdFileFullName;
            pngdFileFullName << g_compatible_dir_name << dirp->d_name;

            fpStrm = fopen(pngdFileFullName.str().c_str(), "rb");
            if (fpStrm == nullptr) {
                closedir(currentDir);
                SAMPLE_PRT("[chn:%d] can't open file %s in send stream thread!\n",
                           pstPngdThreadParam->s32ChnId, pngdFileFullName.str().c_str());
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
                    fclose(fpStrm);
                    SAMPLE_PRT("hi_mpi_dvpp_malloc %d.\n", s32Ret);
                    continue;
                }

                auto aclRet = aclrtMemcpy(inBuffer, s32ReadLen, &fileData[0], s32ReadLen, ACL_MEMCPY_HOST_TO_DEVICE);
                if (aclRet != ACL_SUCCESS) {
                    hi_mpi_dvpp_free(inBuffer);
                    fclose(fpStrm);
                    SAMPLE_PRT("Copy host memcpy to device fail with %d.\n", aclRet);
                    continue;
                }
                stStream.addr  = (uint8_t *)&fileData[0];
            } else {
                s32Ret = hi_mpi_dvpp_malloc(0, (void**)&inBuffer, fileSize);
                if ((s32Ret != 0) || (inBuffer == NULL)) {
                    SAMPLE_PRT("[chn:%d] can't alloc %d in send thread!\n", pstPngdThreadParam->s32ChnId, fileSize);
                    fclose(fpStrm);
                    continue;
                }

                SAMPLE_PRT("[chn:%d] [%s] stream addr:0x%lx size: %d\n", pstPngdThreadParam->s32ChnId,
                           pngdFileFullName.str().c_str(), (uint64_t)(uintptr_t)inBuffer, fileSize);
                s32ReadLen = fread(inBuffer, 1, fileSize, fpStrm);
                if (s32ReadLen == 0) {
                    hi_mpi_dvpp_free((void*)inBuffer);
                    fclose(fpStrm);
                    continue;
                }
                stStream.addr = (uint8_t *)inBuffer;
            }
            fclose(fpStrm);

            stStream.type = HI_PT_PNG;
            stStream.pts  = pstPngdThreadParam->u64PtsInit;
            stStream.len  = s32ReadLen;

            s32Ret = hi_mpi_png_get_image_info(&stStream, &stImgInfo);
            if (s32Ret != HI_SUCCESS) {
                SAMPLE_PRT("[chn:%d] hi_mpi_png_get_image_info faild! ret:%#x\n", pstPngdThreadParam->s32ChnId, s32Ret);
                hi_mpi_dvpp_free(inBuffer);
                continue;
            }

            stStream.addr  = (uint8_t *)inBuffer;
            outPicInfo.picture_width  = stImgInfo.width;
            outPicInfo.picture_height = stImgInfo.height;
            outPicInfo.picture_width_stride  = stImgInfo.width_stride;
            outPicInfo.picture_height_stride = stImgInfo.height_stride;
            outPicInfo.picture_buffer_size   = stImgInfo.img_buf_size;
            outPicInfo.picture_format = HI_PIXEL_FORMAT_UNKNOWN;

            s32Ret = hi_mpi_dvpp_malloc(0, &outBuffer, outPicInfo.picture_buffer_size);
            if (s32Ret != 0) {
                SAMPLE_PRT("hi_mpi_dvpp_malloc out buf Failed\n");
                hi_mpi_dvpp_free(inBuffer);
                continue;
            }
            bufAllocCount++;
            outPicInfo.picture_address = outBuffer;

            SAMPLE_PRT("[chn:%d] [%s] w:%u h:%u pixel:%u. idx:%u buffer:0x%lx size:%u\n",
                       pstPngdThreadParam->s32ChnId, pngdFileFullName.str().c_str(),
                       outPicInfo.picture_width, outPicInfo.picture_height,
                       outPicInfo.picture_format, bufAllocCount,
                       (uint64_t)(uintptr_t)outBuffer, outPicInfo.picture_buffer_size);

            do {
                s32Ret = hi_mpi_pngd_send_stream(pstPngdThreadParam->s32ChnId,
                                                 &stStream, &outPicInfo,
                                                 pstPngdThreadParam->s32MilliSec);
                if (s32Ret != HI_ERR_PNGD_BUF_FULL) {
                    errCnt++;
                    if (errCnt > 100) {
                        break; // break do...while
                    }
                }

                usleep(pstPngdThreadParam->s32IntervalTime);
            } while ((s32Ret != HI_SUCCESS) && (pstPngdThreadParam->enSendThreadCtrl == THREAD_CTRL_START));
            if (s32Ret != HI_SUCCESS) {
                SAMPLE_PRT("[chn:%d] hi_mpi_pngd_send_stream Send [%s] failed for %u times! addr:%#lx, len:%u\n",
                           pstPngdThreadParam->s32ChnId, pngdFileFullName.str().c_str(), errCnt,
                           (uint64_t)(uintptr_t)stStream.addr, stStream.len);

                s32Ret = hi_mpi_dvpp_free(inBuffer);
                if (s32Ret != 0) {
                    SAMPLE_PRT("hi_mpi_dvpp_free inBuffer failed\n");
                }
                s32Ret = hi_mpi_dvpp_free(outBuffer);
                if (s32Ret != 0) {
                    SAMPLE_PRT("hi_mpi_dvpp_free outBuffer failed\n");
                }
            }

            errCnt = 0;
            SAMPLE_PRT("[chn:%d] hi_mpi_pngd_send_stream Send [%u] Again ok! addr:%#lx, len:%u\n",
                       pstPngdThreadParam->s32ChnId, bufAllocCount,
                       (uint64_t)(uintptr_t)stStream.addr, stStream.len);
            g_pngd_thread_param[pstPngdThreadParam->s32ChnId].u32SendSucc++;

            usleep(pstPngdThreadParam->s32IntervalTime);
        }

        SAMPLE_PRT("[chn:%d] s32CircleSend:%d\n", pstPngdThreadParam->s32ChnId, pstPngdThreadParam->s32CircleSend);
        seekdir(currentDir, 0);

        if (pstPngdThreadParam->enSendThreadCtrl == THREAD_CTRL_STOP) {
            break; // break while(1)
        }

        if ((pstPngdThreadParam->s32CircleSend == 0) ||
            (pstPngdThreadParam->s32CircleSend == 1)) {
            break;
        } else if (pstPngdThreadParam->s32CircleSend > 0) {
            pstPngdThreadParam->s32CircleSend--;
        }
    }
    closedir(currentDir);

    SAMPLE_PRT("[chn:%d] send steam thread return\n", pstPngdThreadParam->s32ChnId);

    return (void *)HI_SUCCESS;
}

void *pngd_send_stream_performance(void *pArgs)
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
    hi_img_stream stStream{};
    hi_pic_info outPicInfo{};
    hi_img_info stImgInfo{};
    PNGD_THREAD_PARAM_S *pstPngdThreadParam = (PNGD_THREAD_PARAM_S *)pArgs;

    if (aclrtSetCurrentContext(g_context)) {
        SAMPLE_PRT("set context error\n");
        return (void *)(HI_FAILURE);
    }

    std::ostringstream pthreadName;
    pthreadName << "PngSendPerf_" << pstPngdThreadParam->s32ChnId;
    prctl(PR_SET_NAME, pthreadName.str().c_str(), 0, 0, 0);

    fpStrm = fopen(pstPngdThreadParam->cFileName, "rb");
    if (fpStrm == nullptr) {
        SAMPLE_PRT("[chn:%d] can't open file %s!\n",
                   pstPngdThreadParam->s32ChnId, pstPngdThreadParam->cFileName);
        return (void *)(HI_FAILURE);
    }

    fseek(fpStrm, 0L, SEEK_END);
    fileSize = ftell(fpStrm);

    s32Ret = hi_mpi_dvpp_malloc(0, (void**)&pstPngdThreadParam->perfStreamBuf, fileSize);
    if ((s32Ret != 0) || (pstPngdThreadParam->perfStreamBuf == nullptr)) {
        SAMPLE_PRT("[chn:%d] can't alloc %d in send stream thread!\n",
                   pstPngdThreadParam->s32ChnId, fileSize);
        fclose(fpStrm);
        return (void *)(HI_FAILURE);
    }
    fflush(stdout);

    fseek(fpStrm, 0L, SEEK_SET);
    std::vector<uint8_t> fileData;

    if (g_run_mode == ACL_HOST) {
        fileData.resize(fileSize);
        s32ReadLen = fread(&fileData[0], 1, fileSize, fpStrm);
        auto aclRet = aclrtMemcpy(pstPngdThreadParam->perfStreamBuf, s32ReadLen,
                                  &fileData[0], s32ReadLen,
                                  ACL_MEMCPY_HOST_TO_DEVICE);
        if (aclRet != ACL_SUCCESS) {
            hi_mpi_dvpp_free(pstPngdThreadParam->perfStreamBuf);
            pstPngdThreadParam->perfStreamBuf = nullptr;
            fclose(fpStrm);
            SAMPLE_PRT("Copy host memcpy to device fail with %d.\n", aclRet);
            return (void *)(HI_FAILURE);
        }
        stStream.addr = (uint8_t *)&fileData[0];
    } else {
        s32ReadLen = fread(pstPngdThreadParam->perfStreamBuf, 1, fileSize, fpStrm);
        stStream.addr = pstPngdThreadParam->perfStreamBuf;
    }
    fflush(stdout);
    fclose(fpStrm);

    stStream.type = HI_PT_PNG;
    stStream.pts = pstPngdThreadParam->u64PtsInit;
    stStream.len = fileSize;
    s32Ret = hi_mpi_png_get_image_info(&stStream, &stImgInfo);
    if (s32Ret != HI_SUCCESS) {
        hi_mpi_dvpp_free(pstPngdThreadParam->perfStreamBuf);
        pstPngdThreadParam->perfStreamBuf = nullptr;
        SAMPLE_PRT("[chn:%d] hi_mpi_png_get_image_info faild! ret:%#x\n", pstPngdThreadParam->s32ChnId, s32Ret);
        return (void *)(HI_FAILURE);
    }

    outPicInfo.picture_height_stride = stImgInfo.height_stride;
    if (pstPngdThreadParam->enPixFormat == HI_PIXEL_FORMAT_UNKNOWN) {
        outPicInfo.picture_width_stride  = stImgInfo.width_stride;
    } else {
        if (pstPngdThreadParam->enPixFormat == HI_PIXEL_FORMAT_RGB_888) {
            outPicInfo.picture_width_stride = ALIGN_UP(stImgInfo.width, g_align) * 3;
        } else if (pstPngdThreadParam->enPixFormat == HI_PIXEL_FORMAT_RGBA_8888) {
            outPicInfo.picture_width_stride = ALIGN_UP(stImgInfo.width, g_align) * 4;
        }
    }
    outBufferSize = outPicInfo.picture_width_stride * outPicInfo.picture_height_stride;
    SAMPLE_PRT("[chn:%d] outBufferSize:%u. pix:%d stream file:%s, filesize:%d Start! w_stride:%u h_stride:%u\n",
               pstPngdThreadParam->s32ChnId, outBufferSize, pstPngdThreadParam->enPixFormat,
               pstPngdThreadParam->cFileName, fileSize,
               outPicInfo.picture_width_stride, outPicInfo.picture_height_stride);

    for (idxChn = 0; idxChn < PNGD_PERFOR_MODE_BUF_NUM;) {
        s32Ret = hi_mpi_dvpp_malloc(0, &outBuffer, outBufferSize);
        if ((s32Ret != 0) || (outBuffer == nullptr)) {
            SAMPLE_PRT("[chn:%d] DvppMalloc From Pool Failed. ret:%d\n", pstPngdThreadParam->s32ChnId, s32Ret);
        } else {
            idxChn++;
            g_out_buffer_pool[pstPngdThreadParam->s32ChnId].push_back(outBuffer);
        }
    }

    delay_exec(g_start_time, g_delay_time);

    pngd_get_time_stamp_us(&g_start_send_time[pstPngdThreadParam->s32ChnId]);
    while (1) {
        if (pstPngdThreadParam->enSendThreadCtrl == THREAD_CTRL_STOP) {
            break;
        } else if (pstPngdThreadParam->enSendThreadCtrl == THREAD_CTRL_PAUSE) {
            sleep(1);
            continue;
        }

        stStream.type = HI_PT_PNG;
        stStream.pts  = pstPngdThreadParam->u64PtsInit;
        stStream.addr = pstPngdThreadParam->perfStreamBuf;
        stStream.len  = s32ReadLen;

        for (mallocCount = 0; mallocCount < PNGD_PERFOR_MODE_QUERY_CNT; mallocCount++) {
            (void)pthread_mutex_lock(&g_out_buffer_pool_lock[pstPngdThreadParam->s32ChnId]);
            if (g_out_buffer_pool[pstPngdThreadParam->s32ChnId].empty() == false) {
                outBuffer = g_out_buffer_pool[pstPngdThreadParam->s32ChnId].back();
                g_out_buffer_pool[pstPngdThreadParam->s32ChnId].pop_back();
                (void)pthread_mutex_unlock(&g_out_buffer_pool_lock[pstPngdThreadParam->s32ChnId]);
                break;
            } else {
                (void)pthread_mutex_unlock(&g_out_buffer_pool_lock[pstPngdThreadParam->s32ChnId]);
                usleep(1000);
            }

            if (pstPngdThreadParam->enSendThreadCtrl == THREAD_CTRL_STOP) {
                break;
            }
        }

        if (mallocCount >= PNGD_PERFOR_MODE_QUERY_CNT) {
            SAMPLE_PRT("[chn:%d] DvppMalloc From Pool Failed for %u times. \n",
                       pstPngdThreadParam->s32ChnId, mallocCount);
            break;
        }

        outPicInfo.picture_address  = outBuffer;
        outPicInfo.picture_buffer_size = outBufferSize;
        outPicInfo.picture_width  = g_width;
        outPicInfo.picture_height = g_height;
        outPicInfo.picture_format = pstPngdThreadParam->enPixFormat;

        do {
            s32Ret = hi_mpi_pngd_send_stream(pstPngdThreadParam->s32ChnId,
                                             &stStream, &outPicInfo,
                                             pstPngdThreadParam->s32MilliSec);
            if (s32Ret != HI_ERR_PNGD_BUF_FULL) {
                errCnt++;
                if (errCnt > 100) {
                    break; // break do...while
                }
            }

            usleep(pstPngdThreadParam->s32IntervalTime);
        } while ((s32Ret != HI_SUCCESS) && (pstPngdThreadParam->enSendThreadCtrl == THREAD_CTRL_START));
        if (s32Ret != HI_SUCCESS) {
            if (errCnt != 0) {
                SAMPLE_PRT("[chn:%d] SendStream failed %u times! ret:%#x addr:%#lx, len:%u, outBuffer:0x%lx, size:%u\n",
                           pstPngdThreadParam->s32ChnId, errCnt, s32Ret,
                           (uint64_t)(uintptr_t)stStream.addr, stStream.len,
                           (uint64_t)(uintptr_t)outBuffer, outBufferSize);
            }
            break; // break while(1)
        }

        errCnt = 0;
        g_pngd_thread_param[pstPngdThreadParam->s32ChnId].u32SendSucc++;

        if ((pstPngdThreadParam->s32CircleSend == 0) ||
            (pstPngdThreadParam->s32CircleSend == 1)) {
            break;
        } else if (pstPngdThreadParam->s32CircleSend > 0) {
            pstPngdThreadParam->s32CircleSend--;
        }
    }

    return (void *)HI_SUCCESS;
}

void *pngd_send_stream(void *pArgs)
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
    hi_img_stream stStream{};
    hi_pic_info outPicInfo{};
    hi_img_info stImgInfo{};
    PNGD_THREAD_PARAM_S *pstPngdThreadParam = (PNGD_THREAD_PARAM_S *)pArgs;

    if (aclrtSetCurrentContext(g_context)) {
        SAMPLE_PRT("set context error\n");
        return (void *)(HI_FAILURE);
    }

    std::ostringstream pthreadName;
    pthreadName << "PngSend_" << pstPngdThreadParam->s32ChnId;
    prctl(PR_SET_NAME, pthreadName.str().c_str(), 0, 0, 0);

    fpStrm = fopen(pstPngdThreadParam->cFileName, "rb");
    if (fpStrm == nullptr) {
        SAMPLE_PRT("[chn:%d] can't open file %s in send stream thread!\n",
                   pstPngdThreadParam->s32ChnId, pstPngdThreadParam->cFileName);
        return (void *)(HI_FAILURE);
    }

    fseek(fpStrm, 0L, SEEK_END);
    fileSize = ftell(fpStrm);
    fflush(stdout);

    while (1) {
        if (pstPngdThreadParam->enSendThreadCtrl == THREAD_CTRL_STOP) {
            break;
        } else if (pstPngdThreadParam->enSendThreadCtrl == THREAD_CTRL_PAUSE) {
            sleep(1);
            continue;
        }

        std::vector<uint8_t*> fileData;

        s32Ret = hi_mpi_dvpp_malloc(0, (void**)&pu8Buf, fileSize);
        if ((s32Ret != 0) || (pu8Buf == nullptr)) {
            SAMPLE_PRT("[chn:%d] can't alloc %d in send stream thread ret = %d!\n",
                       pstPngdThreadParam->s32ChnId, fileSize, s32Ret);
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
                SAMPLE_PRT("Copy host memcpy to device fail with %d.\n", aclRet);
                return (void *)(HI_FAILURE);
            }

            stStream.addr  = (uint8_t *)&fileData[0];
        } else {
            stStream.addr = (uint8_t *)pu8Buf;
        }

        stStream.type = HI_PT_PNG;
        stStream.pts  = pstPngdThreadParam->u64PtsInit;
        stStream.len  = s32ReadLen;
        s32Ret = hi_mpi_png_get_image_info(&stStream, &stImgInfo);
        if (s32Ret != HI_SUCCESS) {
            SAMPLE_PRT("[chn:%d] hi_mpi_png_get_image_info faild! ret:%#x\n", pstPngdThreadParam->s32ChnId, s32Ret);
            hi_mpi_dvpp_free(pu8Buf);
            break;
        }
        if (pstPngdThreadParam->enPixFormat == HI_PIXEL_FORMAT_UNKNOWN) {
            outPicInfo.picture_width_stride  = stImgInfo.width_stride;
        } else {
            if (pstPngdThreadParam->enPixFormat == HI_PIXEL_FORMAT_RGB_888) {
                outPicInfo.picture_width_stride = ALIGN_UP(stImgInfo.width, g_align) * 3;
            } else if (pstPngdThreadParam->enPixFormat == HI_PIXEL_FORMAT_RGBA_8888) {
                outPicInfo.picture_width_stride = ALIGN_UP(stImgInfo.width, g_align) * 4;
            }
        }
        outPicInfo.picture_height_stride = stImgInfo.height_stride;
        outBufferSize = outPicInfo.picture_width_stride * outPicInfo.picture_height_stride;
        SAMPLE_PRT("[chn:%d] outBufferSize:%u. pix:%d stream file:%s, filesize:%d Start! w_stride:%u h_stride:%u\n",
                   pstPngdThreadParam->s32ChnId, outBufferSize, pstPngdThreadParam->enPixFormat,
                   pstPngdThreadParam->cFileName, fileSize,
                   outPicInfo.picture_width_stride, outPicInfo.picture_height_stride);

        stStream.addr = (uint8_t *)pu8Buf;
        s32Ret = hi_mpi_dvpp_malloc(0, &outBuffer, outBufferSize);
        if (s32Ret != 0) {
            SAMPLE_PRT("[chn:%d] hi_mpi_dvpp_malloc %u Failed. ret:%d.\n",
                       pstPngdThreadParam->s32ChnId, outBufferSize, s32Ret);
            hi_mpi_dvpp_free((void*)pu8Buf);
            break;
        }
        bufAllocCount++;

        outPicInfo.picture_address = outBuffer;
        outPicInfo.picture_buffer_size = outBufferSize;
        outPicInfo.picture_width  = g_width;
        outPicInfo.picture_height = g_height;
        outPicInfo.picture_format = pstPngdThreadParam->enPixFormat;

        do {
            s32Ret = hi_mpi_pngd_send_stream(pstPngdThreadParam->s32ChnId,
                                             &stStream, &outPicInfo, pstPngdThreadParam->s32MilliSec);
            if (s32Ret != HI_ERR_PNGD_BUF_FULL) {
                errCnt++;
                if (errCnt > 100) {
                    break; // break do...while
                }
            }

            usleep(pstPngdThreadParam->s32IntervalTime);
        } while ((s32Ret != HI_SUCCESS) && (pstPngdThreadParam->enSendThreadCtrl == THREAD_CTRL_START));
        if (s32Ret != HI_SUCCESS) {
            if (errCnt != 0) {
                SAMPLE_PRT("[chn:%d] SendStream [%u]failed %u times! addr:%#lx, len:%u, outBuffer:0x%lx, size:%u\n",
                           pstPngdThreadParam->s32ChnId, bufAllocCount, errCnt,
                           (uint64_t)(uintptr_t)stStream.addr, stStream.len,
                           (uint64_t)(uintptr_t)outBuffer, outBufferSize);
                s32Ret = hi_mpi_dvpp_free((void*)pu8Buf);
                if (s32Ret != 0) {
                    SAMPLE_PRT("[chn:%d] Free stream failed\n", pstPngdThreadParam->s32ChnId);
                }
                s32Ret = hi_mpi_dvpp_free((void *)outBuffer);
                if (s32Ret != 0) {
                    SAMPLE_PRT("[chn:%d] Free outBuffer failed\n", pstPngdThreadParam->s32ChnId);
                }
            }

            break; // break while(1)
        }

        g_pngd_thread_param[pstPngdThreadParam->s32ChnId].u32SendSucc++;
        errCnt = 0;
        SAMPLE_PRT("[chn:%d] hi_mpi_pngd_send_stream Send[%u] ok! addr:%#lx, len:%d, outBuffer:0x%lx, size:%u\n",
                   pstPngdThreadParam->s32ChnId, bufAllocCount,
                   (uint64_t)(uintptr_t)stStream.addr, stStream.len,
                   (uint64_t)(uintptr_t)outBuffer, outBufferSize);

        if ((pstPngdThreadParam->s32CircleSend == 0) ||
            (pstPngdThreadParam->s32CircleSend == 1)) {
            break;
        } else if (pstPngdThreadParam->s32CircleSend > 0) {
            pstPngdThreadParam->s32CircleSend--;
            usleep(pstPngdThreadParam->s32IntervalTime);
        } else {
            usleep(pstPngdThreadParam->s32IntervalTime);
        }
    }
    fflush(stdout);
    fclose(fpStrm);

    SAMPLE_PRT("[chn:%d] send steam thread return\n", pstPngdThreadParam->s32ChnId);
    return (void *)HI_SUCCESS;
}

void pngd_cmd_ctrl(pthread_t *ptr_pngd_send_tid, pthread_t *ptr_pngd_get_tid)
{
    int32_t s32Ret;
    uint64_t nextexcTime;
    uint64_t tmpCurtime;

    uint64_t curTime = 0;
    uint64_t lastTime = 0;
    struct timeval currentTime;

    if (g_wait_time > 0) {
        nextexcTime = g_start_time + static_cast<uint64_t>(g_wait_time * MULTIPLE_S_TO_US);
    }

    for (uint32_t idxChn = g_start_channel; idxChn < g_chn_num + g_start_channel; idxChn++) {
        if (ptr_pngd_send_tid[idxChn] != 0) {
            s32Ret = pthread_join(ptr_pngd_send_tid[idxChn], NULL);
            ptr_pngd_send_tid[idxChn] = 0;
        }
        g_pngd_thread_param[idxChn].enSendThreadCtrl = THREAD_CTRL_STOP;

        while (1) {
            // jump out if no decoding data
            if (g_pngd_thread_param[idxChn].u32SendSucc == g_pngd_thread_param[idxChn].u32Decoded) {
                if (g_pngd_thread_param[idxChn].enGetThreadCtrl == THREAD_CTRL_START) {
                    if (g_performance <= 0) {
                        print_pngd_chn_status(g_pngd_thread_param[idxChn].s32ChnId);
                    }
                    g_pngd_thread_param[idxChn].enGetThreadCtrl = THREAD_CTRL_STOP;
                    break;
                }
            }

            // jump out when timeout
            if (g_wait_time > 0) {
                gettimeofday(&currentTime, nullptr);
                tmpCurtime = currentTime.tv_sec * MULTIPLE_S_TO_US + currentTime.tv_usec;
                if (tmpCurtime > nextexcTime) {
                    SAMPLE_PRT("[chn:%u] Running Time out. Try to exit!\n", idxChn);
                    g_pngd_thread_param[idxChn].enGetThreadCtrl = THREAD_CTRL_STOP;
                    break;
                }
            }

            if (g_performance <= 0) {
                pngd_get_time_stamp_us(&curTime);
                if (curTime - lastTime > 5000000) {
                    SAMPLE_PRT("[chn:%u] Waiting status LeftPics:%u. getThreadCtrl:%d\n",
                               idxChn, g_pngd_thread_param[idxChn].u32SendSucc - g_pngd_thread_param[idxChn].u32Decoded,
                               g_pngd_thread_param[idxChn].enGetThreadCtrl);
                    lastTime = curTime;
                }
            }

            usleep(1000);
        }
    }

    for (uint32_t idxChn = g_start_channel; idxChn < g_chn_num + g_start_channel; idxChn++) {
        if (ptr_pngd_get_tid[idxChn] != 0) {
            pthread_join(ptr_pngd_get_tid[idxChn], nullptr);
            ptr_pngd_get_tid[idxChn] = 0;
        }
    }

    return;
}

int32_t pngd_start_send_stream(pthread_t *ptr_pngd_send_tid)
{
    int32_t ret = -1;

    for (uint32_t idxChn = g_start_channel; idxChn < g_chn_num + g_start_channel; idxChn++) {
        ptr_pngd_send_tid[idxChn] = 0;
        g_pngd_thread_param[idxChn].enSendThreadCtrl = THREAD_CTRL_START;
        if (g_pngd_thread_param[idxChn].bWholeDirDecode == HI_TRUE) {
            ret = pthread_create(&ptr_pngd_send_tid[idxChn], 0,
                                 pngd_send_stream_compatible,
                                 (void *)&g_pngd_thread_param[idxChn]);
            if (ret != 0) {
                SAMPLE_PRT("[chn:%u] create send pic thread Fail, ret = %d \n", idxChn, ret);
                ptr_pngd_send_tid[idxChn] = 0;
                g_pngd_thread_param[idxChn].enSendThreadCtrl = THREAD_CTRL_STOP;
                return ret;
            }
        } else if (g_performance > 0) {
            ret = pthread_create(&ptr_pngd_send_tid[idxChn], 0,
                                 pngd_send_stream_performance,
                                 (void *)&g_pngd_thread_param[idxChn]);
            if (ret != 0) {
                SAMPLE_PRT("[chn:%u] create send pic thread Fail, ret = %d \n", idxChn, ret);
                ptr_pngd_send_tid[idxChn] = 0;
                g_pngd_thread_param[idxChn].enSendThreadCtrl = THREAD_CTRL_STOP;
                return ret;
            }
        } else {
            ret = pthread_create(&ptr_pngd_send_tid[idxChn], 0,
                                 pngd_send_stream,
                                 (void *)&g_pngd_thread_param[idxChn]);
            if (ret != 0) {
                SAMPLE_PRT("[chn:%u] create send pic thread Fail, ret = %d \n", idxChn, ret);
                ptr_pngd_send_tid[idxChn] = 0;
                g_pngd_thread_param[idxChn].enSendThreadCtrl = THREAD_CTRL_STOP;
                return ret;
            }
        }
    }

    return 0;
}

void pngd_stop_send_stream()
{
    SAMPLE_PRT(" PNGD stop send stream threads\n");
    for (uint32_t idxChn = g_start_channel; idxChn < g_chn_num + g_start_channel; idxChn++) {
        g_pngd_thread_param[idxChn].enSendThreadCtrl = THREAD_CTRL_STOP;
    }
}

void pngd_show_decode_state()
{
    if (g_performance <= 0) {
        return;
    }

    for (uint32_t idxChn = g_start_channel; idxChn < g_chn_num + g_start_channel; ++idxChn) {
        uint64_t u64DiffTime = g_end_get_time[idxChn] - g_start_send_time[idxChn];
        if (u64DiffTime == 0) {
            continue;
        }
        SAMPLE_PRT("\033[0;33m -------------------------------------------------------------------------\033[0;39m\n");
        double actualFrameRate = ((double)g_pngd_thread_param[idxChn].u32Decoded * MULTIPLE_S_TO_US) / u64DiffTime;
        SAMPLE_PRT("\033[0;33m chnId %u, actualFrameRate %.1f, SendCnt %u, GetCnt %u DiffTime %lu\033[0;39m\n",
                   idxChn, actualFrameRate, g_pngd_thread_param[idxChn].u32SendSucc,
                   g_pngd_thread_param[idxChn].u32Decoded, u64DiffTime);
        SAMPLE_PRT("\033[0;33m -------------------------------------------------------------------------\033[0;39m\n");
    }

    return;
}

void pngd_release_mem(hi_s32 chn, hi_pic_info &picInfo, hi_img_stream &imgStream)
{
    if (picInfo.picture_address != nullptr) {
        hi_mpi_dvpp_free((void *)picInfo.picture_address);
        picInfo.picture_address = nullptr;
    }

    if (imgStream.addr != nullptr) {
        hi_mpi_dvpp_free((void*)imgStream.addr);
        imgStream.addr = nullptr;
    }
}

void *pngd_get_pic(void *pArgs)
{
    int32_t s32Ret        = 0;
    hi_pic_info stPicInfo{};
    hi_img_stream stStream{};
    PNGD_THREAD_PARAM_S *pstPngdThreadParam = (PNGD_THREAD_PARAM_S *)pArgs;

    if (aclrtSetCurrentContext(g_context)) {
        SAMPLE_PRT("set context error\n");
        return (void *)(HI_FAILURE);
    }

    std::ostringstream pthreadName;
    pthreadName << "PngdGetPic_" << pstPngdThreadParam->s32ChnId;
    prctl(PR_SET_NAME, pthreadName.str().c_str(), 0, 0, 0);

    SAMPLE_PRT("[chn:%d] pngd_get_pic Start\n", pstPngdThreadParam->s32ChnId);
    while (1) {
        if (pstPngdThreadParam->enGetThreadCtrl == THREAD_CTRL_STOP) {
            SAMPLE_PRT("[chn:%d] pngd_get_pic break out\n", pstPngdThreadParam->s32ChnId);
            break;
        }

        s32Ret = hi_mpi_pngd_get_image_data(pstPngdThreadParam->s32ChnId,
                                            &stPicInfo, &stStream, pstPngdThreadParam->s32MilliSec);
        if (s32Ret == HI_SUCCESS) {
            g_pngd_thread_param[pstPngdThreadParam->s32ChnId].u32DecodeSucc++;
            g_pngd_thread_param[pstPngdThreadParam->s32ChnId].u32Decoded++;

            if (g_write_file > 0) {
                pngd_save_file(pstPngdThreadParam->s32ChnId,
                    g_pngd_thread_param[pstPngdThreadParam->s32ChnId].u32DecodeSucc, stPicInfo);
            }

            pngd_release_mem(pstPngdThreadParam->s32ChnId, stPicInfo, stStream);
            SAMPLE_PRT("[chn:%d] Yes, GetFrame Success [%u].\n",
                pstPngdThreadParam->s32ChnId, g_pngd_thread_param[pstPngdThreadParam->s32ChnId].u32DecodeSucc);
        } else if (s32Ret != HI_ERR_PNGD_BUF_EMPTY) {
            g_pngd_thread_param[pstPngdThreadParam->s32ChnId].u32DecodeFail++;
            g_pngd_thread_param[pstPngdThreadParam->s32ChnId].u32Decoded++;

            pngd_release_mem(pstPngdThreadParam->s32ChnId, stPicInfo, stStream);
            SAMPLE_PRT("[chn:%d] Yes, GetFrame Fail(Decoder Fail) [%u]. ret:%#x\n", pstPngdThreadParam->s32ChnId,
                       g_pngd_thread_param[pstPngdThreadParam->s32ChnId].u32DecodeFail, s32Ret);
        } else {
            usleep(300);
        }
    }

    SAMPLE_PRT("[chn:%d] Get thread end\n", pstPngdThreadParam->s32ChnId);
    return (void *)HI_SUCCESS;
}

void *pngd_get_pic_performance(void *pArgs)
{
    int32_t s32Ret;
    hi_img_stream stStream{};
    hi_pic_info stPicInfo{};
    PNGD_THREAD_PARAM_S *pstPngdThreadParam = (PNGD_THREAD_PARAM_S *)pArgs;

    std::ostringstream pthreadName;
    pthreadName << "PngdGetPerf_" << pstPngdThreadParam->s32ChnId;
    prctl(PR_SET_NAME, pthreadName.str().c_str(), 0, 0, 0);

    while (1) {
        if (pstPngdThreadParam->enGetThreadCtrl == THREAD_CTRL_STOP) {
            break;
        }

        s32Ret = hi_mpi_pngd_get_image_data(pstPngdThreadParam->s32ChnId,
                                            &stPicInfo, &stStream, pstPngdThreadParam->s32MilliSec);
        if (s32Ret == HI_SUCCESS) {
            pngd_get_time_stamp_us(&g_end_get_time[pstPngdThreadParam->s32ChnId]);

            g_pngd_thread_param[pstPngdThreadParam->s32ChnId].u32DecodeSucc++;
            g_pngd_thread_param[pstPngdThreadParam->s32ChnId].u32Decoded++;

            (void)pthread_mutex_lock(&g_out_buffer_pool_lock[pstPngdThreadParam->s32ChnId]);
            g_out_buffer_pool[pstPngdThreadParam->s32ChnId].push_back((void *)stPicInfo.picture_address);
            (void)pthread_mutex_unlock(&g_out_buffer_pool_lock[pstPngdThreadParam->s32ChnId]);
        } else if (s32Ret != HI_ERR_PNGD_BUF_EMPTY) {
            g_pngd_thread_param[pstPngdThreadParam->s32ChnId].u32DecodeFail++;
            g_pngd_thread_param[pstPngdThreadParam->s32ChnId].u32Decoded++;

            (void)pthread_mutex_lock(&g_out_buffer_pool_lock[pstPngdThreadParam->s32ChnId]);
            g_out_buffer_pool[pstPngdThreadParam->s32ChnId].push_back((void *)stPicInfo.picture_address);
            (void)pthread_mutex_unlock(&g_out_buffer_pool_lock[pstPngdThreadParam->s32ChnId]);
            SAMPLE_PRT("[chn:%d] GetFrame Fail. ret:%#x, Decoder Fail %u times\n",
                pstPngdThreadParam->s32ChnId, s32Ret, g_pngd_thread_param[pstPngdThreadParam->s32ChnId].u32DecodeFail);
        } else {
            usleep(300);
        }
    }

    return (void *)HI_SUCCESS;
}

int32_t pngd_start_get_pic(pthread_t *ptr_pngd_get_tid)
{
    int32_t ret = -1;

    for (uint32_t idxChn = g_start_channel; idxChn < g_chn_num + g_start_channel; idxChn++) {
        ptr_pngd_get_tid[idxChn] = 0;
        g_pngd_thread_param[idxChn].enGetThreadCtrl = THREAD_CTRL_START;
        if (g_performance > 0) {
            ret = pthread_create(&ptr_pngd_get_tid[idxChn], 0,
                                 pngd_get_pic_performance, (void *)&g_pngd_thread_param[idxChn]);
            if (ret != 0) {
                SAMPLE_PRT("[chn:%u] create get pic thread Fail, ret = %d \n", idxChn, ret);
                ptr_pngd_get_tid[idxChn] = 0;
                g_pngd_thread_param[idxChn].enGetThreadCtrl = THREAD_CTRL_STOP;
                return ret;
            }
        } else {
            ret = pthread_create(&ptr_pngd_get_tid[idxChn], 0,
                                 pngd_get_pic, (void *)&g_pngd_thread_param[idxChn]);
            if (ret != 0) {
                SAMPLE_PRT(" chn %u, create get pic thread Fail, ret = %d \n", idxChn, ret);
                ptr_pngd_get_tid[idxChn] = 0;
                g_pngd_thread_param[idxChn].enGetThreadCtrl = THREAD_CTRL_STOP;
                return ret;
            }
        }
    }

    return 0;
}

void pngd_stop_get_pic()
{
    SAMPLE_PRT(" PNGD stop get pic threads\n");
    for (uint32_t idxChn = g_start_channel; idxChn < g_chn_num + g_start_channel; idxChn++) {
        g_pngd_thread_param[idxChn].enGetThreadCtrl = THREAD_CTRL_STOP;
    }
}

int32_t pngd_create()
{
    uint32_t idxChn = 0;
    int32_t  s32Ret = HI_SUCCESS;
    hi_pngd_chn_attr stChnAttr[PNGD_MAX_CHN_NUM]{};

    for (idxChn = g_start_channel; idxChn < g_chn_num + g_start_channel; idxChn++) {
        pthread_mutex_init(&g_out_buffer_pool_lock[idxChn], NULL);
    }

    for (idxChn = g_start_channel; idxChn < g_chn_num + g_start_channel; idxChn++) {
        CHECK_CHN_RET(hi_mpi_pngd_create_chn(idxChn, &stChnAttr[idxChn]), idxChn, "hi_mpi_pngd_create_chn");

        s32Ret = snprintf(g_pngd_thread_param[idxChn].cFileName, FILE_NAME_LEN - 1, "%s", g_input_file_name);
        if (s32Ret <= 0) {
            SAMPLE_PRT(" snprintf fileNmae fail for %#x!\n", s32Ret);
            return HI_FAILURE;
        } else {
            s32Ret = HI_SUCCESS;
        }

        g_pngd_thread_param[idxChn].s32ChnId = idxChn;
        g_pngd_thread_param[idxChn].s32IntervalTime = 1000;
        g_pngd_thread_param[idxChn].u64PtsInit = 0;
        g_pngd_thread_param[idxChn].enSendThreadCtrl = THREAD_CTRL_INIT;
        g_pngd_thread_param[idxChn].enGetThreadCtrl  = THREAD_CTRL_INIT;
        g_pngd_thread_param[idxChn].perfStreamBuf = nullptr;
        g_pngd_thread_param[idxChn].s32CircleSend = g_send_circle;
        g_pngd_thread_param[idxChn].bWholeDirDecode = g_whole_dir_decode;
        g_pngd_thread_param[idxChn].s32MilliSec = 1000;
        g_pngd_thread_param[idxChn].enPixFormat = g_pixel_format;
    }

    return HI_SUCCESS;
}

int32_t pngd_destroy()
{
    for (uint32_t idxChn = g_start_channel; idxChn < g_chn_num + g_start_channel; idxChn++) {
        CHECK_CHN_RET(hi_mpi_pngd_destroy_chn(idxChn), idxChn, "hi_mpi_pngd_destroy_chn");

        // Release PNG stream mem
        if (g_pngd_thread_param[idxChn].perfStreamBuf != nullptr) {
            hi_mpi_dvpp_free((void*)g_pngd_thread_param[idxChn].perfStreamBuf);
            g_pngd_thread_param[idxChn].perfStreamBuf = nullptr;
        }

        // Release RGB picture mem
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
    if (g_context) {
        aclrtDestroyContext(g_context);
        g_context = nullptr;
        aclrtResetDevice(0);
        aclFinalize();
    }
}

int32_t deinit_pngd_mod()
{
    int32_t ret = HI_SUCCESS;

    ret = pngd_destroy();
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("pngd_destroy failed. ret code:%#x\n", ret);
    }

    ret = hi_mpi_sys_exit();
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_sys_exit failed. ret code:%#x\n", ret);
    }

    destroy_acl_device();

    return ret;
}