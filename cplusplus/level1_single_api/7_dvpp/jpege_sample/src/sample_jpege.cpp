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
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <getopt.h>
#include "sample_comm.h"
#include <iostream>
#include <map>

// Input file name
char g_input_file_name[500] = "";
// Output file name
char g_output_file_name[500] = "";
uint32_t g_set_outfile = 0;
// Channel width, Value range:[32, 8192]
uint32_t g_chn_width = 0;
// Channel height, Value range:[32, 8192]
uint32_t g_chn_height = 0;
// Picture width, Value range:[32, 8192]
uint32_t g_width = 128;
// Picture height, Value range:[32, 8192]
uint32_t g_height = 128;
// Input file format, 1(yuv420sp nv12),2(yuv420sp nv21),7(yuv422packed YUYV),8(yuv422packed UYVY)
// 9(yuv422packed YVYU), 10(yuv422packed VYUY)
uint32_t g_format = 1;
// The number of channel, Value range:[1, MAX_JPEGE_CHN]
uint32_t g_chn_num = 1;
// PIcture bitwidth, Only support 8bit
uint32_t g_bitwidth = 8;
// Encoding quality, Value range:[1, 100]
uint32_t g_level = 100;
// Save Code Stream, 0:do not save, Non-zero:save
uint32_t g_save = 1;
// The start channel id, Value range:[0, MAX_JPEGE_CHN - 1]
uint32_t g_chn_num_start = 0;
// 0:function test, Non-zero:performance test
uint32_t g_performance = 0;
// Number of performance statistics cycles
uint32_t g_per_count = 300;
// Number of applied input buffer
uint32_t g_mem_count = 100;
// Notify user to obtain streams, 0:Single-thread and multi-channel, Non-zero:Single-thread single-channel
uint32_t g_select_one_thread = 1;
// 1: synchronize encode mode; 0: asynchronous mode
uint32_t g_is_syn_enc = 0;
// Frame count of input file
uint32_t g_frameCount = 1;
// Zero copy
uint32_t g_isZeroCopy = 0;

aclrtContext g_context = NULL;
// ACL_HOST or ACL_DEVICE
aclrtRunMode g_run_mode = ACL_HOST;

/*
* function:    jpege_usage(int argc, char **argv)
* Description: usage help
* input:       无
* output:      无
* return:      无
* others:      无
*/
void jpege_usage()
{
    printf("\n/*********************************************************/\n");
    printf("Usage :\n");
    printf("\t: ./jpege_demo --in_image_file 1920x1080_YUV420_SP.yuv"
           " --img_width 1920 --img_height 1080 --in_format 1 --chn_num 1\n");
    printf("\t\n");
    printf("\t in_image_file: input image name.\n");
    printf("\t img_width: input image width.\n");
    printf("\t img_height: input image height.\n");
    printf("\t in_format: input YUV format.\n");
    printf("\t\t 1: HI_PIXEL_FORMAT_YUV_SEMIPLANAR_420\n");
    printf("\t\t 2: HI_PIXEL_FORMAT_YVU_SEMIPLANAR_420\n");
    printf("\t\t 7: HI_PIXEL_FORMAT_YUYV_PACKED_422\n");
    printf("\t\t 8: HI_PIXEL_FORMAT_UYVY_PACKED_422\n");
    printf("\t\t 9: HI_PIXEL_FORMAT_YVYU_PACKED_422\n");
    printf("\t\t 10: HI_PIXEL_FORMAT_VYUY_PACKED_422\n");
    printf("\t chn_num: jpege channel num.\n");
    printf("/*********************************************************/\n\n");
}

/*
* function:    get_option(int argc, char **argv)
* Description: Command Line Parameters
* input:       argc:total number of parameters in the command line,
               argv:array of pointers to command-line string parameters
* output:      none
* return:      none
* others:      none
*/
void get_option(int argc, char **argv)
{
    while (1) {
        hi_s32 optionIndex = 0;
        hi_s32 c;
        struct option longOptions[] {
            {"chn_width"      , 1, 0, 'W'},
            {"chn_height"     , 1, 0, 'H'},
            {"img_width"      , 1, 0, 'w'},
            {"img_height"     , 1, 0, 'h'},
            {"in_format"      , 1, 0, 'f'},
            {"in_bitwidth"    , 1, 0, 'b'},
            {"chn_num"        , 1, 0, 'c'},
            {"in_image_file"  , 1, 0, 'i'},
            {"out_image_file" , 1, 0, 'o'},
            {"level"          , 1, 0, 'l'},
            {"save"           , 1, 0, 's'},
            {"chn_start"       , 1, 0, 'B'},
            {"performance"    , 1, 0, 'p'},
            {"per_count"       , 1, 0, 'O'},
            {"mem_count"       , 1, 0, 'Q'},
            {"one_thread"      , 1, 0, 'T'},
            {"sync_enc"        , 1, 0, 'S'},
            {"frame_count"     , 1, 0, 'n'},
            {"zero_copy"       , 1, 0, 'z'},
            {NULL, 0, 0, 0}
        };
        c = getopt_long(argc, argv, "W:H:w:h:f:b:c:i:o:l:s:B:p:O:Q:T:S:n:z", longOptions, &optionIndex);
        if (c == -1) {
            break;
        }
        switch (c) {
            case 'W':
                g_chn_width = atoi(optarg);
                break;
            case 'H':
                g_chn_height = atoi(optarg);
                break;
            case 'w':
                g_width = atoi(optarg);
                break;
            case 'h':
                g_height = atoi(optarg);
                break;
            case 'f':
                g_format = atoi(optarg);
                break;
            case 'b':
                g_bitwidth = atoi(optarg);
                break;
            case 'c':
                g_chn_num = atoi(optarg);
                break;
            case 'i':
                strcpy(g_input_file_name, optarg);
                break;
            case 'o':
                strcpy(g_output_file_name, optarg);
                g_set_outfile = 1;
                break;
            case 'l':
                g_level = atoi(optarg);
                break;
            case 's':
                g_save = atoi(optarg);
                break;
            case 'B':
                g_chn_num_start = atoi(optarg);
                break;
            case 'p':
                g_performance = atoi(optarg);
                break;
            case 'O':
                g_per_count = atoi(optarg);
                break;
            case 'Q':
                g_mem_count = atoi(optarg);
                break;
            case 'T':
                g_select_one_thread = atoi(optarg);
                break;
            case 'S':
                g_is_syn_enc = atoi(optarg);
                break;
            case 'n':
                g_frameCount = atoi(optarg);
                break;
            case 'z':
                g_isZeroCopy = atoi(optarg);
                break;
            default:
                SAMPLE_PRT("bad arg!\n");
                break;
        }
    }
    return;
}

/*
* function:    jpege_sys_init(void)
* Description: System Initialization
* input:       none
* output:      Indicates whether the system is initialized
* return:      HI_SUCCESS:success, HI_FAILURE:fail
* others:      none
*/
int32_t jpege_sys_init()
{
    hi_s32 ret = 0;

    aclError acl_ret = aclInit(NULL);
    if (acl_ret != ACL_SUCCESS) {
        SAMPLE_PRT("aclInit fail with %d.\n", acl_ret);
        return HI_FAILURE;
    }

    acl_ret = aclrtSetDevice(0);
    if (acl_ret != ACL_SUCCESS) {
        SAMPLE_PRT("aclrtSetDevice(0) fail with %d.\n", acl_ret);
        return HI_FAILURE;
    }

    acl_ret = aclrtCreateContext(&g_context, 0);
    if (acl_ret != ACL_SUCCESS) {
        SAMPLE_PRT("acl create context failed with %d.\n", acl_ret);
        return HI_FAILURE;
    }

    acl_ret = aclrtGetCurrentContext(&g_context);
    if (acl_ret != ACL_SUCCESS) {
        SAMPLE_PRT("get current context failed with %d \n", acl_ret);
        return HI_FAILURE;
    }
    acl_ret = aclrtGetRunMode(&g_run_mode);
    if (acl_ret != ACL_SUCCESS) {
        SAMPLE_PRT("get run mode failed with %d \n", acl_ret);
        return HI_FAILURE;
    }
    ret = hi_mpi_sys_init();
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_sys_init failed ret:0x%x \n", ret);
        return HI_FAILURE;
    }

    return HI_SUCCESS;
}

/*
* function:    jpege_sys_exit(void)
* Description: System deinitialization
* input:       none
* output:      Indicates whether the system is going to exit
* return:      HI_SUCCESS:success, HI_FAILURE:fail
* others:      none
*/
hi_s32 jpege_sys_exit()
{
    hi_s32 ret = 0;
    aclError acl_ret = ACL_SUCCESS;

    ret = hi_mpi_sys_exit();
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_sys_exit failed ret:0x%x.\n", ret);
    }

    acl_ret = aclrtDestroyContext(g_context);
    if (acl_ret != ACL_SUCCESS) {
        SAMPLE_PRT("destroy context failed with %d.\n", acl_ret);
    }
    acl_ret = aclrtResetDevice(0);
    if (acl_ret != ACL_SUCCESS) {
        SAMPLE_PRT("reset device(0) fail with %d.\n", acl_ret);
    }
    acl_ret = aclFinalize();
    if (acl_ret != ACL_SUCCESS) {
        SAMPLE_PRT("finalize acl failed with %d.\n", acl_ret);
    }

    return HI_SUCCESS;
}

/*
* function:    jpeg_encode(void)
* Description: JPEGE Encoder Configuration
* input:       none
* output:      Check whether the encoder is configured successfully
* return:      HI_SUCCESS success, Error Code:fail
* others:      none
*/
hi_s32 jpeg_encode(void)
{
    hi_s32 i;
    hi_s32 ret;
    hi_video_size size[MAX_JPEGE_CHN];
    hi_s32 chnNum;
    hi_venc_chn vencChn[MAX_JPEGE_CHN];
    hi_bool supportDc = HI_FALSE;

    if (g_chn_num_start + g_chn_num > MAX_JPEGE_CHN) {
        SAMPLE_PRT("g_chn_num_start:%d + g_chn_num:%d > MAX_JPEGE_CHN: %d \n",
            g_chn_num_start, g_chn_num, MAX_JPEGE_CHN);
        goto EXIT_JPEG_STOP;
    }

    chnNum = g_chn_num;
    if (chnNum > MAX_JPEGE_CHN) {
        SAMPLE_PRT("g_chn_num = %d, should between [0, %d]!\n", chnNum, MAX_JPEGE_CHN);
        goto EXIT_JPEG_STOP;
    }

    // If the channel width and height are not setted,
    // the image width and height are used as the channel width and height by default
    if ((g_chn_width == 0) || (g_chn_height == 0)) {
        g_chn_width = g_width;
        g_chn_height = g_height;
    }

    for (i = 0; i < chnNum; ++i) {
        size[i].width = g_chn_width;
        size[i].height = g_chn_height;
    }

    for (i = 0; i < chnNum; ++i) {
        vencChn[i] = g_chn_num_start + i; // chn no
    }

    // Initialize the system
    ret = jpege_sys_init();
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("sample_venc_sys_init failed!\n");
        return ret;
    }

    HiSampleVencSendFrameInput inputPara; // input parameters
    inputPara.width = g_width;
    inputPara.height = g_height;
    inputPara.align = JPEGE_ALIGN;
    inputPara.pixelFormat = (hi_pixel_format)g_format;
    inputPara.bitWidth = HI_DATA_BIT_WIDTH_8;
    inputPara.cmpMode = HI_COMPRESS_MODE_NONE;

    snprintf(inputPara.inputFileName, 64, "%s", g_input_file_name);

    // Create channel and enable the channel to receive input pictures
    for (i = 0; i < chnNum; ++i) {
        ret = jpege_snap_start(vencChn[i], &size[i], supportDc, g_level);
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("Venc Start failed for %#x in chnl %d!\n", ret, vencChn[i]);
            goto EXIT_JPEG_STOP;
        }
    }
    if (g_is_syn_enc) {
        // synchonize mode: one in one out
        ret = jpege_start_sync_enc(vencChn, chnNum, g_performance, &inputPara);
        jpege_snap_stop(vencChn, chnNum);
        jpege_sys_exit();
        return ret;
    }
    // start get stream
    ret = jpege_start_get_stream(vencChn, chnNum, g_performance, g_save);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("Start jpege_start_get_stream failed!\n");
        goto EXIT_JPEG_STOP;
    }

    // start send stream
    ret = jpege_start_send_frame(vencChn, chnNum, g_performance, &inputPara);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("snap send frame failed %#x!\n", ret);
        goto EXIT_JPEG_STOP;
    }

    // Extend the time and exit the main function after all encoding is complete
    ret = wait_encoder_complete(g_chn_num_start, chnNum);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("wait encode complete failed!\n");
        goto EXIT_JPEG_STOP;
    }

    jpege_stop_send_frame(chnNum);
    jpege_stop_get_stream(chnNum);
    jpege_snap_stop(vencChn, chnNum);
    jpege_sys_exit();
    return HI_SUCCESS;

EXIT_JPEG_STOP:
    jpege_stop_send_frame(chnNum);
    jpege_stop_get_stream(chnNum);
    jpege_snap_stop(vencChn, chnNum);
    jpege_sys_exit();
    return HI_FAILURE;
}

/*
* function:    main(int argc, char *argv[])
* Description: JPEGE DDK Sample Main Function
* input:       argc:Total number of parameters in the command line, argv[]:argc parameters
* output:      Check whether the program is started successfully
* return:      HI_SUCCESS:success, HI_FAILURE:fail
* others:      none
*/
int main(int argc, char *argv[])
{
    if (argc < 2) {
        jpege_usage();
        return HI_FAILURE;
    }

    get_option(argc, &(*argv));

    hi_s32 ret = jpeg_encode();
    if (ret == HI_SUCCESS) {
        printf("program exit normally!\n");
    } else {
        printf("program exit abnormally!\n");
    }
    return ret;
}
