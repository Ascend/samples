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
#include <unistd.h>
#include <signal.h>
#include <getopt.h>
#include <exception>
#include <iostream>
#include <vector>
#include <thread>
#include "sample_comm.h"

using namespace std;

typedef int (*TEST_FUNC)(FuncInput);
typedef int (*TEST_ENTRY)(TEST_FUNC);
vector<pair<TEST_ENTRY, TEST_FUNC>> test_entry;

VpcAttr g_vpc_attribute;
aclrtRunMode g_run_mode;

void sample_vpc_handle_sig(int32_t signo)
{
    if (SIGINT == signo || SIGTSTP == signo || SIGTERM == signo) {
            hi_mpi_sys_exit();
        SAMPLE_PRT("\033[0;31mprogram exit abnormally!\033[0;39m\n");
    }

    exit(0);
}

void get_option_1(int c)
{
    switch (c) {
        case '2':
            g_vpc_attribute.paddingTopSize = atoi(optarg);
            break;
        case '3':
            g_vpc_attribute.paddingBottomSize = atoi(optarg);
            break;
        case '4':
            g_vpc_attribute.paddingLeftSize = atoi(optarg);
            break;
        case '5':
            g_vpc_attribute.paddingRightSize = atoi(optarg);
            break;
        case '6':
            g_vpc_attribute.paddingValR = atof(optarg);
            break;
        case '7':
            g_vpc_attribute.paddingValG = atof(optarg);
            break;
        case '8':
            g_vpc_attribute.paddingValB = atof(optarg);
            break;
        case 'Q':
            g_vpc_attribute.filterLevel = atof(optarg);
            break;
        case 'U':
            g_vpc_attribute.divisor = atof(optarg);
            break;
        case 'P':
            g_vpc_attribute.paddingMode = (hi_vpc_bord_type)atoi(optarg);
            break;
        case 'X':
            g_vpc_attribute.inWidthAlign = atoi(optarg);
            break;
        case 'Y':
            g_vpc_attribute.inHeightAlign = atoi(optarg);
            break;
        case 'Z':
            g_vpc_attribute.outWidthAlign = atoi(optarg);
            break;
        case 'V':
            g_vpc_attribute.outHeightAlign = atoi(optarg);
            break;
        case 'l':
            g_vpc_attribute.queueLen = atoi(optarg);
            break;
        default:
            SAMPLE_PRT("this is default!\n");
        break;
    }
}

void get_option(int argc, char **argv)
{
    while (1) {
        int32_t option_index = 0;
        struct option long_options[] = {
            {"img_width", 1, 0, 'w'},
            {"img_height", 1, 0, 'h'},
            {"in_format", 1, 0, 'f'},
            {"in_bitwidth", 1, 0, 'b'},
            {"chn_num", 1, 0, 'c'},
            {"queue_len", 1, 0, 'l'},
            {"thread_num", 1, 0, 't'},
            {"out_width", 1, 0, 'd'},
            {"out_height", 1, 0, 'g'},
            {"out_format", 1, 0, 'e'},
            {"crop_x", 1, 0, 'm'},
            {"crop_y", 1, 0, 'r'},
            {"crop_width", 1, 0, 's'},
            {"crop_height", 1, 0, 'p'},
            {"resize_width", 1, 0, 'u'},
            {"resize_height", 1, 0, 'v'},
            {"fx", 1, 0, 'x'},
            {"fy", 1, 0, 'y'},
            {"interpolation", 1, 0, 'i'},
            {"dest_left_offset", 1, 0, 'L'},
            {"dest_top_offset", 1, 0, 'T'},
            {"test_type", 1, 0, 'C'},
            {"loop", 1, 0, 'D'},
            {"in_image_file", 1, 0, 'F'},
            {"out_image_file", 1, 0, 'O'},
            {"write_file", 1, 0, 'W'},
            {"multi_count", 1, 0, 'M'},
            {"write_file_index", 1, 0, 'I'},
            {"pyramid_padding_mode", 1, 0, '0'},
            {"pyramid_padding_value", 1, 0, '1'},
            {"top_padding_size", 1, 0, '2'},
            {"bottom_padding_size", 1, 0, '3'},
            {"left_padding_size", 1, 0, '4'},
            {"right_padding_size", 1, 0, '5'},
            {"padding_val_r", 1, 0, '6'},
            {"padding_val_g", 1, 0, '7'},
            {"padding_val_b", 1, 0, '8'},
            {"filter_level", 1, 0, 'Q'},
            {"divisor", 1, 0, 'U'},
            {"padding_mode", 1, 0, 'P'},
            {"in_image_num", 1, 0, 'N'},
            {"in_width_align", 1, 0, 'X'},
            {"in_height_align", 1, 0, 'Y'},
            {"out_width_align", 1, 0, 'Z'},
            {"out_height_align", 1, 0, 'V'},
            {nullptr, 1, 0, 'U'},
        };

        int32_t c = getopt_long(argc, argv,
            "w:h:f:b:c:l:t:d:g:e:m:r:s:p:u:v:x:y:i:L:T:C:D:F:O:W:M:I:0:1:2:3:4:5:6:7:8:Q:U:P:N:X:Y:Z:V",
            long_options, &option_index);
        if (c == -1) {
            break;
        }
        switch (c) {
            case 'w':
                g_vpc_attribute.width = atoi(optarg);
                break;
            case 'h':
                g_vpc_attribute.height = atoi(optarg);
                break;
            case 'f':
                g_vpc_attribute.format = atoi(optarg);
                break;
            case 'b':
                g_vpc_attribute.bitwidth = atoi(optarg);
                break;
            case 'c':
                g_vpc_attribute.chnNum = atoi(optarg);
                break;
            case 't':
                g_vpc_attribute.threadNum = atoi(optarg);
                break;
            case 'd':
                g_vpc_attribute.outWidth = atoi(optarg);
                break;
            case 'g':
                g_vpc_attribute.outHeight = atoi(optarg);
                break;
            case 'e':
                g_vpc_attribute.outFormat = atoi(optarg);
                break;
            case 'm':
                g_vpc_attribute.cropX = atoi(optarg);
                break;
            case 'r':
                g_vpc_attribute.cropY = atoi(optarg);
                break;
            case 's':
                g_vpc_attribute.cropWidth = atoi(optarg);
                break;
            case 'p':
                g_vpc_attribute.cropHeight = atoi(optarg);
                break;
            case 'u':
                g_vpc_attribute.resizeWidth = atoi(optarg);
                break;
            case 'v':
                g_vpc_attribute.resizeHeight = atoi(optarg);
                break;
            case 'x':
                g_vpc_attribute.fx = strtod(optarg, NULL);
                break;
            case 'y':
                g_vpc_attribute.fy = strtod(optarg, NULL);
                break;
            case 'i':
                g_vpc_attribute.interpolation = atoi(optarg);
                break;
            case 'L':
                g_vpc_attribute.destLeftOffset = atoi(optarg);
                break;
            case 'T':
                g_vpc_attribute.destTopOffset = atoi(optarg);
                break;
            case 'C':
                g_vpc_attribute.testType = atoi(optarg);
                break;
            case 'D':
                g_vpc_attribute.loop = atoi(optarg);
                break;
            case 'F':
                strcpy(g_vpc_attribute.inputFileName, optarg);
                break;
            case 'O':
                strcpy(g_vpc_attribute.outputFileName, optarg);
                break;
            case 'W':
                g_vpc_attribute.writeFile = atoi(optarg);
                break;
            case 'M':
                g_vpc_attribute.multiCount = atoi(optarg);
                break;
            case 'I':
                g_vpc_attribute.writeFileIndex = atoi(optarg);
                break;
            case '0':
                g_vpc_attribute.pyramidPaddingMode = atoi(optarg);
                break;
            case '1':
                g_vpc_attribute.pyramidPaddingValue = atoi(optarg);
                break;
            case 'N':
                g_vpc_attribute.srcPicNum = atoi(optarg);
                break;
            default:
                get_option_1(c);
                break;
        }
    }
    return;
}

int32_t test_entry_single_chnl(TEST_FUNC test_func)
{
    int32_t s32Ret = get_run_mode();
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    s32Ret = acl_init();
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    s32Ret = hi_mpi_sys_init();
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_sys_init failed, ret = %#x!\n", s32Ret);
        acl_deinit();
        return s32Ret;
    }

    // create vpc channel
    hi_vpc_chn chnId;
    hi_vpc_chn_attr stChnAttr {};
    stChnAttr.attr = g_vpc_attribute.queueLen;
    s32Ret = hi_mpi_vpc_sys_create_chn(&chnId, &stChnAttr);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("Call hi_mpi_vpc_sys_create_chn failed, ret = %#x\n", s32Ret);
        hi_mpi_sys_exit();
        acl_deinit();
        return s32Ret;
    }

    // By default, threadNum is 1. It can be set to the number of thread you need.
    vector<thread> Thread;
    FuncInput funcInput;
    funcInput.chnId = chnId;
    funcInput.g_vpc_attribute = g_vpc_attribute;
    uint32_t currentThreadNum = 0;
    try {
        for (uint32_t i = 0; i < g_vpc_attribute.threadNum; i++) {
            Thread.push_back(std::thread(test_func, funcInput));
            ++currentThreadNum;
        }
    } catch (exception& e) {
        cout << e.what() << endl;
        cout << "actual channel number is " << currentThreadNum << endl;
        g_vpc_attribute.threadNum = currentThreadNum;
    }

    for (auto& t : Thread) {
        t.join();
    }

    // after callling vpc interfaces, destroy the channel.
    s32Ret = hi_mpi_vpc_destroy_chn(chnId);
    if (s32Ret != HI_SUCCESS) {
        SAMPLE_PRT("Call hi_mpi_vpc_destroy_chn failed, ret = %#x\n", s32Ret);
    }

    hi_mpi_sys_exit();
    acl_deinit();

    if (s32Ret == HI_SUCCESS) {
        SAMPLE_PRT("program exit normally!\n");
    } else {
        SAMPLE_PRT("program exit abnormally!\n");
    }
    return s32Ret;
}

void init_test_entry_single_chnl()
{
    SAMPLE_PRT("test_type:%u, ", g_vpc_attribute.testType);
    switch (g_vpc_attribute.testType) {
        case 1: // test_type 1: crop
            SAMPLE_PRT("crop\n");
            test_entry.push_back(make_pair(test_entry_single_chnl, sample_comm_vpc_crop));
            break;
        case 2: // test_type 2: resize
            SAMPLE_PRT("resize\n");
            test_entry.push_back(make_pair(test_entry_single_chnl, sample_comm_vpc_resize));
            break;
        case 3: // test_type 3: cropResize
            SAMPLE_PRT("cropResize\n");
            test_entry.push_back(make_pair(test_entry_single_chnl, sample_comm_vpc_crop_resize));
            break;
        case 4: // test_type 4: cropResizePaste
            SAMPLE_PRT("cropResizePaste\n");
            test_entry.push_back(make_pair(test_entry_single_chnl, sample_comm_vpc_crop_resize_paste));
            break;
        case 5: // test_type 5: cropResizeMakeBorder
            SAMPLE_PRT("cropResizeMakeBorder\n");
            test_entry.push_back(make_pair(test_entry_single_chnl, sample_comm_vpc_crop_resize_make_border));
            break;
        case 6: // test_type 6: convertColor
            SAMPLE_PRT("convertColor\n");
            test_entry.push_back(make_pair(test_entry_single_chnl, sample_comm_vpc_convert_color));
            break;
        case 7: // test_type 7: copyMakeBorder
            SAMPLE_PRT("copyMakeBorder\n");
            test_entry.push_back(make_pair(test_entry_single_chnl, sample_comm_vpc_copy_make_border));
            break;
        case 8: // test_type 8: pyrDown
            SAMPLE_PRT("pyrDown\n");
            test_entry.push_back(make_pair(test_entry_single_chnl, sample_comm_vpc_pyrdown));
            break;
        case 9: // test_type 9: calHist
            SAMPLE_PRT("calcHist\n");
            test_entry.push_back(make_pair(test_entry_single_chnl, sample_comm_vpc_calc_hist));
            break;
        case 10: // test_type 10: equalizeHist
            SAMPLE_PRT("equalizeHist\n");
            test_entry.push_back(make_pair(test_entry_single_chnl, sample_comm_vpc_equalize_hist));
            break;
        case 11: // test_type 11: batchCropResizePaste
            SAMPLE_PRT("batchCropResizePaste\n");
            test_entry.push_back(make_pair(test_entry_single_chnl, sample_comm_vpc_batch_crop_resize_paste));
            break;
        case 12: // test_type 12: batchCropResizeMakeBorder
            SAMPLE_PRT("batchCropResizeMakeBorder\n");
            test_entry.push_back(make_pair(test_entry_single_chnl, sample_comm_vpc_batch_crop_resize_make_border));
            break;
        case 20: // test_type 20: preProcess
            SAMPLE_PRT("preProcess\n");
            pre_process(g_vpc_attribute);
            break;
        default:
            break;
    }
}

int main(int argc, char *argv[])
{
    if (argc < 4) { // if argc is less than 4
        SAMPLE_PRT("\nInvalid input!\n");
        return HI_FAILURE;
    }

    SAMPLE_PRT("signal input!\n");
    signal(SIGINT, sample_vpc_handle_sig);
    signal(SIGTERM, sample_vpc_handle_sig);

    // get the options
    get_option(argc, &(*argv));
    // init function entry
    init_test_entry_single_chnl();
    vector<pair<TEST_ENTRY, TEST_FUNC>>::iterator iter_begin = test_entry.begin();
    for (vector<pair<TEST_ENTRY, TEST_FUNC>>::iterator pTest = iter_begin; pTest != test_entry.end(); ++pTest) {
        ((*pTest).first)((*pTest).second);
    }

    return HI_SUCCESS;
}