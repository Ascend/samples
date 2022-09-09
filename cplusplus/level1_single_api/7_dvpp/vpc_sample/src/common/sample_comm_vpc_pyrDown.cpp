/**
 *  Copyright [2022] Huawei Technologies Co., Ltd
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

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/types.h>
#include <string>
#include "sample_comm.h"

using namespace std;

int32_t sample_comm_vpc_pyrdown(FuncInput funcInput)
{
    aclError aclRet = aclrtSetCurrentContext(g_context);
    if (aclRet != ACL_SUCCESS) {
        SAMPLE_PRT("set current context failed, ret = %d\n", aclRet);
        return HI_FAILURE;
    }

    char inputFileName[FILE_NAME_LEN];
    strcpy(inputFileName, funcInput.g_vpc_attribute.inputFileName);
    char outputFileName[FILE_NAME_LEN];
    strcpy(outputFileName, funcInput.g_vpc_attribute.outputFileName);
    uint32_t width = funcInput.g_vpc_attribute.width;
    uint32_t height = funcInput.g_vpc_attribute.height;
    uint32_t format = funcInput.g_vpc_attribute.format;
    hi_vpc_bord_type pyramidPaddingMode = static_cast<hi_vpc_bord_type>(funcInput.g_vpc_attribute.pyramidPaddingMode);
    double pyramidPaddingValue = funcInput.g_vpc_attribute.pyramidPaddingValue;
    hi_vpc_chn chnId = funcInput.chnId;

    // the pyramid down sampling interface can generate up to 4 pictures
    uint32_t filterLevel = funcInput.g_vpc_attribute.filterLevel;
    // configure how you fill the border of the down sampling pictures
    hi_vpc_make_border_info makeBorderInfo;
    makeBorderInfo.border_type = static_cast<hi_vpc_bord_type>(pyramidPaddingMode);
    makeBorderInfo.scalar_value.val[0] = pyramidPaddingValue;
    /*
    These are common filters for down sampling. In this sample, we use gaussFilter.
    sobelXFilter5x5[5][5] = {0,  0, 0, 0, 0,
                             0, -1, 0, 1, 0,
                             0, -2, 0, 2, 0,
                             0, -1, 0, 1, 0,
                             0,  0, 0, 0, 0};
    medianFilter5x5[5][5] = {1, 1, 1, 1, 1,
                             1, 1, 1, 1, 1,
                             1, 1, 1, 1, 1,
                             1, 1, 1, 1, 1,
                             1, 1, 1, 1, 1};
    laplacianFilter[5][5] = {0, 0, 0, 0, 0,
                             0, 1, 1, 1, 0,
                             0, 1,-8, 1, 0,
                             0, 1, 1, 1, 0,
                             0, 0, 0, 0, 0};
    */
    int8_t gaussFilter[5][5] = {1, 4,  6,  4,  1, // 5x5
                                4, 16, 24, 16, 4,
                                6, 24, 36, 24, 6,
                                4, 16, 24, 16, 4,
                                1, 4,  6,  4,  1};
    uint16_t divisor = funcInput.g_vpc_attribute.divisor; // divisor after convolution
    uint32_t dstBufferSize[4];

    // configure input picture
    hi_vpc_pic_info inputPic;
    inputPic.picture_width = width;
    inputPic.picture_height = height;
    inputPic.picture_format = static_cast<hi_pixel_format>(format);
    configure_stride_and_buffer_size(inputPic);

    // The width and height are half of the size of the upper layer.
    hi_vpc_pic_info outputPic[4] = {NULL};
    outputPic[0].picture_width = width % 2 ? (width / 2) + 1 : (width / 2);
    outputPic[0].picture_height = height % 2 ? (height / 2) + 1 : (height / 2);
    outputPic[0].picture_format = HI_PIXEL_FORMAT_YUV_400;
    dstBufferSize[0] = configure_stride_and_buffer_size(outputPic[0]);

    outputPic[1].picture_width = width % 4 ? (width / 4) + 1 : (width / 4);
    outputPic[1].picture_height = height % 4 ? (height / 4) + 1 : (height / 4);
    outputPic[1].picture_format = HI_PIXEL_FORMAT_YUV_400;
    dstBufferSize[1] = configure_stride_and_buffer_size(outputPic[1]);

    outputPic[2].picture_width = width % 8 ? (width / 8) + 1 : (width / 8);
    outputPic[2].picture_height = height % 8 ? (height / 8) + 1 : (height / 8);
    outputPic[2].picture_format = HI_PIXEL_FORMAT_YUV_400;
    dstBufferSize[2] = configure_stride_and_buffer_size(outputPic[2]);

    outputPic[3].picture_width = width % 16 ? (width / 16) + 1 : (width / 16);
    outputPic[3].picture_height = height % 16 ? (height / 16) + 1 : (height / 16);
    outputPic[3].picture_format = HI_PIXEL_FORMAT_YUV_400;
    dstBufferSize[3] = configure_stride_and_buffer_size(outputPic[3]);

    int32_t ret = prepare_input_data(inputPic, inputFileName);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("prepare input data failed!\n");
        return ret;
    }

    // malloc output buffer
    for (uint32_t i = 0; i < filterLevel; i++) {
        ret = dvpp_mem_malloc(&outputPic[i].picture_address, outputPic[i].picture_buffer_size);
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("output buffer(%u) alloc failed!\n", i);
            dvpp_mem_free(inputPic.picture_address);
            inputPic.picture_address = nullptr;
            for (uint32_t j = i - 1; j >= 0; --j) {
                dvpp_mem_free(outputPic[j].picture_address);
                outputPic[j].picture_address = nullptr;
            }
            return HI_FAILURE;
        }
        memset_buffer(outputPic[i]);
    }

    // start to call hi_mpi_vpc_pyrdown interface
    uint32_t taskID = 0;
    ret = hi_mpi_vpc_pyrdown(chnId, &inputPic, outputPic, filterLevel, gaussFilter, divisor, makeBorderInfo,
        &taskID, -1);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_vpc_pyrdown failed, ret = %#x!\n", ret);
        dvpp_mem_free(inputPic.picture_address);
        inputPic.picture_address = nullptr;
        for (uint32_t i = 0; i < filterLevel; i++) {
            dvpp_mem_free(outputPic[i].picture_address);
            outputPic[i].picture_address = nullptr;
        }
        return HI_FAILURE;
    }

    // use the taskID that you get from the interface to get the process result
    uint32_t taskIDResult = taskID;
    ret = hi_mpi_vpc_get_process_result(chnId, taskIDResult, -1);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_vpc_get_process_result failed, ret = %#x!\n", ret);
        dvpp_mem_free(inputPic.picture_address);
        inputPic.picture_address = nullptr;
        for (uint32_t i = 0; i < filterLevel; i++) {
            dvpp_mem_free(outputPic[i].picture_address);
            outputPic[i].picture_address = nullptr;
        }
        return HI_FAILURE;
    }

    // write output file
    string baseName = outputFileName;
    string pyraFileName;
    for (uint32_t i = 0; i < filterLevel; i++) {
        pyraFileName = baseName + ".level_" + to_string(i);
        ret = handle_output_data(outputPic[i], dstBufferSize[i], const_cast<char *>(pyraFileName.c_str()));
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("handle_output_data failed!\n");
            break;
        }
    }

    // free input and output buffer
    dvpp_mem_free(inputPic.picture_address);
    inputPic.picture_address = nullptr;
    for (uint32_t i = 0; i < filterLevel; i++) {
        dvpp_mem_free(outputPic[i].picture_address);
        outputPic[i].picture_address = nullptr;
    }
    return ret;
}

