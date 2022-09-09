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
#include "sample_comm.h"

using namespace std;

int32_t sample_comm_vpc_crop_resize(FuncInput funcInput)
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
    uint32_t outWidth = funcInput.g_vpc_attribute.outWidth;
    uint32_t outHeight = funcInput.g_vpc_attribute.outHeight;
    uint32_t outFormat = funcInput.g_vpc_attribute.outFormat;
    uint32_t cropX = funcInput.g_vpc_attribute.cropX;
    uint32_t cropY = funcInput.g_vpc_attribute.cropY;
    uint32_t cropWidth = funcInput.g_vpc_attribute.cropWidth;
    uint32_t cropHeight = funcInput.g_vpc_attribute.cropHeight;
    uint32_t resizeWidth = funcInput.g_vpc_attribute.resizeWidth;
    uint32_t resizeHeight = funcInput.g_vpc_attribute.resizeHeight;
    int32_t interpolation = funcInput.g_vpc_attribute.interpolation;
    uint32_t multiCount = funcInput.g_vpc_attribute.multiCount;
    hi_vpc_chn chnId = funcInput.chnId;

    hi_vpc_crop_resize_region cropResizeRegionInfos[MAX_MULTI_COUNT];
    uint32_t dstBufferSize[MAX_MULTI_COUNT];

    // configure input picture
    hi_vpc_pic_info inputPic;
    inputPic.picture_width = width;
    inputPic.picture_height = height;
    inputPic.picture_format = static_cast<hi_pixel_format>(format);
    configure_stride_and_buffer_size(inputPic);

    // malloc input buffer
    int32_t ret = prepare_input_data(inputPic, inputFileName);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("prepare input data failed!\n");
        return ret;
    }

    // malloc and configure output buffer
    hi_vpc_pic_info outputPic;
    for (uint32_t i = 0; i < multiCount; i++) {
        outputPic.picture_width = outWidth;
        outputPic.picture_height  = outHeight;
        outputPic.picture_format = static_cast<hi_pixel_format>(outFormat);
        dstBufferSize[i] = configure_stride_and_buffer_size(outputPic);

        // malloc output buffer
        ret = dvpp_mem_malloc(&outputPic.picture_address, outputPic.picture_buffer_size);
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("output buffer(%u) alloc failed!\n", i);
            dvpp_mem_free(inputPic.picture_address);
            inputPic.picture_address = nullptr;
            for (uint32_t j = 0; j < i; j++) {
                dvpp_mem_free(cropResizeRegionInfos[j].dest_pic_info.picture_address);
                cropResizeRegionInfos[j].dest_pic_info.picture_address = nullptr;
            }
            return HI_FAILURE;
        }
        memset_buffer(outputPic);

        // This array can be configured by what you want to crop and resize. All regions are the same here
        cropResizeRegionInfos[i].dest_pic_info = outputPic;
        cropResizeRegionInfos[i].crop_region.left_offset = cropX;
        cropResizeRegionInfos[i].crop_region.top_offset = cropY;
        cropResizeRegionInfos[i].crop_region.crop_width = cropWidth;
        cropResizeRegionInfos[i].crop_region.crop_height = cropHeight;
        cropResizeRegionInfos[i].resize_info.resize_width = resizeWidth;
        cropResizeRegionInfos[i].resize_info.resize_height = resizeHeight;
        cropResizeRegionInfos[i].resize_info.interpolation = interpolation;
    }

    // start to call hi_mpi_vpc_crop_resize interface
    uint32_t taskID = 0;
    ret = hi_mpi_vpc_crop_resize(chnId, &inputPic, cropResizeRegionInfos, multiCount, &taskID, -1);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_vpc_crop_resize failed, ret = %#x!\n", ret);
        dvpp_mem_free(inputPic.picture_address);
        inputPic.picture_address = nullptr;
        for (uint32_t i = 0; i < multiCount; i++) {
            dvpp_mem_free(cropResizeRegionInfos[i].dest_pic_info.picture_address);
            cropResizeRegionInfos[i].dest_pic_info.picture_address = nullptr;
        }
        return HI_FAILURE;
    }

    // asign the taskID that you get from the last interface to get the process result
    uint32_t taskIDResult = taskID;
    ret = hi_mpi_vpc_get_process_result(chnId, taskIDResult, -1);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_vpc_get_process_result failed, ret = %#x!\n", ret);
        dvpp_mem_free(inputPic.picture_address);
        inputPic.picture_address = nullptr;
        for (uint32_t i = 0; i < multiCount; i++) {
            dvpp_mem_free(cropResizeRegionInfos[i].dest_pic_info.picture_address);
            cropResizeRegionInfos[i].dest_pic_info.picture_address = nullptr;
        }
        return HI_FAILURE;
    }

    // write the first picture in the cropResizeRegionInfos array
    uint32_t picIndex = 1;
    ret = handle_output_data(cropResizeRegionInfos[picIndex - 1].dest_pic_info, dstBufferSize[picIndex - 1],
        outputFileName);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("handle_output_data failed!\n");
    }

    // free input and output buffer
    dvpp_mem_free(inputPic.picture_address);
    inputPic.picture_address = nullptr;
    for (uint32_t i = 0; i < multiCount; i++) {
        dvpp_mem_free(cropResizeRegionInfos[i].dest_pic_info.picture_address);
        cropResizeRegionInfos[i].dest_pic_info.picture_address = nullptr;
    }

    return ret;
}

