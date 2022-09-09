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
#include <string>
#include <sys/types.h>
#include "sample_comm.h"

using namespace std;

int32_t sample_comm_vpc_batch_crop_resize_paste(FuncInput funcInput)
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
    uint32_t destLeftOffset = funcInput.g_vpc_attribute.destLeftOffset;
    uint32_t destTopOffset = funcInput.g_vpc_attribute.destTopOffset;
    uint32_t multiCount = funcInput.g_vpc_attribute.multiCount;
    uint32_t srcPicNum = funcInput.g_vpc_attribute.srcPicNum;
    hi_vpc_chn chnId = funcInput.chnId;

    hi_vpc_pic_info *batchInputPic[MAX_MULTI_COUNT] = {nullptr};
    uint32_t regionCount[MAX_MULTI_COUNT];
    uint32_t totalRegionCount = 0;
    hi_vpc_pic_info outputPic;
    uint32_t dstBufferSize[MAX_MULTI_COUNT];

    // malloc input hi_vpc_pic_info memory
    for (uint32_t i = 0; i < srcPicNum; i++) {
        batchInputPic[i] = static_cast<hi_vpc_pic_info*>(malloc(sizeof(hi_vpc_pic_info)));
        if (batchInputPic[i] == nullptr) {
            SAMPLE_PRT("malloc batchInputPic[%u] failed.\n", i);
            for (uint32_t j = 0; j < i; ++j) {
                free(batchInputPic[j]);
                batchInputPic[j] = nullptr;
            }
            return HI_FAILURE;
        }
        batchInputPic[i]->picture_address = nullptr;
    }
    // conf input picture
    int32_t ret = HI_SUCCESS;
    for (uint32_t i = 0; i < srcPicNum; i++) {
        batchInputPic[i]->picture_width = width;
        batchInputPic[i]->picture_height = height;
        batchInputPic[i]->picture_format = static_cast<hi_pixel_format>(format);
        configure_stride_and_buffer_size(*batchInputPic[i]);

        string batchFileName = std::string(inputFileName) + "_" + std::to_string(i);
        ret = prepare_input_data(*batchInputPic[i], const_cast<char*>(batchFileName.c_str()));
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("prepare input data (%u) failed!\n", i);
            for (uint32_t j = 0; j < i; ++j) {
                dvpp_mem_free(batchInputPic[j]->picture_address);
                batchInputPic[j]->picture_address = nullptr;
            }
            for (uint32_t j = 0; j < srcPicNum; j++) {
                free(batchInputPic[i]);
                batchInputPic[i] = nullptr;
            }
            return ret;
        }

        // in this sample, the number of crop regions in input pictures are the same
        regionCount[i] = multiCount;
        totalRegionCount += regionCount[i];
    }

    // configure output region
    hi_vpc_crop_resize_paste_region cropResizePasteInfos[MAX_MULTI_COUNT];
    for (uint32_t i = 0; i < totalRegionCount; i++) {
        outputPic.picture_width = outWidth;
        outputPic.picture_height  = outHeight;
        outputPic.picture_format = static_cast<hi_pixel_format>(outFormat);
        dstBufferSize[i] = configure_stride_and_buffer_size(outputPic);

        ret = dvpp_mem_malloc(&outputPic.picture_address, outputPic.picture_buffer_size);
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("output buffer(%u) alloc failed!\n", i);
            for (uint32_t j = 0; j < srcPicNum; j++) {
                dvpp_mem_free(batchInputPic[j]->picture_address);
                batchInputPic[j]->picture_address = nullptr;
                free(batchInputPic[j]);
                batchInputPic[j] = nullptr;
            }
            for (uint32_t j = 0; j < i; j++) {
                dvpp_mem_free(cropResizePasteInfos[j].dest_pic_info.picture_address);
                cropResizePasteInfos[j].dest_pic_info.picture_address = nullptr;
            }
            return HI_FAILURE;
        }

        // After crop and resize, the picture is pasted to the output picture, which is set to 0 here.
        memset_buffer(outputPic);

        // This array can be configured by what you want to crop, resize and paste. All regions are the same here.
        cropResizePasteInfos[i].dest_pic_info = outputPic;
        cropResizePasteInfos[i].crop_region.left_offset = cropX;
        cropResizePasteInfos[i].crop_region.top_offset = cropY;
        cropResizePasteInfos[i].crop_region.crop_width = cropWidth;
        cropResizePasteInfos[i].crop_region.crop_height = cropHeight;
        cropResizePasteInfos[i].resize_info.resize_width = resizeWidth;
        cropResizePasteInfos[i].resize_info.resize_height = resizeHeight;
        cropResizePasteInfos[i].resize_info.interpolation = interpolation;
        cropResizePasteInfos[i].dest_left_offset = destLeftOffset;
        cropResizePasteInfos[i].dest_top_offset = destTopOffset;
    }

    // start to call hi_mpi_vpc_batch_crop_resize_paste interface
    uint32_t taskID = 0;
    ret = hi_mpi_vpc_batch_crop_resize_paste(chnId, (const hi_vpc_pic_info**)batchInputPic, srcPicNum,
        cropResizePasteInfos, regionCount, &taskID, -1);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_vpc_batch_crop_resize_paste failed, ret = %#x!\n", ret);
        for (uint32_t j = 0; j < srcPicNum; j++) {
            dvpp_mem_free(batchInputPic[j]->picture_address);
            batchInputPic[j]->picture_address = nullptr;
            free(batchInputPic[j]);
            batchInputPic[j] = nullptr;
        }
        for (uint32_t j = 0; j < totalRegionCount; j++) {
            dvpp_mem_free(cropResizePasteInfos[j].dest_pic_info.picture_address);
            cropResizePasteInfos[j].dest_pic_info.picture_address = nullptr;
        }
        return HI_FAILURE;
    }

    // asign the taskID that you get from the last interface to get the process result
    uint32_t taskIDResult = taskID;
    ret = hi_mpi_vpc_get_process_result(chnId, taskIDResult, -1);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_vpc_get_process_result failed, ret = %#x!\n", ret);
        for (uint32_t j = 0; j < srcPicNum; j++) {
            dvpp_mem_free(batchInputPic[j]->picture_address);
            batchInputPic[j]->picture_address = nullptr;
            free(batchInputPic[j]);
            batchInputPic[j] = nullptr;
        }
        for (uint32_t j = 0; j < totalRegionCount; j++) {
            dvpp_mem_free(cropResizePasteInfos[j].dest_pic_info.picture_address);
            cropResizePasteInfos[j].dest_pic_info.picture_address = nullptr;
        }
        return HI_FAILURE;
    }

    // write output file
    for (uint32_t j = 0; j < totalRegionCount; j++) {
        string outFileName = std::string(outputFileName) + "_region" + std::to_string(j);
        ret = handle_output_data(cropResizePasteInfos[j].dest_pic_info, dstBufferSize[j],
            const_cast<char *>(outFileName.c_str()));
        if (ret != HI_SUCCESS) {
            SAMPLE_PRT("handle_output_data failed!\n");
            break;
        }
    }

    // free input and output buffer
    for (uint32_t j = 0; j < srcPicNum; j++) {
        dvpp_mem_free(batchInputPic[j]->picture_address);
        batchInputPic[j]->picture_address = nullptr;
        free(batchInputPic[j]);
        batchInputPic[j] = nullptr;
    }
    for (uint32_t i = 0; i < totalRegionCount; i++) {
        dvpp_mem_free(cropResizePasteInfos[i].dest_pic_info.picture_address);
        cropResizePasteInfos[i].dest_pic_info.picture_address = nullptr;
    }
    return ret;
}

