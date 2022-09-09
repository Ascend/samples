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
#include <string>
#include "sample_comm.h"

using namespace std;

int32_t sample_comm_vpc_calc_hist(FuncInput funcInput)
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
    hi_vpc_chn chnId = funcInput.chnId;

    // configure input picture
    hi_vpc_pic_info inputPic;
    inputPic.picture_width = width;
    inputPic.picture_height = height;
    inputPic.picture_format = static_cast<hi_pixel_format>(format);
    configure_stride_and_buffer_size(inputPic);

    int32_t ret = prepare_input_data(inputPic, inputFileName);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("prepare input data failed!\n");
        return ret;
    }

    // the result of calcutaion is stored in histogram_result
    hi_vpc_histogram_config histogram_result;
    // start to call hi_mpi_vpc_calc_hist interface
    uint32_t taskID = 0;
    ret = hi_mpi_vpc_calc_hist(chnId, &inputPic, &histogram_result, &taskID, -1);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("HI_MPI_VPC_CalHist failed, ret = %#x!\n", ret);
        dvpp_mem_free(inputPic.picture_address);
        inputPic.picture_address = nullptr;
        return HI_FAILURE;
    }

    // use the taskID that you get from the interface to get the process result
    uint32_t taskIDResult = taskID;
    ret = hi_mpi_vpc_get_process_result(chnId, taskIDResult, -1);
    if (ret != HI_SUCCESS) {
        SAMPLE_PRT("hi_mpi_vpc_get_process_result failed, ret = %#x!\n", ret);
        dvpp_mem_free(inputPic.picture_address);
        inputPic.picture_address = nullptr;
        return HI_FAILURE;
    }

    /* write the histogram result to a file. In this sample, the file is written as :
        R/Y 0 : 46136
        G/U 0 : 0
        B/V 0 : 669
        R/Y 1 : 20564
        G/U 1 : 0
        B/V 1 : 68
    */
    FILE* resFp = fopen(outputFileName, "wt");
    if (!resFp) {
        SAMPLE_PRT("open %s failed\n", outputFileName);
        dvpp_mem_free(inputPic.picture_address);
        inputPic.picture_address = nullptr;
        return HI_FAILURE;
    }
    string str;
    int32_t len;
    int8_t ch1[2]; // 2 charactors to generate end of line
    ch1[0] = 13; // 13 is to generate end of line
    ch1[1] = 10; // 10 is to generate end of line
    string s1 = "R/Y ";
    string s2 = "G/U ";
    string s3 = "B/V ";
    string s4 = " : ";
    for (int32_t i = 0; i < 256; ++i) { // 256 elements
        str = s1 + to_string(i) + s4 + to_string(histogram_result.histogram_y_or_r[i]);
        len = str.size();
        fwrite(str.c_str(), 1, len, resFp);
        fwrite(ch1, 2, 1, resFp); // 2 charactors to generate end of line

        str = s2 + to_string(i) + s4 + to_string(histogram_result.histogram_u_or_g[i]);
        len = str.size();
        fwrite(str.c_str(), 1, len, resFp);
        fwrite(ch1, 2, 1, resFp); // 2 charactors to generate end of line

        str = s3 + to_string(i) + s4 + to_string(histogram_result.histogram_v_or_b[i]);
        len = str.size();
        fwrite(str.c_str(), 1, len, resFp);
        fwrite(ch1, 2, 1, resFp); // 2 charactors to generate end of line
    }

    // free input buffer
    dvpp_mem_free(inputPic.picture_address);
    inputPic.picture_address = nullptr;
    fclose(resFp);
    resFp = nullptr;
    return HI_SUCCESS;
}

