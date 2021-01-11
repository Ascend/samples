/**
* @file batchnorm_tiling.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include <cstdint>
#include <algorithm>
#include <iostream>
#include "batchnorm_tiling.h"

using namespace std;

const int32_t SUCCESS = 1;
const uint32_t UINT_MAX = 4294967295;
const uint32_t TILING_0_UB_SIZE = 192*1024;
const uint32_t TILING_1_UB_SIZE = 112*1024;
const uint32_t TILING_2_UB_SIZE = 112*1024;
const uint32_t MAX_W_SIZE = 1024;
const uint32_t MAX_H_SIZE = 1024;
const uint32_t MAX_C_SIZE = 1024;
const uint32_t UB_SIZE = 1024*240;
typedef enum{
    TILING_MODE_1 = 1,
    TILING_MODE_2 = 2,
    TILING_MODE_3 = 3,
}TilingMode;

int32_t CeilDiv(int32_t n, int32_t factor)
{
    if (factor == 0) {
        return 0;
    }
    return (n + factor - 1) / factor;
}

int32_t CeilDivMul(int32_t n, int32_t factor)
{
    if (factor == 0) {
        return 0;
    }
    return factor*((n + factor - 1) / factor);
}

/**
 * batchnorm tiling to
 * @limit: 1. n,c,h,w is the format of NCHW
 *         2. inDtype, outDtype is the multiple of 16, and the dtype is float16
 *         3. mini, core num is 2
 *         4. only support to select one batchnorm schedule
 *         5. not set kernel workspace
 */
void BatchNormTiling(int64_t n, int64_t c, int64_t h, int64_t w,
                     aclDataType inDtype, TilingMode &mode)
{
    int64_t lenwh = w*h;
    int64_t bytesize = 0;
    if(inDtype == ACL_FLOAT16){
        bytesize = 2;
    }
    else{
        cout << "[ERROR] not fp16 datatype not support yet." << endl;
        return;
    }

    if (c > lenwh && lenwh == CeilDivMul(lenwh, 16) && \
        (CeilDivMul(c, 16) * lenwh * bytesize) <= TILING_0_UB_SIZE) {
        mode = TILING_MODE_1;
    }
    else{
        if(lenwh*bytesize > TILING_2_UB_SIZE){
            mode = TILING_MODE_2;
        }
        else if((lenwh*bytesize > TILING_2_UB_SIZE/2) && (lenwh*bytesize < TILING_2_UB_SIZE)){
            mode = TILING_MODE_2;
        }
        else{
            mode = TILING_MODE_3;
        }
    }

    //cout << "[INFO] select tiling mode is:" << mode << endl;
    return;
}

bool CheckDimEqual(int64_t in_n, int64_t in_c, int64_t in_h, int64_t in_w,
                   int64_t out_n, int64_t out_c, int64_t out_h, int64_t out_w) {
    if(in_n != out_n || in_c != out_c || in_h != out_h || in_w != out_w){
        cout << "[ERROR] [SelectAclopBatchNorm] [SelectAclopBatchNorm] " \
                "input dim is not identity with output dims." << endl;
        return false;
    }
    return true;
}

bool CheckDimSize(int64_t in_c, int64_t in_h, int64_t in_w) {
    if(in_c > MAX_C_SIZE || in_h > MAX_H_SIZE || in_w > MAX_W_SIZE){
        cout << "[ERROR] [SelectAclopBatchNorm] C/H/W not support exceed 1024" << endl;
        return false;
    }
    return true;
}

bool GetDims(int64_t& in_n, int64_t& in_c, int64_t& in_h, int64_t& in_w, int64_t& gamma_c, int64_t& beta_c,
             const aclTensorDesc *const inputDesc[], const aclTensorDesc *const outputDesc[],
             int64_t& out_n, int64_t& out_c, int64_t& out_h, int64_t& out_w) {
    if ((aclGetTensorDescDimV2(inputDesc[0], 0, &in_n) != ACL_SUCCESS) ||
        (aclGetTensorDescDimV2(inputDesc[0], 1, &in_c) != ACL_SUCCESS) ||
        (aclGetTensorDescDimV2(inputDesc[0], 2, &in_h) != ACL_SUCCESS) ||
        (aclGetTensorDescDimV2(inputDesc[0], 3, &in_w) != ACL_SUCCESS)) {
        cout << "[ERROR] [SelectAclopBatchNorm] get [N, C, H, W] from inputdesc failed" << endl;
        return false;
    }

    if (aclGetTensorDescDimV2(inputDesc[1], 0, &gamma_c) != ACL_SUCCESS) {
        cout << "[ERROR] [SelectAclopBatchNorm] get gamma failed" << endl;
        return false;
    }

    if (aclGetTensorDescDimV2(inputDesc[2], 0, &beta_c) != ACL_SUCCESS) {
        cout << "[ERROR] [SelectAclopBatchNorm] get beta failed" << endl;
        return false;
    }

    if ((aclGetTensorDescDimV2(outputDesc[0], 0, &out_n) != ACL_SUCCESS) ||
        (aclGetTensorDescDimV2(outputDesc[0], 1, &out_c) != ACL_SUCCESS) ||
        (aclGetTensorDescDimV2(outputDesc[0], 2, &out_h) != ACL_SUCCESS) ||
        (aclGetTensorDescDimV2(outputDesc[0], 3, &out_w) != ACL_SUCCESS)) {
        cout << "[ERROR] [SelectAclopBatchNorm] get [N, C, H, W] from outputdesc failed" << endl;
        return false;
    }

    return true;
}

bool Validation(int numInputs, const aclTensorDesc *const inputDesc[],
                int numOutputs, const aclTensorDesc *const outputDesc[]) {

    int64_t in_n = 0, in_c = 0, in_h = 0, in_w = 0;
    int64_t out_n = 0, out_c = 0, out_h = 0, out_w = 0;
    int64_t gamma_c = 0, beta_c = 0;
    int64_t totallength = 0;
    aclDataType inDtype;
    TilingMode tilingmode;
    BatchNormParam bnparam;

    if (numInputs != 3) {
        cout << "[ERROR] [SelectAclopBatchNorm] numInputs should be 3" << endl;
        return false;
    }

    // get params of input and output
    if (!GetDims(in_n, in_c, in_h, in_w, gamma_c, beta_c, inputDesc, outputDesc, out_n, out_c, out_h, out_w)) {
        cout << "[ERROR] [SelectAclopBatchNorm] get params failed" << endl;
        return false;
    }

    // validation:check input [N, C, H, W]
    inDtype = aclGetTensorDescType(inputDesc[0]);

    if(in_n != 1){
        cout << "[ERROR] [SelectAclopBatchNorm] batch N only support 1" << endl;
        return false;
    }

    if(CheckDimSize(in_c, in_h, in_w) == false){
        return false;
    }

    if(inDtype != ACL_FLOAT16){
        cout << "[ERROR] [SelectAclopBatchNorm] only support fp16" << endl;
        return false;
    }

    totallength = in_n*in_c*in_h*in_w*2;
    if(totallength >= UINT_MAX){
        cout << "[ERROR] [SelectAclopBatchNorm] totalsize support exceed UINT_MAX" << endl;
        return false;
    }
    // validation ends

    if(gamma_c != in_c){
        cout << "[ERROR] [SelectAclopBatchNorm] gamma channels is not same as input!" << endl;
        return false;
    }

    if(beta_c != in_c){
        cout << "[ERROR] [SelectAclopBatchNorm] beta channels is not same as input!" << endl;
        return false;
    }

    // get output dtype
    if (numOutputs != 1) {
        cout << "[ERROR] [SelectAclopBatchNorm]outputNum should be 1." << endl;
        return false;
    }

    if(CheckDimEqual(in_n, in_c, in_h, in_w, out_n, out_c, out_h, out_w) == false) {
        return false;
    }

    return true;
}
extern "C" aclError SelectAclopBatchNorm(int numInputs, const aclTensorDesc *const inputDesc[],
                                         int numOutputs, const aclTensorDesc *const outputDesc[],
                                         const aclopAttr *opAttr, aclopKernelDesc *aclopKernelDesc)
{
    if (Validation(numInputs, inputDesc, numOutputs, outputDesc) == false) {
        return ACL_ERROR_FAILURE;
    }
    int64_t in_n = 0, in_c = 0, in_h = 0, in_w = 0;
    int64_t out_n = 0, out_c = 0, out_h = 0, out_w = 0;
    int64_t gamma_c = 0, beta_c = 0;
    int64_t totallength = 0;
    aclDataType inDtype;
    TilingMode tilingmode;
    BatchNormParam bnparam;
    bnparam.param1 = 0;
    bnparam.param2 = 0;
    bnparam.param3 = 0;
    bnparam.param4 = 0;
    bnparam.param5 = 0;
    bnparam.param6 = 0;
    bnparam.param7 = 0;
    bnparam.param8 = 0;
    bnparam.param9 = 0;
    bnparam.param10 = 0;

    // get params of input and output
    if (!GetDims(in_n, in_c, in_h, in_w, gamma_c, beta_c, inputDesc, outputDesc, out_n, out_c, out_h, out_w)) {
        cout << "[ERROR] [SelectAclopBatchNorm] get params failed" << endl;
        return false;
    }

    inDtype = aclGetTensorDescType(inputDesc[0]);

    totallength = in_n*in_c*in_h*in_w*2;

    cout << "[INFO] [SelectAclopBatchNorm] input shape is: "
         << in_n << " " << in_c << " " << in_h << " " << in_w << endl;

    bnparam.input_n = in_n;
    bnparam.input_c = in_c;
    bnparam.input_h = in_h;
    bnparam.input_w = in_w;
    bnparam.in_datatype = 1;
    bnparam.output_n = out_n;
    bnparam.output_c = out_c;
    bnparam.output_h = out_h;
    bnparam.output_w = out_w;
    bnparam.out_datatype = 1;
    bnparam.gamma_c = gamma_c;
    bnparam.gamma_datatype = 1;
    bnparam.beta_c = beta_c;
    bnparam.beta_datatype = 1;

    /////////////////////////////
    BatchNormTiling(in_n, in_c, in_h, in_w, inDtype, tilingmode);

    string scheduleFlag = "";
    // TILING_MODE_1 means C >= H*W, tiling C
    if(tilingmode == TILING_MODE_1){
        scheduleFlag = "tiling_mode_1__kernel0";
        int32_t chn_num = MAX_C_SIZE;
        int32_t align_16 = CeilDivMul(in_w*in_h, 16);
        int32_t total_use_ub = chn_num*align_16*2*2 + chn_num*2;
        if(total_use_ub <= UB_SIZE){
            // input_wh
            bnparam.param1 = in_h*in_w;
            // align_wh
            bnparam.param2 = CeilDivMul(bnparam.param1, 16);
            // align_c
            bnparam.param3 = CeilDivMul(in_c, 16);
        }
        else{
            // input_wh
            bnparam.param1 = in_h*in_w;
            // iterwh_align16
            bnparam.param2 = CeilDiv(bnparam.param1, 16);
            // repeat_alignc
            bnparam.param3 = CeilDiv(in_c, 16);
            // align_wh
            bnparam.param4 = CeilDivMul(bnparam.param1, 16);
            // align_c
            bnparam.param5 = CeilDivMul(in_c, 16);
            bnparam.param6 = CeilDiv(bnparam.param4, 16) - 1;
        }
    }
    else if(tilingmode == TILING_MODE_2){  // TILING_MODE_2 means C <= H*W and H*W*C(==2) >138KB
        scheduleFlag = "tiling_mode_2__kernel0";
        // input_wh
        bnparam.param1 = in_h*in_w;
        // align_wh
        bnparam.param2 = CeilDivMul(bnparam.param1, 16);
        int32_t tiling_num = (TILING_1_UB_SIZE / 4);
        // iter_ceil
        bnparam.param3 = CeilDiv(bnparam.param1, tiling_num);
        if(bnparam.param3 < 2){
            // iter_mask128
            bnparam.param4 = tiling_num / 128;
            // repeat_mask128
            bnparam.param5 = bnparam.param4/255;
            // repeat_res_mask128
            bnparam.param6 = bnparam.param4 - bnparam.param5*255;
            // res_mask128
            bnparam.param7 = tiling_num -  bnparam.param4*128;
        }
        else{
            // iter_h
            bnparam.param4 = CeilDiv(in_h, bnparam.param3);
            // iter_align16
            bnparam.param5 = CeilDivMul(bnparam.param4*in_w, 16);
            // iter_res_align16
            bnparam.param6 = CeilDivMul(((in_h - bnparam.param4*(bnparam.param3 - 1))*in_w) , 16);
            // repeat_mask128
            bnparam.param7 = CeilDivMul(bnparam.param4*in_w, 128);
            // repeat_res_mask128
            bnparam.param8 = CeilDivMul(((in_h - bnparam.param4*(bnparam.param3 - 1))*in_w), 128);
            bnparam.param9 = in_h - bnparam.param4*bnparam.param3;
        }
    }
    else if(tilingmode == TILING_MODE_3){  // TILING_MODE_2 means C <= H*W and H*W*C(>=2) <= 138KB
        // input_wh
        bnparam.param1 = in_h*in_w;
        // align_wh
        bnparam.param2 = CeilDivMul(bnparam.param1, 16);
        int32_t tiling_num = (TILING_2_UB_SIZE / 4);
        // single_chnum
        bnparam.param3 = tiling_num / bnparam.param2;
        if(bnparam.param3 > in_c){
            bnparam.param3 = in_c;
        }
        // iter_cnum
        bnparam.param4 = (int32_t)(in_c / bnparam.param3);
        if(bnparam.param4 <= 0){
            bnparam.param4 = 1;
        }
        if(bnparam.param1 == bnparam.param2){
            // repeat_length
            bnparam.param5 = bnparam.param1*bnparam.param3;
            // repeat_mask
            bnparam.param6 = bnparam.param1 / 128;
            // repeat_res_mask
            bnparam.param7 = bnparam.param1 -  bnparam.param6*128;
            // res_mask
            bnparam.param8 = 128*bnparam.param6;
            // res_ch_num
            bnparam.param9 = in_c - bnparam.param3*(bnparam.param4);
            // res_repeat_length
            bnparam.param10 = bnparam.param9*bnparam.param1;
        }
        else{
            // repeat_mask128
            bnparam.param5 = bnparam.param1 / 128;
            // repeat_res_mask
            bnparam.param6 = bnparam.param1 - bnparam.param5*128;
        }
        scheduleFlag = "tiling_mode_3__kernel0";
    }
    else{
        cout << "[ERROR] [SelectAclopBatchNorm] tiling mode not support yet! " \
             << tilingmode << endl;
        return ACL_ERROR_FAILURE;
    }

    cout << "[INFO] [SelectAclopBatchNorm] tiling mode is: " << tilingmode << endl;
    cout << "[INFO] [SelectAclopBatchNorm] param is:" << bnparam.param1 << "," \
         << bnparam.param2 << "," << bnparam.param3 << "," << bnparam.param4 << "," \
         << bnparam.param5 << "," << bnparam.param6 << "," << bnparam.param7 << "," \
         << bnparam.param8 << "," << bnparam.param9 << "," << bnparam.param10 << endl;
    // core number
    aclopSetKernelArgs(aclopKernelDesc, scheduleFlag.c_str(),  1, &bnparam, sizeof(BatchNormParam));

    return ACL_SUCCESS;
}

