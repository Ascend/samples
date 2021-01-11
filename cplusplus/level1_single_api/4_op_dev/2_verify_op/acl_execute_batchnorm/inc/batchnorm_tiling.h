/**
* @file batchnorm_tiling.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#ifndef TVM_TOPK_FLOWTABLE_H
#define TVM_TOPK_FLOWTABLE_H
#include "acl/acl.h"

//align with 64
struct BatchNormParam {
    int32_t input_n;
    int32_t input_c;
    int32_t input_h;
    int32_t input_w;
    int32_t in_datatype;
    int32_t output_n;
    int32_t output_c;
    int32_t output_h;
    int32_t output_w;
    int32_t out_datatype;
    int32_t gamma_c;
    int32_t gamma_datatype;
    int32_t beta_c;
    int32_t beta_datatype;
    int32_t param1;
    int32_t param2;
    int32_t param3;
    int32_t param4;
    int32_t param5;
    int32_t param6;
    int32_t param7;
    int32_t param8;
    int32_t param9;
    int32_t param10;
};

extern "C" aclError SelectAclopBatchNorm(int numInputs, const aclTensorDesc *const inputDesc[],
                                         int numOutputs, const aclTensorDesc *const outputDesc[],
                                         const aclopAttr *opAttr, aclopKernelDesc *aclopKernelDesc);

#endif
// TVM_TOPK_FLOWTABLE_H



