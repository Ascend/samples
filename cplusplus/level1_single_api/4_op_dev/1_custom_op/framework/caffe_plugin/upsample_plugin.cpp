/* Copyright (C) 2018. Huawei Technologies Co., Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the Apache License Version 2.0.You may not use this file except in compliance with the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * Apache License for more details at
 * http://www.apache.org/licenses/LICENSE-2.0
 */

#include "register/register.h"
#include <memory>
#include <string>
#include <vector>
#include "graph/operator.h"

using namespace ge;
namespace domi
{
// Caffe ParseParams
Status ParseParams_Upsample(const ge::Operator& op_src, ge::Operator& op_dest)
{
    // trans op_src to op_dest
    // if op_src get required attr failed, need to return Failed
    // if op_src get optional attr failed, need to return Failed or set a default value
    float scale;
    if (ge::GRAPH_SUCCESS == op_src.GetAttr("scale", scale)){
        op_dest.SetAttr("scale", scale);
    }
    int stride;
    int stride_h;
    int stride_w;
    if (ge::GRAPH_SUCCESS == op_src.GetAttr("stride", stride)){
        op_dest.SetAttr("stride_h", stride);
        op_dest.SetAttr("stride_w", stride);
    }else{
        op_src.GetAttr("stride_h", stride_h);
        op_src.GetAttr("stride_w", stride_w);
        op_dest.SetAttr("stride_h", stride_h);
        op_dest.SetAttr("stride_w", stride_w);
    }
    
    return SUCCESS;
}
// test_reduction is the type name of the operator in the OM model. 
// It can be specified randomly and cannot be the same as an existing type name. It is case sensitive. 
REGISTER_CUSTOM_OP("UpsampleTik") 
    .FrameworkType(CAFFE)  // Enumerated type. The options are as follows: CAFFE, TENSORFLOW
    .OriginOpType("UpsampleTik")  // // Reduction indicates the type name of the operator in the caffe framework.
    .ParseParamsByOperatorFn(ParseParams_Upsample)  // AutoMappingFn indicates automatic mapping the parameters of op.
    .ImplyType(ImplyType::TVM);
}  // namespace domi
