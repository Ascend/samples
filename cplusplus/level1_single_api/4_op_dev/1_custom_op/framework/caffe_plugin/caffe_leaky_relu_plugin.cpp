/* Copyright (C) 2019. Huawei Technologies Co., Ltd. All rights reserved.
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
#include "graph/operator.h"

using namespace ge;

namespace domi
{
Status ParseParamsLeakyRelu(const ge::Operator& op_src, ge::Operator& op_dest)
{
 
  // trans op_src to op_dest
  // if op_src get required attr failed, need to return Failed
  // if op_src get optional attr failed, need to return Failed or set a default value
  float negative_slope;
  if (ge::GRAPH_SUCCESS == op_src.GetAttr("negative_slope", negative_slope)){
        op_dest.SetAttr("negative_slope", negative_slope);
    }else{
        op_dest.SetAttr("negative_slope", float(0));
    }
  return SUCCESS;
}

REGISTER_CUSTOM_OP("LeakyReluDemo")
    .FrameworkType(CAFFE)  // type: CAFFE, TENSORFLOW
    .OriginOpType("LeakyReLUDemo")  // name in caffe module
    .ParseParamsByOperatorFn(ParseParamsLeakyRelu)  // AutoMappingFn for Tensorflow, ParseParamsFn need to realize for caffe
    .ImplyType(ImplyType::TVM);
}  // namespace domi
