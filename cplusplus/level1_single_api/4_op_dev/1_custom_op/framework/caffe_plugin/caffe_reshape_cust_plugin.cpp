/* Copyright (C) 2019. Huawei Technologies Co., Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the Apache License Version 2.0.
 * You may not use this file except in compliance with the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * Apache License for more details at
 * http://www.apache.org/licenses/LICENSE-2.0
 */
#include "register/register.h"
using namespace ge;

namespace domi
{
Status ParseParamsReshapeCust(const ge::Operator& op_src, ge::Operator& op_dest)
{
  return SUCCESS;
}

REGISTER_CUSTOM_OP("ReshapeCust")
    .FrameworkType(CAFFE)  // type: CAFFE, TENSORFLOW
    .OriginOpType("ReshapeCust")  // name in caffe module
    .ParseParamsByOperatorFn(ParseParamsReshapeCust)  // AutoMappingFn for Tensorflow, ParseParamsFn need to realize for caffe
    .ImplyType(ImplyType::AI_CPU);
}  // namespace domi
