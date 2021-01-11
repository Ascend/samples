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

#include "graph/operator.h"
#include "register/register.h"
#include <string>
#include <vector>

using namespace ge;
namespace domi {

/* Permute Attr */
const char ATTR_ORDER[] = "order";
Status ParseParamsPermute(const ge::Operator& op_src, ge::Operator& op_dest)
{
    // if op_src get required attr failed, need to return Failed
    // if op_src get optional attr failed, need to return Failed or set a default value
    vector<int64_t> orders;
    if (ge::GRAPH_SUCCESS == op_src.GetAttr(ATTR_ORDER, orders)){
        op_dest.SetAttr(ATTR_ORDER, orders);
    }
    return SUCCESS;
}

// register Permute op info to GE
REGISTER_CUSTOM_OP("PermuteTik")
  .FrameworkType(CAFFE)
  .OriginOpType("Permute")
  .ParseParamsByOperatorFn(ParseParamsPermute);
}  // namespace domi

