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

#include "graph/operator.h"
#include "register/register.h"
#include "json.hpp"

using namespace ge;
using json = nlohmann::json;
namespace domi {
void GetAttrListFromJson(json& attr, std::vector<int32_t>& val) {
  int num = attr["ints"].size();
  for (int i = 0; i < num; ++i) {
    val.push_back(attr["ints"][i].get<int32_t>());
  }
}

Status ParseOnnxParamsCol2im(const ge::Operator& op_src, ge::Operator& op_dest) {
  AscendString attrs_string;
  std::vector<int32_t> kernel_size;
  std::vector<int32_t> dilation;
  std::vector<int32_t> padding;
  std::vector<int32_t> stride;
  if (ge::GRAPH_SUCCESS == op_src.GetAttr("attribute", attrs_string)) {
    json attrs = json::parse(attrs_string.GetString());
    for (json& attr : attrs["attribute"]) {
      if (attr["name"] == "kernel_size") {
        GetAttrListFromJson(attr, kernel_size);
      } else if (attr["name"] == "dilation") {
        GetAttrListFromJson(attr, dilation);
      } else if (attr["name"] == "padding") {
        GetAttrListFromJson(attr, padding);
      } else if (attr["name"] == "stride") {
        GetAttrListFromJson(attr, stride);
      }
    }
  }

  if (kernel_size.empty() || dilation.empty() || padding.empty() || stride.empty()) {
    return FAILED;
  }
  op_dest.SetAttr("kernel_size", kernel_size);
  op_dest.SetAttr("dilation", dilation);
  op_dest.SetAttr("padding", padding);
  op_dest.SetAttr("stride", stride);
  return SUCCESS;
}

REGISTER_CUSTOM_OP("Col2im")
    .FrameworkType(ONNX)
    .OriginOpType({ge::AscendString("ai.onnx::8::Col2im"),
                   ge::AscendString("ai.onnx::9::Col2im"),
                   ge::AscendString("ai.onnx::10::Col2im"),
                   ge::AscendString("ai.onnx::11::Col2im"),
                   ge::AscendString("ai.onnx::12::Col2im"),
                   ge::AscendString("ai.onnx::13::Col2im")})
    .ParseParamsByOperatorFn(ParseOnnxParamsCol2im)
    .ImplyType(ImplyType::TVM);
}  // domi