/* Copyright (C) 2022. Huawei Technologies Co., Ltd. All rights reserved.
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
#include "array_ops.h"
#include "elewise_calculation_ops.h"

namespace domi {
Status ParseParamsAddn(const ge::Operator& op_src, ge::Operator& op_dest) {
  // 1.设置PartitionCall节点(op_dest)的输入、输出个数和原始节点(op_src)一致
  ge::Operator op_ori = const_cast<ge::Operator&>(op_src);
  std::string in_name = "args";
  std::string in_value = "in_num";
  std::string out_name = "output";
  std::string out_value = "out_num";
  op_ori.SetAttr(in_value, 3);
  op_ori.SetAttr(out_value, 1);
  DynamicInputOutputInfo in_values(kInput, in_name.c_str(), in_name.size(), in_value.c_str(), in_value.size());
  DynamicInputOutputInfo out_values(kOutput, out_name.c_str(), out_name.size(), out_value.c_str(), out_value.size());
  AutoMappingByOpFnDynamic(op_ori, op_dest, {in_values, out_values});

  // 如果有属性需要从原始节点(op_src)继承，可以设置到op_dest中

  // 2.设置属性"original_type"为OriginOpType
  op_dest.SetAttr("original_type", "ai.onnx::11::AddN");
  return SUCCESS;
}

static Status ParseOpToGraphAddn(const ge::Operator &op, ge::Graph &graph) {
  // Data标识输入顺序，index顺序和原始节点(op)的输入顺序对应
  auto data_0 = ge::op::Data().set_attr_index(0);
  auto data_1 = ge::op::Data().set_attr_index(1);
  auto data_2 = ge::op::Data().set_attr_index(2);
  
  auto add0 = ge::op::Add("add0")
    .set_input_x1(data_0)
    .set_input_x2(data_1);
  
  auto add1 = ge::op::Add("add1")
    .set_input_x1(data_2)
    .set_input_x2(add0);

  std::vector<ge::Operator> inputs{data_0, data_1, data_2};
  // output设置和原始节点(op)保持一致
  std::vector<std::pair<ge::Operator, std::vector<size_t>>> output_indexs;
  output_indexs.emplace_back(add1, vector<std::size_t>{0});
  graph.SetInputs(inputs).SetOutputs(output_indexs);
  return SUCCESS;
}

REGISTER_CUSTOM_OP("PartitionedCall")
    .FrameworkType(ONNX)
    .OriginOpType("ai.onnx::11::AddN")
    .ParseParamsByOperatorFn(ParseParamsAddn)
    .ParseOpToGraphFn(ParseOpToGraphAddn)
    .ImplyType(ImplyType::TVM);
}  // namespace domi
