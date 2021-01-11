/**
 * Copyright (C)  2019. Huawei Technologies Co., Ltd. All rights reserved.

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the Apache License Version 2.0.You may not use this file except in compliance with the License.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * Apache License for more details at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * @file assign.cpp
 *
 * @brief
 *
 * @version 1.0
 *
 */
#include "./assign.h"
#include <string>
#include <vector>

namespace ge {

bool IsUnknownRankShape(const std::vector<int64_t>& shape_vec) {
  if (shape_vec.size() == 1 && shape_vec[0] == -2) {
    return true;
  }
  return false;
}

bool IsUnKnownShape(const std::vector<int64_t>& shape_vec) {
  auto found = find(shape_vec.begin(), shape_vec.end(), -1);
  return found != shape_vec.end();
}

bool IsUnknown(const std::vector<int64_t>& shape_vec) {
  return (IsUnKnownShape(shape_vec) || IsUnknownRankShape(shape_vec));
}

void MakeUpShapeRange(const std::vector<int64_t>& shape, std::vector<std::pair<int64_t, int64_t>>& range) {
  if (IsUnknownRankShape(shape)) {
    return;
  }

  if (range.empty()) {
    for (size_t i = 0; i < shape.size(); i++) {
      if (shape[i] == -1) {
        range.push_back(std::pair<int64_t, int64_t>(1, -1));
      } else {
        range.push_back(std::pair<int64_t, int64_t>(shape[i], shape[i]));
      }
    }
  }
}

	
bool OneInOneOutDynamicInfer(const Operator& op,
                             const std::string& input_name,
                             const std::vector<std::string>& output_name_list) {
  // get input desc
  auto op_info = OpDescUtils::GetOpDescFromOperator(op);
  auto input_desc = op_info->MutableInputDesc(input_name);
  vector<int64_t> input_shape = input_desc->MutableShape().GetDims();
  DataType input_dtype = input_desc->GetDataType();

  if (IsUnknown(input_shape)) {
    std::vector<std::pair<int64_t, int64_t>> input_range;
    input_desc->GetShapeRange(input_range);
    MakeUpShapeRange(input_shape, input_range);

    auto output_desc = op_info->MutableOutputDesc(0);
    for (const string& output_name : output_name_list) {
      output_desc = op_info->MutableOutputDesc(output_name);
      output_desc->SetShape(GeShape(input_shape));
      output_desc->SetOriginShape(GeShape(input_shape));
      output_desc->SetShapeRange(input_range);
      output_desc->SetDataType(input_dtype);
    }
  } else {
    auto output_desc = op_info->MutableOutputDesc(0);
    for (const string& output_name : output_name_list) {
      output_desc = op_info->MutableOutputDesc(output_name);
      output_desc->SetShape(GeShape(input_shape));
      output_desc->SetDataType(input_dtype);
    }
  }
  return true;
}

//----------------Assign-------------------
IMPLEMT_VERIFIER(Assign, AssignVerify)
{
  if (op.GetInputDesc("ref").GetDataType() != op.GetInputDesc("value").GetDataType()) {
    return GRAPH_FAILED;
  }
  return GRAPH_SUCCESS;
}

// Obtains the processing function of the output tensor description.
IMPLEMT_COMMON_INFERFUNC(AssignInferShape)
{
  if (OneInOneOutDynamicInfer(op, "value", {"ref"})) {
    return GRAPH_SUCCESS;
  }
  return GRAPH_FAILED;

}

//Registered inferfunction
COMMON_INFER_FUNC_REG(Assign, AssignInferShape);

//Registered verify function
VERIFY_FUNC_REG(Assign, AssignVerify);
//----------------Assign-------------------
}
