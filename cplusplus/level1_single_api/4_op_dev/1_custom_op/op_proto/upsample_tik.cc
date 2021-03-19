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
 * @file permute.cpp
 *
 * @brief
 *
 * @version 1.0
 *
 */
#include "./upsample_tik.h"
#include <string>
#include <vector>
#include <algorithm>

namespace ge {
// ----------------Upsample Op Begin-------------------

IMPLEMT_VERIFIER(UpsampleTik, UpsampleTikVerify) { return GRAPH_SUCCESS; }
IMPLEMT_INFERFUNC(UpsampleTik, UpsampleTikInferShape) {
  TensorDesc tensordesc_output = op.GetInputDescByName("x");
  uint32_t stride_h = 2;
  uint32_t stride_w = 2;
  if (op.GetAttr("stride_h", stride_h) != ge::GRAPH_SUCCESS) {
    stride_h = 2;
  }
    if (op.GetAttr("stride_w", stride_w) != ge::GRAPH_SUCCESS) {
    stride_w = 2;
  }
  ge::Shape shape = tensordesc_output.GetShape();
  std::vector<int64_t> dims_input = shape.GetDims();
  std::vector<int64_t> dimVector;
  for (size_t i = 0; i < dims_input.size(); i++) {
    if (i == 2 ) {
      int64_t dims = dims_input[i] * stride_h;
      dimVector.push_back(dims);
    } else if(i == 3) {
      int64_t dims = dims_input[i] * stride_w;
      dimVector.push_back(dims);
    }else {
      int64_t dims = dims_input[i];
      dimVector.push_back(dims);
    }
  }

  Shape outputMaxShape(dimVector);
  tensordesc_output.SetShape(outputMaxShape);
  (void)op.UpdateOutputDesc("y", tensordesc_output);

  return GRAPH_SUCCESS;
}

INFER_FUNC_REG(UpsampleTik, UpsampleTikInferShape);
VERIFY_FUNC_REG(UpsampleTik, UpsampleTikVerify);
// ----------------Upsample Op End-----------------
}
