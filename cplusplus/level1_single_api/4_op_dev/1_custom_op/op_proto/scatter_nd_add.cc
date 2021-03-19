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
 * @file scatter_nd_add.cpp
 *
 * @brief
 *
 * @version 1.0
 *
 */
#include "./scatter_nd_add.h"
#include <string>
#include <vector>

namespace ge {
static bool CheckTwoInputDtypeSame(const Operator &op, const string &input_name1,
                                   const string &input_name2) {
  DataType input_type_x1 = op.GetInputDescByName(input_name1.c_str()).GetDataType();
  DataType input_type_x2 = op.GetInputDescByName(input_name2.c_str()).GetDataType();
  if (input_type_x1 != input_type_x2) {
    return false;
  }

  return true;
}

IMPLEMT_VERIFIER(ScatterNdAdd, ScatterNdAddVerify) {
  if (!CheckTwoInputDtypeSame(op, "var", "updates")) {
    return GRAPH_FAILED;
  }
  return GRAPH_SUCCESS;
}

IMPLEMT_COMMON_INFERFUNC(ScatterNdAddInferShape) {
  Shape var_shape = op.GetInputDescByName("var").GetShape();
  DataType input_dtype = op.GetInputDescByName("var").GetDataType();
  TensorDesc td = op.GetOutputDescByName("var");
  td.SetShape(ge::Shape(var_shape));
  td.SetDataType(input_dtype);
  (void)op.UpdateOutputDesc("var", td);
  return GRAPH_SUCCESS;
}

COMMON_INFER_FUNC_REG(ScatterNdAdd, ScatterNdAddInferShape);
VERIFY_FUNC_REG(ScatterNdAdd, ScatterNdAddVerify);
}
