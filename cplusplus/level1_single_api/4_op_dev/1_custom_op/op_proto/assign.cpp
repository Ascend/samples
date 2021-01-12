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
#include <algorithm>

namespace ge {

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
  Shape x_shape = op.GetInputDesc("ref").GetShape();
  DataType input_dtype = op.GetInputDesc("ref").GetDataType();
  std::vector<std::pair<int64_t, int64_t>> shape_range_x;
  op.GetInputDesc("ref").GetShapeRange(shape_range_x);

  TensorDesc tensordesc_output = op.GetOutputDesc("ref");
  tensordesc_output.SetShape(x_shape);
  tensordesc_output.SetDataType(input_dtype);
  tensordesc_output.SetShapeRange(shape_range_x);
  (void)op.UpdateOutputDesc("ref", tensordesc_output);
}

//Registered inferfunction
COMMON_INFER_FUNC_REG(Assign, AssignInferShape);

//Registered verify function
VERIFY_FUNC_REG(Assign, AssignVerify);
//----------------Assign-------------------
}
