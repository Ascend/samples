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
 * @file leaky_relu.cpp
 *
 * @brief
 *
 * @version 1.0
 *
 */
#include "./leaky_relu_demo.h"
#include <string>
#include <vector>

namespace ge {

IMPLEMT_VERIFIER(LeakyReluDemo, LeakyReluDemoVerify) {
  
  return GRAPH_SUCCESS;
}
IMPLEMT_INFERFUNC(LeakyReluDemo, LeakyReluDemoInferShape) {
  auto x_shape = op.GetInputDescByName("x").GetShape().GetDims();
  DataType x_dtype = op.GetInputDescByName("x").GetDataType();
  TensorDesc y_desc = op.GetOutputDescByName("y");
  y_desc.SetShape(ge::Shape(x_shape));
  y_desc.SetDataType(x_dtype);
  (void)op.UpdateOutputDesc("y", y_desc);
  return GRAPH_SUCCESS;
}
INFER_FUNC_REG(LeakyReluDemo, LeakyReluDemoInferShape);
VERIFY_FUNC_REG(LeakyReluDemo, LeakyReluDemoVerify);

}
