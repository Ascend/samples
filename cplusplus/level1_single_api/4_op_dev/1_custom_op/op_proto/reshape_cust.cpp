/* Copyright 2020 Huawei Technologies Co., Ltd
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "reshape_cust.h"
#include <vector>
#include <string>
#include <iostream>

namespace {
template <typename T>
std::vector<int64_t> AsInt64(const T *data, int64_t data_size) {
  std::vector<int64_t> ret(data_size);
  for (int64_t i = 0; i < data_size; ++i) {
    ret[i] = data[i];
  }
  return ret;
}

int64_t GetElementNum(const std::vector<int64_t> &shape) {
  int64_t ret = 1;
  for (size_t i = 0; i < shape.size(); ++i) {
    ret *= shape[i];
  }
  return ret;
}
}

namespace ge {

IMPLEMT_COMMON_INFERFUNC(ReshapeCustInferShape) {
  TensorDesc tensordesc_tensor = op.GetInputDescByName("tensor");
  TensorDesc tensor_desc_shape = op.GetInputDescByName("shape");
  TensorDesc tensor_desc_output = op.GetOutputDescByName("output");
  Tensor shape_tensor;
  if (op.GetInputConstData("shape", shape_tensor) == GRAPH_SUCCESS) {
    DataType shape_type = tensor_desc_shape.GetDataType();
    std::vector<int64_t> shape_values;
    if (shape_type == DT_INT32) {
      auto shape_data = reinterpret_cast<const int32_t *>(shape_tensor.GetData());
      shape_values = AsInt64<int32_t>(shape_data, shape_tensor.GetSize() / sizeof(int32_t));
    } else {
      auto shape_data = reinterpret_cast<const int64_t *>(shape_tensor.GetData());
      shape_values = AsInt64<int64_t>(shape_data, shape_tensor.GetSize() / sizeof(int64_t));
    }

    std::vector<int64_t> input_shape = tensordesc_tensor.GetShape().GetDims();
    int64_t input_element_num = GetElementNum(input_shape);
    int64_t shape_element_num = GetElementNum(shape_values);
    if (input_element_num != shape_element_num) {
      return GRAPH_FAILED;
    }
    tensor_desc_output.SetShape(Shape(shape_values));
	  tensor_desc_output.SetOriginShape(Shape(shape_values));
  }

  tensor_desc_output.SetDataType(tensordesc_tensor.GetDataType());

  std::vector<std::pair<int64_t,int64_t>> range;
  auto status = op.GetInputDescByName("tensor").GetShapeRange(range);
  if (status != GRAPH_SUCCESS) {
    return GRAPH_FAILED;
  }
  tensor_desc_output.SetShapeRange(range);

  (void)op.UpdateOutputDesc("output", tensor_desc_output);
  return GRAPH_SUCCESS;
}

COMMON_INFER_FUNC_REG(ReshapeCust, ReshapeCustInferShape);

}
