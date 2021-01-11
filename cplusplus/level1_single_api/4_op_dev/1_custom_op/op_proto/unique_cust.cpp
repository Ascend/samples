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

#include "unique_cust.h"
#include <vector>
#include <string>
#include <iostream>

namespace {
ge::graphStatus WithRank(const ge::TensorDesc &tensor, int64_t rank, ge::Shape &out, const char* op_name) {
  if (rank > INT32_MAX) {
    return ge::GRAPH_FAILED;
  }
  ge::Shape s = tensor.GetShape();
  int64_t existing = static_cast<int64_t>(s.GetDimNum());
  if (s.GetDims() == ge::UNKNOWN_RANK) {
    std::vector<int64_t> out_shape(rank, ge::UNKNOWN_DIM);
    out = ge::Shape(out_shape);
    return ge::GRAPH_SUCCESS;
  }
  if (existing != rank) {
    return ge::GRAPH_FAILED;
  }
  out = s;
  return ge::GRAPH_SUCCESS;
}
}

namespace ge {
IMPLEMT_INFERFUNC(UniqueCust, UniqueCustInfer) {
  TensorDesc x_input = op.GetInputDesc("x");

  Shape x_shape;
  if (WithRank(x_input, 1, x_shape, "UniqueCust") != GRAPH_SUCCESS) {
    return GRAPH_FAILED;
  }

  DataType idx_type;
  if (op.GetAttr("out_idx", idx_type) != GRAPH_SUCCESS) {
    return GRAPH_FAILED;
  }

  TensorDesc idx_desc = op.GetOutputDesc("idx");
  idx_desc.SetShape(x_shape);
  idx_desc.SetOriginShape(x_shape);
  idx_desc.SetDataType(idx_type);
  op.UpdateOutputDesc("idx", idx_desc);

  TensorDesc y_desc = op.GetOutputDesc("y");
  y_desc.SetShape(Shape({UNKNOWN_DIM}));
  y_desc.SetOriginShape(Shape({UNKNOWN_DIM}));
  y_desc.SetDataType(x_input.GetDataType());
  if (x_shape.GetShapeSize() != UNKNOWN_DIM) {
    std::vector<std::pair<int64_t, int64_t>> range;
    int64_t max_dim = x_shape.GetDim(0);
    range.emplace_back(std::make_pair(1, max_dim));
    y_desc.SetShapeRange(range);
  }
  op.UpdateOutputDesc("y", y_desc);
  return GRAPH_SUCCESS;
}

INFER_FUNC_REG(UniqueCust, UniqueCustInfer);
}
