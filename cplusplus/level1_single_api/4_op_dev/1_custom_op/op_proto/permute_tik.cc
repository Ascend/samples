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
#include "./permute_tik.h"
#include <string>
#include <vector>
#include <algorithm>

namespace ge {
// ----------------Permute Op Begin-------------------

static graphStatus TransposeCommonInferShape(const std::vector<int64_t>& order_list,
Operator& op) {
    Shape shape = op.GetInputDescByName("x").GetShape();
    size_t dim_num = shape.GetDimNum();
    if (order_list.empty() || (order_list.size() != dim_num)) {
        return GRAPH_FAILED;
    }
    for (size_t i = 0; i < dim_num; ++i) {
        if ((size_t)order_list[i] >= dim_num || (size_t)order_list[i] < 0) {
            return GRAPH_FAILED;
        }
    }

    vector<int64_t> out_vec;
    for (size_t i = 0; i < dim_num; ++i) {
        out_vec.push_back(shape.GetDim(order_list[i]));
    }

    Shape out_shape(out_vec);
    TensorDesc tensordesc_output = op.GetOutputDescByName("y");
    tensordesc_output.SetShape(out_shape);
    tensordesc_output.SetDataType(op.GetInputDescByName("x").GetDataType());
    (void)op.UpdateOutputDesc("y", tensordesc_output);
    return GRAPH_SUCCESS;
}

IMPLEMT_COMMON_INFERFUNC(PermuteTikInferShape) {
    auto input_shape = op.GetInputDescByName("x").GetShape();
    std::vector<int64_t> input_shape_dims = input_shape.GetDims();

    std::vector<int64_t> perm_list;
    if (ge::GRAPH_SUCCESS != op.GetAttr("order", perm_list)) {
        return GRAPH_FAILED;
    }
    for (size_t i = 0; i < input_shape_dims.size(); ++i) {
        if (std::find(perm_list.begin(), perm_list.end(), i) == perm_list.end()) {
            perm_list.push_back((int64_t)i);
        }
    }
    op.SetAttr("order", perm_list);
    return TransposeCommonInferShape(perm_list, op);
}

COMMON_INFER_FUNC_REG(PermuteTik, PermuteTikInferShape);
// ----------------Permute Op End-----------------
}
