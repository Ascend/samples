/* Copyright (c) Huawei Technologies Co., Ltd. 2022. All rights reserved.
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

#include "softmax_tik.h"
#include <string>
#include <vector>

namespace ge {
IMPLEMT_VERIFIER(SoftmaxTik, SoftmaxTikVerify)
{
    std::vector<DataType> support_list;
    support_list.reserve(2);
    support_list.push_back(DT_FLOAT16);
    support_list.push_back(DT_FLOAT);
    return GRAPH_SUCCESS;
}

// Obtains the processing function of the output tensor description.
IMPLEMT_COMMON_INFERFUNC(SoftmaxTikInferShape)
{
    TensorDesc tensordesc_output = op.GetOutputDescByName("y");
    ge::TensorDesc inputTensorDesc = op.GetInputDescByName("x");
    ge::Shape shape = inputTensorDesc.GetShape();
    DataType dtype = inputTensorDesc.GetDataType();
    std::vector<int64_t> dimVector;
    dimVector.push_back(shape.GetDim(0));
    ge::Shape outputShape(dimVector);
    tensordesc_output.SetShape(outputShape);
    tensordesc_output.SetDataType(dtype);
    (void)op.UpdateOutputDesc("y", tensordesc_output);
    return GRAPH_SUCCESS;
}

// Registered inferfunction
COMMON_INFER_FUNC_REG(SoftmaxTik, SoftmaxTikInferShape);

// Registered verify function
VERIFY_FUNC_REG(SoftmaxTik, SoftmaxTikVerify);
}
