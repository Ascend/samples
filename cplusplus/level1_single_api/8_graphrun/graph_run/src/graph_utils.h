/**
 * Copyright 2020 Huawei Technologies Co., Ltd
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef DAVINCI_GRAPH_UTILS_H
#define DAVINCI_GRAPH_UTILS_H
#include <vector>
#include <string>
#include <map>

#include "graph.h"
#include "types.h"
#include "tensor.h"
#include "ge_error_codes.h"
#include "ge_api_types.h"
#include "ge_api.h"
#include "all_ops.h"

#define FAILED -1
#define SUCCESS 0

using namespace ge;
using std::vector;
using std::map;
using std::string;

bool GetTensorFromBin(string input_path, vector<int64_t> shapes, Tensor &input_tensor,
                      Format format=ge::FORMAT_NCHW, DataType data_type=ge::DT_FLOAT);

int32_t GenInitGraph(Graph &graph, vector<TensorDesc> var_desc, const vector<string> &var_name,
                     const vector<float> &var_values);

int32_t GenConvGraph(Graph &graph, const vector<TensorDesc> &var_desc, const vector<string> &var_name,
                     Format format=ge::FORMAT_NCHW);

int32_t GenCheckpointGraph(Graph &graph, const map<string, TensorDesc> &variables);

#endif //DAVINCI_GRAPH_UTILS_H
