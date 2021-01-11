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

/*!
 * \file assign.cc
 * \brief
 */
#include <string>
#include <math.h>
#include <algorithm>
#include <nlohmann/json.hpp>
#include "op_tiling.h"
#include "graph/debug/ge_log.h"
#include "op_log.h"
#include "error_log.h"

// the minimum num for one core
const int64_t CORE_MINIMUM_NUM = 4;
const int64_t BLOCK_SIZE = 32;  // one block size is 32Bytes

namespace optiling {
bool GetAssignCompileParams(const nlohmann::json& op_compile_info, int64_t& core_num, int64_t& ub_size) {
    using namespace nlohmann;
    auto all_vars = op_compile_info["vars"];
    if (all_vars.count("core_num") == 0) {
        OP_LOGE("op [Assign]: GetAssignCompileParams, get core_num error");
        return false;
    }
    core_num = all_vars["core_num"].get<std::int64_t>();

    if (all_vars.count("ub_size") == 0) {
        OP_LOGE("op [Assign]: GetAssignCompileParams, get ub_size error");
        return false;
    }
    ub_size = all_vars["ub_size"].get<std::int64_t>();

    return true;
}

bool AssignTiling(const std::string& op_type, const TeOpParas& op_paras, const nlohmann::json& op_info,
                 OpRunInfo& run_info) {
    using namespace ge;

    CHECK(!op_paras.inputs.empty(), "op [%s] : op_paras.inputs cannot be empty", op_type.c_str());
    CHECK(!op_paras.inputs[1].tensor.empty(), "op [%s] : op_paras.inputs[1].tensor cannot be empty", op_type.c_str());

    const std::vector<int64_t>& value_shape = op_paras.inputs[1].tensor[0].shape;
    const std::string input_dtype = op_paras.inputs[1].tensor[0].dtype;

    int64_t value_num;
    if (value_shape.size() == 0) {
        value_num = 1;
    } else {
        value_num = std::accumulate(value_shape.begin(), value_shape.end(), 1, std::multiplies<int64_t>());
    }

    int64_t core_num = 0;
    int64_t ub_size = 0;
    if (!GetAssignCompileParams(op_info, core_num, ub_size)) {
        return false;
    }

    int64_t ele_size = 0;
    if (input_dtype.find('8') != string::npos) {
        ele_size = sizeof(int8_t);
    } else if (input_dtype.find("16") != string::npos) {
        ele_size = sizeof(int16_t);
    } else if (input_dtype.find("32") != string::npos) {
        ele_size = sizeof(int32_t);
    } else if (input_dtype.find("64") != string::npos) {
        ele_size = sizeof(int64_t);
    } else {
        ele_size = sizeof(float);
    }

    int64_t ele_per_block = BLOCK_SIZE / ele_size;
    int64_t block_count = (value_num + ele_per_block - 1) / ele_per_block; 

    int64_t sigment_total = (block_count + CORE_MINIMUM_NUM - 1) / CORE_MINIMUM_NUM;
    int64_t sigment_per_core = (sigment_total + core_num - 1) / core_num; 

    int64_t core_used_num = sigment_per_core == 0 ? 1 : (sigment_total + sigment_per_core - 1) / sigment_per_core;
    int64_t block_per_core = sigment_per_core * CORE_MINIMUM_NUM;
    int64_t block_tail_core = block_count - (block_per_core * (core_used_num - 1));

    GELOGD("op [Assign]: CompileParams, core_used_num = %d, block_per_core = %d, block_tail_core = %d",
           core_used_num, block_per_core, block_tail_core);

    ByteBufferPut(run_info.tiling_data, core_used_num);
    ByteBufferPut(run_info.tiling_data, block_per_core);
    ByteBufferPut(run_info.tiling_data, block_tail_core);

    run_info.block_dim = core_num;
    std::vector<int64_t> workspace;
    run_info.workspaces = workspace;
    GELOGI("op [%s] tiling run success.", op_type.c_str());
    return true;
}

// register tiling interface of the Assign op.
REGISTER_OP_TILING_FUNC_BUFFERED(Assign, AssignTiling);
}  // namespace optiling
