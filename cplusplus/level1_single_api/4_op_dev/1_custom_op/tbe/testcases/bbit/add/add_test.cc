/* Copyright (C) 2019. Huawei Technologies Co., Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the Apache License Version 2.0.You may not use
 * this file except in compliance with the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * Apache License for more details at
 * http://www.apache.org/licenses/LICENSE-2.0
 */

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string.h>
#include "log.hpp"
#include "./add_test.hpp"
#include "two_in_one_out_layer.hpp"
using namespace std;

bool add_Test::test(string name)
{
    TVM_LOG(CC_LOG_INFO, "TVM add BBIT begin.");
    bool ret = false;

    // Code for invoking testcase here.
    if (name == "all") {
        for (const auto& item : test_calls_) {
            bool ret = item.second();
            if (!ret) {
                TVM_LOG(CC_LOG_ERROR, "%s %s falied!", case_.c_str(), item.first.c_str());
                return false;
            }
        }
    } else {
        auto found = test_calls_.find(name);
        if (found == test_calls_.end()) {
            TVM_LOG(CC_LOG_ERROR, "%s case %s not found!", case_.c_str(), name.c_str());
            return false;
        }

        bool ret = found->second();
        if (!ret) {
            TVM_LOG(CC_LOG_ERROR, "%s %s falied!", case_.c_str(), found->first.c_str());
            return false;
        }
    }

    TVM_LOG(CC_LOG_INFO, "TVM add BBIT end.");
    return true;
}

bool add_Test::test_1_1_float32() {
    std::string op_name = "add";
    std::string inputSizeStr = "1_1_float32";
    uint32_t inputSize = 1*1;
    uint32_t inputBSize = 1*1;
    uint32_t outputSize = 1*1;

    const char *stubFunc =  "cce_add_1_1_float32__kernel0";

    std::string bin_path = "./llt/ops/common/kernel_bin/add/cce_add_1_1_float32";

    std::string tilingName = "cce_add_1_1_float32__kernel0";

    std::string inputArrAPath = "./llt/ops/common/data/add/1_1_float32/add_input1_1_1_float32.data";
    std::string inputArrBPath = "./llt/ops/common/data/add/1_1_float32/add_input2_1_1_float32.data";

    std::string expectOutputDataPath = "./llt/ops/common/data/add/1_1_float32/add_output_1_1_float32.data";
    float ratios[2] = {0.0001 ,0.0001};

    TwoInOneOutLayer<float,float,float> layer{
        op_name,
        inputSizeStr,
        inputSize,
        inputBSize,
        outputSize,
        bin_path,
        tilingName,
        inputArrAPath,
        inputArrBPath,
        expectOutputDataPath,
        ratios,
        (void*)stubFunc
    };

    return layer.test();
}

bool add_Test::test_16_32_float32() {
    std::string op_name = "add";
    std::string inputSizeStr = "16_32_float32";
    uint32_t inputSize = 16*32;
    uint32_t inputBSize = 16*32;
    uint32_t outputSize = 16*32;

    const char *stubFunc =  "cce_add_16_32_float32__kernel0";

    std::string bin_path = "./llt/ops/common/kernel_bin/add/cce_add_16_32_float32";

    std::string tilingName = "cce_add_16_32_float32__kernel0";

    std::string inputArrAPath = "./llt/ops/common/data/add/16_32_float32/add_input1_16_32_float32.data";
    std::string inputArrBPath = "./llt/ops/common/data/add/16_32_float32/add_input2_16_32_float32.data";

    std::string expectOutputDataPath = "./llt/ops/common/data/add/16_32_float32/add_output_16_32_float32.data";
    float ratios[2] = {0.0001 ,0.0001};

    TwoInOneOutLayer<float,float,float> layer{
        op_name,
        inputSizeStr,
        inputSize,
        inputBSize,
        outputSize,
        bin_path,
        tilingName,
        inputArrAPath,
        inputArrBPath,
        expectOutputDataPath,
        ratios,
        (void*)stubFunc
    };

    return layer.test();
}


