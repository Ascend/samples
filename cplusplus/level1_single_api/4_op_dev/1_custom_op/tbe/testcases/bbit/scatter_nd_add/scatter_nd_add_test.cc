#include <stdio.h>
#include <fstream>
#include <iostream>
#include "./../../log.hpp"
#include <string.h>
using namespace std;
#include "three_in_no_out_layer.hpp"
#include "./scatter_nd_add_test.hpp"

bool ScatterNdAddTest::test(string name)
{
    TVM_LOG(CC_LOG_INFO, "TVM ScatterNDAdd BBIT begin.");
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

    TVM_LOG(CC_LOG_INFO, "TVM ScatterNDAdd BBIT end.");
    return true;
}

/*
* op: scatter_nd_add
* input_shape: (2, 2, 2)
* output_shape: (31, 2, 2)
* stype: float32
* dtype: float32
*/
bool ScatterNdAddTest::scatter_nd_add_test_2_2_2_float32()
{
    std::string op_name = "scatter_nd_add";
    std::string inputSizeStr = "2_2_2_float32";
    uint32_t inputSize = 31*2*2;
    uint32_t inputBSize = 2*1;
    uint32_t inputCSize = 2*2*2;
    uint32_t outputSize = 31*2*2;

    const char* stubFunc =  "cce_scatter_nd_add_2_2_2_float32__kernel0";

    std::string bin_path = "./llt/ops/common/kernel_bin/scatter_nd_add/cce_scatter_nd_add_2_2_2_float32";

    std::string tilingName = "cce_scatter_nd_add_2_2_2_float32__kernel0";

    std::string inputArrAPath = "./llt/ops/common/data/scatter_nd_add/2_2_2_float32/scatter_nd_add_input_31_2_2_float32.data";
    std::string inputArrBPath = "./llt/ops/common/data/scatter_nd_add/2_2_2_float32/scatter_nd_add_input_B_2_1_int32.data";
    std::string inputArrCPath = "./llt/ops/common/data/scatter_nd_add/2_2_2_float32/scatter_nd_add_input_C_2_2_2_float32.data";

    std::string expectOutputDataPath = "./llt/ops/common/data/scatter_nd_add/2_2_2_float32/scatter_nd_add_output_31_2_2_float32.data";
    float ratios[2] = {0.001 ,0.001};

    ThreeInNoOutLayer<float,int32_t,float,float> layer{
        op_name,
        inputSizeStr,
        inputSize,
        inputBSize,
        inputCSize,
        outputSize,
        bin_path,
        tilingName,
        inputArrAPath,
        inputArrBPath,
        inputArrCPath,
        expectOutputDataPath,
        ratios,
        (void*)stubFunc,
        false
    };

    return layer.test();
}


/*
* op: scatter_nd_add
* input_shape: (33, 5, 5)
* output_shape: (17, 5, 5)
* stype: float16
* dtype: float16
*/
bool ScatterNdAddTest::scatter_nd_add_test_33_5_5_float16()
{
    std::string op_name = "scatter_nd_add";
    std::string inputSizeStr = "33_5_5_float16";
    uint32_t inputSize = 17*5*5;
    uint32_t inputBSize = 33*1;
    uint32_t inputCSize = 33*5*5;
    uint32_t outputSize = 17*5*5;

    const char* stubFunc =  "cce_scatter_nd_add_33_5_5_float16__kernel0";

    std::string bin_path = "./llt/ops/common/kernel_bin/scatter_nd_add/cce_scatter_nd_add_33_5_5_float16";

    std::string tilingName = "cce_scatter_nd_add_33_5_5_float16__kernel0";

    std::string inputArrAPath = "./llt/ops/common/data/scatter_nd_add/33_5_5_float16/scatter_nd_add_input_17_5_5_float16.data";
    std::string inputArrBPath = "./llt/ops/common/data/scatter_nd_add/33_5_5_float16/scatter_nd_add_input_B_33_1_int32.data";
    std::string inputArrCPath = "./llt/ops/common/data/scatter_nd_add/33_5_5_float16/scatter_nd_add_input_C_33_5_5_float16.data";

    std::string expectOutputDataPath = "./llt/ops/common/data/scatter_nd_add/33_5_5_float16/scatter_nd_add_output_17_5_5_float16.data";
    float ratios[2] = {0.001 ,0.001};

    ThreeInNoOutLayer<fp16_t,int32_t,fp16_t,fp16_t> layer{
        op_name,
        inputSizeStr,
        inputSize,
        inputBSize,
        inputCSize,
        outputSize,
        bin_path,
        tilingName,
        inputArrAPath,
        inputArrBPath,
        inputArrCPath,
        expectOutputDataPath,
        ratios,
        (void*)stubFunc,
        false
    };

    return layer.test();
}
