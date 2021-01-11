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

#include "gtest/gtest.h"
#include "two_in_one_out_layer.hpp"

class ADD_ST : public testing::Test {
protected:
 static void SetUpTestCase() {
 std::cout << "ADD_ST ST SetUp" << std::endl;
 }
 static void TearDownTestCase() {
 std::cout << "ADD_ST ST TearDown" << std::endl;
 }
 // Some expensive resource shared by all tests.
 virtual void SetUp() {}
 virtual void TearDown() {}
};

/*
* op: add
* input_shape: (1, 1)
* output_shape: (1, 1)
* stype: float32
* dtype: float32
*/
TEST_F(ADD_ST, test_add_1_1_float32) {
 std::string op_name = "add";
 std::string inputSizeStr = "1_1_float32";
 uint32_t inputSize = 1*1;
 uint32_t inputBSize = 1*1;
 uint32_t outputSize = 1*1;

 const char *stubFunc =  "cce_add_1_1_float32__kernel0";

 std::string bin_path = "./llt/ops/common/kernel_bin/add/cce_add_1_1_float32.o";

 std::string tilingName = "cce_add_1_1_float32__kernel0";

 std::string inputArrAPath = "./llt/ops/common/data/add/1_1_float32/add_input1_1_1_float32.data";
 std::string inputArrBPath = "./llt/ops/common/data/add/1_1_float32/add_input2_1_1_float32.data";

 std::string expectOutputDataPath = "./llt/ops/common/data/add/1_1_float32/add_output_1_1_float32.data";
 float ratios[2] = {0.0001 ,0.0001};

 TwoInOneOutLayer<float,float> layer{
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

 bool ret = layer.test();

 if(!ret)
 {
  layer.writeBinaryFile((void*)layer.outputData,
  "./llt/ops/common/data/add/1_1_float32/actual_add_output_1_1_float32.data",
  outputSize * sizeof(float));
 }

 assert(true == ret);
}

/*
* op: add
* input_shape: (16, 32)
* output_shape: (16, 32)
* stype: float32
* dtype: float32
*/
TEST_F(ADD_ST, test_add_16_32_float32) {
 std::string op_name = "add";
 std::string inputSizeStr = "16_32_float32";
 uint32_t inputSize = 16*32;
 uint32_t inputBSize = 16*32;
 uint32_t outputSize = 16*32;

 const char *stubFunc =  "cce_add_16_32_float32__kernel0";

 std::string bin_path = "./llt/ops/common/kernel_bin/add/cce_add_16_32_float32.o";

 std::string tilingName = "cce_add_16_32_float32__kernel0";

 std::string inputArrAPath = "./llt/ops/common/data/add/16_32_float32/add_input1_16_32_float32.data";
 std::string inputArrBPath = "./llt/ops/common/data/add/16_32_float32/add_input2_16_32_float32.data";

 std::string expectOutputDataPath = "./llt/ops/common/data/add/16_32_float32/add_output_16_32_float32.data";
 float ratios[2] = {0.0001 ,0.0001};

 TwoInOneOutLayer<float,float> layer{
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

 bool ret = layer.test();

 if(!ret)
 {
  layer.writeBinaryFile((void*)layer.outputData,
  "./llt/ops/common/data/add/16_32_float32/actual_add_output_16_32_float32.data",
  outputSize * sizeof(float));
 }

 assert(true == ret);
}


