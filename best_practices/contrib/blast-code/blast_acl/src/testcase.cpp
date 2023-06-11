/**
* @file testcase.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include "op_test.h"
#include "op_execute.h"
using namespace OpTest;


OP_TEST(Blast, Test_Blast_001_case_001_ND_int16)
{
    
    std::string opType = "Blast";
    OpTestDesc opTestDesc(opType);
    // input parameter init
    opTestDesc.inputShape = {{16}, {32, 16}, {400}};
    opTestDesc.inputDataType = {ACL_INT16, ACL_INT16, ACL_INT16};
    opTestDesc.inputFormat = {(aclFormat)2, (aclFormat)2, (aclFormat)2};
    opTestDesc.inputFilePath = {"test_data/data/Test_Blast_001_case_001_ND_int16_input_0", "test_data/data/Test_Blast_001_case_001_ND_int16_input_1", "test_data/data/Test_Blast_001_case_001_ND_int16_input_2"};
    opTestDesc.inputConst = {false, false, false};
    // output parameter init
    opTestDesc.outputShape = {{32}};
    opTestDesc.outputDataType = {ACL_INT16};
    opTestDesc.outputFormat = {(aclFormat)2};
    opTestDesc.outputFilePath = {"result_files/Test_Blast_001_case_001_ND_int16_output_0"};
    // attr parameter init
    
    // set deviceId
    const uint32_t deviceId = 0;
    EXPECT_EQ_AND_RECORD(true, OpExecute(opTestDesc, deviceId), opTestDesc, "Test_Blast_001_case_001_ND_int16");

}

