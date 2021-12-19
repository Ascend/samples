/**
* @file main.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include <cstdint>
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "acl/acl.h"
#include "op_runner.h"

#include "common.h"

bool g_isDevice = false;

OperatorDesc CreateOpDesc()
{
    // define operator
    
    std::string opType = "LSTMTik";
    aclDataType dataType = ACL_FLOAT16;
    aclFormat format = ACL_FORMAT_ND;
    OperatorDesc opDesc(opType);

    std::vector<int64_t> xShape{16, 2, 2, 16, 16};
    opDesc.AddInputTensorDesc(dataType, xShape.size(),
                              xShape.data(), format);
    std::vector<int64_t> initHShape{2, 2, 16, 16};
    opDesc.AddInputTensorDesc(dataType, initHShape.size(),
                              initHShape.data(), format);
    std::vector<int64_t> initCShape{2, 2, 16, 16};
    opDesc.AddInputTensorDesc(dataType, initCShape.size(),
                              initCShape.data(), format);
    std::vector<int64_t> initWeightShape{4, 8, 16, 16};
    opDesc.AddInputTensorDesc(dataType, initWeightShape.size(),
                              initWeightShape.data(), format);
    std::vector<int64_t> initBiasShape{128};
    opDesc.AddInputTensorDesc(ACL_FLOAT, initBiasShape.size(),
                              initBiasShape.data(), format);
    std::vector<int64_t> outputHShape{16, 2, 2, 16, 16};
    opDesc.AddOutputTensorDesc(dataType, outputHShape.size(),
                               outputHShape.data(), format);
    std::vector<int64_t> outputCShape{2, 2, 16, 16};
    opDesc.AddOutputTensorDesc(dataType, outputCShape.size(),
                               outputCShape.data(), format);
    return opDesc;
}

bool SetInputData(OpRunner &runner)
{
    for (size_t i = 0; i < runner.NumInputs(); ++i) {
        size_t fileSize = 0;
        std::string filePath = "test_data/data/input_" + std::to_string(i) + ".bin";
        char *fileData = ReadFile(filePath, fileSize,
            runner.GetInputBuffer<void>(i), runner.GetInputSize(i));
        if (fileData == nullptr) {
            ERROR_LOG("Read input[%zu] failed", i);
            return false;
        }

        INFO_LOG("Set input[%zu] from %s success.", i, filePath.c_str());
        INFO_LOG("Input[%zu]:", i);
        runner.PrintInput(i);
    }

    return true;
}

bool ProcessOutputData(OpRunner &runner)
{
    for (size_t i = 0; i < runner.NumOutputs(); ++i) {
        INFO_LOG("Output[%zu]:", i);
        runner.PrintOutput(i);

        std::string filePath = "result_files/output_" + std::to_string(i) + ".bin";
        if (!WriteFile(filePath, runner.GetOutputBuffer<void>(i), runner.GetOutputSize(i))) {
            ERROR_LOG("Write output[%zu] failed.", i);
            return false;
        }

        INFO_LOG("Write output[%zu] success. output file = %s", i, filePath.c_str());
    }
    return true;
}

bool RunLSTMOp()
{
    // create op desc
    OperatorDesc opDesc = CreateOpDesc();

    // create Runner
    OpRunner opRunner(&opDesc);
    if (!opRunner.Init()) {
        ERROR_LOG("Init OpRunner failed");
        return false;
    }

    // Load inputs
    if (!SetInputData(opRunner)) {
        return false;
    }

    // Run op
    if (!opRunner.RunOp()) {
        return false;
    }

    // process output data
    if (!ProcessOutputData(opRunner)) {
        return false;
    }

    INFO_LOG("Run op success");
    return true;
}

int main()
{
    std::string output = "./result_files";
    if (access(output.c_str(), 0) == -1) {
        int ret = mkdir(output.c_str(), 0700);
        if (ret == 0) {
            INFO_LOG("make output directory successfully");
        }
        else {
            ERROR_LOG("make output directory fail");
            return FAILED;
        }
    }

    if (aclInit("test_data/config/acl.json") != ACL_SUCCESS) {
        ERROR_LOG("Init acl failed");
        return FAILED;
    }

    int deviceId = 0;
    if (aclopSetModelDir("op_models") != ACL_SUCCESS) {
        std::cerr << "Load single op model failed" << std::endl;
        return FAILED;
    }

    if (aclrtSetDevice(deviceId) != ACL_SUCCESS) {
        std::cerr << "Open device failed. device id = " << deviceId << std::endl;
        return FAILED;
    }
    INFO_LOG("Open device[%d] success", deviceId);

    aclrtRunMode runMode;
    if (aclrtGetRunMode(&runMode) != ACL_SUCCESS) {
        ERROR_LOG("acl get run mode failed");
        return FAILED;
    }
    g_isDevice = (runMode == ACL_DEVICE);

    if (!RunLSTMOp()) {
        (void) aclrtResetDevice(deviceId);
        return FAILED;
    }

    (void) aclrtResetDevice(deviceId);

    if (aclFinalize() != ACL_SUCCESS) {
        ERROR_LOG("Finalize acl failed");
        return FAILED;
    }

    return SUCCESS;
}
