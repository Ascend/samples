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
	// feature shape is [8, 256, 7, 7]
	// kernel shape is [256, 256, 3, 3]
	// output shape is [8, 256, 7, 7]
    std::vector<int64_t> shape{8, 256, 7, 7};
    std::vector<int64_t> shape1{256, 256, 3, 3};  
    std::vector<int64_t> shape2{8, 256, 7, 7};
    std::string opType = "Conv2DTik";
    aclDataType dataType = ACL_FLOAT16;
    aclFormat format = ACL_FORMAT_NCHW;
    OperatorDesc opDesc(opType);
    opDesc.AddInputTensorDesc(dataType, shape.size(), shape.data(), format);
    opDesc.AddInputTensorDesc(dataType, shape1.size(), shape1.data(), format);
    opDesc.AddOutputTensorDesc(dataType, shape2.size(), shape2.data(), format);
    int64_t intList[4]{1, 1, 1, 1};
    auto opAttr = opDesc.opAttr;
    aclopSetAttrListInt(opAttr, "strides", 4, intList);
    aclopSetAttrListInt(opAttr, "pads", 4, intList);
    aclopSetAttrListInt(opAttr, "dilations", 4, intList);
    return opDesc;
}


bool SetInputData(OpRunner &runner)
{
    for (size_t i = 0; i < runner.NumInputs(); ++i) {
        size_t fileSize;
        std::string filePath = "test_data/data/input_" + std::to_string(i) + ".bin";
        char *fileData = ReadFile(filePath, fileSize,
            runner.GetInputBuffer<void>(i), runner.GetInputSize(i));
        if (fileData == nullptr) {
            ERROR_LOG("Read input[%zu] failed", i);
            return false;
        }

        INFO_LOG("Set input[%zu] from %s success", i, filePath.c_str());
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

bool RunConv2DOp()
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
        ERROR_LOG("Set input data failed");
        return false;
    }

    // Run op
    if (!opRunner.RunOp()) {
        ERROR_LOG("Run op failed");
        return false;
    }

    // process output data
    if (!ProcessOutputData(opRunner)) {
        ERROR_LOG("Process output data failed");
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
            INFO_LOG("Make output directory successfully");
        }
        else {
            ERROR_LOG("Make output directory fail, ret = %d", ret);
            return FAILED;
        }
    }

    if (aclInit("test_data/config/acl.json") != ACL_SUCCESS) {
        ERROR_LOG("Init acl failed");
        return FAILED;
    }

    int deviceId = 0;
    if (aclopSetModelDir("op_models") != ACL_SUCCESS) {
        ERROR_LOG("Load single op model failed");
        return FAILED;
    }

    if (aclrtSetDevice(deviceId) != ACL_SUCCESS) {
        ERROR_LOG("Set device failed. deviceId is %d", deviceId);
        return FAILED;
    }
    INFO_LOG("Open device[%d] success", deviceId);

    aclrtRunMode runMode;
    if (aclrtGetRunMode(&runMode) != ACL_SUCCESS) {
        ERROR_LOG("Acl get run mode failed");
        return FAILED;
    }
    g_isDevice = (runMode == ACL_DEVICE);

    if (!RunConv2DOp()) {
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
