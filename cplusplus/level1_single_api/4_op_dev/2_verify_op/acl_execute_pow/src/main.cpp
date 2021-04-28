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
int deviceId = 0;

OperatorDesc CreateOpDesc()
{
    // define operator
    std::vector<int64_t> shape{2, 2};
    std::string opType = "Pow";
    aclDataType dataType = ACL_INT32;
    aclFormat format = ACL_FORMAT_ND;
    OperatorDesc opDesc(opType);
    opDesc.AddInputTensorDesc(dataType, shape.size(), shape.data(), format);
    opDesc.AddInputTensorDesc(dataType, shape.size(), shape.data(), format);
    opDesc.AddOutputTensorDesc(dataType, shape.size(), shape.data(), format);
    return opDesc;
}

bool SetInputData(OpRunner &runner)
{
    for (size_t i = 0; i < runner.NumInputs(); ++i) {
        size_t fileSize = 0;
        std::string filePath = "test_data/data/input_" + std::to_string(i) + ".bin";
        bool result = ReadFile(filePath, fileSize,
            runner.GetInputBuffer<void>(i), runner.GetInputSize(i));
        if (!result) {
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

bool RunAddOp()
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

void DestoryResource()
{
    bool flag = false;
    if (aclrtResetDevice(deviceId) != ACL_SUCCESS) {
        ERROR_LOG("Reset device %d failed", deviceId);
        flag = true;
    }
    if (aclFinalize() != ACL_SUCCESS) {
        ERROR_LOG("Finalize acl failed");
        flag = true;
    }
    if (flag) {
        ERROR_LOG("Destory resource failed");
    } else {
        INFO_LOG("Destory resource success");
    }
}

bool InitResource()
{
    std::string output = "./result_files";
    if (access(output.c_str(), 0) == -1) {
        int ret = mkdir(output.c_str(), 0700);
        if (ret == 0) {
            INFO_LOG("Make output directory successfully");
        }
        else {
            ERROR_LOG("Make output directory fail");
            return false;
        }
    }

    // acl.json is dump or profiling config file
    if (aclInit("test_data/config/acl.json") != ACL_SUCCESS) {
        ERROR_LOG("acl init failed");
        return false;
    }

    if (aclrtSetDevice(deviceId) != ACL_SUCCESS) {
        ERROR_LOG("Set device failed. deviceId is %d", deviceId);
        (void)aclFinalize();
        return false;
    }
    INFO_LOG("Set device[%d] success", deviceId);

    // runMode is ACL_HOST which represents app is running in host
    // runMode is ACL_DEVICE which represents app is running in device
    aclrtRunMode runMode;
    if (aclrtGetRunMode(&runMode) != ACL_SUCCESS) {
        ERROR_LOG("Get run mode failed");
        DestoryResource();
        return false;
    }
    g_isDevice = (runMode == ACL_DEVICE);

    if (aclopSetModelDir("op_models") != ACL_SUCCESS) {
        ERROR_LOG("Load single op model failed");
        DestoryResource();
        return false;
    }

    return true;
}

int main()
{
    if (!InitResource()) {
        ERROR_LOG("Init resource failed");
        return FAILED;
    }
    INFO_LOG("Init resource success");

    if (!RunAddOp()) {
        DestoryResource();
        return FAILED;
    }

    DestoryResource();

    return SUCCESS;
}
