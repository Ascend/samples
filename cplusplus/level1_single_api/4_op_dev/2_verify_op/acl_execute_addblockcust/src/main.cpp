/**
* Copyright (c) Huawei Technologies Co., Ltd. 2023. All rights reserved.
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
#include <cstdint>
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "acl/acl.h"
#include "op_runner.h"

#include "common.h"

OperatorDesc CreateOpDesc()
{
    // define operator
    std::vector<int64_t> inputShape0{2,2};
    std::vector<int64_t> inputShape1{2,2};
    std::vector<int64_t> outputShape{2,2};
    std::string opType = "AddBlockCust";
    aclDataType dataType = ACL_INT32;
    aclFormat format = ACL_FORMAT_ND;
    OperatorDesc opDesc(opType);
    opDesc.AddInputTensorDesc(dataType, inputShape0.size(), inputShape0.data(), format);
    opDesc.AddInputTensorDesc(dataType, inputShape1.size(), inputShape1.data(), format);
    opDesc.AddOutputTensorDesc(dataType, outputShape.size(), outputShape.data(), format);
    return opDesc;
}

bool SetInputData(OpRunner &runner)
{
    for (size_t i = 0; i < runner.NumInputs(); ++i) {
        size_t fileSize = 0;
        std::string filePath = "test_data/data/input_" + std::to_string(i) + ".bin";
        bool ret = ReadFile(filePath, fileSize,
            runner.GetInputBuffer<void>(i), runner.GetInputSize(i));
        if (!ret) {
            ERROR_LOG("Read input[%zu] failed", i);
            return false;
        }
        INFO_LOG("file Size is:%zu.", fileSize);
        INFO_LOG("Set input[%zu] from %s success.", i, filePath.c_str());
        INFO_LOG("Input[%zu]:", i);
        runner.PrintInput(i);
        std::cout << std::endl;
        std::vector<int64_t> inputShape = runner.GetInputShape(i);
        std::string shapeString;
        for (size_t j = 0; j < inputShape.size(); j++) {
            shapeString.append(std::to_string(inputShape[j]));
            shapeString.append(" ");
        }
        INFO_LOG("Input[%zu] shape is:%s", i, shapeString.c_str());
    }

    return true;
}

bool ProcessOutputData(OpRunner &runner)
{
    for (size_t i = 0; i < runner.NumOutputs(); ++i) {
        INFO_LOG("Output[%zu]:", i);
        runner.PrintOutput(i);
        std::cout << std::endl;
        std::string shapeString;
        std::vector<int64_t> outputShape = runner.GetOutputShape(i);
        for (size_t j = 0; j < outputShape.size(); j++) {
            shapeString.append(std::to_string(outputShape[j]));
            shapeString.append(" ");
        }
        INFO_LOG("Output[%zu] shape is:%s", i, shapeString.c_str());

        std::string filePath = "result_files/output_" + std::to_string(i) + ".bin";
        if (!WriteFile(filePath, runner.GetOutputBuffer<void>(i), runner.GetOutputSize(i))) {
            ERROR_LOG("Write output[%zu] failed.", i);
            return false;
        }

        INFO_LOG("Write output[%zu] success. output file = %s", i, filePath.c_str());
    }
    return true;
}

bool RunAddOp(bool isDevice)
{
    // Create op desc
    OperatorDesc opDesc = CreateOpDesc();

    // Create Runner
    OpRunner opRunner(&opDesc, isDevice);
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

    // Process output data
    if (!ProcessOutputData(opRunner)) {
        return false;
    }

    INFO_LOG("Run op success");
    return true;
}

int main()
{
    // create result files path
    std::string output = "./result_files";
    if (access(output.c_str(), 0) == -1) {
        int ret = mkdir(output.c_str(), 0700);
        if (ret == 0) {
            INFO_LOG("Make output directory successfully");
        }
        else {
            ERROR_LOG("Make output directory fail");
            return FAILED;
        }
    }

    // init acl json
    if (aclInit("test_data/config/acl.json") != ACL_SUCCESS) {
        ERROR_LOG("Init acl failed");
        return FAILED;
    }

    // set model path
    int deviceId = 0;
    if (aclopSetModelDir("op_models") != ACL_SUCCESS) {
        std::cerr << "Load single op model failed" << std::endl;
        (void) aclFinalize();
        return FAILED;
    }

    // set device id
    if (aclrtSetDevice(deviceId) != ACL_SUCCESS) {
        std::cerr << "Open device failed. device id = " << deviceId << std::endl;
        (void) aclFinalize();
        return FAILED;
    }
    INFO_LOG("Open device[%d] success", deviceId);

    aclrtRunMode runMode;
    if (aclrtGetRunMode(&runMode) != ACL_SUCCESS) {
        ERROR_LOG("Acl get run mode failed");
        (void) aclrtResetDevice(deviceId);
        (void) aclFinalize();
        return FAILED;
    }
    bool isDevice = (runMode == ACL_DEVICE);

    if (!RunAddOp(isDevice)) {
        (void) aclrtResetDevice(deviceId);
        (void) aclFinalize();
        return FAILED;
    }

    (void) aclrtResetDevice(deviceId);

    if (aclFinalize() != ACL_SUCCESS) {
        ERROR_LOG("Finalize acl failed");
        return FAILED;
    }

    return SUCCESS;
}
