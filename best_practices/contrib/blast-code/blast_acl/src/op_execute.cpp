/**
* @file op_execute.cpp
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
#include <fstream>
#include <string>
#include <climits>
#include <cstring>
#include <cstdio>

#include "acl/acl.h"
#include "op_runner.h"

#include "common.h"

bool CreateOpDesc(OpTestDesc &desc)
{
    if (desc.inputShape.size() < 0 ||
        desc.inputShape.size() != desc.inputDataType.size() ||
        desc.inputShape.size() != desc.inputFormat.size()) {
        ERROR_LOG("Input operator desc error");
        return false;
    }

     if (desc.outputShape.size() <= 0 ||
        desc.outputShape.size() != desc.outputDataType.size() ||
        desc.outputShape.size() != desc.outputFormat.size()) {
        ERROR_LOG("Output operator desc error");
        return false;
    }

    for (std::size_t i = 0; i < desc.inputShape.size(); i++) {
        desc.AddInputTensorDesc(desc.inputDataType[i], desc.inputShape[i].size(),
            desc.inputShape[i].data(), desc.inputFormat[i]);
    }

    for (std::size_t i = 0; i < desc.outputShape.size(); i++) {
        desc.AddOutputTensorDesc(desc.outputDataType[i], desc.outputShape[i].size(),
            desc.outputShape[i].data(), desc.outputFormat[i]);
    }

    for (auto &x : desc.opAttrVec) {
        desc.AddTensorAttr(x);
    }
    return true;
}

bool SetInputData(OpRunner &runner)
{
    for (size_t i = 0; i < runner.NumInputs(); ++i) {
        size_t fileSize;
        if (runner.GetOpTestDesc().inputFilePath[i] == "") {
            INFO_LOG("Input[%zu] is an optional input.", i);
            continue;
        }
        std::string filePath = runner.GetOpTestDesc().inputFilePath[i] + ".bin";
        bool result = ReadFile(filePath, fileSize, runner.GetInputBuffer<void>(i), runner.GetInputSize(i));
        if (!result) {
            ERROR_LOG("Failed to read input[%zu].", i);
            return false;
        }
        char realPath[PATH_MAX];
        if (realpath(filePath.c_str(), realPath)) {
            INFO_LOG("Set input[%zu] from '%s' success.", i, realPath);
        }else {
            ERROR_LOG("The file '%s' is not exist.", filePath.c_str());
            return false;
        }
        bool isConst = runner.GetOpTestDesc().inputConst[i];
        if (isConst) {
            int setConstStatus = aclSetTensorConst(runner.GetOpTestDesc().inputDesc[i], runner.GetInputBuffer<void>(i), runner.GetInputSize(i));
            if (setConstStatus != 0) {
                ERROR_LOG("Set Const[%d] failed.", setConstStatus);
                return false;
            }
        }
    }

    return true;
}

bool ProcessOutputData(OpRunner &runner)
{
    for (size_t i = 0; i < runner.NumOutputs(); ++i) {
        INFO_LOG("Output[%zu]:", i);

        std::string filePath = runner.GetOpTestDesc().outputFilePath[i] + ".bin";
        if (!WriteFile(filePath, runner.GetOutputBuffer<void>(i), runner.GetOutputSize(i))) {
            ERROR_LOG("Failed to write output[%zu].", i);
            return false;
        }
        // char realPath[PATH_MAX];
        // if (realpath(filePath.c_str(), realPath)) {

        //     INFO_LOG("Write output[%zu] success. output file = %s", i, realPath);
        // }else {
        //     ERROR_LOG("The file '%s' is not exist.", filePath.c_str());
        //     return false;
        // }

    }
    return true;
}

bool DoRunOp(OpRunner &opRunner)
{
    if (!opRunner.Init()) {
        ERROR_LOG("Failed to init OpRunner.");
        return false;
    }

    // Load inputs
    if (!SetInputData(opRunner)) {
        ERROR_LOG("Failed to set input data.");
        return false;
    }


    // Run op
    if (!opRunner.RunOp()) {
        ERROR_LOG("Failed to run op test.");
        return false;
    }

    // process output data
    if (!ProcessOutputData(opRunner)) {
        ERROR_LOG("Failed to process output data.");
        return false;
    }

    INFO_LOG("Run op success");

    return true;
}

bool OpExecute(OpTestDesc &opDesc, uint32_t deviceId = 0)
{
    // create and init op desc
    if (CreateOpDesc(opDesc) == false) {
        return false;
    }
    // create Runner
    OpRunner opRunner(&opDesc);
    if (!opRunner.CheckDeviceCount(deviceId)) {
        ERROR_LOG("Failed to get device count.");
        return false;
    }

    if (!opRunner.OpExecuteInit()) {
        ERROR_LOG("Failed to init operator execute.");
        return false;
    }

    if (!opRunner.SetOpExecuteDevice(deviceId)) {
        ERROR_LOG("Failed to set device.");
        return false;
    }
    // get acl run mode.
    if (!opRunner.GetDeviceRunMode()) {
        ERROR_LOG("Failed to get device run mode.");
        return false;
    }

    if (!DoRunOp(opRunner)) {
        if (!opRunner.ResetDevice(deviceId)) {
            ERROR_LOG("Failed to reset device.");
        }
        return false;
    }

    return true;
}
