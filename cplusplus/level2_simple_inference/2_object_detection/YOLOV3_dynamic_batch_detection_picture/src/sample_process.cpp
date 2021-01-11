/**
* @file sample_process.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include "sample_process.h"

using namespace std;
extern bool g_isDevice;

SampleProcess::SampleProcess() : deviceId_(0), context_(nullptr), stream_(nullptr)
{
}

SampleProcess::~SampleProcess()
{
    DestroyResource();
}

Result SampleProcess::InitResource()
{
    // ACL init
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl init failed, errorCode = %d.", static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("acl init success.");

    // set device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl set device %d failed, errorCode = %d.",
            deviceId_, static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("set device %d success.", deviceId_);

    // create context (set current)
    ret = aclrtCreateContext(&context_, deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl create context failed, deviceId = %d, errorCode = %d.",
            deviceId_, static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("create context success.");

    // create stream
    ret = aclrtCreateStream(&stream_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl create stream failed, deviceId = %d, errorCode = %d.",
            deviceId_, static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("create stream success.");

    // get run mode
    aclrtRunMode runMode;
    ret = aclrtGetRunMode(&runMode);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl get run mode failed, errorCode = %d.", static_cast<int32_t>(ret));
        return FAILED;
    }
    g_isDevice = (runMode == ACL_DEVICE);
    INFO_LOG("get run mode success.");
    return SUCCESS;
}

void SampleProcess::DestroyResource()
{
    aclError ret;
    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy stream failed, errorCode = %d.", static_cast<int32_t>(ret));
        }
        stream_ = nullptr;
    }
    INFO_LOG("end to destroy stream.");

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy context failed, errorCode = %d.", static_cast<int32_t>(ret));
        }
        context_ = nullptr;
    }
    INFO_LOG("end to destroy context.");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("reset device %d failed, errorCode = %d.",
            deviceId_, static_cast<int32_t>(ret));
    }
    INFO_LOG("end to reset device: %d.", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("finalize acl failed, errorCode = %d.", static_cast<int32_t>(ret));
    }
    INFO_LOG("end to finalize acl.");

}

Result SampleProcess::CheckAndFillDynamicPara(int argc, char **argv, DynamicInfo &dynamicInfo)
{
    if (argc == 2) {
        uint64_t dynamicBatchNum = atoll(argv[1]);
        if ((dynamicBatchNum != 1) &&
            (dynamicBatchNum != 2) &&
            (dynamicBatchNum != 4) &&
            (dynamicBatchNum != 8)) {
            ERROR_LOG("invalid dynamic batch num, should be 1,2,4 or 8.");
            return FAILED;
        }
        dynamicInfo.dynamicArr[0] = dynamicBatchNum;
        dynamicInfo.dynamicType = DYNAMIC_BATCH;
    } else {
        uint64_t height = atoll(argv[1]);
        uint64_t width = atoll(argv[2]);
        if (!(((height == 416) && (width == 416)) ||
            ((height == 832) && (width == 832)) ||
            ((height == 1248) && (width == 1248)))) {
            ERROR_LOG("invalid dynamic hw, should be 416*416,832*832,1248*1248.");
            return FAILED;
        }
        dynamicInfo.dynamicArr[0] = height;
        dynamicInfo.dynamicArr[1] = width;
        dynamicInfo.dynamicType = DYNAMIC_HW;
    }

    return SUCCESS;
}

Result SampleProcess::SelectDynamicInputPara(DynamicType dynamicType, string &omModelPath,
    std::map<uint64_t, std::string> &testFile)
{
    if (dynamicType == DYNAMIC_BATCH) { // dynamic batch
        omModelPath = "../model/yolov3_dynamic_batch.om";
        testFile = {
            {1, "../data/input_float32_1x3x416x416.bin.in"},
            {2, "../data/input_float32_2x3x416x416.bin.in"},
            {4, "../data/input_float32_4x3x416x416.bin.in"},
            {8, "../data/input_float32_8x3x416x416.bin.in"}
        };
    } else if (dynamicType == DYNAMIC_HW) { // dynamic hw
        omModelPath = "../model/yolov3_dynamic_hw.om";
        testFile = {
            {416, "../data/input_float32_1x3x416x416.bin.in"},
            {832, "../data/input_float32_1x3x832x832.bin.in"},
            {1248, "../data/input_float32_1x3x1248x1248.bin.in"}
        };
    } else {
        ERROR_LOG("invalid dynamic type %d", static_cast<int32_t>(dynamicType));
        return FAILED;
    }

    return SUCCESS;
}

Result SampleProcess::Process(const DynamicInfo &dynamicInfo)
{
    ModelProcess modelProcess;
    string omModelPath = "";
    std::map<uint64_t, std::string> testFile;

    Result ret = SelectDynamicInputPara(dynamicInfo.dynamicType, omModelPath, testFile);
    if (ret != SUCCESS) {
        ERROR_LOG("select dynamic input parameter failed.");
        return FAILED;
    }

    // model init
    ret = modelProcess.LoadModel(omModelPath.c_str());
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadModel failed.");
        return FAILED;
    }

    ret = modelProcess.CreateDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateDesc failed.");
        return FAILED;
    }

    if (testFile.find(dynamicInfo.dynamicArr[0]) == testFile.end()) {
        ERROR_LOG("no corresponding input file for input size %lu", dynamicInfo.dynamicArr[0]);
        return FAILED;
    }
    std::string &inputFile = testFile[dynamicInfo.dynamicArr[0]];
    INFO_LOG("start to process file: %s", inputFile.c_str());
    // create input
    uint32_t devBufferSize;
    void *picDevBuffer = Utils::GetDeviceBufferOfFile(inputFile, devBufferSize);
    if (picDevBuffer == nullptr) {
        ERROR_LOG("get pic device buffer failed, file name[%s].", inputFile.c_str());
        return FAILED;
    }
    ret = modelProcess.CreateInput(picDevBuffer, devBufferSize);
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateInput failed.");
        return FAILED;
    }

    // create output
    ret = modelProcess.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed.");
        return FAILED;
    }

    ret = modelProcess.SetDynamicSize(dynamicInfo);
    if (ret != SUCCESS) {
        ERROR_LOG("execute SetDynamicSize failed.");
        return FAILED;
    }

    modelProcess.PrintModelDescInfo(dynamicInfo.dynamicType);
    modelProcess.PrintModelCurOutputDims();

    ret = modelProcess.Execute();
    if (ret != SUCCESS) {
        ERROR_LOG("execute inference failed.");
        return FAILED;
    }

    modelProcess.DumpModelOutputResult();

    return SUCCESS;
}
