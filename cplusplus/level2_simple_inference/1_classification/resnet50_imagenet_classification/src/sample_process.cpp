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
#include <iostream>
#include "model_process.h"
#include "acl/acl.h"
#include "utils.h"
using namespace std;
extern bool g_isDevice;

SampleProcess::SampleProcess() :deviceId_(0), context_(nullptr), stream_(nullptr)
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
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl init failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("acl init success");

    // set device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl set device %d failed, errorCode = %d", deviceId_, static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("set device %d success", deviceId_);

    // create context (set current)
    ret = aclrtCreateContext(&context_, deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl create context failed, deviceId = %d, errorCode = %d",
            deviceId_, static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("create context success");

    // create stream
    ret = aclrtCreateStream(&stream_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl create stream failed, deviceId = %d, errorCode = %d",
            deviceId_, static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("create stream success");

    // get run mode
    // runMode is ACL_HOST which represents app is running in host
    // runMode is ACL_DEVICE which represents app is running in device
    aclrtRunMode runMode;
    ret = aclrtGetRunMode(&runMode);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl get run mode failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    g_isDevice = (runMode == ACL_DEVICE);
    INFO_LOG("get run mode success");
    return SUCCESS;
}

Result SampleProcess::Process()
{
    // model init
    ModelProcess modelProcess;
    const char* omModelPath = "../model/resnet50.om";
    Result ret = modelProcess.LoadModel(omModelPath);
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadModel failed");
        return FAILED;
    }

    ret = modelProcess.CreateModelDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateModelDesc failed");
        return FAILED;
    }

    string testFile[] = {
        "../data/dog1_1024_683.bin",
        "../data/dog2_1024_683.bin"
    };

    size_t devBufferSize;
    void *picDevBuffer = nullptr;
    ret = modelProcess.GetInputSizeByIndex(0, devBufferSize);
    if (ret != SUCCESS) {
        ERROR_LOG("execute GetInputSizeByIndex failed");
        return FAILED;
    }

    aclError aclRet = aclrtMalloc(&picDevBuffer, devBufferSize, ACL_MEM_MALLOC_NORMAL_ONLY);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("malloc device buffer failed. size is %zu, errorCode is %d",
            devBufferSize, static_cast<int32_t>(aclRet));
        return FAILED;
    }

    ret = modelProcess.CreateInput(picDevBuffer, devBufferSize);
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateInput failed");
        aclrtFree(picDevBuffer);
        return FAILED;
    }

    ret = modelProcess.CreateOutput();
    if (ret != SUCCESS) {
        aclrtFree(picDevBuffer);
        ERROR_LOG("execute CreateOutput failed");
        return FAILED;
    }

    for (size_t index = 0; index < sizeof(testFile) / sizeof(testFile[0]); ++index) {
        INFO_LOG("start to process file:%s", testFile[index].c_str());
        // copy image data to device buffer
        ret = Utils::MemcpyFileToDeviceBuffer(testFile[index], picDevBuffer, devBufferSize);
        if (ret != SUCCESS) {
            aclrtFree(picDevBuffer);
            ERROR_LOG("memcpy device buffer failed, index is %zu", index);
            return FAILED;
        }

        ret = modelProcess.Execute();
        if (ret != SUCCESS) {
            ERROR_LOG("execute inference failed");
            aclrtFree(picDevBuffer);
            return FAILED;
        }

        // Print the top 5 confidence values with indexes.
        // Use function [DumpModelOutputResult] if you want to dump results to file in the current directory.
        modelProcess.OutputModelResult();

    }
    // release model input output
    modelProcess.DestroyInput();
    modelProcess.DestroyOutput();

    aclrtFree(picDevBuffer);
    return SUCCESS;
}

void SampleProcess::DestroyResource()
{
    aclError ret;
    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("destroy stream failed, errorCode = %d", static_cast<int32_t>(ret));
        }
        stream_ = nullptr;
    }
    INFO_LOG("end to destroy stream");

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("destroy context failed, errorCode = %d", static_cast<int32_t>(ret));
        }
        context_ = nullptr;
    }
    INFO_LOG("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("reset device %d failed, errorCode = %d", deviceId_, static_cast<int32_t>(ret));
    }
    INFO_LOG("end to reset device %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("finalize acl failed, errorCode = %d", static_cast<int32_t>(ret));
    }
    INFO_LOG("end to finalize acl");
}
