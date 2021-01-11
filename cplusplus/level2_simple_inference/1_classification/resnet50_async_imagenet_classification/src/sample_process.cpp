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
#include <thread>
#include <sstream>
#include "acl/acl.h"
#include "utils.h"
#include "model_process.h"

using namespace std;
extern bool g_isDevice;
extern size_t g_callbackInterval;
static bool g_isExit = false;

aclrtContext SampleProcess::context_ = nullptr;
aclrtStream SampleProcess::stream_ = nullptr;

SampleProcess::SampleProcess() :deviceId_(0)
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
        ERROR_LOG("acl init failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("acl init success");

    // set device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl set device %d failed, errorCode = %d", deviceId_, static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("set device %d success", deviceId_);

    // create context (set current)
    ret = aclrtCreateContext(&context_, deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl create context failed, deviceId = %d, errorCode = %d",
            deviceId_, static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("create context success");

    // create stream
    ret = aclrtCreateStream(&stream_);
    if (ret != ACL_ERROR_NONE) {
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
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl get run mode failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    g_isDevice = (runMode == ACL_DEVICE);
    INFO_LOG("get run mode success");
    return SUCCESS;
}

void SampleProcess::ProcessCallback(void *arg)
{
    aclrtSetCurrentContext(context_);
    while (g_callbackInterval != 0) {
        // timeout value is 100ms
        (void)aclrtProcessReport(100);
        if(*(static_cast<bool *>(arg)) == true) {
            return;
        }
    }
}

Result SampleProcess::Process()
{
    // model init
    ModelProcess modelProcess(stream_);
    const char* omModelPath = "../model/resnet50.om";
    Result ret = modelProcess.LoadModel(omModelPath);
    if (ret != SUCCESS) {
        ERROR_LOG("load model from file failed");
        return FAILED;
    }

    ret = modelProcess.CreateModelDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("create model description failed");
        return FAILED;
    }

    ret = modelProcess.InitMemPool();
    if (ret != SUCCESS) {
        ERROR_LOG("init memory pool failed");
        return FAILED;
    }
    INFO_LOG("init memory pool success");

    // subscribe report
    g_isExit = false;
    std::thread td(ProcessCallback, &g_isExit);
    std::ostringstream oss;
    oss << td.get_id();
    uint64_t tid = std::stoull(oss.str());
    aclError aclRt = aclrtSubscribeReport(static_cast<uint64_t>(tid), stream_);
    if (aclRt != ACL_ERROR_NONE) {
        g_isExit = true;
        td.join();
        ERROR_LOG("acl subscribe report failed");
        return FAILED;
    }
    INFO_LOG("subscribe report success");

    ret = modelProcess.ExecuteAsync();
    if (ret != SUCCESS) {
        g_isExit = true;
        td.join();
        (void)aclrtUnSubscribeReport(static_cast<uint64_t>(tid), stream_);
        ERROR_LOG("model async execute failed");
        return FAILED;
    }
    ret = modelProcess.SynchronizeStream();
    if (ret != SUCCESS) {
        g_isExit = true;
        td.join();
        (void)aclrtUnSubscribeReport(static_cast<uint64_t>(tid), stream_);
        ERROR_LOG("synchronize stream failed");
        return FAILED;
    }

    INFO_LOG("model execute success");

    g_isExit = true;
    td.join();

    // unsubscribe report
    aclRt = aclrtUnSubscribeReport(static_cast<uint64_t>(tid), stream_);
    if (aclRt != ACL_ERROR_NONE) {
        ERROR_LOG("acl unsubscribe report failed");
        return FAILED;
    }
    INFO_LOG("unsubscribe report success");

    return SUCCESS;
}

void SampleProcess::DestroyResource()
{
    aclError ret;
    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy stream failed, errorCode = %d", static_cast<int32_t>(ret));
        }
        stream_ = nullptr;
    }
    INFO_LOG("end to destroy stream");

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy context failed, errorCode = %d", static_cast<int32_t>(ret));
        }
        context_ = nullptr;
    }
    INFO_LOG("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("reset device %d failed, errorCode = %d", deviceId_, static_cast<int32_t>(ret));
    }
    INFO_LOG("end to reset device %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("finalize acl failed, errorCode = %d", static_cast<int32_t>(ret));
    }
    INFO_LOG("end to finalize acl");

}
