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
#include <sstream>
#include "venc_process.h"
#include "acl/acl.h"
#include "utils.h"
using namespace std;
static bool runFlag = true;

SampleProcess::SampleProcess() : deviceId_(0), context_(nullptr), thread_(),
    outFolder_((char*)"output/"), format_(PIXEL_FORMAT_YUV_SEMIPLANAR_420),
    enType_(H265_MAIN_LEVEL)
{
}

SampleProcess::~SampleProcess()
{
    DestroyResource();
}

void *ThreadFunc(aclrtContext sharedContext)
{
    if (sharedContext == nullptr) {
        ERROR_LOG("sharedContext can not be nullptr");
        return ((void*)(-1));
    }
    INFO_LOG("use shared context for this thread");
    aclError ret = aclrtSetCurrentContext(sharedContext);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclrtSetCurrentContext failed, errorCode = %d", static_cast<int32_t>(ret));
        return ((void*)(-1));
    }

    INFO_LOG("process callback thread start ");
    while (runFlag) {
        // Notice: timeout 1000ms
        (void)aclrtProcessReport(1000);
    }
    return (void*)0;
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

    // get run mode
    // runMode is ACL_HOST which represents app is running in host
    // runMode is ACL_DEVICE which represents app is running in device
    aclrtRunMode runMode;
    ret = aclrtGetRunMode(&runMode);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl get run mode failed, errorCode = %d.", static_cast<int32_t>(ret));
        return FAILED;
    }
    bool isDivece = (runMode == ACL_DEVICE);
    RunStatus::SetDeviceStatus(isDivece);
    INFO_LOG("get run mode success");

    return SUCCESS;
}

Result SampleProcess::DoVencProcess()
{

    // create process callback thread
    thread_ = std::thread(ThreadFunc, context_);
    std::ostringstream oss;
    oss << thread_.get_id();
    uint64_t tid = std::stoull(oss.str());
    INFO_LOG("create process callback thread successfully, threadId = %lu", tid);

    // check output folder
    auto ret = Utils::CheckAndCreateFolder(outFolder_);
    if (ret != SUCCESS) {
        ERROR_LOG("mkdir out folder error.");
        return FAILED;
    }

    // dvpp init
    VencProcess vencProcess;
    ret = vencProcess.InitResource(tid, format_, enType_);
    if (ret != SUCCESS) {
        ERROR_LOG("init dvpp resource failed");
        return FAILED;
    }

    // create input
    const char *fileName = "../data/dvpp_venc_128x128_nv12.yuv";
    void *inBufferDev = nullptr;
    uint32_t inBufferSize = 0;
    if (!Utils::ReadFileToDeviceMem(fileName, inBufferDev, inBufferSize)) {
        ERROR_LOG("read file %s to device mem failed.\n", fileName);
        return FAILED;
    }
    vencProcess.SetInput(inBufferDev, inBufferSize);

    // venc process
    ret = vencProcess.Process();
    if (ret != SUCCESS) {
        ERROR_LOG("dvpp ProcessVenc failed");
        return FAILED;
    }

    return SUCCESS;
}

void SampleProcess::DestroyResource()
{
    aclError ret;
    // destory thread
    runFlag = false;
    if (thread_.joinable()) {
        thread_.join();
    }

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
