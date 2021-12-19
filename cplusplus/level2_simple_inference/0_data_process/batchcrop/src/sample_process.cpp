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
#include "dvpp_process.h"

using namespace std;

SampleProcess::SampleProcess() : deviceId_(0), context_(nullptr), stream_(nullptr)
{
}

SampleProcess::~SampleProcess()
{
    DestroyResource();
}

void SampleProcess::SetDeviceId(int deviceId)
{
    deviceId_ = deviceId;
}

vector<InputArea> SampleProcess::BatchInputCrop()
{
    vector<InputArea> batchInputCrop;
    InputArea inputArea;
    inputArea.inputFileName = "../data/dvpp_vpc_1920x1080_nv12.yuv";
    inputArea.inputFormat = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    inputArea.outputFormat = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    inputArea.inputWidth = 1920;
    inputArea.inputHeight = 1080;
    batchInputCrop.push_back(inputArea);

    return batchInputCrop;
}

Result SampleProcess::InitResource()
{
    aclError ret = aclInit(NULL);
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
    return SUCCESS;
}

Result SampleProcess::BatchCropProcess()
{
    DvppProcess dvppProcess(stream_);
    Result ret = dvppProcess.InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("init dvpp resource failed");
        return FAILED;
    }

    vector<InputArea> batchInputCrop = BatchInputCrop();
    // crop eight images from one image
    Result sizeStatus = dvppProcess.SetBatchSize(1, 8);
    if (sizeStatus == FAILED){
        return FAILED;
    }
    dvppProcess.SetBatchInputCrop(batchInputCrop);
    // crop image size is 224 *224, source image size is 1920 * 1080
    dvppProcess.SetOutArea(224, 224);
    const char *outputFileName = "./cropName";
    dvppProcess.SetOutputFileName(outputFileName);
    ret = dvppProcess.ProcessBatchCrop();

    if (ret != SUCCESS) {
        dvppProcess.DestroyBatchCropResource();
        ERROR_LOG("ProcessBatchCrop failed.");
        return FAILED;
    }
    INFO_LOG("ProcessBatchCrop success.");
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
