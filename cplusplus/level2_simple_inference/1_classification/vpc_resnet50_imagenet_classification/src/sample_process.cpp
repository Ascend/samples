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
#include "dvpp_process.h"
#include "model_process.h"
#include "singleOp_process.h"
#include "acl/acl.h"
#include "utils.h"
using namespace std;

SampleProcess::SampleProcess():deviceId_(0), context_(nullptr), stream_(nullptr)
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
    bool isDivece = (runMode == ACL_DEVICE);
    RunStatus::SetDeviceStatus(isDivece);
    INFO_LOG("get run mode success");
    return SUCCESS;
}

Result SampleProcess::Process()
{
    // dvpp init
    DvppProcess dvppProcess(stream_);
    Result ret = dvppProcess.InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("init dvpp resource failed");
        return FAILED;
    }

    // model init
    ModelProcess modelProcess;
    const char* omModelPath = "../model/resnet50_aipp.om";
    ret = modelProcess.LoadModel(omModelPath);
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadModel failed");
        return FAILED;
    }
    ret = modelProcess.CreateDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateDesc failed");
        return FAILED;
    }
    ret = modelProcess.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed");
        return FAILED;
    }
    int modelInputWidth;
    int modelInputHeight;
    ret = modelProcess.GetModelInputWH(modelInputWidth, modelInputHeight);
    if (ret != SUCCESS) {
        ERROR_LOG("execute GetModelInputWH failed");
        return FAILED;
    }

    // singleOp init
    SingleOpProcess singleOpProcess(stream_);
    const int modelOutputShape = 1000;
    ret = singleOpProcess.Init(modelOutputShape);
    if (ret != SUCCESS) {
        ERROR_LOG("singleOp init failed");
        return FAILED;
    }

    // input image
    PicDesc testPic[] = {
        {"../data/dog1_1024_683.jpg", 0, 0},
        {"../data/dog2_1024_683.jpg", 0, 0},
    };


    for (size_t index = 0; index < sizeof(testPic) / sizeof(testPic[0]); ++index) {
        INFO_LOG("start to process picture:%s", testPic[index].picName.c_str());
        // 1.dvpp process
        uint32_t devPicBufferSize;
        void *picDevBuffer = nullptr;
        // get input image data buffer
        ret = Utils::GetDeviceBufferOfPicture(testPic[index], picDevBuffer, devPicBufferSize);
        if (ret != SUCCESS) {
            ERROR_LOG("get pic device buffer failed, index is %zu", index);
            return FAILED;
        }
        dvppProcess.SetInput(picDevBuffer, devPicBufferSize, testPic[index]);

        ret = dvppProcess.InitDvppOutputPara(modelInputWidth, modelInputHeight);
        if (ret != SUCCESS) {
            ERROR_LOG("init dvpp output para failed");
            (void)acldvppFree(picDevBuffer);
            picDevBuffer = nullptr;
            return FAILED;
        }

        ret = dvppProcess.Process();
        if (ret != SUCCESS) {
            ERROR_LOG("dvpp process failed");
            (void)acldvppFree(picDevBuffer);
            picDevBuffer = nullptr;
            return FAILED;
        }

        (void)acldvppFree(picDevBuffer);
        picDevBuffer = nullptr;

        void *dvppOutputBuffer = nullptr;
        int dvppOutputSize;
        dvppProcess.GetDvppOutput(&dvppOutputBuffer, dvppOutputSize);

        // 2.model process
        ret = modelProcess.CreateInput(dvppOutputBuffer, dvppOutputSize);
        if (ret != SUCCESS) {
            ERROR_LOG("execute CreateInput failed");
            (void)acldvppFree(dvppOutputBuffer);
            return FAILED;
        }

        ret = modelProcess.Execute();
        if (ret != SUCCESS) {
            ERROR_LOG("execute inference failed");
            (void)acldvppFree(dvppOutputBuffer);
            return FAILED;
        }

        // release model input buffer
        (void)acldvppFree(dvppOutputBuffer);
        modelProcess.DestroyInput();

        const aclmdlDataset *modelOutput = modelProcess.GetModelOutputData();

        // 3.singleOp process
        ret = singleOpProcess.InitInput(modelOutput);
        if (ret != SUCCESS) {
            ERROR_LOG("singleOp init input failed");
            singleOpProcess.destroyResource();
            return FAILED;
        }

        ret = singleOpProcess.Process();
        if (ret != SUCCESS) {
            ERROR_LOG("singleOp process failed");
            singleOpProcess.destroyResource();
            return FAILED;
        }

        singleOpProcess.PrintResult();
        singleOpProcess.destroyResource();
    }

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
