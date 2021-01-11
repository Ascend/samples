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
#include <string>
#include "dvpp_process.h"
#include "model_process.h"
#include "acl/acl.h"
#include "utils.h"
using namespace std;

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

Result SampleProcess::JpegeProcess(DvppType dvpptype)
{
    DvppProcess dvppProcess(stream_);
    dvppProcess.SetDvppType(dvpptype);
    Result ret = dvppProcess.InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("init dvpp resource failed");
        return FAILED;
    }

    ret = dvppProcess.ProcessJpegE();
    if (ret != SUCCESS) {
        ERROR_LOG("process jpege failed");
        return FAILED;
    }
    return SUCCESS;
}

Result SampleProcess::Resize8kProcess(DvppType dvpptype)
{
    INFO_LOG("dvpp process 8k resize begin");
    DvppProcess dvppProcess(stream_);
    dvppProcess.SetDvppType(dvpptype);
    Result ret = dvppProcess.InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("init dvpp resource failed");
        return FAILED;
    }

    ret = dvppProcess.Process8kResize();
    if (ret != SUCCESS) {
        ERROR_LOG("dvpp process 8k resize failed");
        return FAILED;
    }
    INFO_LOG("dvpp process 8k resize success");

    return SUCCESS;
}

// jpegd -> vpc -> model execute
Result SampleProcess::JpegdProcess(DvppType dvpptype)
{
    const char* omModelPath = "../model/resnet50_aipp.om";
    std::string modelOutputBinfileName = "./result/model_output_";
    std::string dvppOutputfileName = "./result/dvpp_output_";

    // dvpp init
    DvppProcess dvppProcess(stream_);
    dvppProcess.SetDvppType(dvpptype);
    Result ret = dvppProcess.InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("init dvpp resource failed");
        return FAILED;
    }

    // model init
    ModelProcess modelProcess;
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

    // input image
    PicDesc testPic[] = {
        {"../data/persian_cat_1024_1536_283.jpg", 0, 0},
        {"../data/wood_rabbit_1024_1061_330.jpg", 0, 0},
    };
    INFO_LOG( "-------------------------------------------");
    for (size_t index = 0; index < sizeof(testPic) / sizeof(testPic[0]); ++index) {
        INFO_LOG("start to process picture:%s", testPic[index].picName.c_str());
        // 1.dvpp process
        uint32_t devPicBufferSize;
        char *picDevBuffer = nullptr;
        // get input image data buffer
        ret = Utils::GetPicDevBuffer4JpegD(testPic[index], picDevBuffer, devPicBufferSize);
        if (ret != SUCCESS) {
            ERROR_LOG("get pic device buffer failed, index is %zu", index);
            return FAILED;
        }

        dvppProcess.SetInput4JpegD(picDevBuffer, devPicBufferSize, testPic[index]);

        ret = dvppProcess.InitDvppOutputPara(modelInputWidth, modelInputHeight);
        if (ret != SUCCESS) {
            ERROR_LOG("init dvpp output para failed");
            return FAILED;
        }

        ret = dvppProcess.Process();
        if (ret != SUCCESS) {
            ERROR_LOG("dvpp process failed");
            return FAILED;
        }

        (void)acldvppFree(picDevBuffer);
        picDevBuffer = nullptr;

        void *dvppOutputBuffer = nullptr;
        int dvppOutputSize;
        dvppProcess.GetDvppOutput(&dvppOutputBuffer, dvppOutputSize);

        std::string dvppOutputfileNameCur = dvppOutputfileName + std::to_string(index);
        (void)Utils::SaveDvppOutputData(dvppOutputfileNameCur.c_str(), dvppOutputBuffer, dvppOutputSize);

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

        aclmdlDataset *modelOutput = modelProcess.GetModelOutputData();
        if (modelOutput == nullptr) {
            ERROR_LOG("get model output data failed");
            return FAILED;
        }

        // process model output result
        std::string modelOutputBinfileNameCur = modelOutputBinfileName + std::to_string(index);
        ret = Utils::PullModelOutputData(modelOutput, modelOutputBinfileNameCur.c_str());
        if (ret != SUCCESS) {
            ERROR_LOG("pull model output data failed");
            return FAILED;
        }
        std::string modelOutputTxtfileNameCur = modelOutputBinfileNameCur + ".txt";
        ret = Utils::SaveModelOutputData(modelOutputBinfileNameCur.c_str(), modelOutputTxtfileNameCur.c_str());
        if (ret != SUCCESS) {
            ERROR_LOG("save model output data failed");
            return FAILED;
        }

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
