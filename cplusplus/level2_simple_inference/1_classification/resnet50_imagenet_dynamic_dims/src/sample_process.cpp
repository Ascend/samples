/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at

* http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
#include <iostream>
#include <vector>
#include <memory>
#include <unistd.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgproc/types_c.h"
#include "sample_process.h"
#include "acl/acl.h"
#include "utils.h"

using namespace std;
namespace {
    string g_testFile = "../data";
    const char *g_omModelPath = "../model/resnet50.om";
    int g_classNum = 1001;
    int g_batchSize = 2;
    int g_channels = 3;
    int g_modelWidth224 = 224;
    int g_modelHeight224 = 224;
    int g_modelWidth200 = 200;
    int g_modelHeight200 = 200;
}

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
    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl get run mode failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    isDevice_ = (runMode_ == ACL_DEVICE);
    modelProcess_.GetRunMode(runMode_);
    INFO_LOG("get run mode success");

    // model init
    Result modelRet = modelProcess_.LoadModel(g_omModelPath);
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadModel failed");
        return FAILED;
    }

    modelRet = modelProcess_.CreateModelDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateModelDesc failed");
        return FAILED;
    }

    modelRet = modelProcess_.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed");
        return FAILED;
    }
    return SUCCESS;
}

Result SampleProcess::Process()
{
    size_t devBufferSize;
    void *inputBuff = nullptr;
    int modelHeight = 0;
    int modelWidth = 0;
    aclError ret = modelProcess_.GetInputSizeByIndex(0, devBufferSize);
    if (ret != SUCCESS) {
        ERROR_LOG("execute GetInputSizeByIndex failed");
        return FAILED;
    }
    if (!isDevice_) {
        ret = aclrtMallocHost(&inputBuff, devBufferSize);
        if (inputBuff == nullptr) {
            ERROR_LOG("aclrtMallocHost inputBuff failed, errorCode = %d.", static_cast < int32_t > (ret));
            return FAILED;
        }
    } else {
        aclError ret = aclrtMalloc(&inputBuff, devBufferSize, ACL_MEM_MALLOC_HUGE_FIRST);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("aclrtMalloc device buffer failed. size is %zu, errorCode is %d", devBufferSize,
                static_cast<int32_t>(ret));
            return FAILED;
        }
    }
    for (size_t index = 0; index < g_batchSize; ++index) {
        if (index == 0) {
            modelHeight = g_modelHeight224;
            modelWidth = g_modelWidth224;
            INFO_LOG("ModelSetDynamicInfo g_batchSize:%d, g_channels:%d, modelHeight:%d, modelWidth:%d",
                     g_batchSize, g_channels, modelHeight, modelWidth);
        } else if (index == 1) {
            modelHeight = g_modelHeight200;
            modelWidth = g_modelWidth200;
            INFO_LOG("ModelSetDynamicInfo g_batchSize:%d, g_channels:%d, modelHeight:%d, modelWidth:%d",
                     g_batchSize, g_channels, modelHeight, modelWidth);
        }

        ret = aclrtMemset(inputBuff, devBufferSize, 0, devBufferSize);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("aclrtMemset failed");
        }
        uint32_t oneBatchFileSize = ReadOneBatchPicHwc(g_testFile, inputBuff, g_batchSize, modelWidth, modelHeight);

        if (oneBatchFileSize > devBufferSize) {
            ERROR_LOG("ReadOneBatch failed");
            return FAILED;
        }

        void *imageInfoBuf = Utils::MemcpyToDeviceBuffer(inputBuff, devBufferSize, runMode_);
        if (imageInfoBuf == nullptr) {
            ERROR_LOG("MemcpyToDeviceBuffer failed");
            return FAILED;
        }
        
        ret = modelProcess_.CreateInput(imageInfoBuf, oneBatchFileSize);
        if (ret != SUCCESS) {
            ERROR_LOG("execute CreateInput failed");
            return FAILED;
        }

        ret = modelProcess_.ModelSetDynamicInfo(g_batchSize, modelHeight, modelWidth, g_channels);

        if (ret != ACL_SUCCESS) {
            ERROR_LOG("ModelSetDynamicInfo  failed, errorCode is %d", static_cast<int32_t>(ret));
            return FAILED;
        }
        ret = modelProcess_.Execute();
        if (ret != SUCCESS) {
            ERROR_LOG("execute inference failed");
            return FAILED;
        }
        modelProcess_.DestroyInput();
        modelProcess_.OutputModelResultSoftMax(g_classNum, g_batchSize);

    }
    aclrtFree(inputBuff);
    modelProcess_.DestroyOutput();
    return SUCCESS;
}

uint32_t SampleProcess::ReadOneBatch(string inputImageDir, void *&inputBuff, int batchSize)
{
    vector<string> fileVec;
    Utils::GetAllFiles(inputImageDir, fileVec);
    if (fileVec.empty()) {
        INFO_LOG("Failed to deal all empty path=%s.", inputImageDir.c_str());
        return FAILED;
    }
    uint32_t fileSize = 0;
    uint32_t batchFileSize = 0;
    uint8_t i = 0;
    for (i; i < batchSize; i++) {
        auto ret = Utils::ReadBinFile(fileVec[i], inputBuff, fileSize);
        if (ret != SUCCESS) {
            ERROR_LOG("read bin file failed, file name is %s", fileVec[i].c_str());
            return FAILED;
        }
        INFO_LOG("read bin file , file name is %s", fileVec[i].c_str());
        inputBuff = inputBuff + fileSize;
        batchFileSize = batchFileSize + fileSize;

        if (i == (batchSize - 1)) {
            INFO_LOG("read bin file Num =%d.", i + 1);
            break;
        } else if (i == (fileVec.size() - 1)) {
            WARN_LOG("read bin file Num =%d.", i + 1);
            break;
        }
    }
    inputBuff = inputBuff - (i + 1) * fileSize;
    INFO_LOG("ReadBinFile batchFileSize = %d", batchFileSize);
    return batchFileSize;
}

uint32_t SampleProcess::ReadOneBatchPicHwc(string inputImageDir, void *&inputBuff, int batchSize, int resizeWidth, int resizeHeight)
{
    vector<string> fileVec;
    Utils::GetAllFiles(inputImageDir, fileVec);
    if (fileVec.empty()) {
        INFO_LOG("Failed to deal all empty path=%s.", inputImageDir.c_str());
        return FAILED;
    }
    uint32_t fileSize = 0;
    uint32_t batchFileSize = 0;

    for (uint8_t i = 0; i < batchSize; i++) {
        cv::Mat frame, reiszedImage, rsImageF32;
        frame=cv::imread(fileVec[i]);
        INFO_LOG("read pic file , file name is %s", fileVec[i].c_str());
        cv::resize(frame, reiszedImage, cv::Size(resizeWidth, resizeHeight));

        cv::Mat shipRGB;
        cv::cvtColor(reiszedImage, shipRGB, cv::COLOR_BGR2RGB);
        shipRGB.convertTo(rsImageF32, CV_32FC3);

        std::vector<cv::Mat> channels;
        cv::split(rsImageF32, channels);
        channels[2] -=123.0;
        channels[1] -=117.0;
        channels[0] -=104.0;
        channels[2] *=0.0142857142857143;
        channels[1] *=0.0142857142857143;
        channels[0] *=0.0142857142857143;

        cv::Mat result;
        cv::merge(channels, result);
        fileSize = ((resizeWidth) * (resizeHeight) * 3 * 4);
        memcpy(static_cast<uint8_t *>(inputBuff),  result.data, fileSize);

        inputBuff = inputBuff + fileSize;
        batchFileSize = batchFileSize + fileSize;
    }

    inputBuff = inputBuff - batchFileSize;
    INFO_LOG("Read Pic File batchFileSize = %d", batchFileSize);
    return batchFileSize;
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
