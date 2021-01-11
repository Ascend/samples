/**
* Copyright 2020 Huawei Technologies Co., Ltd
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

* File sample_process.cpp
* Description: handle acl resource
*/
#include "utils.h"
#include <iostream>
#include <pthread.h>
#include "acl/acl.h"
#include "model_process.h"
#include "object_detect.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs/legacy/constants_c.h"

using namespace std;

ObjectDetect::ObjectDetect(const char* modelPath, uint32_t modelWidth,
uint32_t modelHeight)
:deviceId_(0), context_(nullptr), stream_(nullptr), modelWidth_(modelWidth),
modelHeight_(modelHeight), isInited_(false), imageDataBuf_(nullptr), imageInfoBuf_(nullptr){
    imageInfoSize_ = 0;
    modelPath_ = modelPath;
    imageDataSize_ = RGBU8_IMAGE_SIZE(modelWidth, modelHeight);
}

ObjectDetect::~ObjectDetect() {
    DestroyResource();
}

Result ObjectDetect::InitResource() {
    // ACL init
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("Acl init failed");
        return FAILED;
    }
    INFO_LOG("Acl init success");

    // open device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("Acl open device %d failed", deviceId_);
        return FAILED;
    }
    INFO_LOG("Open device %d success", deviceId_);

    // create context (set current)
    ret = aclrtCreateContext(&context_, deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl create context failed");
        return FAILED;
    }
    INFO_LOG("create context success");
        // create stream
        ret = aclrtCreateStream(&stream_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("acl create stream failed");
            return FAILED;
        }
        INFO_LOG("create stream success");
    //Gets whether the current application is running on host or Device
    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl get run mode failed");
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::InitModel(const char* omModelPath) {
    Result ret = model_.LoadModelFromFileWithMem(omModelPath);
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadModelFromFileWithMem failed");
        return FAILED;
    }

    ret = model_.CreateDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateDesc failed");
        return FAILED;
    }

    ret = model_.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed");
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::CreateModelInputdDataset()
{
    //Request image data memory for input model
    aclError aclRet = aclrtMalloc(&imageDataBuf_, imageDataSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("malloc device data buffer failed, aclRet is %d", aclRet);
        return FAILED;
    }
    //The second input to Yolov3 is the input image width and height parameter
    const float imageInfo[4] = {(float)modelWidth_, (float)modelHeight_,
    (float)modelWidth_, (float)modelHeight_};
    imageInfoSize_ = sizeof(imageInfo);
    if (runMode_ == ACL_HOST)
        imageInfoBuf_ = Utils::CopyDataHostToDevice((void *)imageInfo, imageInfoSize_);
    else
        imageInfoBuf_ = Utils::CopyDataDeviceToDevice((void *)imageInfo, imageInfoSize_);
    if (imageInfoBuf_ == nullptr) {
        ERROR_LOG("Copy image info to device failed");
        return FAILED;
    }
    //Use the applied memory to create the model and input dataset. After creation, only update the memory data for each frame of inference, instead of creating the input dataset every time
    Result ret = model_.CreateInput(imageDataBuf_, imageDataSize_,
    imageInfoBuf_, imageInfoSize_);
    if (ret != SUCCESS) {
        ERROR_LOG("Create mode input dataset failed");
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::Init() {
    //If it is already initialized, it is returned
    if (isInited_) {
        INFO_LOG("Classify instance is initied already!");
        return SUCCESS;
    }
    //Initializes the ACL resource
    Result ret = InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("Init acl resource failed");
        return FAILED;
    }
    //Initializes the model management instance
    ret = InitModel(modelPath_);
    if (ret != SUCCESS) {
        ERROR_LOG("Init model failed");
        return FAILED;
    }
    ret = CreateModelInputdDataset();
    if (ret != SUCCESS) {
        ERROR_LOG("Create image info buf failed");
        return FAILED;
    }

    isInited_ = true;
    return SUCCESS;
}

Result ObjectDetect::Inference(aclmdlDataset*& inferenceOutput, cv::Mat& reiszeMat) {
    //Copy the data into the cache of the input dataset
    aclrtMemcpyKind policy = (runMode_ == ACL_HOST)?
    ACL_MEMCPY_HOST_TO_DEVICE:ACL_MEMCPY_DEVICE_TO_DEVICE;
    aclError aclret = aclrtMemcpy(imageDataBuf_, imageDataSize_,
    reiszeMat.ptr<uint8_t>(), imageDataSize_, policy);
    if (aclret != ACL_ERROR_NONE) {
        ERROR_LOG("Copy resized image data to device failed.");
        return FAILED;
    }

    //Perform reasoning
    Result ret = model_.Execute();
    if (ret != SUCCESS) {
        ERROR_LOG("Execute model inference failed");
        return FAILED;
    }
    //Get inference output
    inferenceOutput = model_.GetModelOutputData();

    return SUCCESS;
}

void ObjectDetect::DestroyResource()
{
    aclrtFree(imageDataBuf_);
    aclrtFree(imageInfoBuf_);

    //The ACL resource held by the model instance must be released before the ACL exits or ABORT will be torn down
    model_.DestroyResource();
    aclError ret;

    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy stream failed");
        }
        stream_ = nullptr;
    }
    INFO_LOG("end to destroy stream");
    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy context failed");
        }
        context_ = nullptr;
    }
    INFO_LOG("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("reset device failed");
    }
    INFO_LOG("end to reset device is %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("finalize acl failed");
    }
    INFO_LOG("end to finalize acl");

}
