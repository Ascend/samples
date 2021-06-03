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
#include <iostream>
#include <pthread.h>
#include "acl/acl.h"
#include <time.h>
#include "atlasutil/atlas_model.h"
#include "object_detect.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs/legacy/constants_c.h"

using namespace std;

ObjectDetect::ObjectDetect(const char* modelPath, uint32_t modelWidth,
uint32_t modelHeight)
: modelWidth_(modelWidth), modelHeight_(modelHeight), isInited_(false), 
 imageDataBuf_(nullptr), imageInfoBuf_(nullptr){
    imageInfoSize_ = 0;
    modelPath_ = modelPath;
}

ObjectDetect::~ObjectDetect() {
    DestroyResource();
}

AtlasError ObjectDetect::CreateInput() {
    //Request image data memory for input model
    aclError aclRet = aclrtMalloc(&imageDataBuf_, (size_t)(imageDataSize_), ACL_MEM_MALLOC_HUGE_FIRST);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("malloc device data buffer failed, aclRet is %d", aclRet);
        return ATLAS_ERROR;
    }

    //The second input to Yolov3 is the input image width and height parameter
    const float imageInfo[4] = {(float)modelWidth_, (float)modelHeight_,
    (float)modelWidth_, (float)modelHeight_};
    imageInfoSize_ = sizeof(imageInfo);
    imageInfoBuf_ = CopyDataToDevice((void *)imageInfo, imageInfoSize_,
                                        runMode_, MEMORY_DEVICE);
    if (imageInfoBuf_ == nullptr) {
        ATLAS_LOG_ERROR("Copy image info to device failed");
        return ATLAS_ERROR;
    }

    AtlasError ret = model_.CreateInput(imageDataBuf_, imageDataSize_, 
                                        imageInfoBuf_, imageInfoSize_);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Create mode input dataset failed");
        return ATLAS_ERROR;
    }

    return ATLAS_OK;
}

AtlasError ObjectDetect::Init() {
    //If it is already initialized, it is returned
    if (isInited_) {
        ATLAS_LOG_INFO("Classify instance is initied already!");
        return ATLAS_OK;
    }

    //Gets whether the current application is running on host or Device
    AtlasError ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("acl get run mode failed");
        return ATLAS_ERROR;
    }

    //Initializes the model management instance
    ret = model_.Init(modelPath_);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Init model failed");
        return ATLAS_ERROR;
    }

    imageDataSize_ = model_.GetModelInputSize(0);

    ret = CreateInput();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Create model input failed");
        return ATLAS_ERROR;
    }

    isInited_ = true;
    return ATLAS_OK;
}

AtlasError ObjectDetect::Inference(std::vector<InferenceOutput>& inferenceOutput, cv::Mat& reiszeMat) {

    AtlasError ret = CopyDataToDeviceEx(imageDataBuf_, imageDataSize_, 
                              reiszeMat.ptr<uint8_t>(), imageDataSize_, 
                              runMode_);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Copy resized image data to device failed.");
        return ATLAS_ERROR;
    }
    //Perform reasoning
    ret = model_.Execute(inferenceOutput);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Execute model inference failed");
        return ATLAS_ERROR;
    }
    ATLAS_LOG_INFO("Execute model inference success");

    return ATLAS_OK;
}

void ObjectDetect::DestroyResource()
{
    model_.DestroyInput();
    model_.DestroyResource();

    aclrtFree(imageDataBuf_);
    aclrtFree(imageInfoBuf_);
}

