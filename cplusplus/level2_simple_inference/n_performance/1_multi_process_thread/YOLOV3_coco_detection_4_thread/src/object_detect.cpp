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
#include <pthread.h>
#include <time.h>
#include "acl/acl.h"
#include "acllite/AclLiteModel.h"
#include "object_detect.h"
#include "opencv2/opencv.hpp"

using namespace std;

ObjectDetect::ObjectDetect(const char* modelPath, uint32_t modelWidth,
                           uint32_t modelHeight)
    : g_modelWidth_(modelWidth), g_modelHeight_(modelHeight), g_isInited_(false),
      g_imageDataBuf_(nullptr), g_imageInfoBuf_(nullptr)
{
    g_imageInfoSize_ = 0;
    g_modelPath_ = modelPath;
}

ObjectDetect::~ObjectDetect()
{
    DestroyResource();
}

AclLiteError ObjectDetect::CreateInput()
{
    // Request image data memory for input model
    aclError aclRet = aclrtMalloc(&g_imageDataBuf_, (size_t)(g_imageDataSize_), ACL_MEM_MALLOC_HUGE_FIRST);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("malloc device data buffer failed, aclRet is %d", aclRet);
        return ACLLITE_ERROR;
    }

    // The second input to Yolov3 is the input image width and height parameter
    const float imageInfo[4] = {(float)g_modelWidth_, (float)g_modelHeight_,
                                (float)g_modelWidth_, (float)g_modelHeight_};
     g_imageInfoSize_ = sizeof(imageInfo);
     g_imageInfoBuf_ = CopyDataToDevice((void *)imageInfo,  g_imageInfoSize_,
                                        g_runMode_, MEMORY_DEVICE);
    if (g_imageInfoBuf_ == nullptr) {
        ACLLITE_LOG_ERROR("Copy image info to device failed");
        return ACLLITE_ERROR;
    }

    AclLiteError ret = g_model_.CreateInput(g_imageDataBuf_, g_imageDataSize_,
                                            g_imageInfoBuf_, g_imageInfoSize_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError ObjectDetect::Init()
{
    // If it is already initialized, it is returned
    if (g_isInited_) {
        ACLLITE_LOG_INFO("Classify instance is initied already!");
        return ACLLITE_OK;
    }

    // Gets whether the current application is running on host or Device
    AclLiteError ret = aclrtGetRunMode(& g_runMode_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acl get run mode failed");
        return ACLLITE_ERROR;
    }

    // Initializes the model management instance
    ret =  g_model_.Init( g_modelPath_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Init model failed");
        return ACLLITE_ERROR;
    }

    g_imageDataSize_ = g_model_.GetModelInputSize(0);

    ret = CreateInput();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create model input failed");
        return ACLLITE_ERROR;
    }

    g_isInited_ = true;
    return ACLLITE_OK;
}

AclLiteError ObjectDetect::Inference(std::vector<InferenceOutput>& inferenceOutput, cv::Mat& reiszeMat)
{
    AclLiteError ret = CopyDataToDeviceEx(g_imageDataBuf_, g_imageDataSize_,
                                          reiszeMat.ptr<uint8_t>(), g_imageDataSize_,
                                          g_runMode_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Copy resized image data to device failed.");
        return ACLLITE_ERROR;
    }
    // Perform reasoning
    ret =  g_model_.Execute(inferenceOutput);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Execute model inference failed");
        return ACLLITE_ERROR;
    }
    ACLLITE_LOG_INFO("Execute model inference success");

    return ACLLITE_OK;
}

void ObjectDetect::DestroyResource()
{
    g_model_.DestroyInput();
    g_model_.DestroyResource();

    aclrtFree(g_imageDataBuf_);
    aclrtFree(g_imageInfoBuf_);
}