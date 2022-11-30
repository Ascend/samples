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
#include <cmath>
#include "object_detect.h"
#include "acl/acl.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"

using namespace std;

namespace {
const uint32_t g_BBoxDataBufId = 0;
const uint32_t g_BoxNumDataBufId = 1;
enum BBoxIndex { TOPLEFTX = 0, TOPLEFTY, BOTTOMRIGHTX, BOTTOMRIGHTY, SCORE, LABEL };
uint32_t g_modelWidth = 416;
uint32_t g_modelHeight = 416;
const char* g_modelPath = "../model/yolov3.om";
}

ObjectDetect::ObjectDetect()
    : g_model_(g_modelPath),
      g_isInited_(false),
      g_isReleased_(false)
{
    g_imageInfoSize_ = 0;
    g_imageInfoBuf_ = nullptr;
}

ObjectDetect::~ObjectDetect()
{
    DestroyResource();
}

AclLiteError ObjectDetect::InitModelInput()
{
    const float imageInfo[4] = {(float)g_modelWidth, (float)g_modelHeight,
                                (float)g_modelWidth, (float)g_modelHeight};
    g_imageInfoSize_ = sizeof(imageInfo);
    g_imageInfoBuf_ = CopyDataToDevice((void *)imageInfo, g_imageInfoSize_,
                                       g_runMode_, MEMORY_DEVICE);
    if (g_imageInfoBuf_ == nullptr) {
        ACLLITE_LOG_ERROR("Copy image info to device failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError ObjectDetect::Init()
{
    if (g_isInited_) {
        ACLLITE_LOG_INFO("Object detection is initied already");
        return ACLLITE_OK;
    }

    AclLiteError atlRet = g_dvpp_.Init();
    if (atlRet) {
        ACLLITE_LOG_ERROR("Dvpp init failed, error %d", atlRet);
        return ACLLITE_ERROR;
    }

    atlRet = aclrtGetRunMode(&g_runMode_);
    if (atlRet) {
        ACLLITE_LOG_WARNING("Get runMode failed, error %d", atlRet);
        return ACLLITE_ERROR;
    }

    atlRet = g_model_.Init();
    if (atlRet) {
        ACLLITE_LOG_ERROR("Model init failed, error %d", atlRet);
        return ACLLITE_ERROR;
    }

    atlRet = InitModelInput();
    if (atlRet) {
        ACLLITE_LOG_ERROR("Model Input init failed, error %d", atlRet);
        return ACLLITE_ERROR;
    }
    g_isInited_ = true;
    return ACLLITE_OK;
}

AclLiteError ObjectDetect::PreProcess(ImageData& resizedImage, ImageData& srcImage, ImageData& yuvImage)
{
    ImageData imageDevice;

    AclLiteError ret = CopyImageToDevice(imageDevice, srcImage, g_runMode_, MEMORY_DVPP);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Copy image to device failed");
        return ACLLITE_ERROR;
    }

    ret = g_dvpp_.JpegD(yuvImage, imageDevice);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Convert jpeg to yuv failed");
        return ACLLITE_ERROR;
    }

    ret = g_dvpp_.CropPaste(resizedImage, yuvImage, g_modelWidth, g_modelHeight,
                            0, 0, yuvImage.width, yuvImage.height);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("CropResolution image failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError ObjectDetect::Inference(std::vector<InferenceOutput>& inferenceOutput,
                                     ImageData& resizedImage)
{
    AclLiteError ret = g_model_.CreateInput(resizedImage.data.get(), resizedImage.size,
                                            g_imageInfoBuf_, g_imageInfoSize_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed");
        return ACLLITE_ERROR;
    }

    ret = g_model_.Execute(inferenceOutput);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Execute model inference failed");
        return ACLLITE_ERROR;
    }
    g_model_.DestroyInput();
    return ACLLITE_OK;
}

AclLiteError ObjectDetect::PostProcess(ImageData& image, std::vector<InferenceOutput>& inferenceOutput,
                                       std::vector<CarInfo>& carData)
{
    float* detectData = (float *)inferenceOutput[g_BBoxDataBufId].data.get();
    if (detectData == nullptr) {
        ACLLITE_LOG_ERROR("inferoutput is null\n");
        return ACLLITE_ERROR;
    }
    uint32_t* boxNum = (uint32_t *)inferenceOutput[g_BoxNumDataBufId].data.get();
    uint32_t totalBox = boxNum[0];
    uint32_t scoreLine = 90;

    float widthScale = (float)(image.width) / g_modelWidth;
    float heightScale = (float)(image.height) / g_modelHeight;

    for (uint32_t i = 0; i < totalBox; i++) {
        uint32_t score = uint32_t(detectData[totalBox * SCORE + i] * 100);
        if (score < scoreLine) {
            continue;
        }
        uint32_t objIndex = (uint32_t)detectData[totalBox * LABEL + i];
        uint32_t n = 2;
        if (objIndex == n) {
            CarInfo carInfo;
            carInfo.rectangle.lt.x = detectData[totalBox * TOPLEFTX + i] * widthScale;
            carInfo.rectangle.lt.y = detectData[totalBox * TOPLEFTY + i] * heightScale;
            carInfo.rectangle.rb.x = detectData[totalBox * BOTTOMRIGHTX + i] * widthScale;
            carInfo.rectangle.rb.y = detectData[totalBox * BOTTOMRIGHTY + i] * heightScale;
            carInfo.text = "car" + std::to_string(score) + "\%";
            carData.emplace_back(carInfo);
        }
    }
    return ACLLITE_OK;
}

void ObjectDetect::DestroyResource()
{
    if (!g_isReleased_) {
        g_dvpp_.DestroyResource();
        g_model_.DestroyResource();
        g_isReleased_ = true;
    }
}
