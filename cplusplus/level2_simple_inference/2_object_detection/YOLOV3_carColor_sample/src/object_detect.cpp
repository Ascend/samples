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

* File object_detect.cpp
*/
#include "object_detect.h"
#include <iostream>
#include <cmath>
#include "acl/acl.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"

using namespace std;

namespace {
const uint32_t kBBoxDataBufId = 0;
const uint32_t kBoxNumDataBufId = 1;
enum BBoxIndex { TOPLEFTX = 0, TOPLEFTY, BOTTOMRIGHTX, BOTTOMRIGHTY, SCORE, LABEL };  
uint32_t kModelWidth = 416;
uint32_t kModelHeight = 416;
const char* kModelPath = "../model/yolov3.om";
}

ObjectDetect::ObjectDetect():
model_(kModelPath),
isInited_(false), 
isReleased_(false){
    imageInfoSize_ = 0;
    imageInfoBuf_ = nullptr;
}

ObjectDetect::~ObjectDetect() {
    DestroyResource();
}

AclLiteError ObjectDetect::InitModelInput() {
    const float imageInfo[4] = {(float)kModelWidth, (float)kModelHeight,
                                (float)kModelWidth, (float)kModelHeight};
    imageInfoSize_ = sizeof(imageInfo);
    imageInfoBuf_ = CopyDataToDevice((void *)imageInfo, imageInfoSize_,
                                     runMode_, MEMORY_DEVICE);
    if (imageInfoBuf_ == nullptr) {
        ACLLITE_LOG_ERROR("Copy image info to device failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError ObjectDetect::Init() {
    if (isInited_) {
        ACLLITE_LOG_INFO("Face detection is initied already");
        return ACLLITE_OK;
    }

    AclLiteError atlRet = dvpp_.Init();
    if (atlRet) {
        ACLLITE_LOG_ERROR("Dvpp init failed, error %d", atlRet);
        return ACLLITE_ERROR;
    }

    atlRet = aclrtGetRunMode(&runMode_);
    if (atlRet) {
        ACLLITE_LOG_WARNING("Get runMode failed, error %d", atlRet);
        return ACLLITE_ERROR;
    }

    atlRet = model_.Init();
    if (atlRet) {
        ACLLITE_LOG_ERROR("Model init failed, error %d", atlRet);
        return ACLLITE_ERROR;
    }

    atlRet = InitModelInput();
    if (atlRet) {
        ACLLITE_LOG_ERROR("Model Input init failed, error %d", atlRet);
        return ACLLITE_ERROR;
    }
    isInited_ = true;
    return ACLLITE_OK;
}

AclLiteError ObjectDetect::Preprocess(ImageData& resizedImage, ImageData& srcImage, ImageData& yuvImage) {
    ImageData imageDevice;

    AclLiteError ret = CopyImageToDevice(imageDevice, srcImage, runMode_, MEMORY_DVPP);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Copy image to device failed");
        return ACLLITE_ERROR;
    }

    ret = dvpp_.JpegD(yuvImage, imageDevice);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Convert jpeg to yuv failed");
        return ACLLITE_ERROR;
    }

    ret = dvpp_.CropPaste(resizedImage, yuvImage, kModelWidth, kModelHeight,
                          0, 0, yuvImage.width, yuvImage.height);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("CropResolution image failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError ObjectDetect::Inference(std::vector<InferenceOutput>& inferenceOutput,
                                     ImageData& resizedImage) {
    AclLiteError ret = model_.CreateInput(resizedImage.data.get(), resizedImage.size, 
                                          imageInfoBuf_, imageInfoSize_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed");
        return ACLLITE_ERROR;
    }

    ret = model_.Execute(inferenceOutput);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Execute model inference failed");
        return ACLLITE_ERROR;
    }
    model_.DestroyInput();
    return ACLLITE_OK;
}

AclLiteError ObjectDetect::Postprocess(ImageData& image, std::vector<InferenceOutput>& inferenceOutput,
                                       std::vector<CarInfo>& carData) {
    float* detectData = (float *)inferenceOutput[kBBoxDataBufId].data.get();
    if(detectData == nullptr){
        ACLLITE_LOG_ERROR("inferoutput is null\n");
        return ACLLITE_ERROR;
    }
    uint32_t* boxNum = (uint32_t *)inferenceOutput[kBoxNumDataBufId].data.get();
    uint32_t totalBox = boxNum[0];


    float widthScale = (float)(image.width) / kModelWidth;
    float heightScale = (float)(image.height) / kModelHeight;

    for (uint32_t i = 0; i < totalBox; i++) {
        uint32_t score = uint32_t(detectData[totalBox * SCORE + i] * 100);
        if (score < 90) {
            continue;
        }
        uint32_t objIndex = (uint32_t)detectData[totalBox * LABEL + i];
        if (objIndex == 2){
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

void ObjectDetect::DestroyResource(){
    if (!isReleased_) {
        dvpp_.DestroyResource();
        model_.DestroyResource();
        isReleased_ = true;
    }
}
