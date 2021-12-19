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

#include "acl/acl.h"
#include "acllite/AclLiteModel.h"
#include "face_detect.h"

using namespace std;
using namespace ascend::presenter;

namespace {
uint32_t kModelWidth = 304;
uint32_t kModelHeight = 300;
const string kModelPath = "../model/face_detection.om";
const string kConfigFile = "../scripts/face_detection.conf";

const static std::vector<std::string> ssdLabel = { "background", "face"};
const uint32_t kBBoxDataBufId = 1;
const uint32_t kBoxNumDataBufId = 0;
const uint32_t kItemSize = 8;
enum BBoxIndex { EMPTY = 0, LABEL,SCORE,TOPLEFTX,TOPLEFTY, 
                 BOTTOMRIGHTX, BOTTOMRIGHTY };  
}

FaceDetect::FaceDetect() : 
model_(kModelPath),
presenterChannel_(nullptr),
isInited_(false), 
isReleased_(false){
}

FaceDetect::~FaceDetect() {
    DestroyResource();
}

AclLiteError FaceDetect::Init() {
    if (isInited_) {
        ACLLITE_LOG_INFO("Face detection is initied already");
        return ACLLITE_OK;
    }

    PresenterErrorCode ret = OpenChannelByConfig(presenterChannel_, 
                                                 kConfigFile.c_str());
    if (ret != PresenterErrorCode::kNone) {
        ACLLITE_LOG_ERROR("Open channel failed, error %d", (int)ret);
        return ACLLITE_ERROR;
    }

    AclLiteError atlRet = dvpp_.Init();
    if (atlRet) {
        ACLLITE_LOG_ERROR("Dvpp init failed, error %d", atlRet);
        return ACLLITE_ERROR;
    }

    atlRet = model_.Init();
    if (atlRet) {
        ACLLITE_LOG_ERROR("Model init failed, error %d", atlRet);
        return ACLLITE_ERROR;
    }

    isInited_ = true;
    return ACLLITE_OK;
}

AclLiteError FaceDetect::Process(ImageData& image) {
    ImageData resizedImage;
    AclLiteError ret = dvpp_.Resize(resizedImage, image, 
                                  kModelWidth, kModelHeight);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Resize image failed");
        return ACLLITE_ERROR;
    }

    std::vector<InferenceOutput> inferOutputs;
    ret = Inference(inferOutputs, resizedImage);
    if (ret) {
        ACLLITE_LOG_ERROR("Inference image failed");
        return ACLLITE_ERROR;        
    }

    vector<DetectionResult> detectResults;
    PostProcess(detectResults, image.width, image.height, inferOutputs);

    ImageData jpg;
    ret = dvpp_.JpegE(jpg, image);
    if (ret) {
        ACLLITE_LOG_ERROR("Convert image to jpeg failed");
        return ret;
    }

    return SendImage(jpg, detectResults); 
}

AclLiteError FaceDetect::Inference(std::vector<InferenceOutput>& inferOutputs,
                                 ImageData& resizedImage) {
    AclLiteError ret = model_.CreateInput(resizedImage.data.get(),
                                       resizedImage.size);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed\n");
        return ACLLITE_ERROR;
    }

    ret = model_.Execute(inferOutputs);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Execute model inference failed\n");
    }
    model_.DestroyInput();

    return ret;
}

void FaceDetect::PostProcess(vector<DetectionResult>& detectResults, 
                             uint32_t imageWidth, uint32_t imageHeight,
                             vector<InferenceOutput>& modelOutput) {
    uint32_t dataSize = 0;
    float* detectData = (float *)modelOutput[kBBoxDataBufId].data.get();
    uint32_t* boxNum = (uint32_t *)modelOutput[kBoxNumDataBufId].data.get();

    uint32_t totalBox = boxNum[0];

    for (uint32_t i = 0; i < totalBox; i++) {
        DetectionResult oneResult;
        Point point_lt, point_rb;
        uint32_t score = uint32_t(detectData[SCORE + i * kItemSize] * 100);
        if (score < 70) {
            break;
        }

        point_lt.x = detectData[TOPLEFTX + i * kItemSize] * imageWidth;
        point_lt.y = detectData[TOPLEFTY + i * kItemSize] * imageHeight;
        point_rb.x = detectData[BOTTOMRIGHTX + i * kItemSize] * imageWidth;
        point_rb.y = detectData[BOTTOMRIGHTY + i * kItemSize] * imageHeight;

        uint32_t objIndex = (uint32_t)detectData[LABEL + i * kItemSize];
        oneResult.lt = point_lt;
        oneResult.rb = point_rb;
        oneResult.result_text = ssdLabel[objIndex] + to_string(score) + "\%";
        detectResults.emplace_back(oneResult);
    }
}

AclLiteError FaceDetect::SendImage(ImageData& jpegImage,
                                 vector<DetectionResult>& detRes) {
    ImageFrame frame;
    frame.format = ImageFormat::kJpeg;
    frame.width = jpegImage.width;
    frame.height = jpegImage.height;
    frame.size = jpegImage.size;
    frame.data = jpegImage.data.get();
    frame.detection_results = detRes;

    PresenterErrorCode ret = PresentImage(presenterChannel_, frame);
    if (ret != PresenterErrorCode::kNone) {
        ACLLITE_LOG_ERROR("Send to presenter server failed, error %d", (int)ret);
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

void FaceDetect::DestroyResource() {
    if (!isReleased_) {
        dvpp_.DestroyResource();
        model_.DestroyResource();
        isReleased_ = true;
    }
}

