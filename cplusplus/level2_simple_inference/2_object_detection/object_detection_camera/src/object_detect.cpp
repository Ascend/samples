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
#include "object_detect.h"

using namespace std;
using namespace ascend::presenter;

namespace {
const uint32_t kModelWidth = 416;
const uint32_t kModelHeight = 416;
const string kModelPath = "../model/object_detection.om";
const string kConfigFile = "../scripts/object_detection.conf";
const uint32_t kBBoxDataBufId = 0;
const uint32_t kBoxNumDataBufId = 1;
const static std::vector<std::string> yolov3Label = { "person", "bicycle", "car", "motorbike",
    "aeroplane","bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter",
    "bench", "bird", "cat", "dog", "horse",
    "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag","tie",
    "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana",
    "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair",
    "sofa", "potted plant", "bed", "dining table", "toilet",
    "TV monitor", "laptop", "mouse", "remote", "keyboard",
    "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase","scissors",
    "teddy bear", "hair drier", "toothbrush" };
enum BBoxIndex { TOPLEFTX = 0, TOPLEFTY, BOTTOMRIGHTX, BOTTOMRIGHTY, SCORE, LABEL };   
}

ObjectDetect::ObjectDetect():
model_(kModelPath),
presenterChannel_(nullptr),
isInited_(false), 
isReleased_(false),
imageInfoSize_(0),
imageInfoBuf_(nullptr)
{
}

ObjectDetect::~ObjectDetect() {
    DestroyResource();
}

AclLiteError ObjectDetect::Init() {
    if (isInited_) {
        ACLLITE_LOG_INFO("Object detection is initied already");
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

    atlRet = aclrtGetRunMode(&runMode_);
    if (atlRet) {
        ACLLITE_LOG_ERROR("acl get run mode failed");
        return ACLLITE_ERROR;
    }

    const float imageInfo[4] = {(float)kModelWidth, (float)kModelHeight,
    (float)kModelWidth, (float)kModelHeight};
    imageInfoSize_ = sizeof(imageInfo);
    imageInfoBuf_ = CopyDataToDevice((void *)imageInfo, imageInfoSize_,
                                        runMode_, MEMORY_DEVICE);
    if (imageInfoBuf_ == nullptr) {
        ACLLITE_LOG_ERROR("Copy image info to device failed");
        return ACLLITE_ERROR;
    }

    isInited_ = true;
    return ACLLITE_OK;
}

AclLiteError ObjectDetect::Process(ImageData& image) {
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

AclLiteError ObjectDetect::Inference(std::vector<InferenceOutput>& inferOutputs,
                                 ImageData& resizedImage) {
    AclLiteError ret = model_.CreateInput(resizedImage.data.get(),
                                       resizedImage.size, imageInfoBuf_, imageInfoSize_);
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

void ObjectDetect::PostProcess(vector<DetectionResult>& detectResults, 
                             uint32_t imageWidth, uint32_t imageHeight,
                             vector<InferenceOutput>& modelOutput) {
    uint32_t dataSize = 0;
    float* detectData = (float *)modelOutput[kBBoxDataBufId].data.get();
    uint32_t* boxNum = (uint32_t *)modelOutput[kBoxNumDataBufId].data.get();

    uint32_t totalBox = boxNum[0];
    float widthScale = (float)(imageWidth) / kModelWidth;
    float heightScale = (float)(imageHeight) / kModelHeight;

    for (uint32_t i = 0; i < totalBox; i++) {
        DetectionResult oneResult;
        Point point_lt, point_rb;
        uint32_t score = uint32_t(detectData[totalBox * SCORE + i] * 100);
        if (score < 90) {
            continue;
        }
        point_lt.x = detectData[totalBox * TOPLEFTX + i] * widthScale;
        point_lt.y = detectData[totalBox * TOPLEFTY + i] * heightScale;
        point_rb.x = detectData[totalBox * BOTTOMRIGHTX + i] * widthScale;
        point_rb.y = detectData[totalBox * BOTTOMRIGHTY + i] * heightScale;

        uint32_t objIndex = (uint32_t)detectData[totalBox * LABEL + i];
        oneResult.lt = point_lt;
        oneResult.rb = point_rb;
        oneResult.result_text = yolov3Label[objIndex] + to_string(score) + "\%";
        detectResults.emplace_back(oneResult);
    }
}

AclLiteError ObjectDetect::SendImage(ImageData& jpegImage,
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

void ObjectDetect::DestroyResource() {
    if (!isReleased_) {
        dvpp_.DestroyResource();
        model_.DestroyResource();
        isReleased_ = true;
    }
}

