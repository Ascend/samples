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
#include "object_detection.h"
#include "postprocess.h"
#include "AclLiteUtils.h"
#include "AclLiteApp.h"

using namespace std;

namespace {
const uint32_t kBBoxDataBufId = 0;
const uint32_t kBoxNumDataBufId = 1;
const uint32_t kItemSize = 8;
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

Postprocess::Postprocess(uint32_t modelWidth, uint32_t modelHeight) : 
modelWidth_(modelWidth), modelHeight_(modelHeight){
}

Postprocess::~Postprocess() {
}

AclLiteError Postprocess::Init() {
    return ACLLITE_OK;
}

AclLiteError Postprocess::Process(int msgId, shared_ptr<void> data) {
    AclLiteError ret = ACLLITE_OK;
    shared_ptr<InferOutputMsg> inferMsg = static_pointer_cast<InferOutputMsg>(data);
    
    switch(msgId) {
        case MSG_INFER_OUTPUT:
            ret = InferOutputProcess(inferMsg);
            break;
        case MSG_DECODE_FINISH:
            SendMessage(g_MainThreadId, MSG_APP_EXIT, nullptr);
            break;
        default:
            ACLLITE_LOG_INFO("Postprocess thread ignore msg %d", msgId);
            break;
    }

    return ret;
}

AclLiteError Postprocess::InferOutputProcess(shared_ptr<InferOutputMsg> data) {
    vector<BBox> detectResults;
    AclLiteError ret = AnalyzeInferenceOutput(detectResults, data->frameWidth,
                                            data->frameHeight, data->inferData);
    if(ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Covert image failed, error %d", ret);
        return ret;
    }

    PrintDetectResults(detectResults, data->channelId);

    return ACLLITE_OK;
}

void Postprocess::PrintDetectResults(vector<BBox>& detectResults,
                                     uint32_t channelId) {
    for (size_t i = 0; i < detectResults.size(); i++) {
        ACLLITE_LOG_INFO("channel%d:%d %d %d %d  %s", 
                       channelId, detectResults[i].rect.ltX, detectResults[i].rect.ltY,
                       detectResults[i].rect.rbX, detectResults[i].rect.rbY, 
                       detectResults[i].text.c_str());
    }
}

AclLiteError Postprocess::AnalyzeInferenceOutput(vector<BBox>& detectResults, 
                                               uint32_t imageWidth, uint32_t imageHeight,
                                               vector<InferenceOutput>& modelOutput) {
    uint32_t dataSize = 0;
    float* detectData = (float *)modelOutput[kBBoxDataBufId].data.get();
    uint32_t* boxNum = (uint32_t *)modelOutput[kBoxNumDataBufId].data.get();

    uint32_t totalBox = boxNum[0];
    float widthScale = (float)(imageWidth) / modelWidth_;
    float heightScale = (float)(imageHeight) / modelHeight_;

    for (uint32_t i = 0; i < totalBox; i++) {
        BBox boundBox;
        uint32_t score = uint32_t(detectData[totalBox * SCORE + i] * 100);
        if (score < 90){
            continue;
        }
        boundBox.rect.ltX = detectData[totalBox * TOPLEFTX + i] * widthScale;
        boundBox.rect.ltY = detectData[totalBox * TOPLEFTY + i] * heightScale;
        boundBox.rect.rbX = detectData[totalBox * BOTTOMRIGHTX + i] * widthScale;
        boundBox.rect.rbY = detectData[totalBox * BOTTOMRIGHTY + i] * heightScale;
        uint32_t objIndex = (uint32_t)detectData[totalBox * LABEL + i];
        boundBox.text = yolov3Label[objIndex] + std::to_string(score) + "\%";
        detectResults.emplace_back(boundBox);
    }

    return ACLLITE_OK;
}

