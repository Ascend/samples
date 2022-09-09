/**
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2022. All rights reserved.
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
#include "CarParams.h"
#include "detectPostprocess.h"
#include "AclLiteUtils.h"
#include "AclLiteApp.h"

using namespace std;

namespace {
const uint32_t kBBoxDataBufId = 0;
const uint32_t kBoxNumDataBufId = 1;
const static std::vector<std::string> yolov3Label = { "person", "bicycle", "car", "motorbike",
    "aeroplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter",
    "bench", "bird", "cat", "dog", "horse",
    "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie",
    "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana",
    "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair",
    "sofa", "potted plant", "bed", "dining table", "toilet",
    "TV monitor", "laptop", "mouse", "remote", "keyboard",
    "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors",
    "teddy bear", "hair drier", "toothbrush" };
enum BBoxIndex { TOPLEFTX = 0, TOPLEFTY, BOTTOMRIGHTX, BOTTOMRIGHTY, SCORE, LABEL };
uint32_t kModelWidth = 416;
uint32_t kModelHeight = 416;
}

DetectPostprocessThread::DetectPostprocessThread() {
}

DetectPostprocessThread::~DetectPostprocessThread() {
}

AclLiteError DetectPostprocessThread::Init()
{
    return ACLLITE_OK;
}

AclLiteError DetectPostprocessThread::Process(int msgId, shared_ptr<void> data)
{
    AclLiteError ret = ACLLITE_OK;
    switch (msgId) {
        case MSG_DETECT_INFER_OUTPUT:
            InferOutputProcess(static_pointer_cast<CarDetectDataMsg>(data));
            MsgSend(static_pointer_cast<CarDetectDataMsg>(data));
            break;
        default:
            ACLLITE_LOG_INFO("Detect PostprocessThread thread ignore msg %d", msgId);
            break;
    }

    return ret;
}

AclLiteError DetectPostprocessThread::MsgSend(shared_ptr<CarDetectDataMsg> carDetectDataMsg)
{
    while (1) {
        AclLiteError ret = SendMessage(carDetectDataMsg->classifyPreThreadId,
            MSG_DETECT_POSTPROC_DATA, carDetectDataMsg);
        if (ret == ACLLITE_ERROR_ENQUEUE) {
            usleep(500);
            continue;
        } else if (ret == ACLLITE_OK) {
            break;
        } else {
            ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
            return ret;
        }
    }

    return ACLLITE_OK;
}

AclLiteError DetectPostprocessThread::InferOutputProcess(shared_ptr<CarDetectDataMsg> carDetectDataMsg)
{
    if (carDetectDataMsg->isLastFrame)
        return ACLLITE_OK;
    
    float* detectData = (float *)carDetectDataMsg->detectInferData[kBBoxDataBufId].data.get();
    if (detectData == nullptr) {
        ACLLITE_LOG_ERROR("detect inferoutput is null\n");
        return ACLLITE_ERROR;
    }
    uint32_t* boxNum = (uint32_t *)carDetectDataMsg->detectInferData[kBoxNumDataBufId].data.get();
    uint32_t totalBox = boxNum[0];

    for (uint32_t i = 0; i < totalBox; i++) {
        uint32_t score = uint32_t(detectData[totalBox * SCORE + i] * 100);
        if (score < 60) {
            continue;
        }
        uint32_t objIndex = (uint32_t)detectData[totalBox * LABEL + i];
        if (objIndex == 2) {
            CarInfo carInfo;
            carInfo.rectangle.lt.x = detectData[totalBox * TOPLEFTX + i];
            carInfo.rectangle.lt.y = detectData[totalBox * TOPLEFTY + i];
            carInfo.rectangle.rb.x = detectData[totalBox * BOTTOMRIGHTX + i];
            carInfo.rectangle.rb.y = detectData[totalBox * BOTTOMRIGHTY + i];
            uint32_t objIndex = (uint32_t)detectData[totalBox * LABEL + i];
            carInfo.detect_result = yolov3Label[objIndex] + std::to_string(score) + "\%";
            carDetectDataMsg->carInfo.emplace_back(carInfo);
        }
    }
    return ACLLITE_OK;
}
