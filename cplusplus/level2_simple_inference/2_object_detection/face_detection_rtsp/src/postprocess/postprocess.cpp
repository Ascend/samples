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
#include "face_detection.h"
#include "postprocess.h"
#include "atlas_utils.h"
#include "atlas_app.h"

using namespace std;

namespace {
const static std::vector<std::string> ssdLabel = { "background", "face"};
const uint32_t kBoxNumDataBufId = 0;
const uint32_t kBBoxDataBufId = 1;
const uint32_t kItemSize = 8;
enum BBoxIndex { EMPTY = 0, LABEL,SCORE,TOPLEFTX,TOPLEFTY, 
                 BOTTOMRIGHTX, BOTTOMRIGHTY};        
}


Postprocess::Postprocess(const string& configFile, bool display):
display_(display),
configFile_(configFile),
presenterChan_(nullptr) {
}

Postprocess::~Postprocess() {
    delete presenterChan_;
}


AtlasError Postprocess::Init() {
    if (display_) {
        PresenterErrorCode ret = OpenChannelByConfig(presenterChan_, configFile_.c_str());
        if (ret != PresenterErrorCode::kNone) {
            ATLAS_LOG_ERROR("Open channel failed, error %d\n", (int)ret);
            return ATLAS_ERROR;
        }
    }

    return ATLAS_OK;
}

AtlasError Postprocess::Process(int msgId, shared_ptr<void> data) {
    AtlasError ret = ATLAS_OK;
    shared_ptr<InferOutputMsg> inferMsg = static_pointer_cast<InferOutputMsg>(data);
    
    switch(msgId) {
        case MSG_INFER_OUTPUT:
            ret = InferOutputProcess(inferMsg);
            break;
        case MSG_DECODE_FINISH:
            SendMessage(kMainThreadId, MSG_APP_EXIT, nullptr);
            break;
        default:
            ATLAS_LOG_INFO("Postprocess thread ignore msg %d", msgId);
            break;
    }

    return ret;
}

AtlasError Postprocess::InferOutputProcess(shared_ptr<InferOutputMsg> data) {
    vector<DetectionResult> detectResults;
    AtlasError ret = AnalyzeInferenceOutput(detectResults, data->frameWidth,
                                            data->frameHeight, data->inferData);
    if(ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Covert image failed, error %d", ret);
        return ret;
    }

    PrintDetectResults(detectResults, data->channelId);

    if (display_) {
        SendImage(presenterChan_, data->jpgImage, detectResults);                  
    }

    return ATLAS_OK;
}

void Postprocess::PrintDetectResults(vector<DetectionResult>& detectResults,
                                     uint32_t channelId) {
    for (size_t i = 0; i < detectResults.size(); i++) {
        ATLAS_LOG_INFO("channel%d:%d %d %d %d  %s", 
                       channelId, detectResults[i].lt.x, detectResults[i].lt.y,
                       detectResults[i].rb.x, detectResults[i].rb.y, 
                       detectResults[i].result_text.c_str());
    }
}

AtlasError Postprocess::AnalyzeInferenceOutput(vector<DetectionResult>& detectResults, 
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
        if (score < 70)
            break;

        point_lt.x = detectData[TOPLEFTX + i * kItemSize] * imageWidth;
        point_lt.y = detectData[TOPLEFTY + i * kItemSize] * imageHeight;
        point_rb.x = detectData[BOTTOMRIGHTX + i * kItemSize] * imageWidth;
        point_rb.y = detectData[BOTTOMRIGHTY + i * kItemSize] * imageHeight;

        uint32_t objIndex = (uint32_t)detectData[LABEL + i * kItemSize];
        oneResult.lt = point_lt;
        oneResult.rb = point_rb;
        oneResult.result_text = ssdLabel[objIndex] + std::to_string(score) + "\%";
        detectResults.emplace_back(oneResult);
    }

    return ATLAS_OK;
}


AtlasError Postprocess::SendImage(Channel* channel, 
                                  ImageData& jpegImage,
                                  vector<DetectionResult>& detRes) {
    ImageFrame frame;
    frame.format = ImageFormat::kJpeg;
    frame.width = jpegImage.width;
    frame.height = jpegImage.height;
    frame.size = jpegImage.size;
    frame.data = jpegImage.data.get();
    frame.detection_results = detRes;

    PresenterErrorCode ret = PresentImage(channel, frame);
    if (ret != PresenterErrorCode::kNone) {
        ATLAS_LOG_ERROR("Send JPEG image to presenter failed, error %d", (int)ret);
        return ATLAS_ERROR;
    }

    return ATLAS_OK;
}


