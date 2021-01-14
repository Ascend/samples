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
#include <sys/timeb.h>

#include "acl/acl.h"
#include "atlas_app.h"
#include "atlas_model.h"
#include "inference.h"
#include "face_detection.h"


using namespace std;

Inference::Inference(const string& modelPath,
                     uint32_t modelWidth, uint32_t modelHeight):                       
model_(modelPath), modelWidth_(modelWidth), modelHeight_(modelHeight){
}

Inference::~Inference() {
    model_.DestroyResource();
}

AtlasError Inference::Init() {
    AtlasError ret = model_.Init();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Model init failed, error:%d", ret);
        return ret;
    }

    return ATLAS_OK;
}

AtlasError Inference::Execute(vector<InferenceOutput>& inferenceOutput,
                                       ImageData& resizedImage) {
    AtlasError ret = model_.CreateInput(resizedImage.data.get(), 
                                        resizedImage.size);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Create mode input dataset failed, error:%d", ret);
        return ATLAS_ERROR;
    }

    ret = model_.Execute(inferenceOutput);
    if (ret != ATLAS_OK) {
        model_.DestroyInput();
        ATLAS_LOG_ERROR("Execute model inference failed, error: %d", ret);
        return ATLAS_ERROR;
    }

    model_.DestroyInput();

    return ATLAS_OK;
}

int execCnt = 0;

AtlasError Inference::FrameImageProcess(
    shared_ptr<PreprocDataMsg> preprocData) {
    shared_ptr<InferOutputMsg> inferMsg = make_shared<InferOutputMsg>();
    AtlasError ret = Execute(inferMsg->inferData, preprocData->resizedImage);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Inference frame failed, error %d", ret);
        return ret;
    }   

    inferMsg->display = preprocData->display;
    inferMsg->channelId = preprocData->channelId;
    inferMsg->frameWidth = preprocData->frameWidth;
    inferMsg->frameHeight = preprocData->frameHeight;
    inferMsg->jpgImage = preprocData->jpgImage;

    SendMessage(preprocData->postprocThreadId, MSG_INFER_OUTPUT, inferMsg);

    return ATLAS_OK;
}

AtlasError Inference::Process(int msgId, shared_ptr<void> data) {
    switch(msgId) {
        case MSG_PREPROC_DATA:
            FrameImageProcess(static_pointer_cast<PreprocDataMsg>(data));
            break;
        default:
            ATLAS_LOG_INFO("Inference thread ignore msg %d", msgId);
            break;
    }

    return ATLAS_OK;
}

