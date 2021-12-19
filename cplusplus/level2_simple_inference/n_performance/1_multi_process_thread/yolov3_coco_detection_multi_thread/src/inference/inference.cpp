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
#include "AclLiteApp.h"
#include "AclLiteModel.h"
#include "inference.h"
#include "object_detection.h"

using namespace std;

Inference::Inference(const string& modelPath,
                     uint32_t modelWidth, uint32_t modelHeight) : 
model_(modelPath), modelWidth_(modelWidth), modelHeight_(modelHeight){
    imageInfoSize_ = 0;
    imageInfoBuf_ = nullptr;
}

Inference::~Inference() {
    model_.DestroyResource();
}

AclLiteError Inference::Init() {
    AclLiteError ret = model_.Init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Model init failed, error:%d", ret);
        return ret;
    }

    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acl get run mode failed");
        return ACLLITE_ERROR;
    }

    const float imageInfo[4] = {(float)modelWidth_, (float)modelHeight_,
    (float)modelWidth_, (float)modelHeight_};
    imageInfoSize_ = sizeof(imageInfo);
    imageInfoBuf_ = CopyDataToDevice((void *)imageInfo, imageInfoSize_,
                                        runMode_, MEMORY_DEVICE);
    if (imageInfoBuf_ == nullptr) {
        ACLLITE_LOG_ERROR("Copy image info to device failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError Inference::Execute(vector<InferenceOutput>& inferenceOutput,
                                       ImageData& resizedImage) {
    AclLiteError ret = model_.CreateInput(resizedImage.data.get(), 
                                        resizedImage.size, imageInfoBuf_, imageInfoSize_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed, error:%d", ret);
        return ACLLITE_ERROR;
    }

    ret = model_.Execute(inferenceOutput);
    if (ret != ACLLITE_OK) {
        model_.DestroyInput();
        ACLLITE_LOG_ERROR("Execute model inference failed, error: %d", ret);
        return ACLLITE_ERROR;
    }

    model_.DestroyInput();

    return ACLLITE_OK;
}

AclLiteError Inference::FrameImageProcess(
    shared_ptr<PreprocDataMsg> preprocData) {
    shared_ptr<InferOutputMsg> inferMsg = make_shared<InferOutputMsg>();

    if (preprocData->isLastFrame) {
        inferMsg->isLastFrame = preprocData->isLastFrame;
        SendMessage(preprocData->postprocThreadId, MSG_DECODE_FINISH, inferMsg);
        return ACLLITE_OK;
    }

    AclLiteError ret = Execute(inferMsg->inferData, preprocData->resizedImage);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Inference frame failed, error %d", ret);
        return ret;
    }   

    inferMsg->isLastFrame = preprocData->isLastFrame;
    inferMsg->frameWidth = preprocData->frameWidth;
    inferMsg->frameHeight = preprocData->frameHeight;
    inferMsg->channelId = preprocData->channelId;

    SendMessage(preprocData->postprocThreadId, MSG_INFER_OUTPUT, inferMsg);

    return ACLLITE_OK;
}

AclLiteError Inference::Process(int msgId, shared_ptr<void> data) {
    switch(msgId) {
        case MSG_PREPROC_DATA:
            FrameImageProcess(static_pointer_cast<PreprocDataMsg>(data));
            break;
        default:
            ACLLITE_LOG_INFO("Inference thread ignore msg %d", msgId);
            break;
    }

    return ACLLITE_OK;
}

