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
#include "AclLiteVideoProc.h"
#include "object_detection.h"
#include "preprocess.h"
#include "AclLiteApp.h"

using namespace std;

Preprocess::Preprocess(string& streamName, uint32_t modelWidth, 
                       uint32_t modelHeight, uint32_t channelId) : 
streamName_(streamName),
cap_(nullptr),
stream_(nullptr),
modelWidth_(modelWidth),
modelHeight_(modelHeight),
postThreadIndex_(channelId),
channelId_(channelId),
selfThreadId_(INVALID_INSTANCE_ID),
nextThreadId_(INVALID_INSTANCE_ID),
postprocThreadId_(INVALID_INSTANCE_ID),
frameCnt_(0) {    
}

Preprocess::~Preprocess() {
    if (cap_ != nullptr) {
        cap_->Close();
        delete cap_;
    }
    dvpp_.DestroyResource();
}

AclLiteError Preprocess::Init() {
    if (ACLLITE_OK != OpenVideoCapture()) {
        return ACLLITE_ERROR;
    }

    aclError aclRet = aclrtCreateStream(&stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Create acl stream failed, error %d", aclRet);
        return ACLLITE_ERROR_CREATE_STREAM;
    }
    ACLLITE_LOG_INFO("Create stream for dvpp success");

    AclLiteError ret = dvpp_.Init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Init dvpp failed");
        return ret;
    }

    if (ACLLITE_OK != GetThreadInstanceId()) {
        ACLLITE_LOG_ERROR("Get self and next thread instance id failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError Preprocess::OpenVideoCapture() {
    if (IsDigitStr(streamName_)) {
        int cameraId = atoi(streamName_.c_str());
        if ((cameraId < 0) || (cameraId >= CAMERA_ID_INVALID)) {
            ACLLITE_LOG_ERROR("Invalid camera id arg %s, only allow %d and %d",
            streamName_.c_str(), CAMERA_ID_0, CAMERA_ID_1);
            return ACLLITE_ERROR;
        }
        cap_ = new AclLiteVideoProc(cameraId);
    } else if (IsRtspAddr(streamName_)) {
        cap_ = new AclLiteVideoProc(streamName_);
    } else if (IsVideoFile(streamName_)) {
        if (!IsPathExist(streamName_)) {
            ACLLITE_LOG_ERROR("The %s is inaccessible", streamName_.c_str());
            return ACLLITE_ERROR;
        }
        cap_ = new AclLiteVideoProc(streamName_);
    } else {
        ACLLITE_LOG_ERROR("Invalid param. The arg should be accessible rtsp,"
                        " video file or camera id");
        return ACLLITE_ERROR;
    }

    if(!cap_->IsOpened()) {
        delete cap_;
        ACLLITE_LOG_ERROR("Failed to open video");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError Preprocess::GetThreadInstanceId() {
    selfThreadId_ = SelfInstanceId();
    nextThreadId_ = GetAclLiteThreadIdByName(kInferName);
    postprocThreadId_ = GetAclLiteThreadIdByName(kPostprocName[postThreadIndex_]);
    if ((selfThreadId_ == INVALID_INSTANCE_ID) ||
        (nextThreadId_ == INVALID_INSTANCE_ID) ||
        (postprocThreadId_ == INVALID_INSTANCE_ID)) {
        ACLLITE_LOG_ERROR("Self instance id %d, next instance id %d, "
                        "postprocess instance id %d",
                        selfThreadId_, nextThreadId_, postprocThreadId_);
                        
        return ACLLITE_ERROR;
    }
    return ACLLITE_OK;
}

AclLiteError Preprocess::Process(int msgId, shared_ptr<void> msgData) {
    AclLiteError ret = ACLLITE_OK;
    switch(msgId) {
        case MSG_APP_START:
            ret = AppStartMsgProcess();
            break;
        case MSG_READ_FRAME:
            ret = ReadFrameMsgProcess();
            break;
        default:
            ACLLITE_LOG_ERROR("Preprocess thread receive unknow msg %d", msgId);
            break;
    }

    return ret;
}

AclLiteError Preprocess::AppStartMsgProcess() {
    AclLiteError ret = SendMessage(selfThreadId_, MSG_READ_FRAME, nullptr);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Process app start message failed, error %d", ret);
    }    

    return ret;
}

AclLiteError Preprocess::ReadFrameMsgProcess() {
    ImageData image;
    AclLiteError ret = cap_->Read(image);
    if (ret != ACLLITE_OK) { 
        shared_ptr<PreprocDataMsg> preprocData = make_shared<PreprocDataMsg>();
        preprocData->postprocThreadId = postprocThreadId_;
        preprocData->isLastFrame = 1;
        ret = SendMessage(nextThreadId_, MSG_PREPROC_DATA, preprocData);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Send preprocess data failed, error %d", ret);
        }
        ACLLITE_LOG_ERROR("Channel %d: read frame failed, error %d", 
                        channelId_, ret);
        return ret;                
    }

    frameCnt_++;
    if ((frameCnt_ % 7) == 0) {
        ProcessImage(image);
    } 

    ret = SendMessage(selfThreadId_, MSG_READ_FRAME, nullptr);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
        return ret;
    } 

    return ACLLITE_OK;
}

void Preprocess::ProcessImage(ImageData image) {
    shared_ptr<PreprocDataMsg> preprocData = make_shared<PreprocDataMsg>();
    AclLiteError ret = dvpp_.Resize(preprocData->resizedImage,image,
                       modelWidth_, modelHeight_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Resize image failed");
        return;
    }
    
    preprocData->postprocThreadId = postprocThreadId_;
    preprocData->isLastFrame = 0;
    preprocData->frameWidth = image.width;
    preprocData->frameHeight = image.height;
    preprocData->channelId = channelId_;
    ret = SendMessage(nextThreadId_, MSG_PREPROC_DATA, preprocData);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Send preprocess data failed, error %d", ret);
    }

    return;
}
