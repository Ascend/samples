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

#include "atlas_videocapture.h"
#include "face_detection.h"
#include "preprocess.h"
#include "atlas_app.h"


using namespace std;

namespace {
}

Preprocess::Preprocess(string& streamName, uint32_t modelWidth, 
                       uint32_t modelHeight, uint32_t channelId, bool display) : 
streamName_(streamName),
cap_(nullptr),
stream_(nullptr),
display_(display),
modelWidth_(modelWidth),
modelHeight_(modelHeight),
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

AtlasError Preprocess::Init() {
    if (ATLAS_OK != OpenVideoCapture()) {
        return ATLAS_ERROR;
    }

    aclError aclRet = aclrtCreateStream(&stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Create acl stream failed, error %d", aclRet);
        return ATLAS_ERROR_CREATE_STREAM;
    }
    ATLAS_LOG_INFO("Create stream for dvpp success");

    AtlasError ret = dvpp_.Init();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Init dvpp failed");
        return ret;
    }

    if (ATLAS_OK != GetThreadInstanceId()) {
        ATLAS_LOG_ERROR("Get self and next thread instance id failed");
        return ATLAS_ERROR;
    }

    return ATLAS_OK;
}

AtlasError Preprocess::OpenVideoCapture() {
    if (IsDigitStr(streamName_)) {
        int cameraId = atoi(streamName_.c_str());
        if ((cameraId < 0) || (cameraId >= CAMERA_ID_INVALID)) {
            ATLAS_LOG_ERROR("Invalid camera id arg %s, only allow %d and %d",
            streamName_.c_str(), CAMERA_ID_0, CAMERA_ID_1);
            return ATLAS_ERROR;
        }
        cap_ = new AtlasVideoCapture(cameraId);
    } else if (IsRtspAddr(streamName_)) {
        cap_ = new AtlasVideoCapture(streamName_);
    } else if (IsVideoFile(streamName_)) {
        if (!IsPathExist(streamName_)) {
            ATLAS_LOG_ERROR("The %s is inaccessible", streamName_.c_str());
            return ATLAS_ERROR;
        }
        cap_ = new AtlasVideoCapture(streamName_);
    } else {
        ATLAS_LOG_ERROR("Invalid param. The arg should be accessible rtsp,"
                        " video file or camera id");
        return ATLAS_ERROR;
    }

    if(!cap_->IsOpened()) {
        delete cap_;
        ATLAS_LOG_ERROR("Failed to open video");
        return ATLAS_ERROR;
    }

    return ATLAS_OK;
}

AtlasError Preprocess::GetThreadInstanceId() {
    selfThreadId_ = SelfInstanceId();
    nextThreadId_ = GetAtlasThreadIdByName(kInferName);
    if (display_) {
        postprocThreadId_ = GetAtlasThreadIdByName(kPostprocName); 
    } else {
        postprocThreadId_ = GetAtlasThreadIdByName(kPostprocName2); 
        if (postprocThreadId_ == INVALID_INSTANCE_ID) {
            postprocThreadId_ = GetAtlasThreadIdByName(kPostprocName); 
        }
    }  

    if ((selfThreadId_ == INVALID_INSTANCE_ID) ||
        (nextThreadId_ == INVALID_INSTANCE_ID) ||
        (postprocThreadId_ == INVALID_INSTANCE_ID)) {
        ATLAS_LOG_ERROR("Self instance id %d, next instance id %d, "
                        "postprocess instance id %d",
                        selfThreadId_, nextThreadId_, postprocThreadId_);
                        
        return ATLAS_ERROR;
    }

    return ATLAS_OK;
}


AtlasError Preprocess::Process(int msgId, shared_ptr<void> msgData) {
    AtlasError ret = ATLAS_OK;
    switch(msgId) {
        case MSG_APP_START:
            ret = AppStartMsgProcess();
            break;
        case MSG_READ_FRAME:
            ret = ReadFrameMsgProcess();
            break;
        default:
            ATLAS_LOG_ERROR("Preprocess thread receive unknow msg %d", msgId);
            break;
    }

    return ret;
}

AtlasError Preprocess::AppStartMsgProcess() {
    AtlasError ret = SendMessage(selfThreadId_, MSG_READ_FRAME, nullptr);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Process app start message failed, error %d", ret);
    }    

    return ret;
}

AtlasError Preprocess::ReadFrameMsgProcess() {
    ImageData image;
    AtlasError ret = cap_->Read(image);
    if (ret != ATLAS_OK) { 
        ATLAS_LOG_ERROR("Channel %d: read frame failed, error %d", 
                        channelId_, ret);
        return ret;                
    }

    frameCnt_++;
    if ((frameCnt_ % 5) == 0) {
        ProcessImage(image);
    } 

    ret = SendMessage(selfThreadId_, MSG_READ_FRAME, nullptr);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Send read frame message failed, error %d", ret);
        return ret;
    } 

    return ATLAS_OK;
}

void Preprocess::ProcessImage(ImageData image) {
    shared_ptr<PreprocDataMsg> preprocData = make_shared<PreprocDataMsg>();
   
    //预处理图片:读取图片,讲图片缩放到模型输入要求的尺寸
    AtlasError ret = dvpp_.Resize(preprocData->resizedImage,image,
                       modelWidth_, modelHeight_);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Resize image failed");
        return;
    }

    if (display_) {
        ret = ConvertImage(preprocData->jpgImage, image);
        if (ret != ATLAS_OK) {
            return;
        }
    }
    
    preprocData->display = display_;
    preprocData->channelId = channelId_;
    preprocData->postprocThreadId = postprocThreadId_;
    preprocData->frameWidth = image.width;
    preprocData->frameHeight = image.height;
    ret = SendMessage(nextThreadId_, MSG_PREPROC_DATA, preprocData);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Send preprocess data failed, error %d", ret);
    }

    return;
}

AtlasError Preprocess::ConvertImage(ImageData& destImage, 
                                    ImageData& srcImage) {
    ImageData jpgImage;

    AtlasError ret = dvpp_.JpegE(jpgImage, srcImage);
    if(ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Convert yuv to jpeg failed");
        return ATLAS_ERROR;
    }
    
    aclrtRunMode runMode = GetRunMode();
    if (runMode == ACL_HOST) {
        AtlasError ret = CopyImageToLocal(destImage, jpgImage, runMode);
        if (ret != ATLAS_OK) {
            ATLAS_LOG_ERROR("Copy jpeg image to local failed");
            return ret;
        }
    } else {
        destImage = jpgImage;
    }

    return ATLAS_OK;
}