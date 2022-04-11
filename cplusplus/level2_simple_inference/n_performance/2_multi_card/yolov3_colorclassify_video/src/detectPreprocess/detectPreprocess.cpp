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
#include "params.h"
#include "detectPreprocess.h"
#include "AclLiteApp.h"

using namespace std;

struct timespec time1 = {0, 0};
struct timespec time2 = {0, 0};

namespace {
uint32_t kModelWidth = 416;
uint32_t kModelHeight = 416;
}

DetectPreprocessThread::DetectPreprocessThread(const char*& configFile, int32_t i, aclrtRunMode& runMode) : 
configFile_(configFile),
cap_(nullptr),
runMode_(runMode),
modelWidth_(kModelWidth),
modelHeight_(kModelHeight),
selfThreadId_(INVALID_INSTANCE_ID),
inferThreadId_(INVALID_INSTANCE_ID),
detectPostThreadId_(INVALID_INSTANCE_ID),
classifyPreThreadId_(INVALID_INSTANCE_ID),
classifyPostThreadId_(INVALID_INSTANCE_ID),
frameCnt_(0),
channelId_(i){
}

DetectPreprocessThread::~DetectPreprocessThread() {
    if (cap_ != nullptr) {
        cap_->Close();
        delete cap_;
    }
    dvpp_.DestroyResource();
}

AclLiteError DetectPreprocessThread::OpenVideoCapture() {
    if (IsRtspAddr(videoPath_)) {
        cap_ = new AclLiteVideoProc(videoPath_);
    } else if (IsVideoFile(videoPath_)) {
        if (!IsPathExist(videoPath_)) {
            ACLLITE_LOG_ERROR("The %s is inaccessible", videoPath_.c_str());
            return ACLLITE_ERROR;
        }
        cap_ = new AclLiteVideoProc(videoPath_);
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

AclLiteError DetectPreprocessThread::GetBaseConfig(std::string& videoPath, uint32_t channelId) {
    std::string videoPathKey = "videoPath_" + to_string(channelId);
    std::map<std::string, std::string> config;
    if(!ReadConfig(config, configFile_)) {
        return ACLLITE_ERROR;
    }
    
    std::map<std::string, std::string>::const_iterator mIter = config.begin();
    for (; mIter != config.end(); ++mIter) {
        if (mIter->first == videoPathKey) {
            videoPath.assign(mIter->second.c_str());
            ACLLITE_LOG_INFO("video %d path: %s", 
                            channelId, videoPath.c_str());
        }
    }

    return ACLLITE_OK;
}

AclLiteError DetectPreprocessThread::Init() {

    AclLiteError aclRet = GetBaseConfig(videoPath_, channelId_);
    if (aclRet) {
        ACLLITE_LOG_ERROR("GetBaseConfig failed, error %d", aclRet);
        return ACLLITE_ERROR;
    }

    if (ACLLITE_OK != OpenVideoCapture()) {
        return ACLLITE_ERROR;
    }

    aclRet = dvpp_.Init();
    if (aclRet) {
        ACLLITE_LOG_ERROR("Dvpp init failed, error %d", aclRet);
        return ACLLITE_ERROR;
    }

    //Get the relevant thread instance id
    selfThreadId_ = SelfInstanceId();
    inferThreadId_ = GetAclLiteThreadIdByName(kInferName[channelId_]);
    detectPostThreadId_ = GetAclLiteThreadIdByName(kDetectPostName[channelId_]);
    classifyPreThreadId_ = GetAclLiteThreadIdByName(kClassifyPreName[channelId_]);
    classifyPostThreadId_ = GetAclLiteThreadIdByName(kClassifyPostName[channelId_]);
    if ((selfThreadId_ == INVALID_INSTANCE_ID) ||
        (inferThreadId_ == INVALID_INSTANCE_ID) ||
        (detectPostThreadId_ == INVALID_INSTANCE_ID) ||
        (classifyPreThreadId_ == INVALID_INSTANCE_ID) ||
        (classifyPostThreadId_ == INVALID_INSTANCE_ID)) {
        ACLLITE_LOG_ERROR("Self instance id %d, infer instance id %d, detectPost instance id %d, classifyPre instance id %d, classifyPost instance id %d", 
                           selfThreadId_, inferThreadId_, detectPostThreadId_, 
                           classifyPreThreadId_, classifyPostThreadId_);
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError DetectPreprocessThread::Process(int msgId, 
                             shared_ptr<void> msgData) {
    AclLiteError ret = ACLLITE_OK;
    shared_ptr<CarDetectDataMsg> carDetectDataMsg = make_shared<CarDetectDataMsg>();
    switch(msgId) {
        case MSG_APP_START:
            ret = AppStart();
            break;
        case MSG_READ_FRAME:
            clock_gettime(CLOCK_REALTIME, &time1);
            ret = ReadFrame(carDetectDataMsg);
            ret = MsgProcess(carDetectDataMsg);
            ret = MsgSend(carDetectDataMsg);
            clock_gettime(CLOCK_REALTIME, &time2);
            //cout << "preprocess time is: " << (time2.tv_sec - time1.tv_sec)*1000 + (time2.tv_nsec - time1.tv_nsec)/1000000 << "ms" << endl;
            break;
        default:
            ACLLITE_LOG_ERROR("Detect Preprocess thread receive unknow msg %d", msgId);
            break;
    }

    return ret;
}

AclLiteError DetectPreprocessThread::AppStart() {
    AclLiteError ret = SendMessage(selfThreadId_, MSG_READ_FRAME, nullptr);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Process app start message failed, error %d", ret);
    }    

    return ret;
}

AclLiteError DetectPreprocessThread::ReadFrame(shared_ptr<CarDetectDataMsg> &carDetectDataMsg) {
    
    carDetectDataMsg->inferThreadId = inferThreadId_;
    carDetectDataMsg->detectPostThreadId = detectPostThreadId_;
    carDetectDataMsg->classifyPreThreadId = classifyPreThreadId_;
    carDetectDataMsg->classifyPostThreadId = classifyPostThreadId_;

    carDetectDataMsg->channelId = channelId_;
    carDetectDataMsg->frameNum = frameCnt_;
    carDetectDataMsg->isLastFrame = 0;

    AclLiteError ret = cap_->Read(carDetectDataMsg->imageFrame);
    if (ret != ACLLITE_OK) { 
        carDetectDataMsg->isLastFrame = 1;
        return ret;
    }
    frameCnt_++;
    return ACLLITE_OK;
}

AclLiteError DetectPreprocessThread::MsgProcess(shared_ptr<CarDetectDataMsg> &carDetectDataMsg) {
    if(carDetectDataMsg->isLastFrame == 1)
        return ACLLITE_OK;
    ImageData imageDevice;
    AclLiteError ret = CopyImageToDevice(imageDevice, carDetectDataMsg->imageFrame, runMode_, MEMORY_DVPP);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Copy image to device failed");
        return ACLLITE_ERROR;
    }

    ret = dvpp_.Resize(carDetectDataMsg->resizedMat, imageDevice, modelWidth_, modelHeight_);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("dvpp_resize image failed");
        return ACLLITE_ERROR;
    }

    ImageData yuvImage;
    ret = CopyImageToLocal(yuvImage, carDetectDataMsg->imageFrame, runMode_);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Copy image to host failed");
        return ACLLITE_ERROR;
    }

    cv::Mat yuvimg(yuvImage.height * 3 / 2, yuvImage.width, CV_8UC1, yuvImage.data.get());
    cv::cvtColor(yuvimg, carDetectDataMsg->frame, CV_YUV2BGR_NV12);

    return ACLLITE_OK;
}

AclLiteError DetectPreprocessThread::MsgSend(shared_ptr<CarDetectDataMsg> &carDetectDataMsg) {
    AclLiteError ret;
    if(carDetectDataMsg->isLastFrame == 0){
        while(1){
            ret = SendMessage(carDetectDataMsg->inferThreadId, MSG_DETECT_PREPROC_DATA, carDetectDataMsg);
            if(ret == ACLLITE_ERROR_ENQUEUE)
            {
                usleep(500);
                continue;
            }
            else if(ret == ACLLITE_OK)
            {
                break;
            }
            else
            {
                ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
                return ret;
            }
        }

        ret = SendMessage(selfThreadId_, MSG_READ_FRAME, nullptr);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
            return ret;
        } 
    }
    else{
            shared_ptr<CarDetectDataMsg> carDetectDataMsgEnd = make_shared<CarDetectDataMsg>();
            carDetectDataMsgEnd->inferThreadId = inferThreadId_;
            carDetectDataMsgEnd->detectPostThreadId = detectPostThreadId_;
            carDetectDataMsgEnd->classifyPreThreadId = classifyPreThreadId_;
            carDetectDataMsgEnd->classifyPostThreadId = classifyPostThreadId_;
            carDetectDataMsgEnd->channelId = carDetectDataMsg->channelId;
            carDetectDataMsgEnd->frameNum = carDetectDataMsg->frameNum;
            carDetectDataMsgEnd->isLastFrame = carDetectDataMsg->isLastFrame;
            while(1)
            {
                ret = SendMessage(carDetectDataMsgEnd->inferThreadId, MSG_DETECT_PREPROC_DATA, carDetectDataMsgEnd);
                if(ret == ACLLITE_ERROR_ENQUEUE)
                {
                    usleep(500);
                    continue;
                }
                else if(ret == ACLLITE_OK)
                {
                    break;
                }
                else
                {
                    ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
                    return ret;
                }
            }
    }

    return ACLLITE_OK;
}
