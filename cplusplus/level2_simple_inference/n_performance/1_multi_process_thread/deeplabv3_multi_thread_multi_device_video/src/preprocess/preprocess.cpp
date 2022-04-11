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
#include "object_detection.h"
#include "preprocess.h"
#include "acllite/AclLiteApp.h"

using namespace std;

struct timespec time1 = {0, 0};
struct timespec time2 = {0, 0};

PreprocessThread::PreprocessThread(string& videoPath, uint32_t modelWidth, 
                                   uint32_t modelHeight, uint32_t postThreadNum, 
                                   uint32_t inferThreadNum, aclrtContext& context) : 
videoPath_(videoPath),
context_(context),
cap_(nullptr),
stream_(nullptr),
modelWidth_(modelWidth),
modelHeight_(modelHeight),
selfThreadId_(INVALID_INSTANCE_ID),
videoThreadId_(INVALID_INSTANCE_ID),
inferThreadNum_(inferThreadNum),
postThreadNum_(postThreadNum),
inferChannel_(0),
postChannel_(0),
indexCount_(postThreadNum - 1),
frameCnt_(0) {
    for(int i = 0; i < inferThreadNum; i++){
        nextThreadId_.push_back(INVALID_INSTANCE_ID);
        for(int j = 0; j < postThreadNum; j++){
            postprocThreadId_.push_back(INVALID_INSTANCE_ID);
        }
    }
}

PreprocessThread::~PreprocessThread() {
    aclError ret = aclrtSetCurrentContext(context_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("PreprocessThread destructor set context failed, error: %d", ret);
    }
    if (cap_ != nullptr) {
        cap_->Close();
        delete cap_;
    }
    dvpp_.DestroyResource();
}

AclLiteError PreprocessThread::OpenVideoCapture() {
    if (IsDigitStr(videoPath_)) {
        int cameraId = atoi(videoPath_.c_str());
        if ((cameraId < 0) || (cameraId >= CAMERA_ID_INVALID)) {
            ACLLITE_LOG_ERROR("Invalid camera id arg %s, only allow %d and %d",
            videoPath_.c_str(), CAMERA_ID_0, CAMERA_ID_1);
            return ACLLITE_ERROR;
        }
        cap_ = new AclLiteVideoProc(cameraId);
    } else if (IsRtspAddr(videoPath_)) {
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

AclLiteError PreprocessThread::Init() {

    if (ACLLITE_OK != OpenVideoCapture()) {
        return ACLLITE_ERROR;
    }

    aclError aclRet = aclrtGetRunMode(&runMode_);
    if (aclRet != ACL_ERROR_NONE) {
        ACLLITE_LOG_ERROR("acl get run mode failed");
        return ACLLITE_ERROR_GET_RUM_MODE;
    }

    aclRet = dvpp_.Init();
    if (aclRet) {
        ACLLITE_LOG_ERROR("Dvpp init failed, error %d", aclRet);
        return ACLLITE_ERROR;
    }

    //Get the relevant thread instance id
    selfThreadId_ = SelfInstanceId();
    videoThreadId_ = GetAclLiteThreadIdByName(kVideoprocName);
    if ((selfThreadId_ == INVALID_INSTANCE_ID) ||
        (videoThreadId_ == INVALID_INSTANCE_ID)) {
        ACLLITE_LOG_ERROR("Self instance id %d, video process instance id %d", selfThreadId_, videoThreadId_);
        return ACLLITE_ERROR;
    }

    for(int i = 0; i < inferThreadNum_; i++)
    {
        nextThreadId_[i] = GetAclLiteThreadIdByName(kInferName[i]);
        if (nextThreadId_[i] == INVALID_INSTANCE_ID){
            ACLLITE_LOG_ERROR("%d inference instance id %d",
                        i, nextThreadId_[i]);
            return ACLLITE_ERROR;
        }
        for(int j = 0; j < postThreadNum_; j++)
        {
            postprocThreadId_[i*postThreadNum_+j] = GetAclLiteThreadIdByName(kPostprocName[i*postThreadNum_+j]);
            if (postprocThreadId_[i*postThreadNum_+j] == INVALID_INSTANCE_ID){
                ACLLITE_LOG_ERROR("%d postprocess instance id %d",
                            i*postThreadNum_+j, postprocThreadId_[i*postThreadNum_+j]);
                return ACLLITE_ERROR;
            }
        }
    }

    return ACLLITE_OK;
}

AclLiteError PreprocessThread::Process(int msgId, 
                             shared_ptr<void> msgData) {
    AclLiteError ret = ACLLITE_OK;
    shared_ptr<PreprocDataMsg> preprocDataMsg = make_shared<PreprocDataMsg>();
    switch(msgId) {
        case MSG_APP_START:
            ret = AppStart();
            break;
        case MSG_READ_FRAME:
            clock_gettime(CLOCK_REALTIME, &time1);
            ret = ReadFrame(preprocDataMsg);
            ret = MsgProcess(preprocDataMsg);
            ret = MsgSend(preprocDataMsg);
            clock_gettime(CLOCK_REALTIME, &time2);
            //cout << "preprocess time is: " << (time2.tv_sec - time1.tv_sec)*1000 + (time2.tv_nsec - time1.tv_nsec)/1000000 << "ms" << endl;
            break;
        default:
            ACLLITE_LOG_ERROR("Preprocess thread receive unknow msg %d", msgId);
            break;
    }

    return ret;
}

AclLiteError PreprocessThread::AppStart() {
    AclLiteError ret = SendMessage(selfThreadId_, MSG_READ_FRAME, nullptr);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Process app start message failed, error %d", ret);
    }    

    return ret;
}

AclLiteError PreprocessThread::ReadFrame(shared_ptr<PreprocDataMsg> &preprocDataMsg) {
    inferChannel_ = frameCnt_ % inferThreadNum_;
    if(inferChannel_ == 0){
        indexCount_ = (++indexCount_) % postThreadNum_;
    }
    postChannel_ = inferChannel_ * postThreadNum_ + indexCount_;

    preprocDataMsg->inferThreadId = nextThreadId_[inferChannel_];
    preprocDataMsg->postprocThreadId = postprocThreadId_[postChannel_];
    preprocDataMsg->videoProcessThreadId = videoThreadId_;
    preprocDataMsg->channelId = postChannel_;
    preprocDataMsg->frameNum = frameCnt_;
    preprocDataMsg->isLastFrame = 0;

    AclLiteError ret = cap_->Read(preprocDataMsg->imageFrame);
    if (ret != ACLLITE_OK) { 
        preprocDataMsg->isLastFrame = 1;
        return ret;
    }
    frameCnt_++;
    return ACLLITE_OK;
}

AclLiteError PreprocessThread::MsgProcess(shared_ptr<PreprocDataMsg> &preprocDataMsg) {
    if(preprocDataMsg->isLastFrame == 1)
        return ACLLITE_OK;
    ImageData imageDevice;
    AclLiteError ret = CopyImageToDevice(imageDevice, preprocDataMsg->imageFrame, runMode_, MEMORY_DVPP);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Copy image to device failed");
        return ACLLITE_ERROR;
    }

    ImageData resizedImage;
    ret = dvpp_.Resize(resizedImage, imageDevice, modelWidth_, modelHeight_);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("dvpp_resize image failed");
        return ACLLITE_ERROR;
    }

    ret = CopyImageToLocal(preprocDataMsg->resizedMat, resizedImage, runMode_);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Copy image to host failed");
        return ACLLITE_ERROR;
    }

    ImageData yuvImage;
    ret = CopyImageToLocal(yuvImage, preprocDataMsg->imageFrame, runMode_);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Copy image to host failed");
        return ACLLITE_ERROR;
    }

    cv::Mat yuvimg(yuvImage.height * 3 / 2, yuvImage.width, CV_8UC1, yuvImage.data.get());
    cv::cvtColor(yuvimg, preprocDataMsg->frame, CV_YUV2BGR_NV12);

    return ACLLITE_OK;
}

AclLiteError PreprocessThread::MsgSend(shared_ptr<PreprocDataMsg> &preprocDataMsg) {
    AclLiteError ret;
    if(preprocDataMsg->isLastFrame == 0){
        while(1){
            ret = SendMessage(preprocDataMsg->inferThreadId, MSG_PREPROC_DATA, preprocDataMsg);
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
        for(int i = 0; i < inferThreadNum_; i++)
        {
            for(int j = 0; j < postThreadNum_; j++){
                shared_ptr<PreprocDataMsg> preprocDataMsgEnd = make_shared<PreprocDataMsg>();
                preprocDataMsgEnd->videoProcessThreadId = preprocDataMsg->videoProcessThreadId;
                preprocDataMsgEnd->channelId = preprocDataMsg->channelId;
                preprocDataMsgEnd->frameNum = preprocDataMsg->frameNum;
                preprocDataMsgEnd->isLastFrame = preprocDataMsg->isLastFrame;
                preprocDataMsgEnd->inferThreadId = nextThreadId_[i];
                preprocDataMsgEnd->postprocThreadId = postprocThreadId_[i*postThreadNum_+j];
                while(1)
                {
                    ret = SendMessage(preprocDataMsgEnd->inferThreadId, MSG_PREPROC_DATA, preprocDataMsgEnd);
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
        }
    }

    return ACLLITE_OK;
}
