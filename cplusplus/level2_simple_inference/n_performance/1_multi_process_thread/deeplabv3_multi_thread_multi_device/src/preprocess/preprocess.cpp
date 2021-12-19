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
namespace {
ImageData imageFrame;
vector<string> fileVec;
vector<string>::iterator imageFile;
}
struct timespec time1 = {0, 0};
struct timespec time2 = {0, 0};

PreprocessThread::PreprocessThread(string& filePath, uint32_t modelWidth, 
                       uint32_t modelHeight, uint32_t postThreadNum, uint32_t inferThreadNum) : 
filePath_(filePath),
stream_(nullptr),
modelWidth_(modelWidth),
modelHeight_(modelHeight),
selfThreadId_(INVALID_INSTANCE_ID),
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
    dvpp_.DestroyResource();
}

AclLiteError PreprocessThread::Init() {
    GetAllFiles(filePath_, fileVec);
    if (fileVec.empty()) {
        ACLLITE_LOG_ERROR("Failed to deal all empty path=%s.", filePath_.c_str());
    }
    imageFile = fileVec.begin();

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
    if (selfThreadId_ == INVALID_INSTANCE_ID) {
        ACLLITE_LOG_ERROR("Self instance id %d", selfThreadId_);
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
            ret = MsgProcess(imageFrame, preprocDataMsg);
            ret = MsgSend(preprocDataMsg);
            clock_gettime(CLOCK_REALTIME, &time2);
            cout << "preprocess time is: " << (time2.tv_sec - time1.tv_sec)*1000 + (time2.tv_nsec - time1.tv_nsec)/1000000 << "ms" << endl;
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
        indexCount_ = (++indexCount_) % postThreadNum_; //postcount初始值为postnum-1;
    }
    postChannel_ = inferChannel_ * postThreadNum_ + indexCount_;

    preprocDataMsg->inferThreadId = nextThreadId_[inferChannel_];
    preprocDataMsg->postprocThreadId = postprocThreadId_[postChannel_];

    preprocDataMsg->frameNum = frameCnt_;
    preprocDataMsg->isLastFrame = 0;
    if (imageFile == fileVec.end()) { 
        preprocDataMsg->isLastFrame = 1;
        return ACLLITE_OK;
    }
    ReadJpeg(imageFrame, *imageFile);
    if (imageFrame.data == nullptr) {
        ACLLITE_LOG_ERROR("Read image %s failed", (*imageFile).c_str());
        return ACLLITE_ERROR;
    }
    preprocDataMsg->imageFileName = *imageFile;
    imageFile++;
    frameCnt_++;
    return ACLLITE_OK;
}

AclLiteError PreprocessThread::MsgProcess(ImageData& imageFrame, 
                         shared_ptr<PreprocDataMsg> &preprocDataMsg) {
    if(preprocDataMsg->isLastFrame == 1)
        return ACLLITE_OK;
    ImageData imageDevice;
    AclLiteError ret = CopyImageToDevice(imageDevice, imageFrame, runMode_, MEMORY_DVPP);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Copy image to device failed");
        return ACLLITE_ERROR;
    }

    ImageData yuvImage;
    ret = dvpp_.JpegD(yuvImage, imageDevice);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Convert jpeg to yuv failed");
        return ACLLITE_ERROR;
    }

    yuvImage.width = imageDevice.width;
    yuvImage.height = imageDevice.height;

    ImageData resizedImage;
    ret = dvpp_.CropResolution(resizedImage, yuvImage, modelWidth_, modelHeight_);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("dvpp_cropandpaste image failed");
        return ACLLITE_ERROR;
    }
    //拷贝resizedImage  preprocDataMsg->resizedMat
    ret = CopyImageToLocal(preprocDataMsg->resizedMat, resizedImage, runMode_);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Copy image to host failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError PreprocessThread::MsgSend(shared_ptr<PreprocDataMsg> &preprocDataMsg) {
    AclLiteError ret;
    if(preprocDataMsg->isLastFrame == 0){
        ret = SendMessage(preprocDataMsg->inferThreadId, MSG_PREPROC_DATA, preprocDataMsg);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Send preprocess data failed, error %d", ret);
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
                preprocDataMsgEnd->inferThreadId = nextThreadId_[i];
                preprocDataMsgEnd->postprocThreadId = postprocThreadId_[i*postThreadNum_+j];
                ret = SendMessage(preprocDataMsgEnd->inferThreadId, MSG_PREPROC_END, preprocDataMsgEnd);
                if (ret != ACLLITE_OK) {
                    ACLLITE_LOG_ERROR("Send preprocess data failed, error %d", ret);
                }
            }
        }
    }

    return ACLLITE_OK;
}
