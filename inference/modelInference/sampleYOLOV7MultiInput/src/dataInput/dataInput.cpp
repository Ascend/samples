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
#include "Params.h"
#include "dataInput.h"
#include <sys/time.h>

namespace{
    const uint32_t kYuvMultiplier = 3;
    const uint32_t kYuvDivisor = 2;
    const uint32_t kSleepTime = 500;
    const uint32_t kOneSec = 1000000;
    const uint32_t kOneMSec = 1000;
}
using namespace std;

DataInputThread::DataInputThread(
    int32_t deviceId, int32_t channelId, aclrtRunMode& runMode,
    string inputDataType, string inputDataPath, string inferName,
    int postThreadNum, uint32_t batch, int framesPerSecond)
    :deviceId_(deviceId), channelId_(channelId), runMode_(runMode), postproId_(0),
    inputDataType_(inputDataType), inputDataPath_(inputDataPath),
    inferName_(inferName), cap_(nullptr), frameCnt_(0), postThreadNum_(postThreadNum),
    selfThreadId_(INVALID_INSTANCE_ID), preThreadId_(INVALID_INSTANCE_ID),
    inferThreadId_(INVALID_INSTANCE_ID), dataOutputThreadId_(INVALID_INSTANCE_ID),
    rtspDisplayThreadId_(INVALID_INSTANCE_ID), batch_(batch), framesPerSecond_(framesPerSecond),
    msgNum_(0)
{
    for (int i = 0; i < postThreadNum; i++) {
        postThreadId_.push_back(INVALID_INSTANCE_ID);
    }
}

DataInputThread::~DataInputThread()
{
    if (inputDataType_ == "pic") {
        dvpp_.DestroyResource();
    }
    if (cap_ != nullptr) {
        cap_->Close();
        delete cap_;
        cap_ = nullptr;
    }
}

AclLiteError DataInputThread::OpenPicsDir()
{
    string inputImageDir = inputDataPath_;
    GetAllFiles(inputImageDir, fileVec_);
    if (fileVec_.empty()) {
        ACLLITE_LOG_ERROR("Failed to deal all empty path=%s.", inputImageDir.c_str());
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError DataInputThread::OpenVideoCapture()
{
    if (IsRtspAddr(inputDataPath_)) {
        cap_ = new AclLiteVideoProc(inputDataPath_, deviceId_);
    } else if (IsVideoFile(inputDataPath_)) {
        if (!IsPathExist(inputDataPath_)) {
            ACLLITE_LOG_ERROR("The %s is inaccessible", inputDataPath_.c_str());
            return ACLLITE_ERROR;
        }
        cap_ = new AclLiteVideoProc(inputDataPath_, deviceId_);
    } else {
        ACLLITE_LOG_ERROR("Invalid param. The arg should be accessible rtsp,"
                          " video file or camera id");
        return ACLLITE_ERROR;
    }

    if (!cap_->IsOpened()) {
        delete cap_;
        cap_ = nullptr;
        ACLLITE_LOG_ERROR("Failed to open video");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError DataInputThread::Init()
{
    AclLiteError aclRet;
    if (inputDataType_ == "pic") {
        aclRet = OpenPicsDir();
        if (aclRet != ACLLITE_OK) {
            return ACLLITE_ERROR;
        }
        aclRet = dvpp_.Init("DVPP_CHNMODE_JPEGD");
        if (aclRet) {
            ACLLITE_LOG_ERROR("Dvpp init failed, error %d", aclRet);
            return ACLLITE_ERROR;
        }
    } else {
        aclRet = OpenVideoCapture();
        if (aclRet != ACLLITE_OK) {
            return ACLLITE_ERROR;
        }
    }
    // Get the relevant thread instance id
    selfThreadId_ = SelfInstanceId();
    inferThreadId_ = GetAclLiteThreadIdByName(inferName_);
    preThreadId_ = GetAclLiteThreadIdByName(kPreName + to_string(channelId_));
    dataOutputThreadId_ = GetAclLiteThreadIdByName(kDataOutputName + to_string(channelId_));
    rtspDisplayThreadId_ = GetAclLiteThreadIdByName(kRtspDisplayName + to_string(channelId_));
    for (int i = 0; i < postThreadNum_; i++) {
        postThreadId_[i] = GetAclLiteThreadIdByName(kPostName + to_string(channelId_) + "_" + to_string(i));
        if (postThreadId_[i] == INVALID_INSTANCE_ID) {
            ACLLITE_LOG_ERROR("%d postprocess instance id %d",
                i, postThreadId_[i]);
            return ACLLITE_ERROR;
        }
    }

    if ((selfThreadId_ == INVALID_INSTANCE_ID) ||
        (preThreadId_ == INVALID_INSTANCE_ID) ||
        (inferThreadId_ == INVALID_INSTANCE_ID) ||
        (dataOutputThreadId_ == INVALID_INSTANCE_ID)) {
        ACLLITE_LOG_ERROR("Self instance id %d, pre instance id %d, infer instance id %d,"
                          "dataOutput instance id %d",
                          selfThreadId_, preThreadId_, inferThreadId_,
                          dataOutputThreadId_);
        return ACLLITE_ERROR;
    }
    int oneSecond = 1000;
    lastDecodeTime_ = 0;
    waitTime_  = oneSecond / framesPerSecond_;

    return ACLLITE_OK;
}

AclLiteError DataInputThread::Process(int msgId, shared_ptr<void> msgData)
{
    shared_ptr<DetectDataMsg> detectDataMsg = make_shared<DetectDataMsg>();
    switch (msgId) {
        case MSG_APP_START:
            AppStart();
            break;
        case MSG_READ_FRAME:
            MsgRead(detectDataMsg);
            MsgSend(detectDataMsg);
            break;
        default:
            ACLLITE_LOG_ERROR("Detect Preprocess thread receive unknow msg %d", msgId);
            break;
    }

    return ACLLITE_OK;
}

AclLiteError DataInputThread::AppStart()
{
    AclLiteError ret = SendMessage(selfThreadId_, MSG_READ_FRAME, nullptr);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Process app start message failed, error %d", ret);
    }

    return ret;
}

AclLiteError DataInputThread::ReadPic(shared_ptr<DetectDataMsg> &detectDataMsg)
{
    AclLiteError ret = ACLLITE_OK;
    if (frameCnt_ == fileVec_.size()) {
        detectDataMsg->isLastFrame = true;
        return ACLLITE_OK;
    }
    ImageData jpgImg, dvppImg, decodedImg;
    string picFile = fileVec_[frameCnt_];
    ret = ReadJpeg(jpgImg, picFile);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Read Jpeg image failed");
        return ACLLITE_ERROR;
    }
    ret = CopyImageToDevice(dvppImg, jpgImg, runMode_, MEMORY_DVPP);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Copy image to device failed");
        return ACLLITE_ERROR;
    }
    ret = dvpp_.JpegD(decodedImg, dvppImg);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Pic decode failed");
        return ACLLITE_ERROR;
    }
    cv::Mat frame = cv::imread(picFile);
    detectDataMsg->decodedImg.push_back(decodedImg);
    detectDataMsg->frame.push_back(frame);
    return ACLLITE_OK;
}

AclLiteError DataInputThread::ReadStream(shared_ptr<DetectDataMsg> &detectDataMsg)
{
    // get time now
    timeval tv;
    gettimeofday(&tv, 0);
    int64_t now = ((int64_t)tv.tv_sec * kOneSec + (int64_t)tv.tv_usec) / kOneMSec;
    // get time last record
    if (lastDecodeTime_ == 0) {
        lastDecodeTime_ = now;
    }
    // calculate interval
    realWaitTime_ = (now - lastDecodeTime_);
    
    AclLiteError ret = ACLLITE_OK;
    ImageData decodedImg;

    while (realWaitTime_ < waitTime_) {
        ret = cap_->Read(decodedImg);
        if(ret == ACLLITE_RETRY)
        {
            ret = cap_->Read(decodedImg);
        }
        if (ret != ACLLITE_OK) {
            if (ret == ACLLITE_ERROR_DECODE_FINISH) {
                detectDataMsg->isLastFrame = true;
                return ACLLITE_ERROR_DECODE_FINISH;
            } else {
                detectDataMsg->isLastFrame = true;
                ACLLITE_LOG_ERROR("Read frame failed, error %d", ret);
                return ACLLITE_ERROR;
            }
        }
        // get time now
        gettimeofday(&tv, 0);
        now = ((int64_t)tv.tv_sec * kOneSec + (int64_t)tv.tv_usec) / kOneMSec;
        // calculate interval agian
        realWaitTime_ = (now - lastDecodeTime_);
    }
    
    ret = cap_->Read(decodedImg);
    if(ret == ACLLITE_RETRY)
    {
        ret = cap_->Read(decodedImg);
    }
    if (ret != ACLLITE_OK) {
        if (ret == ACLLITE_ERROR_DECODE_FINISH) {
            detectDataMsg->isLastFrame = true;
            return ACLLITE_ERROR_DECODE_FINISH;
        } else {
            detectDataMsg->isLastFrame = true;
            ACLLITE_LOG_ERROR("Read frame failed, error %d", ret);
            return ACLLITE_ERROR;
        }
    }
    // get frame
    ImageData yuvImage;
    ret = CopyImageToLocal(yuvImage, decodedImg, runMode_);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Copy image to host failed");
        return ACLLITE_ERROR;
    }
    cv::Mat frame;
    cv::Mat yuvimg(yuvImage.height * kYuvMultiplier / kYuvDivisor, yuvImage.width, CV_8UC1, yuvImage.data.get());
    cv::cvtColor(yuvimg, frame, CV_YUV2BGR_NV12);
    detectDataMsg->decodedImg.push_back(decodedImg);
    detectDataMsg->frame.push_back(frame);
    lastDecodeTime_ = now;
    return ACLLITE_OK;
}

AclLiteError DataInputThread::GetOneFrame(shared_ptr<DetectDataMsg> &detectDataMsg)
{
    AclLiteError ret;
    if (inputDataType_ == "pic") {
        ret = ReadPic(detectDataMsg);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Read pic failed, error %d", ret);
            return ACLLITE_ERROR;
        }
    } else if (inputDataType_ == "video" ||
                inputDataType_ == "rtsp") {
        ret = ReadStream(detectDataMsg);
        if (ret != ACLLITE_OK) {
            return ACLLITE_ERROR;
        }
    } else {
        ACLLITE_LOG_ERROR("Invalid input data type, Please check your input file!");
        return ACLLITE_ERROR;
    }
    return ACLLITE_OK;
}

AclLiteError DataInputThread::MsgRead(shared_ptr<DetectDataMsg> &detectDataMsg)
{
    AclLiteError ret;
    postproId_ = msgNum_ % postThreadNum_;
    detectDataMsg->isLastFrame = false;
    detectDataMsg->detectPreThreadId = preThreadId_;
    detectDataMsg->detectInferThreadId = inferThreadId_;
    detectDataMsg->detectPostThreadId = postThreadId_[postproId_];
    detectDataMsg->postId = postproId_;
    detectDataMsg->dataOutputThreadId = dataOutputThreadId_;
    detectDataMsg->rtspDisplayThreadId = rtspDisplayThreadId_;
    detectDataMsg->deviceId = deviceId_;
    detectDataMsg->channelId = channelId_;
    detectDataMsg->msgNum = msgNum_;
    msgNum_++;
    GetOneFrame(detectDataMsg);
    if (detectDataMsg->isLastFrame) {
        return ACLLITE_OK;
    }
    frameCnt_++;
    while (frameCnt_ % batch_) {
        GetOneFrame(detectDataMsg);
        if (detectDataMsg->isLastFrame) {
            break;
        }
        frameCnt_++;
    }

    return ACLLITE_OK;
}

AclLiteError DataInputThread::MsgSend(shared_ptr<DetectDataMsg> &detectDataMsg)
{
    AclLiteError ret;
    if (detectDataMsg->isLastFrame == false) {
        while (1) {
            ret = SendMessage(detectDataMsg->detectPreThreadId, MSG_PREPROC_DETECTDATA, detectDataMsg);
            if (ret == ACLLITE_ERROR_ENQUEUE) {
                usleep(kSleepTime);
                continue;
            } else if(ret == ACLLITE_OK) {
                break;
            } else {
                ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
                return ret;
            }
        }

        ret = SendMessage(selfThreadId_, MSG_READ_FRAME, nullptr);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
            return ret;
        }
    } else {
        for (int i = 0; i < postThreadNum_; i++) {
            while (1) {
                ret = SendMessage(detectDataMsg->detectPreThreadId, MSG_PREPROC_DETECTDATA, detectDataMsg);
                if (ret == ACLLITE_ERROR_ENQUEUE) {
                    usleep(kSleepTime);
                    continue;
                } else if (ret == ACLLITE_OK) {
                    break;
                } else {
                    ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
                    return ret;
                }
            }
        }
    }
    return ACLLITE_OK;
}
