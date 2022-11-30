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
#include <sys/timeb.h>
#include "CarParams.h"
#include "detectPreprocess.h"
#include "AclLiteApp.h"

using namespace std;

namespace {
uint32_t kModelWidth = 416;
uint32_t kModelHeight = 416;
}

DetectPreprocessThread::DetectPreprocessThread(const char*& configFile,
    int32_t deviceId, int32_t channelId, aclrtRunMode& runMode, bool display)
    :display_(display), configFile_(configFile), cap_(nullptr),
    runMode_(runMode), modelWidth_(kModelWidth), modelHeight_(kModelHeight),
    selfThreadId_(INVALID_INSTANCE_ID), inferThreadId_(INVALID_INSTANCE_ID),
    detectPostThreadId_(INVALID_INSTANCE_ID), classifyPreThreadId_(INVALID_INSTANCE_ID),
    classifyPostThreadId_(INVALID_INSTANCE_ID), presentAgentDisplayThreadId_(INVALID_INSTANCE_ID),
    rtspDisplayThreadId_(INVALID_INSTANCE_ID), frameCnt_(0), deviceId_(deviceId), channelId_(channelId)
{
}

DetectPreprocessThread::~DetectPreprocessThread()
{
    if (cap_ != nullptr) {
        cap_->Close();
        delete cap_;
    }
    dvpp_.DestroyResource();
}

AclLiteError DetectPreprocessThread::OpenPicsDir()
{
    string inputImageDir = inputDataPath_;
    GetAllFiles(inputImageDir, fileVec_);
    if (fileVec_.empty()) {
        ACLLITE_LOG_ERROR("Failed to deal all empty path=%s.", inputImageDir.c_str());
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError DetectPreprocessThread::OpenVideoCapture()
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
        ACLLITE_LOG_ERROR("Failed to open video");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError DetectPreprocessThread::GetPicsDirConfig(std::string& picsDirPath, uint32_t channelId)
{
    std::string picsDirPathKey = "picsDirPath_" + to_string(channelId);
    std::map<std::string, std::string> config;
    if (!ReadConfig(config, configFile_)) {
        return ACLLITE_ERROR;
    }

    std::map<std::string, std::string>::const_iterator mIter = config.begin();
    for (; mIter != config.end(); ++mIter) {
        if (mIter->first == picsDirPathKey) {
            picsDirPath.assign(mIter->second.c_str());
            ACLLITE_LOG_INFO("pics directory %d path: %s",
                             channelId, picsDirPath.c_str());
        }
    }

    return ACLLITE_OK;
}

AclLiteError DetectPreprocessThread::GetInputDataType(std::string& inputType, uint32_t channelId)
{
    std::string inputTypeKey = "inputType_" + to_string(channelId);
    std::map<std::string, std::string> config;
    if (!ReadConfig(config, configFile_)) {
        return ACLLITE_ERROR;
    }

    std::map<std::string, std::string>::const_iterator mIter = config.begin();
    for (; mIter != config.end(); ++mIter) {
        if (mIter->first == inputTypeKey) {
            inputType.assign(mIter->second.c_str());
            ACLLITE_LOG_INFO("device %d input type is : %s",
                             channelId, inputType.c_str());
        }
    }
    if (inputType.empty() || (inputType != "video" &&
        inputType != "pic" && inputType != "rtsp")) {
        ACLLITE_LOG_ERROR("device %d input type is invalid", channelId);
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError DetectPreprocessThread::GetInputDataPath(std::string& inputDataPath, uint32_t channelId)
{
    std::string inputDataPathKey = "inputDataPath_" + to_string(channelId);
    std::map<std::string, std::string> config;
    if (!ReadConfig(config, configFile_)) {
        return ACLLITE_ERROR;
    }

    std::map<std::string, std::string>::const_iterator mIter = config.begin();
    for (; mIter != config.end(); ++mIter) {
        if (mIter->first == inputDataPathKey) {
            inputDataPath.assign(mIter->second.c_str());
            ACLLITE_LOG_INFO("device %d input data path is : %s",
                             channelId, inputDataPath.c_str());
        }
    }
    if (inputDataPath.empty()) {
        ACLLITE_LOG_ERROR("device %d input data path is invalid", channelId);
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError DetectPreprocessThread::Init()
{
    AclLiteError aclRet = GetInputDataPath(inputDataPath_, channelId_);
    if (aclRet != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("GetInputDataPath failed, error %d", aclRet);
        return ACLLITE_ERROR;
    }

    aclRet = GetInputDataType(inputType_, channelId_);
    if (aclRet != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("GetInputDataType failed, error %d", aclRet);
        return ACLLITE_ERROR;
    }

    if (inputType_ == "pic") {
        aclRet = OpenPicsDir();
    } else {
        aclRet = OpenVideoCapture();
    }
    if (aclRet != ACLLITE_OK) {
        return ACLLITE_ERROR;
    }

    aclRet = dvpp_.Init();
    if (aclRet) {
        ACLLITE_LOG_ERROR("Dvpp init failed, error %d", aclRet);
        return ACLLITE_ERROR;
    }

    // Get the relevant thread instance id
    selfThreadId_ = SelfInstanceId();
    inferThreadId_ = GetAclLiteThreadIdByName(kInferName + to_string(deviceId_));
    detectPostThreadId_ = GetAclLiteThreadIdByName(kDetectPostName + to_string(channelId_));
    classifyPreThreadId_ = GetAclLiteThreadIdByName(kClassifyPreName + to_string(channelId_));
    classifyPostThreadId_ = GetAclLiteThreadIdByName(kClassifyPostName + to_string(channelId_));
    rtspDisplayThreadId_ = GetAclLiteThreadIdByName(kRtspDisplayName + to_string(channelId_));
    if (display_) {
        presentAgentDisplayThreadId_ = GetAclLiteThreadIdByName(kPresentAgentDisplayName.c_str());
    }
    if ((selfThreadId_ == INVALID_INSTANCE_ID) ||
        (inferThreadId_ == INVALID_INSTANCE_ID) ||
        (detectPostThreadId_ == INVALID_INSTANCE_ID) ||
        (classifyPreThreadId_ == INVALID_INSTANCE_ID) ||
        (classifyPostThreadId_ == INVALID_INSTANCE_ID)) {
        ACLLITE_LOG_ERROR("Self instance id %d, infer instance id %d, detectPost instance id %d,"
                          " classifyPre instance id %d, classifyPost instance id %d",
                          selfThreadId_, inferThreadId_, detectPostThreadId_,
                          classifyPreThreadId_, classifyPostThreadId_);
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError DetectPreprocessThread::Process(int msgId, shared_ptr<void> msgData)
{
    AclLiteError ret = ACLLITE_OK;
    shared_ptr<CarDetectDataMsg> carDetectDataMsg = make_shared<CarDetectDataMsg>();
    switch (msgId) {
        case MSG_APP_START:
            ret = AppStart();
            break;
        case MSG_READ_FRAME:
            ret = MsgRead(carDetectDataMsg);
            ret = MsgProcess(carDetectDataMsg);
            ret = MsgSend(carDetectDataMsg);
            break;
        default:
            ACLLITE_LOG_ERROR("Detect Preprocess thread receive unknow msg %d", msgId);
            break;
    }

    return ret;
}

AclLiteError DetectPreprocessThread::AppStart()
{
    AclLiteError ret = SendMessage(selfThreadId_, MSG_READ_FRAME, nullptr);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Process app start message failed, error %d", ret);
    }

    return ret;
}

AclLiteError DetectPreprocessThread::ReadPic(shared_ptr<CarDetectDataMsg> &carDetectDataMsg)
{
    carDetectDataMsg->inferThreadId = inferThreadId_;
    carDetectDataMsg->detectPostThreadId = detectPostThreadId_;
    carDetectDataMsg->classifyPreThreadId = classifyPreThreadId_;
    carDetectDataMsg->classifyPostThreadId = classifyPostThreadId_;
    carDetectDataMsg->presentAgentDisplayThreadId = presentAgentDisplayThreadId_;
    carDetectDataMsg->rtspDisplayThreadId = rtspDisplayThreadId_;
    carDetectDataMsg->deviceId = deviceId_;
    carDetectDataMsg->channelId = channelId_;
    carDetectDataMsg->frameNum = frameCnt_;
    carDetectDataMsg->isLastFrame = 0;

    if (frameCnt_ == fileVec_.size()) {
        carDetectDataMsg->isLastFrame = 1;
        return ACLLITE_OK;
    }
    string picFile = fileVec_[frameCnt_];
    AclLiteError ret = ReadJpeg(carDetectDataMsg->imageFrame, picFile);
    carDetectDataMsg->frame = cv::imread(picFile);
    frameCnt_++;
    return ACLLITE_OK;
}

AclLiteError DetectPreprocessThread::ReadStream(shared_ptr<CarDetectDataMsg> &carDetectDataMsg)
{
    carDetectDataMsg->inferThreadId = inferThreadId_;
    carDetectDataMsg->detectPostThreadId = detectPostThreadId_;
    carDetectDataMsg->classifyPreThreadId = classifyPreThreadId_;
    carDetectDataMsg->classifyPostThreadId = classifyPostThreadId_;
    carDetectDataMsg->presentAgentDisplayThreadId = presentAgentDisplayThreadId_;
    carDetectDataMsg->rtspDisplayThreadId = rtspDisplayThreadId_;
    carDetectDataMsg->deviceId = deviceId_;
    carDetectDataMsg->channelId = channelId_;
    carDetectDataMsg->frameNum = frameCnt_;
    carDetectDataMsg->isLastFrame = 0;
    AclLiteError ret = cap_->Read(carDetectDataMsg->imageFrame);
    if (ret != ACLLITE_OK) {
        if (ret == ACLLITE_ERROR_DECODE_FINISH) {
        carDetectDataMsg->isLastFrame = 1;
        return ACLLITE_OK;
        } else {
        carDetectDataMsg->isLastFrame = 1;
        return ret;
        }
    }
    frameCnt_++;
    return ACLLITE_OK;
}

AclLiteError DetectPreprocessThread::ProcessPic(shared_ptr<CarDetectDataMsg> &carDetectDataMsg)
{
    if (carDetectDataMsg->isLastFrame == 1)
        return ACLLITE_OK;
    ImageData imageDevice, yuvImage;

    AclLiteError ret = CopyImageToDevice(imageDevice, carDetectDataMsg->imageFrame, runMode_, MEMORY_DVPP);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Copy image to device failed");
        return ACLLITE_ERROR;
    }

    ret = dvpp_.JpegD(carDetectDataMsg->imageFrame, imageDevice);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Pic decode failed");
        return ACLLITE_ERROR;
    }

    ret = dvpp_.ProportionPasteCenter(carDetectDataMsg->resizedFrame,
        carDetectDataMsg->imageFrame, modelWidth_, modelHeight_);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Pic decode failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError DetectPreprocessThread::ProcessStreamFrame(shared_ptr<CarDetectDataMsg> &carDetectDataMsg)
{
    if (carDetectDataMsg->isLastFrame == 1)
        return ACLLITE_OK;
    ImageData imageDevice;
    AclLiteError ret = CopyImageToDevice(imageDevice, carDetectDataMsg->imageFrame, runMode_, MEMORY_DVPP);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Copy image to device failed");
        return ACLLITE_ERROR;
    }

    ret = dvpp_.ProportionPasteCenter(carDetectDataMsg->resizedFrame, imageDevice, modelWidth_, modelHeight_);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("dvpp_cropandpaste image failed");
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

AclLiteError DetectPreprocessThread::MsgRead(shared_ptr<CarDetectDataMsg> &carDetectDataMsg)
{
    AclLiteError ret;
    if (inputType_ == "pic") {
        ret = ReadPic(carDetectDataMsg);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Read pic failed, error %d", ret);
            return ACLLITE_ERROR;
        }
    } else {
        ret = ReadStream(carDetectDataMsg);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Read frame failed, error %d", ret);
            return ACLLITE_ERROR;
        }
    }

    return ACLLITE_OK;
}

AclLiteError DetectPreprocessThread::MsgProcess(shared_ptr<CarDetectDataMsg> &carDetectDataMsg)
{
    AclLiteError ret;
    if (inputType_ == "pic") {
        ret = ProcessPic(carDetectDataMsg);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Process pic failed, error %d", ret);
            return ACLLITE_ERROR;
        }
    } else {
        ret = ProcessStreamFrame(carDetectDataMsg);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Process stream frame failed, error %d", ret);
            return ACLLITE_ERROR;
        }
    }

    return ACLLITE_OK;
}

AclLiteError DetectPreprocessThread::MsgSend(shared_ptr<CarDetectDataMsg> &carDetectDataMsg)
{
    AclLiteError ret;
    if (carDetectDataMsg->isLastFrame == 0){
        while (1) {
            ret = SendMessage(carDetectDataMsg->inferThreadId, MSG_DETECT_PREPROC_DATA, carDetectDataMsg);
            if (ret == ACLLITE_ERROR_ENQUEUE) {
                usleep(500);
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
            shared_ptr<CarDetectDataMsg> carDetectDataMsgEnd = make_shared<CarDetectDataMsg>();
            carDetectDataMsgEnd->inferThreadId = inferThreadId_;
            carDetectDataMsgEnd->detectPostThreadId = detectPostThreadId_;
            carDetectDataMsgEnd->classifyPreThreadId = classifyPreThreadId_;
            carDetectDataMsgEnd->classifyPostThreadId = classifyPostThreadId_;
            carDetectDataMsgEnd->presentAgentDisplayThreadId = presentAgentDisplayThreadId_;
            carDetectDataMsgEnd->rtspDisplayThreadId = rtspDisplayThreadId_;
            carDetectDataMsgEnd->deviceId = deviceId_;
            carDetectDataMsgEnd->channelId = channelId_;
            carDetectDataMsgEnd->frameNum = carDetectDataMsg->frameNum;
            carDetectDataMsgEnd->isLastFrame = carDetectDataMsg->isLastFrame;
            while (1) {
                ret = SendMessage(carDetectDataMsgEnd->inferThreadId, MSG_DETECT_PREPROC_DATA, carDetectDataMsgEnd);
                if (ret == ACLLITE_ERROR_ENQUEUE) {
                    usleep(500);
                    continue;
                } else if(ret == ACLLITE_OK) {
                    break;
                } else {
                    ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
                    return ret;
                }
            }
    }
    return ACLLITE_OK;
}
