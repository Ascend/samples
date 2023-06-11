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
#include "acl/acl.h"
#include "label.h"
#include "dataOutput.h"
#include <sys/time.h>

using namespace std;

namespace {
const uint32_t kOutputWidth = 540;
const uint32_t kOutputHeigth = 270;
const uint32_t kSleepTime = 500;
uint32_t kWaitTime = 1000;
const uint32_t kOneSec = 1000000;
const uint32_t kOneMSec = 1000;
const uint32_t kCountFps = 100;
}

DataOutputThread::DataOutputThread(aclrtRunMode& runMode, string outputDataType, string outputPath, int postThreadNum)
    :runMode_(runMode), outputDataType_(outputDataType),
    outputPath_(outputPath), shutdown_(0), postNum_(postThreadNum)
{
}

DataOutputThread::~DataOutputThread() {
    if (outputDataType_ == "video") {
        outputVideo_.release();
    }
}

AclLiteError DataOutputThread::SetOutputVideo()
{
    stringstream sstream;
    sstream.str("");
    sstream << outputPath_;
    int fps = 25;
    outputVideo_.open(sstream.str(), cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
        fps, cv::Size(kOutputWidth, kOutputHeigth));
    return ACLLITE_OK;
}

AclLiteError DataOutputThread::Init()
{
    if (outputDataType_ == "video") {
        AclLiteError ret = SetOutputVideo();
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("SetOutputVideo failed, error %d", ret);
            return ACLLITE_ERROR;
        }
    }
    if (outputDataType_ == "imshow") {
        kWaitTime = 1;
    }
    return ACLLITE_OK;
}

AclLiteError DataOutputThread::Process(int msgId, shared_ptr<void> data)
{
    AclLiteError ret = ACLLITE_OK;
    switch (msgId) {
        case MSG_OUTPUT_FRAME:
            RecordQueue(static_pointer_cast<DetectDataMsg>(data));
            DataProcess();
            break;
        case MSG_ENCODE_FINISH:
            shutdown_++;
            if (shutdown_ == postNum_) {
                ShutDownProcess();
            }
            break;
        default:
            ACLLITE_LOG_INFO("Detect PostprocessThread thread ignore msg %d", msgId);
            break;
    }

    return ret;
}

AclLiteError DataOutputThread::ShutDownProcess()
{
    int flag = 0;
    for (int i = 0; i < postNum_; i++) {
        if (!postQueue_[i].empty()) {
            shared_ptr<DetectDataMsg> detectDataMsg = postQueue_[i].front();
            ProcessOutput(detectDataMsg);
            postQueue_[i].pop();
            flag++;
        }
    }
    if (outputDataType_ != "rtsp") {
        SendMessage(g_MainThreadId, MSG_APP_EXIT, nullptr);
    }
    return ACLLITE_OK;
}

AclLiteError DataOutputThread::RecordQueue(shared_ptr<DetectDataMsg> detectDataMsg)
{
    if (detectDataMsg->postId >= postNum_) {
        ACLLITE_LOG_ERROR("Support up to 4 post-processing of 1 inputdata.");
        return ACLLITE_ERROR;
    }
    postQueue_[detectDataMsg->postId].push(detectDataMsg);
    return ACLLITE_OK;
}

AclLiteError DataOutputThread::DataProcess()
{
    int flag = 0;
    for (int i = 0; i < postNum_; i++) {
        if (!postQueue_[i].empty())
            flag++;
    }
    if (flag == postNum_) {
        for (int i = 0; i < postNum_; i++) {
            shared_ptr<DetectDataMsg> detectDataMsg = postQueue_[i].front();
            ProcessOutput(detectDataMsg);
            postQueue_[i].pop();
        }
    }
    return ACLLITE_OK;
}

AclLiteError DataOutputThread::ProcessOutput(shared_ptr<DetectDataMsg> detectDataMsg)
{
    AclLiteError ret;
    if (outputDataType_ == "video") {
        ret = SaveResultVideo(detectDataMsg);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Draw classify result on video failed, error %d", ret);
            return ACLLITE_ERROR;
        }
    } else if (outputDataType_ == "pic") {
        ret = SaveResultPic(detectDataMsg);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Draw classify result on pic failed, error %d", ret);
            return ACLLITE_ERROR;
        }
    } else if (outputDataType_ == "stdout") {
        ret = PrintResult(detectDataMsg);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("stdout result on screen failed, error %d", ret);
            return ACLLITE_ERROR;
        }
    } else if (outputDataType_ == "imshow") {
        ret = SendCVImshow(detectDataMsg);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Draw classify result on pic failed, error %d", ret);
            return ACLLITE_ERROR;
        }
    } else if (outputDataType_ == "rtsp") {
        ret = SendImageToRtsp(detectDataMsg);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Send image to rtsp failed, error %d", ret);
            return ACLLITE_ERROR;
        }
    }

    return ACLLITE_OK;
}

AclLiteError DataOutputThread::SaveResultVideo(shared_ptr<DetectDataMsg> &detectDataMsg)
{
    for(int i = 0; i < detectDataMsg->frame.size(); i++) {
        cv::resize(detectDataMsg->frame[i], detectDataMsg->frame[i],
            cv::Size(kOutputWidth, kOutputHeigth),
            0, 0, cv::INTER_LINEAR);
        outputVideo_ << detectDataMsg->frame[i];
    }
    return ACLLITE_OK;
}

AclLiteError DataOutputThread::SaveResultPic(shared_ptr<DetectDataMsg> &detectDataMsg)
{
    for(int i = 0; i < detectDataMsg->frame.size(); i++) {
        stringstream sstream;
        sstream.str("");
        sstream << "../out/channel_" << detectDataMsg->channelId
            << "_out_pic_" << detectDataMsg->msgNum << i << ".jpg";
        cv::imwrite(sstream.str(), detectDataMsg->frame[i]);
    }
    return ACLLITE_OK;
}

AclLiteError DataOutputThread::PrintResult(shared_ptr<DetectDataMsg> &detectDataMsg)
{
    for(int i = 0; i < detectDataMsg->textPrint.size(); i++) {
        // get time now
        timeval tv;
        gettimeofday(&tv, 0);
        int64_t now = ((int64_t)tv.tv_sec * kOneSec + (int64_t)tv.tv_usec) / kOneMSec;
        if (lastDecodeTime_ == 0) {
            lastDecodeTime_ = now;
        }
        uint32_t lastIntervalTime = now - lastDecodeTime_;
        lastDecodeTime_ = now;
        detectDataMsg->textPrint[i] = detectDataMsg->textPrint[i] + "[" + to_string(lastIntervalTime) + "ms]";
        if (!(frameCnt_ % kCountFps)) {
            if (lastRecordTime_ == 0) {
                lastRecordTime_ = now;
            } else {
                uint32_t fps = kCountFps / ((now - lastRecordTime_) / kOneMSec);
                lastRecordTime_ = now;
                detectDataMsg->textPrint[i] = detectDataMsg->textPrint[i] + "[fps:" + to_string(fps) + "]";
            }
        }
        frameCnt_++;
        cout<<detectDataMsg->textPrint[i]<<endl;
    }
    return ACLLITE_OK;
}

AclLiteError DataOutputThread::SendCVImshow(shared_ptr<DetectDataMsg> &detectDataMsg)
{
    for(int i = 0; i < detectDataMsg->frame.size(); i++) {
        cv::resize(detectDataMsg->frame[i], detectDataMsg->frame[i], cv::Size(kOutputWidth, kOutputHeigth));
        cv::imshow("frame", detectDataMsg->frame[i]);
        cv::waitKey(kWaitTime);
    }
    return ACLLITE_OK;
}

AclLiteError DataOutputThread::DisplayMsgSend(shared_ptr<DetectDataMsg> detectDataMsg)
{
    AclLiteError ret;
    while (1) {
        if (outputDataType_ == "rtsp") {
            ret = SendMessage(detectDataMsg->rtspDisplayThreadId, MSG_RTSP_DISPLAY, detectDataMsg);
        }
        if (ret == ACLLITE_ERROR_ENQUEUE) {
            usleep(kSleepTime);
            continue;
        } else if (ret == ACLLITE_OK) {
            break;
        } else {
            ACLLITE_LOG_ERROR("Send rtsp display message failed, error %d", ret);
            return ret;
        }
    }

    return ACLLITE_OK;
}

AclLiteError DataOutputThread::SendImageToRtsp(shared_ptr<DetectDataMsg> &detectDataMsg)
{
    AclLiteError ret = DisplayMsgSend(detectDataMsg);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Send display msg failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}
