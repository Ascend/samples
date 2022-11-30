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
#include "classifyPostprocess.h"
#include "AclLiteApp.h"
using namespace std;
namespace {
const uint32_t kEachResultTensorNum = 9;
const string kCarColorClass[9] = {"black", "blue", "brown",
                                  "green", "pink", "red",
                                  "silver", "white", "yellow"};
const uint32_t kLineSolid = 2;
const uint32_t kLabelOffset = 11;
const double kFountScale = 0.5;
const cv::Scalar kFontColor(0, 0, 255);
const vector<cv::Scalar> kColors {
    cv::Scalar(237, 149, 100),
    cv::Scalar(0, 215, 255),
    cv::Scalar(50, 205, 50),
    cv::Scalar(139, 85, 26) };
}

ClassifyPostprocessThread::ClassifyPostprocessThread(const char*& configFile, int channelId)
    :configFile_(configFile), channelId_(channelId)
{
}

ClassifyPostprocessThread::~ClassifyPostprocessThread()
{
    configFile_ = nullptr;
}

AclLiteError ClassifyPostprocessThread::GetOutputFrameResolution(int& frameWidth, int& frameHeight, uint32_t channelId)
{
    std::string outputFrameWidthKey = "outputFrameWidth_" + to_string(channelId);
    std::string outputFrameHeightKey = "outputFrameHeight_" + to_string(channelId);

    std::map<std::string, std::string> config;
    if (!ReadConfig(config, configFile_)) {
        return ACLLITE_ERROR;
    }
    
    std::map<std::string, std::string>::const_iterator mIter = config.begin();
    for (; mIter != config.end(); ++mIter) {
        if (mIter->first == outputFrameWidthKey) {
            frameWidth = atoi(mIter->second.c_str());
            ACLLITE_LOG_INFO("video %d width %d",
                             channelId, frameWidth);
        } else if (mIter->first == outputFrameHeightKey) {
            frameHeight = atoi(mIter->second.c_str());
            ACLLITE_LOG_INFO("video %d height %d",
                             channelId, frameHeight);
        }
    }

    return ACLLITE_OK;
}

AclLiteError ClassifyPostprocessThread::GetOutputDataType(std::string& outputType, uint32_t channelId)
{
    std::string outputTypeKey = "outputType_" + to_string(channelId);
    std::map<std::string, std::string> config;
    if (!ReadConfig(config, configFile_)) {
        return ACLLITE_ERROR;
    }

    std::map<std::string, std::string>::const_iterator mIter = config.begin();
    for (; mIter != config.end(); ++mIter) {
        if (mIter->first == outputTypeKey) {
            outputType.assign(mIter->second.c_str());
            ACLLITE_LOG_INFO("device %d output type is : %s",
                             channelId, outputType.c_str());
        }
    }
    if (outputType.empty() || (outputType != "video" &&
        outputType != "pic" && outputType != "presentagent" &&
        outputType != "stdout" && outputType != "rtsp")) {
        ACLLITE_LOG_ERROR("device %d output type is invalid", channelId);
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError ClassifyPostprocessThread::SetOutputVideo()
{
    stringstream sstream;
    sstream.str("");
    sstream << "../out/output/out_test" << channelId_<<".mp4";
    int fps = 25;
    outputVideo_.open(sstream.str(), cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps, cv::Size(outputFrameWidth_, outputFrameHeight_));
    return ACLLITE_OK;
}

AclLiteError ClassifyPostprocessThread::Init()
{
    AclLiteError ret = GetOutputDataType(outputType_, channelId_);
    if (ret != ACLLITE_OK) {
        return ACLLITE_ERROR;
    }

    if (outputType_ == "video") {
        ret = GetOutputFrameResolution(outputFrameWidth_, outputFrameHeight_, channelId_);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Set output frame resolution failed, error %d", ret);
            return ACLLITE_ERROR;
        }
        ret = SetOutputVideo();
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("SetOutputVideo failed, error %d", ret);
            return ACLLITE_ERROR;
        }
    }

    return ret;
}

AclLiteError ClassifyPostprocessThread::Process(int msgId, shared_ptr<void> data)
{
    AclLiteError ret = ACLLITE_OK;
    switch (msgId) {
        case MSG_CLASSIFY_INFER_OUTPUT:
            InferOutputProcess(static_pointer_cast<CarDetectDataMsg>(data));
            break;
        case MSG_ENCODE_FINISH:
            SendMessage(g_MainThreadId, MSG_APP_EXIT, nullptr);
            break;
        default:
            ACLLITE_LOG_INFO("Classify Postprocess thread ignore msg %d", msgId);
            break;
    }

    return ret;
}

AclLiteError ClassifyPostprocessThread::DisplayMsgSend(shared_ptr<CarDetectDataMsg> carDetectDataMsg)
{
    AclLiteError ret;
    while (1) {
        if (outputType_ == "presentagent") {
            #ifdef USE_PRESENT
            ret = SendMessage(carDetectDataMsg->presentAgentDisplayThreadId, MSG_PRESENT_AGENT_DISPLAY, carDetectDataMsg);
            #endif
        }
        if (outputType_ == "rtsp") {
            ret = SendMessage(carDetectDataMsg->rtspDisplayThreadId, MSG_RTSP_DISPLAY, carDetectDataMsg);
        }
        if (ret == ACLLITE_ERROR_ENQUEUE) {
            usleep(500);
            continue;
        } else if (ret == ACLLITE_OK) {
            break;
        } else {
            ACLLITE_LOG_ERROR("Send present agent display message failed, error %d", ret);
            return ret;
        }
    }

    return ACLLITE_OK;
}

AclLiteError ClassifyPostprocessThread::PrintResult(shared_ptr<CarDetectDataMsg> &carDetectDataMsg)
{
    cout<<"[rtsp video "<<carDetectDataMsg->channelId<<"]: { ";
    for (int i = 0; i < carDetectDataMsg->carInfo.size(); ++i) {
        cout<<"[car"<<i<<",("<<carDetectDataMsg->carInfo[i].rectangle.lt.x
        <<","<<carDetectDataMsg->carInfo[i].rectangle.lt.y<<"),("
        <<carDetectDataMsg->carInfo[i].rectangle.rb.x
        <<","<<carDetectDataMsg->carInfo[i].rectangle.rb.y<<"),"
        <<carDetectDataMsg->carInfo[i].carColor_result<<"] ";
    }
    cout<<"}"<<endl;
    return ACLLITE_OK;
}

AclLiteError ClassifyPostprocessThread::DrawResultOnPic(shared_ptr<CarDetectDataMsg> &carDetectDataMsg)
{
    for (int i = 0; i < carDetectDataMsg->carInfo.size(); ++i) {
        cv::rectangle(carDetectDataMsg->frame, carDetectDataMsg->carInfo[i].rectangle.lt,
            carDetectDataMsg->carInfo[i].rectangle.rb,
            kColors[i % kColors.size()], kLineSolid);
        cv::putText(carDetectDataMsg->frame, carDetectDataMsg->carInfo[i].detect_result,
            cv::Point(carDetectDataMsg->carInfo[i].rectangle.lt.x - kLabelOffset,
            carDetectDataMsg->carInfo[i].rectangle.lt.y - kLabelOffset),
            cv::FONT_HERSHEY_COMPLEX, kFountScale, kFontColor);
        cv::putText(carDetectDataMsg->frame, carDetectDataMsg->carInfo[i].carColor_result,
            cv::Point(carDetectDataMsg->carInfo[i].rectangle.lt.x,
            carDetectDataMsg->carInfo[i].rectangle.lt.y + kLabelOffset),
            cv::FONT_HERSHEY_COMPLEX, kFountScale, kFontColor);
    }

    stringstream sstream;
    sstream.str("");
    sstream << "../out/output/device_" << carDetectDataMsg->channelId
        << "_out_pic_" << carDetectDataMsg->frameNum << ".jpg";
    cv::imwrite(sstream.str(), carDetectDataMsg->frame);
    return ACLLITE_OK;
}

AclLiteError ClassifyPostprocessThread::DrawResultOnVideo(shared_ptr<CarDetectDataMsg> &carDetectDataMsg)
{
    for (int i = 0; i < carDetectDataMsg->carInfo.size(); ++i) {
        cv::rectangle(carDetectDataMsg->frame, carDetectDataMsg->carInfo[i].rectangle.lt,
            carDetectDataMsg->carInfo[i].rectangle.rb,
            kColors[i % kColors.size()], kLineSolid);
        cv::putText(carDetectDataMsg->frame, carDetectDataMsg->carInfo[i].detect_result,
            cv::Point(carDetectDataMsg->carInfo[i].rectangle.lt.x - kLabelOffset,
            carDetectDataMsg->carInfo[i].rectangle.lt.y - kLabelOffset),
            cv::FONT_HERSHEY_COMPLEX, kFountScale, kFontColor);
        cv::putText(carDetectDataMsg->frame, carDetectDataMsg->carInfo[i].carColor_result,
            cv::Point(carDetectDataMsg->carInfo[i].rectangle.lt.x,
            carDetectDataMsg->carInfo[i].rectangle.lt.y + kLabelOffset),
            cv::FONT_HERSHEY_COMPLEX, kFountScale, kFontColor);
    }
    resize(carDetectDataMsg->frame, carDetectDataMsg->frame,
            cv::Size(outputFrameWidth_, outputFrameHeight_),
            0, 0, cv::INTER_LINEAR);
    outputVideo_ << carDetectDataMsg->frame;
    return ACLLITE_OK;
}

AclLiteError ClassifyPostprocessThread::SendImage(shared_ptr<CarDetectDataMsg> &carDetectDataMsg)
{
    for (int i = 0; i < carDetectDataMsg->carInfo.size(); ++i) {
        cv::rectangle(carDetectDataMsg->frame, carDetectDataMsg->carInfo[i].rectangle.lt,
            carDetectDataMsg->carInfo[i].rectangle.rb,
            kColors[i % kColors.size()], kLineSolid);
        cv::putText(carDetectDataMsg->frame, carDetectDataMsg->carInfo[i].detect_result,
            cv::Point(carDetectDataMsg->carInfo[i].rectangle.lt.x - kLabelOffset,
            carDetectDataMsg->carInfo[i].rectangle.lt.y - kLabelOffset),
            cv::FONT_HERSHEY_COMPLEX, kFountScale, kFontColor);
        cv::putText(carDetectDataMsg->frame, carDetectDataMsg->carInfo[i].carColor_result,
            cv::Point(carDetectDataMsg->carInfo[i].rectangle.lt.x,
            carDetectDataMsg->carInfo[i].rectangle.lt.y + kLabelOffset),
            cv::FONT_HERSHEY_COMPLEX, kFountScale, kFontColor);
    }

    AclLiteError ret = DisplayMsgSend(carDetectDataMsg);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Send display msg failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError ClassifyPostprocessThread::InferOutputProcess(shared_ptr<CarDetectDataMsg> carDetectDataMsg)
{
    if (carDetectDataMsg->isLastFrame == 1) {
        if (outputType_ == "video") {
            outputVideo_.release();
        }
        if (outputType_ != "presentagent" && outputType_ != "rtsp") {
            SendMessage(carDetectDataMsg->classifyPostThreadId, MSG_ENCODE_FINISH, nullptr);
            ACLLITE_LOG_INFO("it is lastframe in classifyPost without presentagent");
            return ACLLITE_OK;
        } else if (outputType_ == "pic") {
        stringstream sstream;
        sstream.str("");
        sstream << "../out/output/device_" << carDetectDataMsg->deviceId
            << "_out_pic_" << carDetectDataMsg->frameNum << ".jpg";
        cv::imwrite(sstream.str(), carDetectDataMsg->frame);
        return ACLLITE_OK;
        } else {
            AclLiteError ret = DisplayMsgSend(carDetectDataMsg);
            if (ret != ACLLITE_OK) {
                ACLLITE_LOG_ERROR("Send last msg in classifyPost failed, error %d", (int)ret);
                return ACLLITE_ERROR;
            }
            ACLLITE_LOG_INFO("it is lastframe in classifyPost with presentagent/rtsp");
            return ACLLITE_OK;
        }
    }

    if (carDetectDataMsg->flag == 1) {
        // there is no car detected
        if (outputType_ == "video") {
            outputVideo_ << carDetectDataMsg->frame;
            return ACLLITE_OK;
        } else if (outputType_ == "presentagent" || outputType_ == "rtsp") {
            AclLiteError ret = DisplayMsgSend(carDetectDataMsg);
            if (ret != ACLLITE_OK) {
                ACLLITE_LOG_ERROR("Send msg failed, error %d", (int)ret);
                return ACLLITE_ERROR;
            }
            return ACLLITE_OK;
        }
        return ACLLITE_OK;
    }

    void* data = (void *)carDetectDataMsg->classifyInferData[0].data.get();
    if (data == nullptr) {
        return ACLLITE_ERROR;
    }
    float* outData = NULL;
    outData = reinterpret_cast<float*>(data);

    for (int i = 0; i < carDetectDataMsg->carInfo.size(); i++) {
        int maxConfidentIndex = i * kEachResultTensorNum;
        for (int j = 0; j < kEachResultTensorNum; j++) {
            int index = i * kEachResultTensorNum + j;
            if (outData[index] > outData[maxConfidentIndex]) {
                maxConfidentIndex = index;
            }
        }
        int colorIndex = maxConfidentIndex - i * kEachResultTensorNum;
        carDetectDataMsg->carInfo[i].carColor_result = kCarColorClass[colorIndex];
    }

    AclLiteError ret;
    if (outputType_ == "video") {
        ret = DrawResultOnVideo(carDetectDataMsg);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Draw classify result on video failed, error %d", ret);
            return ACLLITE_ERROR;
        }
    } else if (outputType_ == "presentagent") {
        ret = SendImage(carDetectDataMsg);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Send image to presentAgent failed, error %d", ret);
            return ACLLITE_ERROR;
        }
    } else if (outputType_ == "rtsp") {
        ret = SendImage(carDetectDataMsg);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Send image to rtsp failed, error %d", ret);
            return ACLLITE_ERROR;
        }
    } else if (outputType_ == "stdout") {
        ret = PrintResult(carDetectDataMsg);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("stdout result on screen failed, error %d", ret);
            return ACLLITE_ERROR;
        }
    } else {
        ret = DrawResultOnPic(carDetectDataMsg);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Send image to presentAgent failed, error %d", ret);
            return ACLLITE_ERROR;
        }
    }

    return ACLLITE_OK;
}
