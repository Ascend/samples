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
#include "classifyPostprocess.h"
#include "AclLiteApp.h"

using namespace std;

struct timespec time9 = {0, 0};
struct timespec time10 = {0, 0};

namespace {
const uint32_t kEachResultTensorNum = 9;
const string kCarColorClass[9] = { "black", "blue", "brown", "green", "pink", "red",
    "silver", "white", "yellow"};
const uint32_t kLineSolid = 2;
const uint32_t kLabelOffset = 11;
const double kFountScale = 0.5;
const cv::Scalar kFontColor(0, 0, 255);
const vector<cv::Scalar> kColors{
  cv::Scalar(237, 149, 100), cv::Scalar(0, 215, 255), cv::Scalar(50, 205, 50),
  cv::Scalar(139, 85, 26) };
}

ClassifyPostprocessThread::ClassifyPostprocessThread(const char*& configFile, int channelId): 
configFile_(configFile),
channelId_(channelId) {
}

ClassifyPostprocessThread::~ClassifyPostprocessThread() {
}

AclLiteError ClassifyPostprocessThread::GetBaseConfig(int& videoWidth, int& videoHeight, uint32_t channelId) {
    std::string videoWidthKey = "videoWidth_" + to_string(channelId);
    std::string videoHeightKey = "videoHeight_" + to_string(channelId);

    std::map<std::string, std::string> config;
    if(!ReadConfig(config, configFile_)) {
        return ACLLITE_ERROR;
    }
    
    std::map<std::string, std::string>::const_iterator mIter = config.begin();
    for (; mIter != config.end(); ++mIter) {
        if (mIter->first == videoWidthKey) {
            videoWidth = atoi(mIter->second.c_str());
            ACLLITE_LOG_INFO("video %d width %d", 
                            channelId, videoWidth);
        } else if (mIter->first == videoHeightKey) {
            videoHeight = atoi(mIter->second.c_str());
            ACLLITE_LOG_INFO("video %d height %d", 
                            channelId, videoHeight);
        }
    }

    return ACLLITE_OK;
}

AclLiteError ClassifyPostprocessThread::Init() {
    AclLiteError aclRet = GetBaseConfig(videoWidth_, videoHeight_, channelId_);
    if (aclRet) {
        ACLLITE_LOG_ERROR("GetBaseConfig failed, error %d", aclRet);
        return ACLLITE_ERROR;
    }

    stringstream sstream;
    sstream.str("");
    sstream << "../out/out_test" << channelId_<<".mp4";
    outputVideo_.open(sstream.str(), cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 25.0, cv::Size(videoWidth_,videoHeight_));
    return ACLLITE_OK;
}

AclLiteError ClassifyPostprocessThread::Process(int msgId, 
                             shared_ptr<void> data) {
    AclLiteError ret = ACLLITE_OK;
    switch(msgId) {
        case MSG_CLASSIFY_INFER_OUTPUT:
            clock_gettime(CLOCK_REALTIME, &time9);
            InferOutputProcess(static_pointer_cast<CarDetectDataMsg>(data));
            clock_gettime(CLOCK_REALTIME, &time10);
            //cout << "preprocess time is: " << (time10.tv_sec - time9.tv_sec)*1000 + (time10.tv_nsec - time9.tv_nsec)/1000000 << "ms" << endl;
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

AclLiteError ClassifyPostprocessThread::DrawResult(shared_ptr<CarDetectDataMsg> &carDetectDataMsg) {
    
    for (int i = 0; i < carDetectDataMsg->carInfo.size(); ++i) {
        cv::rectangle(carDetectDataMsg->frame, carDetectDataMsg->carInfo[i].rectangle.lt, carDetectDataMsg->carInfo[i].rectangle.rb,
                      kColors[i % kColors.size()], kLineSolid);
        cv::putText(carDetectDataMsg->frame, carDetectDataMsg->carInfo[i].detect_result, cv::Point(carDetectDataMsg->carInfo[i].rectangle.lt.x - kLabelOffset, 
                     carDetectDataMsg->carInfo[i].rectangle.lt.y - kLabelOffset),
                     cv::FONT_HERSHEY_COMPLEX, kFountScale, kFontColor);
        cv::putText(carDetectDataMsg->frame, carDetectDataMsg->carInfo[i].carColor_result, cv::Point(carDetectDataMsg->carInfo[i].rectangle.lt.x, 
                     carDetectDataMsg->carInfo[i].rectangle.lt.y + kLabelOffset),
                     cv::FONT_HERSHEY_COMPLEX, kFountScale, kFontColor);
        // ACLLITE_LOG_INFO("%d %d %d %d  %s", carDetectDataMsg->carInfo[i].rectangle.lt.x, carDetectDataMsg->carInfo[i].rectangle.lt.y,
        //                carDetectDataMsg->carInfo[i].rectangle.rb.x, carDetectDataMsg->carInfo[i].rectangle.rb.y, 
        //                carDetectDataMsg->carInfo[i].detect_result.c_str());
    }

    outputVideo_ << carDetectDataMsg->frame;
    return ACLLITE_OK;
}

AclLiteError ClassifyPostprocessThread::InferOutputProcess(shared_ptr<CarDetectDataMsg> carDetectDataMsg) {
    if (carDetectDataMsg->isLastFrame == 1) {
        outputVideo_.release();
        SendMessage(carDetectDataMsg->classifyPostThreadId, MSG_ENCODE_FINISH, nullptr);
        return ACLLITE_OK;
    }

    if (carDetectDataMsg->flag == 1) {
        return ACLLITE_OK;
    }

    void* data = (void *)carDetectDataMsg->classifyInferData[0].data.get();
    if (data == nullptr) {
        return ACLLITE_ERROR;
    }
    float* outData = NULL;
    outData = reinterpret_cast<float*>(data);

    for(int i = 0; i < carDetectDataMsg->carInfo.size(); i++){
        int maxConfidentIndex = i * kEachResultTensorNum;
        for(int j = 0; j < kEachResultTensorNum; j++){
            int index = i * kEachResultTensorNum + j;
            if(outData[index] > outData[maxConfidentIndex]){
                maxConfidentIndex = index;
            }
        }
        int colorIndex = maxConfidentIndex - i * kEachResultTensorNum;
        carDetectDataMsg->carInfo[i].carColor_result = kCarColorClass[colorIndex];
    }
    DrawResult(carDetectDataMsg);

    return ACLLITE_OK;
}
