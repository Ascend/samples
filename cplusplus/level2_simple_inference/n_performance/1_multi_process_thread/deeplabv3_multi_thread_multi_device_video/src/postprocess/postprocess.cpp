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
#include "acl/acl.h"
#include "object_detection.h"
#include "postprocess.h"
#include "AclLiteUtils.h"
#include "acllite/AclLiteApp.h"

using namespace std;



namespace {
    const vector<int> kColors_1 = {
        230, 150, 87, 34, 65, 92, 139, 210, 60, 189, 240, 21,
        40, 56, 77, 101, 168, 179, 199, 222};
    const vector<int> kColors_2 = {
        46, 235, 187, 234, 165, 222, 188, 0, 153, 43, 70, 221,
        184, 177, 217, 101, 32, 209, 199, 32};
    const vector<int> kColors_3 = {
        166, 50, 287, 134, 55, 192, 39, 210, 90, 109, 170, 161,
        40, 156, 177, 49, 48, 79, 89, 122};
}

PostprocessThread::PostprocessThread(uint32_t outputWidth, uint32_t outputHeight) :
outputWidth_(outputWidth), outputHeight_(outputHeight){
}

PostprocessThread::~PostprocessThread() {
}

AclLiteError PostprocessThread::Init() {
    return ACLLITE_OK;
}

AclLiteError PostprocessThread::Process(int msgId, shared_ptr<void> data) {
    struct timespec time5 = {0, 0};
    struct timespec time6 = {0, 0};
    AclLiteError ret = ACLLITE_OK;
    shared_ptr<PostOutputMsg> postOutputMsg = make_shared<PostOutputMsg>();
    switch(msgId) {
        case MSG_INFER_OUTPUT:
            clock_gettime(CLOCK_REALTIME, &time5);
            InferOutputProcess(static_pointer_cast<InferOutputMsg>(data), postOutputMsg);
            MsgSend(static_pointer_cast<InferOutputMsg>(data), postOutputMsg);
            clock_gettime(CLOCK_REALTIME, &time6);
            //cout << "postprocess time is: " << (time6.tv_sec - time5.tv_sec)*1000 + (time6.tv_nsec - time5.tv_nsec)/1000000 << "ms" << endl;
            break;
        default:
            ACLLITE_LOG_INFO("PostprocessThread thread ignore msg %d", msgId);
            break;
    }

    return ret;
}

AclLiteError PostprocessThread::MsgSend(shared_ptr<InferOutputMsg> inferMsg,
                                       shared_ptr<PostOutputMsg> &postOutputMsg) {
    int sendFlag = MSG_VIDEO_ENCODE;
    if(inferMsg->isLastFrame == 1){
        sendFlag = MSG_ENCODE_FINISH;
    }
    while(1)
    {
        AclLiteError ret = SendMessage(inferMsg->videoProcessThreadId, sendFlag, postOutputMsg);
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

    return ACLLITE_OK;
}

AclLiteError PostprocessThread::InferOutputProcess(shared_ptr<InferOutputMsg> inferMsg,
                                                shared_ptr<PostOutputMsg> &postOutputMsg) {
    postOutputMsg->isLastFrame = inferMsg->isLastFrame;
    postOutputMsg->channelId = inferMsg->channelId;
    postOutputMsg->frameNum = inferMsg->frameNum;
    if (postOutputMsg->isLastFrame) 
        return ACLLITE_OK;

    float* data = (float *)inferMsg->inferData[0].data.get();
    if(data == nullptr){
        ACLLITE_LOG_ERROR("inferoutput is null\n");
        return ACLLITE_ERROR;
    }
    uint32_t dataSize = inferMsg->inferData[0].size;
    uint32_t size = static_cast<uint32_t>(dataSize) / sizeof(float);

    cv::Mat resizeImage;
    cv::resize(inferMsg->frame, resizeImage, cv::Size(outputWidth_, outputHeight_));

    cv::Mat mat_b_up, mat_g_up, mat_r_up;

    //对每一张单通道图
    for(int i = 1; i <21; i++){
        //保存为单通道图
        for(int j=i * (size / 21); j < (i + 1) * (size / 21); j++){
            if(data[j] > 0.5){
                data[j] = 1;
            }
            else{
                data[j] = 0;
            }
        }
        cv::Mat mat_a(513, 513, CV_32FC1, const_cast<float*>((float*)data) + (size / 21)*i);
        int iVal255 = cv::countNonZero(mat_a);
        if (iVal255){
            cv::Mat mat_a_up(outputHeight_, outputWidth_, CV_32FC1);
            cv::resize(mat_a, mat_a_up, cv::Size(outputWidth_, outputHeight_));        //现在拿到一个原图掩码

            cv::multiply(mat_a_up, mat_a_up, mat_b_up, kColors_1[i % kColors_1.size()]);
            cv::multiply(mat_a_up, mat_a_up, mat_g_up, kColors_2[i % kColors_2.size()]);
            cv::multiply(mat_a_up, mat_a_up, mat_r_up, kColors_3[i % kColors_3.size()]);
            mat_b_up.convertTo(mat_b_up, CV_8U);
            mat_g_up.convertTo(mat_g_up, CV_8U);
            mat_r_up.convertTo(mat_r_up, CV_8U);
            cv::Mat newChannels[3] = { mat_b_up, mat_g_up, mat_r_up };
            cv::Mat resultImage;
            cv::merge(newChannels, 3, resultImage);
            cv::addWeighted(resizeImage,1,resultImage,1,0,resizeImage);
        }
    }
    postOutputMsg->resultImage = resizeImage;
    return ACLLITE_OK;
}

