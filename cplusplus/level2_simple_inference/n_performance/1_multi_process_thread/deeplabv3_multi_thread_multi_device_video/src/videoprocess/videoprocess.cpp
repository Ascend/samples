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
#include <queue>
#include <cstdio>
#include "acl/acl.h"
#include "object_detection.h"
#include "videoprocess.h"
#include "AclLiteUtils.h"
#include "acllite/AclLiteApp.h"

using namespace std;
struct timespec time7 = {0, 0};
struct timespec time8 = {0, 0};


VideoprocessThread::VideoprocessThread(int videoHeight, int videoWidth,  int postNum) : 
videoWidth_(videoWidth), videoHeight_(videoHeight), postNum_(postNum), shutdown_(0){
}

VideoprocessThread::~VideoprocessThread() {
}

AclLiteError VideoprocessThread::Init() {
    outputVideoPath_ = "./test1.mp4";
    cout << "videoWidth_ and videoHeight_ is" << " " << videoWidth_ << " " << videoHeight_ << endl;
    outputVideo_.open(outputVideoPath_, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 25.0, cv::Size(videoWidth_,videoHeight_));
    return ACLLITE_OK;
}

AclLiteError VideoprocessThread::Process(int msgId, shared_ptr<void> data) {
    AclLiteError ret = ACLLITE_OK;
    shared_ptr<PostOutputMsg> postOutputMsg = static_pointer_cast<PostOutputMsg>(data);
    
    switch(msgId) {
        case MSG_VIDEO_ENCODE:
            clock_gettime(CLOCK_REALTIME, &time7);
            RecordQueue(postOutputMsg);
            DataProcess();
            clock_gettime(CLOCK_REALTIME, &time8);
            //cout << "videoprocess time is: " << (time8.tv_sec - time7.tv_sec)*1000 + (time8.tv_nsec - time7.tv_nsec)/1000000 << "ms" << endl;
            break;
        case MSG_ENCODE_FINISH:
            clock_gettime(CLOCK_REALTIME, &time7);
            shutdown_++;
            cout << "shutdown_ is " << shutdown_ << endl;
            if(shutdown_ == postNum_){
                ShutDownProcess();
                SendMessage(g_MainThreadId, MSG_APP_EXIT, nullptr);
            }
            clock_gettime(CLOCK_REALTIME, &time8);
            cout << "videoprocess shutdown time is: " << (time8.tv_sec - time7.tv_sec)*1000 + (time8.tv_nsec - time7.tv_nsec)/1000000 << "ms" << endl;
            break;
        default:
            ACLLITE_LOG_INFO("PostprocessThread thread ignore msg %d", msgId);
            break;
    }
    return ret;
}
AclLiteError VideoprocessThread::ShutDownProcess() {
    int flag = 0;
    shared_ptr<PostOutputMsg> outputData[postNum_];
    int noEmptyQueue[postNum_];
    for(int i = 0; i < postNum_; i++)
    {
        if(!postqueue_[i].empty()){
            noEmptyQueue[flag] = i;
            flag++;
        }
    }
    if(flag != 0)
    {
        for(int i = 0; i < flag; i++)
        {
            outputData[i] = postqueue_[noEmptyQueue[i]].front();
            postqueue_[i].pop();
        }
        quick_sort(outputData, 0, flag-1);
        for(int i = 0; i < flag; i++)
        {
	    cout << "Current frame num is: " << outputData[i]->frameNum << endl;
            outputVideo_ << outputData[i]->resultImage;
        }
    }
    outputVideo_.release();
    return ACLLITE_OK;
}
AclLiteError VideoprocessThread::RecordQueue(shared_ptr<PostOutputMsg> postOutputMsg) {
    if(postOutputMsg->channelId > 16)
    {
        ACLLITE_LOG_ERROR("Support up to 16 channels of post-processing.");
        return ACLLITE_ERROR;
    }
    postqueue_[postOutputMsg->channelId].push(postOutputMsg);
    return ACLLITE_OK;
}
AclLiteError VideoprocessThread::quick_sort(shared_ptr<PostOutputMsg> outputData[], int low, int high) {
    if(low > high)
        return ACLLITE_OK;
    int i = low;
    int j = high;
    shared_ptr<PostOutputMsg> key = outputData[low];
    while (i < j)
    {
        while(i < j && outputData[j]->frameNum >= key->frameNum)
            j--;
        outputData[i] = outputData[j];
        while(i < j && outputData[j]->frameNum <= key->frameNum)
            i++;
        outputData[j] = outputData[i];

    }
    outputData[i] = key;

    quick_sort(outputData, low, i-1);
    quick_sort(outputData, i+1, high);
    return ACLLITE_OK;
}
AclLiteError VideoprocessThread::DataProcess() {
    int flag = 0;
    shared_ptr<PostOutputMsg> outputData[postNum_];
    for(int i = 0; i < postNum_; i++)
    {
        if(!postqueue_[i].empty())
            flag++;
    }
    if(flag == postNum_)
    {
        for(int i = 0; i < postNum_; i++)
        {
            outputData[i] = postqueue_[i].front();
            postqueue_[i].pop();
        }
        quick_sort(outputData, 0, postNum_-1);
        for(int i = 0; i < postNum_; i++)
        {
	    cout << "Current frame num is: " << outputData[i]->frameNum << endl;
            outputVideo_ << outputData[i]->resultImage;
        }
    }
    return ACLLITE_OK;
}

