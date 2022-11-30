/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
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
*/

#ifndef ANIMEGAN_MULTI_DEVICE_ONE_VIDEO_VIDEOPROCESS_VIDEOPROCESS_H
#define ANIMEGAN_MULTI_DEVICE_ONE_VIDEO_VIDEOPROCESS_VIDEOPROCESS_H

#pragma once

#include <iostream>
#include <mutex>
#include <queue>
#include <unistd.h>
#include "acl/acl.h"
#include "AclLiteError.h"
#include "acllite/AclLiteImageProc.h"
#include "acllite/AclLiteThread.h"
#include "object_detection.h"

using namespace std;

class VideoprocessThread : public AclLiteThread {
public:
    VideoprocessThread(int videoHeight, int videoWidth, int postNum);
    ~VideoprocessThread();

    AclLiteError Init();
    AclLiteError Process(int msgId, shared_ptr<void> data);

private:
    void DestroyResource();
    AclLiteError ShutDownProcess();
    AclLiteError RecordQueue(shared_ptr<PostOutputMsg> postOutputMsg);
    AclLiteError quick_sort(shared_ptr<PostOutputMsg> outputData[], int low, int high);
    AclLiteError DataProcess();

private:
    int videoWidth_;
    int videoHeight_;
    int postNum_;
    int shutdown_;
    string outputVideoPath_;
    cv::VideoWriter outputVideo_;
    queue<shared_ptr<PostOutputMsg>> postQueue_[16];
};

#endif