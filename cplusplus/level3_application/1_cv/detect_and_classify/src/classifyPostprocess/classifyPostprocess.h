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

* File sample_process.h
* Description: handle acl resource
*/
#pragma once
#include <iostream>
#include <mutex>
#include <unistd.h>
#include "presenter/agent/presenter_channel.h"
#include "presenter/agent/presenter_types.h"
#include "acl/acl.h"
#include "VideoCapture.h"
#include "AclLiteThread.h"
#include "AclLiteImageProc.h"
#include "CarParams.h"

using namespace std;
using namespace ascend::presenter;

class ClassifyPostprocessThread : public AclLiteThread {
public:
    ClassifyPostprocessThread(const char*& configFile, int deviceId);
    ~ClassifyPostprocessThread();

    AclLiteError Init();
    AclLiteError Process(int msgId, shared_ptr<void> msgData);

private:
    AclLiteError GetOutputFrameResolution(int& videoWidth, int& videoHeight, uint32_t deviceId);
    AclLiteError GetOutputDataType(std::string& outputType, uint32_t deviceId);
    AclLiteError SetOutputVideo();
    AclLiteError DrawResultOnPic(shared_ptr<CarDetectDataMsg> &carDetectDataMsg);
    AclLiteError DrawResultOnVideo(shared_ptr<CarDetectDataMsg> &carDetectDataMsg);
    AclLiteError SendImage(shared_ptr<CarDetectDataMsg> &carDetectDataMsg);
    AclLiteError DisplayMsgSend(shared_ptr<CarDetectDataMsg> carDetectDataMsg);
    AclLiteError InferOutputProcess(shared_ptr<CarDetectDataMsg> carDetectDataMsg);

private:
    const char* configFile_;
    string outputType_;
    int outputFrameWidth_;
    int outputFrameHeight_;
    int deviceId_;
    cv::VideoWriter outputVideo_;
};

