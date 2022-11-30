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

* File sample_process.h
* Description: handle acl resource
*/
#ifndef CLASSIFYPOSTPROCESSTHREAD_H
#define CLASSIFYPOSTPROCESSTHREAD_H
#pragma once
#include <iostream>
#include <mutex>
#include <unistd.h>

#ifdef USE_PRESENT
#include "presenter/agent/presenter_channel.h"
#include "presenter/agent/presenter_types.h"
#endif

#include "acl/acl.h"
#include "VideoCapture.h"
#include "AclLiteThread.h"
#include "AclLiteImageProc.h"
#include "CarParams.h"

class ClassifyPostprocessThread : public AclLiteThread {
public:
    ClassifyPostprocessThread(const char*& configFile, int channelId);
    ~ClassifyPostprocessThread();

    AclLiteError Init();
    AclLiteError Process(int msgId, std::shared_ptr<void> msgData);

private:
    AclLiteError GetOutputFrameResolution(int& videoWidth, int& videoHeight, uint32_t channelId);
    AclLiteError GetOutputDataType(std::string& outputType, uint32_t channelId);
    AclLiteError SetOutputVideo();
    AclLiteError PrintResult(std::shared_ptr<CarDetectDataMsg> &carDetectDataMsg);
    AclLiteError DrawResultOnPic(std::shared_ptr<CarDetectDataMsg> &carDetectDataMsg);
    AclLiteError DrawResultOnVideo(std::shared_ptr<CarDetectDataMsg> &carDetectDataMsg);
    AclLiteError SendImage(std::shared_ptr<CarDetectDataMsg> &carDetectDataMsg);
    AclLiteError DisplayMsgSend(std::shared_ptr<CarDetectDataMsg> carDetectDataMsg);
    AclLiteError InferOutputProcess(std::shared_ptr<CarDetectDataMsg> carDetectDataMsg);

private:
    const char* configFile_;
    std::string outputType_;
    int outputFrameWidth_;
    int outputFrameHeight_;
    int channelId_;
    cv::VideoWriter outputVideo_;
};

#endif