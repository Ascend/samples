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
#include "acl/acl.h"
#include "VideoCapture.h"
#include "AclLiteThread.h"
#include "AclLiteImageProc.h"
#include "params.h"

class DetectPreprocessThread : public AclLiteThread {
public:
    DetectPreprocessThread(const char*& configFile, int32_t i, aclrtRunMode& runMode);
    ~DetectPreprocessThread();

    AclLiteError Init();
    AclLiteError Process(int msgId, shared_ptr<void> msgData);
    
private:
    AclLiteError AppStart();
    AclLiteError OpenVideoCapture();
    AclLiteError MsgSend(shared_ptr<CarDetectDataMsg> &carDetectDataMsg);
    AclLiteError MsgProcess(shared_ptr<CarDetectDataMsg> &carDetectDataMsg);
    AclLiteError ReadFrame(shared_ptr<CarDetectDataMsg> &carDetectDataMsg);
    AclLiteError GetBaseConfig(std::string& videoPath, uint32_t channelId);

private:
    const char* configFile_;
    string videoPath_;
    AclLiteVideoProc* cap_;
    aclrtRunMode runMode_;
    uint32_t channelId_;
    uint32_t modelWidth_;
    uint32_t modelHeight_;
    int frameCnt_;
    int selfThreadId_;
    int inferThreadId_;
    int detectPostThreadId_;
    int classifyPreThreadId_;
    int classifyPostThreadId_;
    AclLiteImageProc dvpp_;
};

