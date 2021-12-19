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
#include "acllite/AclLiteThread.h"
#include "object_detection.h"
#include "acllite/AclLiteImageProc.h"

class PreprocessThread : public AclLiteThread {
public:
    PreprocessThread(string& fileName, uint32_t modelWidth, 
               uint32_t modelHeight, uint32_t channelId);
    ~PreprocessThread();

    AclLiteError Init();
    AclLiteError Process(int msgId, shared_ptr<void> msgData);
    
private:
    AclLiteError AppStart();
    AclLiteError MsgSend(shared_ptr<PreprocDataMsg> &preprocDataMsg);
    AclLiteError MsgProcess(ImageData& imageFrame, shared_ptr<PreprocDataMsg> &preprocDataMsg);
    AclLiteError ReadFrame(shared_ptr<PreprocDataMsg> &preprocDataMsg);

private:
    string fileName_;
    aclrtStream stream_;
    aclrtRunMode runMode_;
    uint32_t modelWidth_;
    uint32_t modelHeight_;
    uint32_t postThreadIndex_;
    uint32_t channelId_;
    int selfThreadId_;
    int nextThreadId_;
    vector<int> postprocThreadId_;
    int frameCnt_;
    AclLiteImageProc dvpp_;
};

