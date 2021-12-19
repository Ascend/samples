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
#include "AclLiteThread.h"
#include "AclLiteVideoProc.h"
#include "AclLiteImageProc.h"

class Preprocess : public AclLiteThread {
public:
    Preprocess(string& streamName, uint32_t modelWidth, 
               uint32_t modelHeight, uint32_t channelId);
    ~Preprocess();

    AclLiteError Init();
    AclLiteError Process(int msgId, shared_ptr<void> msgData);
    
private:
    AclLiteError AppStartMsgProcess();
    AclLiteError ReadFrameMsgProcess();
    AclLiteError OpenVideoCapture();
    AclLiteError GetThreadInstanceId();
    void ProcessImage(ImageData image);

private:
    string streamName_;
    AclLiteVideoProc* cap_;
    aclrtStream stream_;
    AclLiteImageProc dvpp_;

    uint32_t modelWidth_;
    uint32_t modelHeight_;
    uint32_t postThreadIndex_;
    uint32_t channelId_;
    int selfThreadId_;
    int nextThreadId_;
    int postprocThreadId_;
    int frameCnt_;    
};

