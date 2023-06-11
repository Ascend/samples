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
#ifndef DATAINPUTTHREAD_H
#define DATAINPUTTHREAD_H
#pragma once
#include <iostream>
#include <mutex>
#include <unistd.h>
#include "acl/acl.h"
#include "VideoCapture.h"
#include "AclLiteThread.h"
#include "AclLiteImageProc.h"
#include "AclLiteApp.h"
#include "Params.h"

class DataInputThread : public AclLiteThread {
public:
    DataInputThread(int32_t deviceId, int32_t channelId, aclrtRunMode& runMode,
        std::string inputDataType, std::string inputDataPath,
        std::string inferName, int postThreadNum, uint32_t batch, int framesPerSecond);

    ~DataInputThread();
    AclLiteError Init();
    AclLiteError Process(int msgId, std::shared_ptr<void> msgData);
    
private:
    AclLiteError AppStart();
    AclLiteError MsgRead(std::shared_ptr<DetectDataMsg> &detectDataMsg);
    AclLiteError MsgSend(std::shared_ptr<DetectDataMsg> &detectDataMsg);
    AclLiteError OpenPicsDir();
    AclLiteError OpenVideoCapture();
    AclLiteError ReadPic(std::shared_ptr<DetectDataMsg> &detectDataMsg);
    AclLiteError ReadStream(std::shared_ptr<DetectDataMsg> &detectDataMsg);
    AclLiteError GetOneFrame(std::shared_ptr<DetectDataMsg> &detectDataMsg);

private:
    uint32_t deviceId_;
    uint32_t channelId_;
    int frameCnt_;
    int msgNum_;
    uint32_t batch_;
    
    std::string inputDataType_;
    std::string inputDataPath_;
    std::string inferName_;
    int postThreadNum_;
    int postproId_;

    aclrtRunMode runMode_;
    AclLiteVideoProc* cap_;
    AclLiteImageProc dvpp_;

    int selfThreadId_;
    int preThreadId_;
    int inferThreadId_;
    std::vector<int> postThreadId_;
    int dataOutputThreadId_;
    int rtspDisplayThreadId_;
    std::vector<std::string> fileVec_;

    int64_t lastDecodeTime_;
    int64_t realWaitTime_;
    int64_t waitTime_;
    int framesPerSecond_;
};

#endif