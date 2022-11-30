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
#ifndef DETECTPREPROCESSTHREAD_H
#define DETECTPREPROCESSTHREAD_H
#pragma once
#include <iostream>
#include <mutex>
#include <unistd.h>
#include "acl/acl.h"
#include "VideoCapture.h"
#include "AclLiteThread.h"
#include "AclLiteImageProc.h"
#include "CarParams.h"

class DetectPreprocessThread : public AclLiteThread {
public:
    DetectPreprocessThread(const char*& configFile, int32_t deviceId,
        int32_t channelId, aclrtRunMode& runMode, bool display = false);
    ~DetectPreprocessThread();

    AclLiteError Init();
    AclLiteError Process(int msgId, std::shared_ptr<void> msgData);
    
private:
    AclLiteError AppStart();
    AclLiteError OpenPicsDir();
    AclLiteError OpenVideoCapture();
    AclLiteError MsgRead(std::shared_ptr<CarDetectDataMsg> &carDetectDataMsg);
    AclLiteError MsgSend(std::shared_ptr<CarDetectDataMsg> &carDetectDataMsg);
    AclLiteError MsgProcess(std::shared_ptr<CarDetectDataMsg> &carDetectDataMsg);
    AclLiteError ReadPic(std::shared_ptr<CarDetectDataMsg> &carDetectDataMsg);
    AclLiteError ReadStream(std::shared_ptr<CarDetectDataMsg> &carDetectDataMsg);
    AclLiteError ProcessPic(std::shared_ptr<CarDetectDataMsg> &carDetectDataMsg);
    AclLiteError ProcessStreamFrame(std::shared_ptr<CarDetectDataMsg> &carDetectDataMsg);
    AclLiteError GetPicsDirConfig(std::string& picsDirPath, uint32_t channelId);
    AclLiteError GetVideoConfig(std::string& videoPath, uint32_t channelId);
    AclLiteError GetRtspConfig(std::string& rtspPath, uint32_t channelId);
    AclLiteError GetInputDataType(std::string& inputType, uint32_t channelId);
    AclLiteError GetInputDataPath(std::string& inputDataPath, uint32_t channelId);

private:
    bool display_;
    const char* configFile_;
    std::string inputType_;
    std::string inputDataPath_;
    AclLiteVideoProc* cap_;
    aclrtRunMode runMode_;
    uint32_t deviceId_;
    uint32_t channelId_;
    uint32_t modelWidth_;
    uint32_t modelHeight_;
    int frameCnt_;
    bool isSkip_;
    int selfThreadId_;
    int inferThreadId_;
    int detectPostThreadId_;
    int classifyPreThreadId_;
    int classifyPostThreadId_;
    int presentAgentDisplayThreadId_;
    int rtspDisplayThreadId_;
    AclLiteImageProc dvpp_;
    std::vector<std::string> fileVec_;
};

#endif
