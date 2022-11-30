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

#ifndef ANIMEGAN_MULTI_DEVICE_ONE_VIDEO_PREPROCESS_PREPROCESS_H
#define ANIMEGAN_MULTI_DEVICE_ONE_VIDEO_PREPROCESS_PREPROCESS_H

#pragma once
#include <iostream>
#include <mutex>
#include <unistd.h>
#include "acl/acl.h"
#include "acllite/AclLiteThread.h"
#include "object_detection.h"
#include "VideoCapture.h"
#include "acllite/AclLiteImageProc.h"

class PreprocessThread : public AclLiteThread {
public:
    PreprocessThread(string& fileName, uint32_t modelWidth,
                     uint32_t modelHeight, uint32_t postThreadNum,
                     uint32_t inferThreadNum, aclrtContext& context);
    ~PreprocessThread();

    AclLiteError Init();
    AclLiteError Process(int msgId, shared_ptr<void> msgData);
    
private:
    AclLiteError AppStart();
    AclLiteError OpenVideoCapture();
    AclLiteError MsgSend(shared_ptr<PreprocDataMsg> &preprocDataMsg);
    AclLiteError MsgProcess(shared_ptr<PreprocDataMsg> &preprocDataMsg);
    AclLiteError ReadFrame(shared_ptr<PreprocDataMsg> &preprocDataMsg);

private:
    string videoPath_;
    AclLiteVideoProc* cap_;
    aclrtStream stream_;
    aclrtRunMode runMode_;
    uint32_t modelWidth_;
    uint32_t modelHeight_;
    int selfThreadId_;
    int videoThreadId_;
    vector<int> nextThreadId_;
    vector<int> postprocThreadId_;
    uint32_t inferThreadNum_;
    uint32_t postThreadNum_;
    int frameCnt_;
    AclLiteImageProc dvpp_;

    int inferChannel_;
    int postChannel_;
    int indexCount_;
    aclrtContext context_;
};

#endif