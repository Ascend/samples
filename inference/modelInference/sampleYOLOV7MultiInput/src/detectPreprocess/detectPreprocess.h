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
#include "Params.h"

class DetectPreprocessThread : public AclLiteThread {
public:
    DetectPreprocessThread(uint32_t modelWidth, uint32_t modelHeight,
    uint32_t batch);
    ~DetectPreprocessThread();
    AclLiteError Init();
    AclLiteError Process(int msgId, std::shared_ptr<void> data);
    
private:
    AclLiteError MsgProcess(std::shared_ptr<DetectDataMsg> detectDataMsg);
    AclLiteError MsgSend(std::shared_ptr<DetectDataMsg> detectDataMsg);

private:
    uint32_t modelWidth_;
    uint32_t modelHeight_;
    AclLiteImageProc dvpp_;
    bool isReleased;
    uint32_t batch_;
};

#endif
