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

class ClassifyPreprocessThread : public AclLiteThread {
public:
    ClassifyPreprocessThread(aclrtRunMode& runMode);
    ~ClassifyPreprocessThread();

    AclLiteError Init();
    AclLiteError Process(int msgId, shared_ptr<void> msgData);
    
private:
    AclLiteError Crop(vector<CarInfo> &carImgs, ImageData &orgImg);
    AclLiteError Resize(vector<CarInfo> &carImgs);
    AclLiteError MsgProcess(shared_ptr<CarDetectDataMsg> carDetectDataMsg);
    AclLiteError MsgSend(shared_ptr<CarDetectDataMsg> carDetectDataMsg);
private:
    aclrtRunMode runMode_;
    uint32_t modelWidth_;
    uint32_t modelHeight_;
    AclLiteImageProc dvpp_;
};

