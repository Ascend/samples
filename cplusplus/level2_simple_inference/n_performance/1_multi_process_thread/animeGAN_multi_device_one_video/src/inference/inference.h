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
#include "AclLiteModel.h"
#include "acllite/AclLiteImageProc.h"
#include "acllite/AclLiteThread.h"
#include "object_detection.h"

using namespace std;

/**
* ClassifyProcess
*/
class InferenceThread : public AclLiteThread {
public:
    InferenceThread(const string& modelPath,
                    uint32_t modelWidth, uint32_t modelHeight, 
                    aclrtContext& contex);
    ~InferenceThread();

    AclLiteError Init();
    AclLiteError Process(int msgId, shared_ptr<void> data);
private:
    AclLiteError CreateImageInfoBuffer();
    AclLiteError MsgSend(shared_ptr<PreprocDataMsg> preprocDataMsg,
                            shared_ptr<InferOutputMsg> &inferOutputMsg);
    AclLiteError MsgSendEnd(shared_ptr<PreprocDataMsg> preprocDataMsg);
    AclLiteError ModelExecute(shared_ptr<PreprocDataMsg> preprocDataMsg,
                                shared_ptr<InferOutputMsg> &inferOutputMsg);

    void DestroyResource();
private:   
    AclLiteModel model_;
    uint32_t modelWidth_;
    uint32_t modelHeight_;
    aclrtRunMode runMode_;
    aclrtStream stream_;
    aclrtContext context_;
};

