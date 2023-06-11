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

* File AclLiteThread.h
* Description: handle AclLiteThread operations
*/
#ifndef ACLLITE_THREAD_H
#define ACLLITE_THREAD_H
#pragma once
#include <iostream>
#include <memory>
#include <thread>
#include <unistd.h>
#include "ThreadSafeQueue.h"
#include "acl/acl.h"
#include "AclLiteError.h"

#define INVALID_INSTANCE_ID (-1)
class AclLiteThread {
public:
    AclLiteThread();
    virtual ~AclLiteThread() {};
    virtual int Init()
    {
        return ACLLITE_OK;
    };
    virtual int Process(int msgId, std::shared_ptr<void> msgData) = 0;
    int SelfInstanceId()
    {
        return instanceId_;
    }
    std::string& SelfInstanceName()
    {
        return instanceName_;
    }
    aclrtContext GetContext()
    {
        return context_;
    }
    aclrtRunMode GetRunMode()
    {
        return runMode_;
    }
    AclLiteError BaseConfig(int instanceId, const std::string& threadName,
                            aclrtContext context, aclrtRunMode runMode);
private:
    aclrtContext context_;
    aclrtRunMode runMode_;
    int instanceId_;
    std::string instanceName_;
    bool baseConfiged_;
    bool isExit_;
};

struct AclLiteThreadParam {
    AclLiteThread* threadInst = nullptr;
    std::string threadInstName = "";
    aclrtContext context = nullptr;
    aclrtRunMode runMode = ACL_HOST;
    int threadInstId = INVALID_INSTANCE_ID;
    uint32_t queueSize = 256;
};
#endif