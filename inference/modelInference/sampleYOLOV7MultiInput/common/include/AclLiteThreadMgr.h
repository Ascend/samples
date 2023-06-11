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

* File AclLiteThreadMgr.h
* Description: handle AclLiteThreadMgr operations
*/
#ifndef ACLLITE_THREADMGR_H
#define ACLLITE_THREADMGR_H
#pragma once
#include <iostream>
#include <memory>
#include <thread>
#include <unistd.h>
#include "AclLiteUtils.h"
#include "ThreadSafeQueue.h"
#include "AclLiteThread.h"

enum AclLiteThreadStatus {
    THREAD_READY = 0,
    THREAD_RUNNING = 1,
    THREAD_EXITING = 2,
    THREAD_EXITED = 3,
    THREAD_ERROR = 4,
};

class AclLiteThreadMgr {
public:
    AclLiteThreadMgr(AclLiteThread* userThreadInstance,
                     const std::string& threadName, const uint32_t msgQueueSize);
    ~AclLiteThreadMgr();
    // Thread function
    static void ThreadEntry(void* data);
    AclLiteThread* GetUserInstance()
    {
        return this->userInstance_;
    }
    const std::string& GetThreadName()
    {
        return name_;
    }
    // Send AclLiteMessage data to the queue
    AclLiteError PushMsgToQueue(std::shared_ptr<AclLiteMessage>& pMessage);
    // Get AclLiteMessage data from the queue
    std::shared_ptr<AclLiteMessage> PopMsgFromQueue()
    {
        return this->msgQueue_.Pop();
    }
    void CreateThread();
    void SetStatus(AclLiteThreadStatus status)
    {
        status_ = status;
    }
    AclLiteThreadStatus GetStatus()
    {
        return status_;
    }
    AclLiteError WaitThreadInitEnd();
 
public:
    bool isExit_;
    AclLiteThreadStatus status_;
    AclLiteThread* userInstance_;
    std::string name_;
    ThreadSafeQueue<std::shared_ptr<AclLiteMessage>> msgQueue_;
};
#endif