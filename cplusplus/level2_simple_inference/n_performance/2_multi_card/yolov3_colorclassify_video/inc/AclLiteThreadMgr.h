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

class AclLiteThreadMgr{
public:
    AclLiteThreadMgr(AclLiteThread* userThreadInstance, 
                   const std::string& threadName);
    ~AclLiteThreadMgr();
    // Thread function
    static void ThreadEntry(void* data);
    AclLiteThread* GetUserInstance() { return this->userInstance_; }
    const std::string& GetThreadName(){ return name_; }
    // Send AclLiteMessage data to the queue
    AclLiteError PushMsgToQueue(shared_ptr<AclLiteMessage>& pMessage);
    // Get AclLiteMessage data from the queue
    shared_ptr<AclLiteMessage> PopMsgFromQueue(){ return this->msgQueue_.Pop(); }
    void CreateThread();
    void SetStatus(AclLiteThreadStatus status) { status_ = status; }
    AclLiteThreadStatus GetStatus() { return status_; }
    AclLiteError WaitThreadInitEnd();
 
public:
    bool isExit_;
    AclLiteThreadStatus status_;
    AclLiteThread* userInstance_;
    std::string name_;
    ThreadSafeQueue<shared_ptr<AclLiteMessage>> msgQueue_;
};