#pragma once
#include <iostream>
#include <memory>
#include <thread>
#include <unistd.h>
#include "ThreadSafeQueue.h"
#include "acl/acl.h"
#include "AclLiteError.h"

using namespace std;

#define INVALID_INSTANCE_ID (-1)

class AclLiteThread{
public:
    AclLiteThread();
    virtual ~AclLiteThread() {};
    virtual int Init() { return ACLLITE_OK; };
    virtual int Process(int msgId, shared_ptr<void> msgData) = 0;                         
    int SelfInstanceId() { return instanceId_; }
    string& SelfInstanceName() { return instanceName_; }
    aclrtContext GetContext() { return context_; }
    aclrtRunMode GetRunMode() { return runMode_; }
    AclLiteError BaseConfig(int instanceId, const string& threadName,
                          aclrtContext context, aclrtRunMode runMode);
private:   
    aclrtContext context_;
    aclrtRunMode runMode_;
    int instanceId_;
    string instanceName_;
    bool baseConfiged_;
    bool isExit_;
};

struct AclLiteThreadParam {
    AclLiteThread* threadInst = nullptr;
    string threadInstName = "";
    aclrtContext context = nullptr;
    aclrtRunMode runMode = ACL_HOST;
    int threadInstId = INVALID_INSTANCE_ID;
};