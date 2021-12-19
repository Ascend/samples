/**
* @file sample_process.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#pragma once

#include "acl/acl.h"
#include "AclLiteThreadMgr.h"

namespace {
    int kMainThreadId = 0;
}

typedef int (*AclLiteMsgProcess)(uint32_t msgId, shared_ptr<void> msgData, void* userData);

class AclLiteApp {
public:
    /**
    * @brief Constructor
    */
    AclLiteApp();
	AclLiteApp(const AclLiteApp&) = delete;
	AclLiteApp& operator=(const AclLiteApp&) = delete;

    /**
    * @brief Destructor
    */
    ~AclLiteApp();

    /**
     * @brief Get the single instance of AclLiteApp
     * @return Instance of AclLiteApp
     */ 
	static AclLiteApp& GetInstance() {
		static AclLiteApp instance;
		return instance;
	}

    /**
     * @brief Create one app thread
     * @return Result of create thread
     */ 
    int CreateAclLiteThread(AclLiteThread* thInst, const std::string& instName,
                          aclrtContext context, aclrtRunMode runMode);
    int Start(vector<AclLiteThreadParam>& threadParamTbl);
    void Wait();
    void Wait(AclLiteMsgProcess msgProcess, void* param);
    int GetAclLiteThreadIdByName(const std::string& threadName);
    AclLiteError SendMessage(int dest, int msgId, shared_ptr<void> data);
    void WaitEnd() { waitEnd_ = true; }
    void Exit();

private:
    AclLiteError Init();
    int CreateAclLiteThreadMgr(AclLiteThread* thInst, const std::string& instName,
                             aclrtContext context, aclrtRunMode runMode);
    bool CheckThreadAbnormal();
    bool CheckThreadNameUnique(const std::string& theadName);
    void ReleaseThreads();
    void DestroyResource();

private:
    bool isReleased_;
    bool waitEnd_;
    std::vector<AclLiteThreadMgr*> threadList_;
};

AclLiteApp& CreateAclLiteAppInstance();
AclLiteApp& GetAclLiteAppInstance();
AclLiteError SendMessage(int dest, int msgId, shared_ptr<void> data);
int GetAclLiteThreadIdByName(const std::string& threadName);