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
#include "atlas_thread_mgr.h"

namespace {
    int kMainThreadId = 0;
}

typedef int (*AtlasMsgProcess)(uint32_t msgId, shared_ptr<void> msgData, void* userData);

class AtlasApp {
public:
    /**
    * @brief Constructor
    */
    AtlasApp();
	AtlasApp(const AtlasApp&) = delete;
	AtlasApp& operator=(const AtlasApp&) = delete;

    /**
    * @brief Destructor
    */
    ~AtlasApp();

    /**
    * @brief init reousce
    * @return result
    */   
	static AtlasApp& GetInstance() {
		static AtlasApp instance;
		return instance;
	}

    AtlasError Init();
    int CreateAtlasThread(AtlasThread* thInst, const std::string& instName,
                          aclrtContext context, aclrtRunMode runMode);
    int Start(vector<AtlasThreadParam>& threadParamTbl);
    void Wait();
    void Wait(AtlasMsgProcess msgProcess, void* param);
    int GetAtlasThreadIdByName(const std::string& threadName);
    AtlasError SendMessage(int dest, int msgId, shared_ptr<void> data);
    void WaitEnd() { waitEnd_ = true; }
    void Exit();

private:
    int CreateAtlasThreadMgr(AtlasThread* thInst, const std::string& instName,
                             aclrtContext context, aclrtRunMode runMode);
    bool CheckThreadAbnormal();
    bool CheckThreadNameUnique(const std::string& theadName);
    void ReleaseThreads();
    void DestroyResource();

private:
    bool isReleased_;
    bool waitEnd_;
    std::vector<AtlasThreadMgr*> threadList_;
};

AtlasApp& CreateAtlasAppInstance();
AtlasApp& GetAtlasAppInstance();
AtlasError SendMessage(int dest, int msgId, shared_ptr<void> data);
int GetAtlasThreadIdByName(const std::string& threadName);

