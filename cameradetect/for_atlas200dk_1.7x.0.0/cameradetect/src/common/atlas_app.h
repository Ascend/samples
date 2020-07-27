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
/**
* SampleProcess
*/
using namespace std;

typedef int (*AtlasMsgProcess)(uint32_t msgId, std::shared_ptr<void> msgData, void* userData);

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
	static AtlasApp& getInstance() {
		static AtlasApp instance;
		return instance;
	}

    int Init();
    void Wait();
    void Wait(AtlasMsgProcess msgProcess, void* param);

    int32_t GetDeviceId() { return deviceId_; }
    aclrtContext &GetContext() { return context_; }
    aclrtStream &GetStream() { return stream_; }
    bool IsRunModeDevice() { return isDevice_; }
    int CreateAtlasThread(AtlasThread* userThreadInstance, const char* threadName = nullptr);
    int GetAtlasThreadIdByName(const string& threadName);
    int SendMessage(int dest, int msgId, std::shared_ptr<void> data);
    void Exit();

private:
    int InitResource();
    void ReleaseThreads();
    void DestroyResource();

private:
    int32_t deviceId_;
    aclrtContext context_;
    aclrtStream stream_;
    
    bool isDevice_;
    std::vector<AtlasThreadMgr*> threadList_;
};

