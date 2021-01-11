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

typedef int (*AtlasMsgProcess)(uint32_t msgId, void* msgData, void* userData);

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
    void Wait();
    void Wait(AtlasMsgProcess msgProcess, void* param);

    int CreateAtlasThread(AtlasThread* userThreadInstance, const char* threadName = nullptr);
    int GetAtlasThreadIdByName(const string& threadName);
    AtlasError SendMessage(int dest, int msgId, void* data);
    void Exit();

private:
    void ReleaseThreads();
    void DestroyResource();

private:
    std::vector<AtlasThreadMgr*> threadList_;
};

