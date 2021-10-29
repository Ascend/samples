/**
* @file sample_process.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include "acl/acl.h"
#include "atlas_app.h"

using namespace std;
extern bool g_isDevice;

namespace {
    const uint32_t kWaitInterval = 10000;
}

AtlasApp::AtlasApp(){

}

AtlasApp::~AtlasApp(){
    ReleaseThreads();
}

AtlasError AtlasApp::Init() {
    AtlasThreadMgr* thMgr = new AtlasThreadMgr(nullptr, "main");
    threadList_.push_back(thMgr);
    return ATLAS_ERROR_APP_INIT;
}

int AtlasApp::CreateAtlasThread(AtlasThread* userThreadInstance, const char* threadName)
{
    AtlasThreadMgr* thMgr = new AtlasThreadMgr(userThreadInstance, threadName);

	threadList_.push_back(thMgr);
    thMgr->CreateThread();

	return 	threadList_.size() - 1;
}

int AtlasApp::GetAtlasThreadIdByName(const string& threadName) {
    if (threadName.empty()) {
        ATLAS_LOG_ERROR("search name is empty");
	    return INVALID_THREAD_ID;
	}

	for (uint32_t i = 0; i < threadList_.size(); i++) {
		if (threadList_[i]->GetThreadName() == threadName) {
			return i;
		}
	}
	
	return INVALID_THREAD_ID;
}

AtlasError AtlasApp::SendMessage(int dest, int msgId, void* data) {
	if ((uint32_t)dest > threadList_.size()) {
		ATLAS_LOG_ERROR("Send message to %d failed for thread not exist", dest);
		return ATLAS_ERROR_DEST_INVALID;
	}
	
	shared_ptr<AtlasMessage> pMessage = make_shared<AtlasMessage>();
	pMessage->dest = dest;
	pMessage->msgId = msgId;
	pMessage->data = data;

	return threadList_[dest]->PushMsgToQueue(pMessage);
}

void AtlasApp::Wait() {
    while (true) {
        usleep(kWaitInterval);
    }
}

void AtlasApp::Wait(AtlasMsgProcess msgProcess, void* param) {
    AtlasThreadMgr* mainMgr = threadList_[0];

    if (mainMgr == nullptr) {
        ATLAS_LOG_ERROR("Atlas app wait exit for message process function is nullptr");
        return;
    }

    while (true) {
        shared_ptr<AtlasMessage> msg = mainMgr->PopMsgFromQueue();
        if (msg == nullptr) {
            usleep(kWaitInterval);
            continue;
        }
        int ret = msgProcess(msg->msgId, msg->data, param);
        if (ret) {
            ATLAS_LOG_ERROR("Atlas app exit for message %d process error:%d", msg->msgId, ret);
            break;
        }
    }
}

void AtlasApp::Exit() {
    ReleaseThreads();
}

void AtlasApp::ReleaseThreads() {
	for (uint32_t i = 0; i < threadList_.size(); i++) {
		threadList_[i]->SetStatus(THREAD_EXITING);
		int retry = 3; 
		while (retry >= 0) {
			if (THREAD_EXITED == threadList_[i]->GetStatus()) {
				delete threadList_[i];
				threadList_[i] = nullptr;
				ATLAS_LOG_INFO("Atlas thread %d released", i);
				break;
			}
			sleep(1);
		}
	}
}

AtlasApp& CreateAtlasAppInstance() {
	return AtlasApp::GetInstance();
}

AtlasApp& GetAtlasAppInstance() {
	return AtlasApp::GetInstance();
}

AtlasError SendMessage(int dest, int msgId, void* data) {
	AtlasApp& app = AtlasApp::GetInstance();
	return app.SendMessage(dest, msgId, data);
}
