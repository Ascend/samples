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
#include "atlas_utils_common.h"
#include "atlas_app.h"

using namespace std;
extern bool g_isDevice;

namespace {
    const uint32_t kWaitInterval = 10000;
}

AtlasApp::AtlasApp() :deviceId_(0), context_(nullptr), stream_(nullptr)
{

}

AtlasApp::~AtlasApp()
{
    DestroyResource();
}

int AtlasApp::InitResource()
{
    // ACL init
    char aclConfigFile[32] = { 0 };
    aclError ret = aclInit(aclConfigFile);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("acl init failed");
        return STATUS_ERROR;
    }
    ASC_LOG_INFO("acl init success");

    // open device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("acl open device %d failed", deviceId_);
        return STATUS_ERROR;
    }
    ASC_LOG_INFO("open device %d success", deviceId_);

    // get run mode
    aclrtRunMode runMode;
    ret = aclrtGetRunMode(&runMode);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("acl get run mode failed");
        return STATUS_ERROR;
    }
    isDevice_ = (runMode == ACL_DEVICE);
    ASC_LOG_INFO("get run mode success");

    return STATUS_OK;
}

int AtlasApp::Init() {

    int ret = InitResource();
    if (ret != STATUS_OK) {
        ASC_LOG_ERROR("Atlas app init failed for acl resouce init error");
        return STATUS_ERROR;
    }

    // parse preapare
    // why nullptr?
    // pthreadMgr name : "main"
    AtlasThreadMgr* thMgr = new AtlasThreadMgr(nullptr, "main");
    threadList_.push_back(thMgr);
    return STATUS_OK;
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
        ASC_LOG_ERROR("search name is empty");
	    return -1;
	}

	for (uint32_t i = 0; i < threadList_.size(); i++) {
		if (threadList_[i]->GetThreadName() == threadName) {
			return i;
		}
	}
	
	return -1;
}

int AtlasApp::SendMessage(int dest, int msgId, std::shared_ptr<void> data) {
	if ((uint32_t)dest > threadList_.size()) {
		ASC_LOG_ERROR("Send message to %d failed for thread not exist", dest);
		return STATUS_ERROR;
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
        ASC_LOG_ERROR("Atlas app wait exit for message process function is nullptr");
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
            ASC_LOG_ERROR("Atlas app exit for message %d process error:%d", msg->msgId, ret);
            break;
        }
    }
}

void AtlasApp::Exit() {
    ReleaseThreads();
    DestroyResource();
}

void AtlasApp::ReleaseThreads() {
	for (uint32_t i = 0; i < threadList_.size(); i++) {
		threadList_[i]->SetStatus(THREAD_EXITING);
		int retry = 3; 
		while (retry >= 0) {
			if (THREAD_EXITED == threadList_[i]->GetStatus()) {
				delete threadList_[i];
				threadList_[i] = nullptr;
				ASC_LOG_INFO("Atlas thread %d released", i);
				break;
			}
			sleep(1);
		}
	}
}


void AtlasApp::DestroyResource()
{
	ReleaseThreads();

    aclError ret;

    ASC_LOG_INFO("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("reset device failed");
    }
    ASC_LOG_INFO("end to reset device is %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("finalize acl failed");
    }
    ASC_LOG_INFO("end to finalize acl");

}


