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
#include "atlas_thread_mgr.h"

using namespace std;

namespace {
const uint32_t kWaitInterval = 10000;
const uint32_t kThreadExitRetry = 3;
}

AtlasApp::AtlasApp():isReleased_(false), waitEnd_(false){
    Init();
}

AtlasApp::~AtlasApp(){
    ReleaseThreads();
}

AtlasError AtlasApp::Init() {
    AtlasThreadMgr* thMgr = new AtlasThreadMgr(nullptr, "main");
    threadList_.push_back(thMgr);
    thMgr->SetStatus(THREAD_RUNNING);
    return ATLAS_OK;
}

int AtlasApp::CreateAtlasThread(AtlasThread* thInst, const string& instName,
                                aclrtContext context, aclrtRunMode runMode)
{
    int instId = CreateAtlasThreadMgr(thInst, instName, context, runMode);
    if (instId == INVALID_INSTANCE_ID) {
        ATLAS_LOG_ERROR("Add thread instance %s failed", instName.c_str());
        return INVALID_INSTANCE_ID;
    }

    threadList_[instId]->CreateThread();	
    AtlasError ret = threadList_[instId]->WaitThreadInitEnd();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Create thread failed, error %d", ret);
        return INVALID_INSTANCE_ID;
    }

    return 	instId;
}

int AtlasApp::CreateAtlasThreadMgr(AtlasThread* thInst, const string& instName,
                                aclrtContext context, aclrtRunMode runMode){
    if (!CheckThreadNameUnique(instName)) {
        ATLAS_LOG_ERROR("The thread instance name is not unique");
        return INVALID_INSTANCE_ID;
    }

    int instId = threadList_.size();	
    AtlasError ret = thInst->BaseConfig(instId, instName, context, runMode);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Create thread instance failed for error %d", ret);
        return INVALID_INSTANCE_ID;
    }
        
    AtlasThreadMgr* thMgr = new AtlasThreadMgr(thInst, instName);	
    threadList_.push_back(thMgr);

    return instId;
}

bool AtlasApp::CheckThreadNameUnique(const string& threadName) {
    if (threadName.size() == 0) return true;

    for (size_t i = 0; i < threadList_.size(); i++) {
        if (threadName == threadList_[i]->GetThreadName()) {
            return false;
        }
    }

    return true;
}

int AtlasApp::Start(vector<AtlasThreadParam>& threadParamTbl) {
    for (size_t i = 0; i < threadParamTbl.size(); i++) {
        int instId = CreateAtlasThreadMgr(threadParamTbl[i].threadInst,
                                        threadParamTbl[i].threadInstName,
                                        threadParamTbl[i].context, 
                                        threadParamTbl[i].runMode);
        if (instId == INVALID_INSTANCE_ID) {
            ATLAS_LOG_ERROR("Create thread instance failed");
            return ATLAS_ERROR;
        }
        threadParamTbl[i].threadInstId = instId;
    }
    //Note:The instance id must generate first, then create thread,
    //for the user thread get other thread instance id in Init function
    for (size_t i = 0; i < threadParamTbl.size(); i++) {
        threadList_[threadParamTbl[i].threadInstId]->CreateThread();
    }

    for (size_t i = 0; i < threadParamTbl.size(); i++) {
        int instId = threadParamTbl[i].threadInstId;
        AtlasError ret = threadList_[instId]->WaitThreadInitEnd();
        if (ret != ATLAS_OK) {
            ATLAS_LOG_ERROR("Create thread %s failed, error %d", 
                            threadParamTbl[i].threadInstName.c_str(), ret);
            return ret;
        } 
    }
    
    return ATLAS_OK;
}

int AtlasApp::GetAtlasThreadIdByName(const string& threadName) {
    if (threadName.empty()) {
        ATLAS_LOG_ERROR("search name is empty");
        return INVALID_INSTANCE_ID;
    }

    for (uint32_t i = 0; i < threadList_.size(); i++) {
        if (threadList_[i]->GetThreadName() == threadName) {
            return i;
        }
    }
    
    return INVALID_INSTANCE_ID;
}

AtlasError AtlasApp::SendMessage(int dest, int msgId, shared_ptr<void> data) {
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
        if (waitEnd_) break;
    }
    threadList_[kMainThreadId]->SetStatus(THREAD_EXITED);
}

bool AtlasApp::CheckThreadAbnormal() {
    for (size_t i = 0; i < threadList_.size(); i++) {
        if (threadList_[i]->GetStatus() == THREAD_ERROR) {
            return true;
        }
    }

    return false;
}

void AtlasApp::Wait(AtlasMsgProcess msgProcess, void* param) {
    AtlasThreadMgr* mainMgr = threadList_[0];

    if (mainMgr == nullptr) {
        ATLAS_LOG_ERROR("Atlas app wait exit for message process function is nullptr");
        return;
    }

    while (true) {
        if (waitEnd_) break;

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
    threadList_[kMainThreadId]->SetStatus(THREAD_EXITED);
}

void AtlasApp::Exit() {
    ReleaseThreads();
}

void AtlasApp::ReleaseThreads() {
    if (isReleased_) return;
    threadList_[kMainThreadId]->SetStatus(THREAD_EXITED);

    for (uint32_t i = 1; i < threadList_.size(); i++) {
        if ((threadList_[i] != nullptr) && 
            (threadList_[i]->GetStatus() == THREAD_RUNNING))
            threadList_[i]->SetStatus(THREAD_EXITING);
    }

    int retry = kThreadExitRetry; 
    while(retry >= 0) {
        bool exitFinish = true;
        for (uint32_t i = 0; i < threadList_.size(); i++) {
            if (threadList_[i] == nullptr) 
                continue;
            if (threadList_[i]->GetStatus() > THREAD_EXITING) {
                delete threadList_[i];
                threadList_[i] = nullptr;
                ATLAS_LOG_INFO("Atlas thread %d released", i);
            } else {
                exitFinish = false;
            }
        }

        if (exitFinish)
            break;

        sleep(1);
        retry--;
    }
    isReleased_ = true;
}

AtlasApp& CreateAtlasAppInstance() {
    return AtlasApp::GetInstance();
}

AtlasApp& GetAtlasAppInstance() {
    return AtlasApp::GetInstance();
}

AtlasError SendMessage(int dest, int msgId, shared_ptr<void> data) {
    AtlasApp& app = AtlasApp::GetInstance();
    return app.SendMessage(dest, msgId, data);
}

int GetAtlasThreadIdByName(const string& threadName) {
    AtlasApp& app = AtlasApp::GetInstance();
    return app.GetAtlasThreadIdByName(threadName);
}
