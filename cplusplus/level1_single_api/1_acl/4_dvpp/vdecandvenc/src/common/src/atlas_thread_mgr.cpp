#include "atlas_thread_mgr.h"
#include "atlas_utils.h"

namespace {
    const uint32_t kMsgQueueSize = 256;
    const uint32_t kWait10Milliseconds = 10000;
}


AtlasThreadMgr::AtlasThreadMgr(AtlasThread* userThreadInstance, const char* threadName)
:msgQueue_(kMsgQueueSize) {
    if (threadName == nullptr) {
        name_ = "";
    } else {
        name_.assign(threadName);
    }
    userInstance_ = userThreadInstance;
    isExit_ = false;
    status_ = THREAD_READY;
}

void AtlasThreadMgr::CreateThread() {
    thread engine(&AtlasThreadMgr::ThreadEntry, (void *)this);
    engine.detach();
}


void AtlasThreadMgr::ThreadEntry(void* arg){
    int ret;
    AtlasThreadMgr* thMgr = (AtlasThreadMgr*)arg;
    AtlasThread* userInstance = thMgr->GetUserInstance();
    
    if (userInstance == nullptr) {
        ATLAS_LOG_ERROR("Atlas thread exit for user thread instance is null");
        return;
    }

    ret = userInstance->Init();
    if (ret) {
        ATLAS_LOG_ERROR("Call user atlas thread init function return error %d, thread exit", ret);
        thMgr->SetStatus(THREAD_ERROR);
        return;
    }

    thMgr->SetStatus(THREAD_RUNNING);
    while(THREAD_EXITING != thMgr->GetStatus()) {
        // 从队列中取数据
        shared_ptr<AtlasMessage> msg = thMgr->PopMsgFromQueue();
        if(msg == nullptr) {
            usleep(kWait10Milliseconds);
            continue;
        }
        // 线程消息处理函数
        ret = userInstance->Process(msg->msgId, msg->data);
        msg->data = nullptr;
        if (ret) {
            ATLAS_LOG_ERROR("Atlas thread call user process function return error %d, thread exit", ret);
            break;
        } 
        usleep(0);
    }
    thMgr->SetStatus(THREAD_EXITED);

    return;
}

bool AtlasThreadMgr::PushMsgToQueue(shared_ptr<AtlasMessage>& pMessage) {
    bool ret;
    ret = msgQueue_.Push(pMessage);
    return ret;
}



