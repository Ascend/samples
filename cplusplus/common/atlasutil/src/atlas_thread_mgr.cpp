#include "atlas_thread_mgr.h"
#include "atlas_utils.h"

namespace {
    const uint32_t kMsgQueueSize = 256;
    const uint32_t kWait10Milliseconds = 10000;
    const uint32_t kWaitThreadStart = 1000;
}


AtlasThreadMgr::AtlasThreadMgr(AtlasThread* userThreadInstance, const string& threadName)
:name_(threadName),
userInstance_(userThreadInstance),
isExit_(false),
status_(THREAD_READY),
msgQueue_(kMsgQueueSize) {
}

AtlasThreadMgr::~AtlasThreadMgr() {
    userInstance_ = nullptr;
    while(!msgQueue_.Empty()) {
        msgQueue_.Pop();
    }
}

void AtlasThreadMgr::CreateThread() {
    thread engine(&AtlasThreadMgr::ThreadEntry, (void *)this);
    engine.detach();
}

void AtlasThreadMgr::ThreadEntry(void* arg){
    AtlasThreadMgr* thMgr = (AtlasThreadMgr*)arg;
    AtlasThread* userInstance = thMgr->GetUserInstance();
    if (userInstance == nullptr) {
        ATLAS_LOG_ERROR("Atlas thread exit for user thread instance is null");
        return;
    }
    
    string& instName = userInstance->SelfInstanceName();
    aclrtContext context = userInstance->GetContext();
    aclError aclRet = aclrtSetCurrentContext(context);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Thread %s set context failed, error: %d", 
                        instName.c_str(), aclRet);
        return;
    }
    
    int ret = userInstance->Init();
    if (ret) {
        ATLAS_LOG_ERROR("Thread %s init error %d, thread exit", 
                        instName.c_str(), ret);
        thMgr->SetStatus(THREAD_ERROR);
        return;
    }

    thMgr->SetStatus(THREAD_RUNNING);
    while(THREAD_RUNNING == thMgr->GetStatus()) {
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
            ATLAS_LOG_ERROR("Thread %s process function return "
                            "error %d, thread exit", instName.c_str(), ret);
            thMgr->SetStatus(THREAD_ERROR);
            return;
        } 
        usleep(0);
    }
    thMgr->SetStatus(THREAD_EXITED);

    return;
}

AtlasError AtlasThreadMgr::WaitThreadInitEnd() {
	while(true) {
		if (status_ == THREAD_RUNNING) {
			break;
		} else if (status_ > THREAD_RUNNING) {
            string& instName = userInstance_->SelfInstanceName();
			ATLAS_LOG_ERROR("Thread instance %s status change to %d, "
							"app start failed", instName.c_str(), status_);							
			return ATLAS_ERROR_START_THREAD;
		} else {
			usleep(kWaitThreadStart);
		}
	}

	return ATLAS_OK;
}

AtlasError AtlasThreadMgr::PushMsgToQueue(shared_ptr<AtlasMessage>& pMessage) {
    if (status_ != THREAD_RUNNING) {
        ATLAS_LOG_ERROR("Thread instance %s status(%d) is invalid, "
                        "can not reveive message", name_.c_str(), status_);
        return ATLAS_ERROR_THREAD_ABNORMAL;
    }
    return msgQueue_.Push(pMessage)? ATLAS_OK : ATLAS_ERROR_ENQUEUE;
}



