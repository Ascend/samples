#pragma once
#include <iostream>
#include <sys/timeb.h>
#include "pictortsp.h"
#include "acl/acl.h"
#include "AclLiteThread.h"
#include "Params.h"

class PushRtspThread: public AclLiteThread
{
public:
    PushRtspThread(std::string rtspUrl);
    ~PushRtspThread();
    AclLiteError Init();
    AclLiteError Process(int msgId, std::shared_ptr<void> msgData);
    AclLiteError DisplayMsgProcess(std::shared_ptr<DetectDataMsg> detectDataMsg);
private:
    PicToRtsp g_picToRtsp;
    uint64_t g_frameSeq;
    std::string g_rtspUrl;
};
