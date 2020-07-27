#pragma once
#include <iostream>
#include <memory>
#include <thread>
#include <unistd.h>
#include "thread_safe_queue.h"
#include "atlas_utils_common.h"

using namespace std;

class AtlasThread{
public:
    // 构造函数
    AtlasThread() {};
    ~AtlasThread() {};
    virtual int Init() { return STATUS_OK; };
    virtual int Process(int msgId, shared_ptr<void> msgData) = 0;
};
