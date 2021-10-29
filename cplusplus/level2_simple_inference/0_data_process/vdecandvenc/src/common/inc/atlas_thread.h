#pragma once
#include <iostream>
#include <memory>
#include <thread>
#include <unistd.h>
#include "thread_safe_queue.h"
#include "atlas_error.h"

using namespace std;

#define INVALID_THREAD_ID (-1)

class AtlasThread{
public:
    // 构造函数
    AtlasThread() {};
    ~AtlasThread() {};
    virtual int Init() { return ATLAS_OK; };
    virtual int Process(int msgId, void* msgData) = 0;
};

class AtlasApp;

AtlasApp& CreateAtlasAppInstance();
AtlasApp& GetAtlasAppInstance();
AtlasError SendMessage(int dest, int msgId, void* data);
