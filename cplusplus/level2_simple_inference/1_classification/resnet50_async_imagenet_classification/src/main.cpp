/**
* @file main.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include <iostream>
#include "sample_process.h"
#include "utils.h"

using namespace std;

bool g_isDevice = false;
size_t g_executeTimes = 100; // model async execute times
size_t g_callbackInterval = 1; // launch callback interval
size_t g_memoryPoolSize = 100; // number of memory blocks

int main(int argc, char *argv[])
{
    INFO_LOG("./main param1 param2 param3, param1 is execute model times(default 100),"
        " param2 is callback interval(default 1), param3 is memory pool size(default 100)");
    if (argc > 1) {
        g_executeTimes = atoi(argv[1]);
    }
    INFO_LOG("execute times = %zu", g_executeTimes);

    if (argc > 2) {
        g_callbackInterval = atoi(argv[2]);
    }
    INFO_LOG("callback interval = %zu", g_callbackInterval);

    if (argc > 3) {
        g_memoryPoolSize = atoi(argv[3]);
    }
    INFO_LOG("memory pool size = %zu", g_memoryPoolSize);

    SampleProcess sampleProcess;
    Result ret = sampleProcess.InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("sample init resource failed");
        return FAILED;
    }

    ret = sampleProcess.Process();
    if (ret != SUCCESS) {
        ERROR_LOG("sample process failed");
        return FAILED;
    }

    INFO_LOG("execute sample success");
    return SUCCESS;
}