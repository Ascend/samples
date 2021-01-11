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
size_t g_vencCnt = 16;

int main(int argc, char *argv[])
{
    INFO_LOG("./main param, param is execute venc times(default 16)");
    if (argc > 1) {
        g_vencCnt = atoi(argv[1]);
    }
    SampleProcess sampleProcess;
    Result ret = sampleProcess.InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("sample init resource failed");
        return FAILED;
    }

    ret = sampleProcess.DoVencProcess();
    if (ret != SUCCESS) {
        ERROR_LOG("sample venc failed");
        return FAILED;
    }

    INFO_LOG("execute sample success");
    return SUCCESS;
}