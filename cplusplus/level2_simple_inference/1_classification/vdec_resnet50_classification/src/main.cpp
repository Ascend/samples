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

int main()
{
    SampleProcess sampleProcess;
    Result ret = sampleProcess.InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("sample init resource failed");

        return FAILED;
    }

    ret = sampleProcess.DoVdecProcess();
    if (ret != SUCCESS) {
        ERROR_LOG("sample vdec process failed");
        return FAILED;
    }

    ret = sampleProcess.DoModelProcess();
    if (ret != SUCCESS) {
        ERROR_LOG("sample model process failed");
        return FAILED;
    }

    INFO_LOG("execute sample success");
    return SUCCESS;
}