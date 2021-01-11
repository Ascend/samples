/**
* @file main.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include "sample_process.h"
#include "utils.h"

using namespace std;
bool g_isDevice = false;

int main(int argc, char **argv)
{
    INFO_LOG("1: ./main [param], [param] is dynamic batch. It should be 1,2,4 or 8. For example: ./main 8;");
    INFO_LOG("2: ./main [param1] [param2], [param1] is dynamic height. [param1] is dynamic width. "
        "It should be 416, 416; 832, 832; 1248, 1248. For example: ./main 416 416");

    if (argc < 2) {
        ERROR_LOG("invalid parameter number, must input one or two parameters.");
        return FAILED;
    }

    SampleProcess sampleProcess;
    Result ret = sampleProcess.InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("sample init resource failed.");
        return FAILED;
    }

    DynamicInfo dynamicInfo;
    ret = sampleProcess.CheckAndFillDynamicPara(argc, argv, dynamicInfo);
    if (ret != SUCCESS) {
        ERROR_LOG("sample check and fill dynamic parameter failed.");
        return FAILED;
    }

    ret = sampleProcess.Process(dynamicInfo);
    if (ret != SUCCESS) {
        ERROR_LOG("sample process failed.");
        return FAILED;
    }

    INFO_LOG("execute sample success.");
    return SUCCESS;
}