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

int main(int argc, char *argv[])
{
    SampleProcess sampleProcess;
    Result ret = sampleProcess.InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("sample init resource failed");
        return FAILED;
    }

    ret = sampleProcess.GetInputOption(argc, argv);
    if (ret != SUCCESS) {
        ERROR_LOG("input parameter invalid");
        return FAILED;
    }

    ret = sampleProcess.VpcProcess();
    if (ret != SUCCESS) {
        ERROR_LOG("vpc process failed");
        return FAILED;
    }

    INFO_LOG("execute sample success");
    return SUCCESS;
}
