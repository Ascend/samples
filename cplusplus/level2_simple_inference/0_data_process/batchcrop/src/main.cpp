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
#include <stdlib.h>
#include "sample_process.h"
using namespace std;

int main(int argc, char *argv[])
{
    SampleProcess sampleProcess;
    sampleProcess.SetDeviceId(0);
    Result ret = sampleProcess.InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("sampleProcess init resource failed");
        return FAILED;
    }

    ret = sampleProcess.BatchCropProcess();
    if (ret != SUCCESS) {
        ERROR_LOG("sampleProcess BatchCropProcess failed");
        return FAILED;
    }

    return SUCCESS;
}
