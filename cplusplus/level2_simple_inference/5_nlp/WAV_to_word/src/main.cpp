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
#include "acllite/AclLiteUtils.h"
using namespace std;

int main()
{
    SampleProcess processSample;
    AclLiteError ret = processSample.InitResource();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("sample init resource failed");
        return 1;
    }

    ret = processSample.Process();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("sample process failed");
        return 1;
    }

    ACLLITE_LOG_INFO("execute sample success");
    return ACLLITE_OK;
}
