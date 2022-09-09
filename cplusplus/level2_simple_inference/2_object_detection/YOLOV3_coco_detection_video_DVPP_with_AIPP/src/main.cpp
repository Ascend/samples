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
using namespace std;

int main(int argc, char *argv[])
{
    //Check the input when the application executes, which takes the path to the input video file
    if((argc < 2) || (argv[1] == nullptr)){
        ACLLITE_LOG_ERROR("Please input: ./main <image_dir>");
        return ACLLITE_ERROR;
    }
    string streamName = string(argv[1]);
    SampleProcess sampleProcess(streamName);
    AclLiteError ret = sampleProcess.InitResource();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("sample init resource failed");
        return ACLLITE_ERROR;
    }

    ret = sampleProcess.Process();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("sample process failed");
        return ACLLITE_ERROR;
    }

    ACLLITE_LOG_INFO("execute sample success");
    return ACLLITE_OK;
}