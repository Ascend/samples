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

int main(int argc, char *argv[])
{
    //Check the input when the application executes, which takes the path to the input video file
    if((argc < 2) || (argv[1] == nullptr)){
        ERROR_LOG("Please input: ./main <image_dir>");
        return FAILED;
    }
    string streamName = string(argv[1]);
    SampleProcess sampleProcess(streamName);
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