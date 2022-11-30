/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at

* http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include <iostream>
#include "sample_process.h"
using namespace std;

int main(int argc, char *argv[])
{
    // Check the input when the application executes, which takes the path to the input video file
    int argNum = 2;
    if ((argc < argNum) || (argv[1] == nullptr)) {
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