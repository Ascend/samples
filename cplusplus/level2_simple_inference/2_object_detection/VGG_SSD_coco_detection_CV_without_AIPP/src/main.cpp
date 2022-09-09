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
#include <cstdlib>
#include <dirent.h>
#include "sample_process.h"
#include "utils.h"
using namespace std;

int main(int argc, char *argv[])
{
    // ACL init
    SampleProcess processSample;
    Result ret = processSample.InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("sample init resource failed");
        return FAILED;
    }

    string input_path = "../data";
    ret = processSample.MainProcess(input_path);
    if (ret != SUCCESS) {
        ERROR_LOG("sample model process failed");
        return FAILED;
    }
    INFO_LOG("execute sample success");
    return SUCCESS;
}
