/**
* Copyright 2020 Huawei Technologies Co., Ltd
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

* File main.cpp
* Description: objectdetection video yolov3 sample main func
*/

#include <iostream>
#include <stdlib.h>
#include <dirent.h>
#include "sample_process.h"
#include "utils.h"
using namespace std;

int main(int argc, char *argv[])
{
    if(argc<2){
        ERROR_LOG("please input: ./main videopath");
	return FAILED;
    }

    if(!Utils::IsPathExist(argv[1])){
        ERROR_LOG("video path %s not found",argv[1]);
	return FAILED;
    }

    SampleProcess processSample;
    Result ret = processSample.InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("sample init resource failed");
        return FAILED;
    }


    //input video path
    string input_path = string(argv[1]);

    ret = processSample.VideoProcess(input_path);
    if (ret != SUCCESS) {
        ERROR_LOG("sample model process failed");
        return FAILED;
    }
    INFO_LOG("execute sample success");
    return SUCCESS;
}
