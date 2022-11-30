/*
* Copyright (C) Huawei Technologies Co., Ltd. 2020-2022. All rights reserved.
*/
/**
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
* Description: dvpp sample main func
*/

#include <iostream>
#include <cstdlib>
#include <dirent.h>

#include "classify_process.h"
#include "acllite/AclLiteUtils.h"
#include "acllite/AclLiteError.h"
#include "acllite/AclLiteResource.h"
using namespace std;

int main(int argc, char *argv[]) 
{
    if((argc < 2) || (argv[1] == nullptr)){
        ACLLITE_LOG_ERROR("Please input: ./main <image_dir>");
        return ACLLITE_ERROR;
    }

    AclLiteResource aclDev;
    AclLiteError ret = aclDev.Init();
    if (ret) {
        ACLLITE_LOG_ERROR("Init resource failed, error %d", ret);
        return ACLLITE_ERROR;
    }
    aclrtRunMode RunMode = aclDev.GetRunMode();  

    ClassifyProcess classify;
    ret = classify.Init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Classification Init resource failed");
        return ACLLITE_ERROR;
    }

    string inputImageDir = string(argv[1]);
    vector<string> fileVec;
    GetAllFiles(inputImageDir, fileVec);
    if (fileVec.empty()) {
        ACLLITE_LOG_ERROR("Failed to deal all empty path=%s.", inputImageDir.c_str());
        return ACLLITE_ERROR;
    }

    ret = classify.Process(fileVec, RunMode);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Classification Excute Inference failed");
        return ACLLITE_ERROR;
    }else {
        ACLLITE_LOG_INFO("Classification Excute Inference success");
    }

    ACLLITE_LOG_INFO("Execute sample finish");
    
    classify.DestroyResource();
    aclDev.Release();
    
    return ACLLITE_OK;
}