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
* Description: dvpp sample main func
*/

#include <iostream>
#include <stdlib.h>
#include <dirent.h>

#include "classify_process.h"
#include "atlasutil/atlas_utils.h"
#include "atlasutil/atlas_error.h"
#include "atlasutil/acl_device.h"
using namespace std;

int main(int argc, char *argv[]) {

    if((argc < 2) || (argv[1] == nullptr)){
        ATLAS_LOG_ERROR("Please input: ./main <image_dir>");
        return ATLAS_ERROR;
    }

    AclDevice aclDev;
    AtlasError ret = aclDev.Init();
    if (ret) {
        ATLAS_LOG_ERROR("Init resource failed, error %d", ret);
        return ATLAS_ERROR;
    }  
    aclrtRunMode RunMode = aclDev.GetRunMode();  

    ClassifyProcess classify;
    ret = classify.Init();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Classification Init resource failed");
        return ATLAS_ERROR;
    }

    string inputImageDir = string(argv[1]);
    vector<string> fileVec;
    GetAllFiles(inputImageDir, fileVec);
    if (fileVec.empty()) {
        ATLAS_LOG_ERROR("Failed to deal all empty path=%s.", inputImageDir.c_str());
        return ATLAS_ERROR;
    }

    ret = classify.Process(fileVec, RunMode);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Classification Excute Inference failed");
        return ATLAS_ERROR;
    }
    else{
        ATLAS_LOG_INFO("Classification Excute Inference success");
    }

    ATLAS_LOG_INFO("Execute sample finish");
    
    classify.DestroyResource();
    aclDev.Release();
    
    return ATLAS_OK;
}
