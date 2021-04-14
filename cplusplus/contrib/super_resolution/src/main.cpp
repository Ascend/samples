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

#include "super_resolution_process.h"
#include "utils.h"
using namespace std;

namespace {
}

int main(int argc, char *argv[]) {
    if((argc < 2) || (argv[1] == nullptr)){
        ERROR_LOG("Please input: ./main <image_dir>");
        return FAILED;
    }

    // model type 
    // 0 - SRCNN
    // 1 - FSRCNN
    // 2 - ESPCN
    // (defalt: SRCNN)
    uint8_t kModelType;
    if (argv[2] == nullptr){
        kModelType = 0;
    }
    else{
        kModelType = atoi(argv[2]);
    }

    SuperResolutionProcess SR(kModelType);
    //init acl resources
    Result ret = SR.init();
    if (ret != SUCCESS) {
        ERROR_LOG("Init resource failed");
        return FAILED;
    }
    //get input Image Dir
    string inputImageDir = string(argv[1]);
    vector<string> fileVec;
    Utils::get_all_files(inputImageDir, fileVec);
    if (fileVec.empty()) {
        ERROR_LOG("Failed to deal all empty path=%s.", inputImageDir.c_str());
        return FAILED;
    }

    //inference
    for (string imageFile : fileVec) {
        ret = SR.preprocess(imageFile);
        if (ret != SUCCESS) {
            ERROR_LOG("Deal file %s failed, continue to read next",
                      imageFile.c_str());                
            continue;
        }
        aclmdlDataset* inferenceOutput = nullptr;
        ret = SR.inference(inferenceOutput);
        if ((ret != SUCCESS) || (inferenceOutput == nullptr)) {
            ERROR_LOG("Inference model inference output data failed");
            return FAILED;
        }
        ret = SR.postprocess(imageFile, inferenceOutput);
        if (ret != SUCCESS) {
            ERROR_LOG("Process model inference output data failed");
            return FAILED;
        }
        // Destroy model
        SR.destroy_model();
    }

    INFO_LOG("Execute sample success");
    return SUCCESS;
}
