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

int main(int argc, char **argv) {
    
    if((argc < 4)){
        ERROR_LOG("invalid parameter number, must input four parameters.");
        ERROR_LOG("Please input: ./main <image_dir> height , width");
        return FAILED;
    }else{
        uint64_t height = atoll(argv[2]);
        uint64_t width = atoll(argv[3]);
        if (!(((height == 256) && (width == 256)) ||
            ((height == 512) && (width == 512)) ||
            ((height == 288) && (width == 288)))) {
            ERROR_LOG("invalid dynamic hw, should be 256*256,512*512,288*288.");
            return FAILED;
            }
    }

    SuperResolutionProcess SR(argv);
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

