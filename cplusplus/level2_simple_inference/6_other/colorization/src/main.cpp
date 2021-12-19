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
#include "colorize_process.h"
#include "acllite/AclLiteUtils.h"
#include "acllite/AclLiteError.h"
using namespace std;

namespace {
uint32_t kModelWidth = 224;
uint32_t kModelHeight = 224;
const char* kModelPath = "../model/colorization.om";
}

int main(int argc, char *argv[]) {
    //check input param
    if((argc < 2) || (argv[1] == nullptr)){
        ACLLITE_LOG_ERROR("Please input: ./main <image_dir>");
        return 1;
    }
    //init instance
    ColorizeProcess colorize(kModelPath, kModelWidth, kModelHeight);
    //init resource
    AclLiteError ret = colorize.init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Classification Init resource failed");
        return 1;
    }
    //get file
    string inputImageDir = string(argv[1]);
    vector<string> fileVec;
    GetAllFiles(inputImageDir, fileVec);
    if (fileVec.empty()) {
        ACLLITE_LOG_ERROR("Failed to deal all empty path=%s.", inputImageDir.c_str());
        return 1;
    }

    for (string imageFile : fileVec) {
        //preprocess
        ret = colorize.preprocess(imageFile);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Read file %s failed, continue to read next",
                      imageFile.c_str());                
            continue;
        }
        //inference
        std::vector<InferenceOutput> inferenceOutput;
        ret = colorize.inference(inferenceOutput);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Inference model inference output data failed");
            return 1;
        }
        //postprocess
        ret = colorize.postprocess(imageFile, inferenceOutput);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Process model inference output data failed");
            return 1;
        }
    }

    ACLLITE_LOG_INFO("Execute sample success");
    return 0;
}

