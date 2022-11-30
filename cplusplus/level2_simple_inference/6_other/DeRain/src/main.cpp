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
#include "utils.h"
#include "ctime"
using namespace std;

namespace {
uint32_t kModelWidth = 256;
uint32_t kModelHeight = 256;
const char* kModelPath = "../model/DeRain.om";
}

int main(int argc, char *argv[]) {
    //init instance
    ClassifyProcess classify(kModelPath, kModelWidth, kModelHeight);
    //init acl resource
    Result ret = classify.Init();
    if (ret != SUCCESS) {
        ERROR_LOG("Classification Init resource failed");
        return FAILED;
    }
    //get file name
    string inputImageDir = string("../data");
    vector<string> fileVec;
    Utils::GetAllFiles(inputImageDir, fileVec);
    if (fileVec.empty()) {
        ERROR_LOG("Failed to deal all empty path=%s.", inputImageDir.c_str());
        return FAILED;
    }
    //process pic one by one
    for (string imageFile : fileVec) {
        int posofPoint = imageFile.find_last_of(".");
        int posofUnderline = imageFile.find_last_of("_");
        if(posofPoint-posofUnderline-1>0){
            string PicType(imageFile.substr(posofUnderline + 1,posofPoint-posofUnderline-1));
            if(PicType == "GT"){
                continue;
            }
        }
        //preprocess:read pic, resize to dst size
        Result ret = classify.Preprocess(imageFile);
        if (ret != SUCCESS) {
            ERROR_LOG("Read file %s failed, continue to read next",
                      imageFile.c_str());                
            continue;
        }

        //inference && get infer result
        aclmdlDataset* inferenceOutput = nullptr;
        ret = classify.Inference(inferenceOutput);
        if ((ret != SUCCESS) || (inferenceOutput == nullptr)) {
            ERROR_LOG("Inference model inference output data failed");
            return FAILED;
        }

        //postprocess:analyse result and paste label to pic
        ret = classify.Postprocess(imageFile, inferenceOutput);
        if (ret != SUCCESS) {
            ERROR_LOG("Process model inference output data failed");
            return FAILED;
        }
    }
    classify.PrintMeanPSNR();
    INFO_LOG("Execute sample success");
    return SUCCESS;
}
