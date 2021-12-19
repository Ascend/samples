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
#include <vector>
#include "object_detect.h"
#include "acllite/AclLiteUtils.h"
#include "acllite/AclLiteError.h"
#include "acllite/AclLiteResource.h"
using namespace std;

namespace {
uint32_t kModelWidth = 416;
uint32_t kModelHeight = 416;
const char* kModelPath = "../model/yolov3.om";
}

int main(int argc, char *argv[]) {

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

    ObjectDetect detect;
    ret = detect.Init();
    if (ret) {
        ACLLITE_LOG_ERROR("Init resource failed, error %d", ret);
        return ACLLITE_ERROR;
    }

    string inputImageDir = string(argv[1]);
    vector<string> fileVec;
    GetAllFiles(inputImageDir, fileVec);
    if (fileVec.empty()) {
        ACLLITE_LOG_ERROR("Failed to deal all empty path=%s.", inputImageDir.c_str());
        return ACLLITE_ERROR;
    }

    ImageData image;
    for (string imageFile : fileVec) {
        ReadJpeg(image, imageFile);
        if (image.data == nullptr) {
            ACLLITE_LOG_ERROR("Read image %s failed", imageFile.c_str());
            return ACLLITE_ERROR;
        }

        ImageData resizedImage;
        ret = detect.Preprocess(resizedImage, image, RunMode);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Read file %s failed, continue to read next",
                      imageFile.c_str());                
            continue;
        }
        std::vector<InferenceOutput> inferenceOutput;
        ret = detect.Inference(inferenceOutput, resizedImage);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Inference model inference output data failed");
            return ACLLITE_ERROR;
        }
 
        ret = detect.Postprocess(image, inferenceOutput, imageFile);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Process model inference output data failed");
            return ACLLITE_ERROR;
        }
    }
    ACLLITE_LOG_INFO("Execute sample success");
    return ACLLITE_OK;
}
