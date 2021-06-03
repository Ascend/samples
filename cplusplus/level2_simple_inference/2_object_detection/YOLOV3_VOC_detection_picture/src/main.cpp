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
#include "atlasutil/atlas_utils.h"
#include "atlasutil/atlas_error.h"
#include "atlasutil/acl_device.h"
using namespace std;

namespace {
uint32_t kModelWidth = 416;
uint32_t kModelHeight = 416;
const char* kModelPath = "../model/yolov3.om";
}

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

    ObjectDetect detect;
    ret = detect.Init();
    if (ret) {
        ATLAS_LOG_ERROR("Init resource failed, error %d", ret);
        return ATLAS_ERROR;
    }

    string inputImageDir = string(argv[1]);
    vector<string> fileVec;
    GetAllFiles(inputImageDir, fileVec);
    if (fileVec.empty()) {
        ATLAS_LOG_ERROR("Failed to deal all empty path=%s.", inputImageDir.c_str());
        return ATLAS_ERROR;
    }

    ImageData image;
    for (string imageFile : fileVec) {
        ReadJpeg(image, imageFile);
        if (image.data == nullptr) {
            ATLAS_LOG_ERROR("Read image %s failed", imageFile.c_str());
            return ATLAS_ERROR;
        }

        ImageData resizedImage;
        ret = detect.Preprocess(resizedImage, image, RunMode);
        if (ret != ATLAS_OK) {
            ATLAS_LOG_ERROR("Read file %s failed, continue to read next",
                      imageFile.c_str());                
            continue;
        }
        std::vector<InferenceOutput> inferenceOutput;
        ret = detect.Inference(inferenceOutput, resizedImage);
        if (ret != ATLAS_OK) {
            ATLAS_LOG_ERROR("Inference model inference output data failed");
            return ATLAS_ERROR;
        }
 
        ret = detect.Postprocess(image, inferenceOutput, imageFile);
        if (ret != ATLAS_OK) {
            ATLAS_LOG_ERROR("Process model inference output data failed");
            return ATLAS_ERROR;
        }
    }
    ATLAS_LOG_INFO("Execute sample success");
    return ATLAS_OK;
}
