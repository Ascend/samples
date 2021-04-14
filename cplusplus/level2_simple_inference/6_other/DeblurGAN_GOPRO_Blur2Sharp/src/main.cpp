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
#include <stdio.h>
#include <unistd.h>
#include "business_imp.h"
#include "atlasutil/atlas_utils.h"
#include "atlasutil/atlas_error.h"
using namespace std;

namespace {
uint32_t kModelWidth = 1280;
uint32_t kModelHeight = 720;
const char* kModelPath = "../model/blurtosharp_pad_1280_720.om";
}

int main(int argc, char *argv[]) {
    //check the input, need the picture directory.
    if((argc < 2) || (argv[1] == nullptr)){
        ATLAS_LOG_ERROR("Please input: ./main <image_dir>");
        return FAILED;
    }

    // according argv[0], get the work directory.
    char work_file[200];
    ATLAS_LOG_INFO("argv[0]:%s", argv[0]);
    
    realpath(argv[0], work_file);
    // ATLAS_LOG_INFO("work_path = %s", work_file); 
    string work_file_tmp = work_file;

    string work_path = work_file_tmp.substr(0, work_file_tmp.find_last_of("/"));
    ATLAS_LOG_INFO("work_path = %s", work_path.c_str()); 

    //Business implementation, need model path, model input image widht and height
    BusinessImp businessImp(kModelPath, kModelWidth, kModelHeight, work_path);
    //Initialization of ACL resource, model and memory
    AtlasError ret = businessImp.init();
    if (ret != SUCCESS) {
        ATLAS_LOG_ERROR("Classification Init resource failed");
        return FAILED;
    }
    //Get all the image file names in the image directory
    string inputImageDir = string(argv[1]);
    vector<string> fileVec;
    GetAllFiles(inputImageDir, fileVec);
    if (fileVec.empty()) {
        ATLAS_LOG_ERROR("Failed to deal all empty path=%s.", inputImageDir.c_str());
        return FAILED;
    }
    
    for (string imageFile : fileVec) {
        //Preprocessing image: read the image and zoom it to the size required by the model input
        ret = businessImp.preprocess(imageFile);
        if (ret != SUCCESS) {
            ATLAS_LOG_ERROR("Read file %s failed, continue to read next",
                      imageFile.c_str());                
            continue;
        }
        //The preprocessed images are sent to the model for inference
        std::vector<InferenceOutput> inferenceOutput;
        ret = businessImp.inference(inferenceOutput);
        if (ret != SUCCESS) {
            ATLAS_LOG_ERROR("Inference model inference output data failed");
            return FAILED;
        }

        //Get inference output, and resize to the original size.
        ret = businessImp.postprocess(imageFile, inferenceOutput);
        if (ret != SUCCESS) {
            ATLAS_LOG_ERROR("Process model inference output data failed");
            return FAILED;
        }
    }

    ATLAS_LOG_INFO("Execute sample success");
    return SUCCESS;
}
