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
#include "object_detect.h"
#include "AclLiteError.h"

using namespace std;

namespace {
    uint32_t kModelWidth;
    uint32_t kModelHeight;
    const char* kModelPath;
}

int main(int argc, char *argv[]) {
    if((argc < 3)){
        ACLLITE_LOG_ERROR("invalid parameter number, must input four parameters.");
        ACLLITE_LOG_ERROR("Please input: ./main imgdir size size");
        return ACLLITE_ERROR;
    }else{
        uint64_t width = atoll(argv[2]);
        uint64_t height = atoll(argv[2]);
        if (!(((height == 256) && (width == 256)) ||
            ((height == 512) && (width == 512)) ||
            ((height == 1024) && (width == 1024)))) {
            ACLLITE_LOG_ERROR("invalid dynamic hw, should be 256*256,512*512,1024*1024.");
            return ACLLITE_ERROR;
        }
    }

    kModelWidth = atoi(argv[2]);
    kModelHeight = atoi(argv[2]);
    if(256 == atoi(argv[2])){
        kModelPath = "../model/AnimeGANv2_256.om";
    }else if(512 == atoi(argv[2])){ 
        kModelPath = "../model/AnimeGANv2_512.om";
    }else{
        kModelPath = "../model/AnimeGANv2_1024.om";
    }

    //Instantiate the object detection class
    ObjectDetect detect(kModelPath, kModelWidth, kModelHeight);
    //Initialize the acl resources, dvpp, load model, 
    //and malloc input memory of input which is const
    AclLiteError ret = detect.init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Classification Init resource failed");
        return ACLLITE_ERROR;
    }

    //Get all the image file path in the image directory
    string inputImageDir = string(argv[1]);
    vector<string> fileVec;
    GetAllFiles(inputImageDir, fileVec);
    if (fileVec.empty()) {
        ACLLITE_LOG_ERROR("Failed to deal all empty path=%s.", inputImageDir.c_str());
        return 1;
    }

    for (string imageFile : fileVec) {
        ImageData image;
        ReadJpeg(image, imageFile);
        if (image.data == nullptr) {
            ACLLITE_LOG_ERROR("Read image %s failed", imageFile.c_str());
            return ACLLITE_ERROR;
        }
        //Preprocess: copy image to device, convert to yuv, and resize
        ImageData resizedImage;
        ret = detect.preprocess(resizedImage, image);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Read file %s failed, continue to read next",
                        imageFile.c_str());                
            continue;
        }
        //Send the resized picture to the model for inference 
        //and get the inference results
        vector<InferenceOutput> inferenceOutput;
        ret = detect.inference(inferenceOutput, resizedImage);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Inference model inference output data failed");
            return ACLLITE_ERROR;
        }
        //Analyze the inference output, mark the object category and 
        //location by the inference result
        ret = detect.postprocess(inferenceOutput, imageFile, resizedImage);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Process model inference output data failed");
            return ACLLITE_ERROR;
        }
    }

    ACLLITE_LOG_INFO("Execute sample success");
    return ACLLITE_OK;
}
