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

#include <cstdio>
#include <iostream>
#include <stdlib.h>
#include <dirent.h>

#include "object_detect.h"
#include "utils.h"
#include "camera.h"
using namespace std;

namespace {

    uint32_t kPreviewWidth = 256;
    uint32_t kPreviewHeight = 256;
    uint32_t kModelWidth1 = 224;
    uint32_t kModelHeight1 = 224;
    uint32_t kModelWidth2 = 224;
    uint32_t kModelHeight2 = 220;
    uint32_t kModelWidth3 = 224;
    uint32_t kModelHeight3 = 224;
    const char* kModelPath1 = "../model/collision_avoidance_model.om";
    const char* kModelPath2 = "../model/road_object_detection_deploy.om";
    const char* kModelPath3 = "../model/road_following_model.om";
}

int main(int argc, char *argv[]) {
    bool bSetChannelId = true;
    ModelInfoParams param;
    param.modelPath1   = kModelPath1;
    param.modelWidth1  = kModelWidth1;
    param.modelHeight1 = kModelHeight1;
    param.modelPath2   = kModelPath2;
    param.modelWidth2  = kModelWidth2;
    param.modelHeight2 = kModelHeight2;
    param.modelPath3   = kModelPath3;
    param.modelWidth3  = kModelWidth3;
    param.modelHeight3 = kModelHeight3;
    param.PreviewWidth  = kPreviewWidth;
    param.PreviewHeight = kPreviewHeight;

    int channelId = 0;
    //check camera id
    if((argc < 2) || (argv[1] == nullptr)){
        ERROR_LOG("Please input: ./main ChannelID");
        bSetChannelId =false;
    }

    //init object instance
    ObjectDetect detect;
    Result ret = detect.init(param);
    if (ret != SUCCESS) {
        ERROR_LOG("detect Init resource failed");
        return FAILED;
    }

    //get channel id
    if(bSetChannelId)
    {
        string channelName = string(argv[1]);
        Utils::GetChannelID(channelName, channelId);
        if(0xFF == channelId){
            ERROR_LOG("channelId = %d  ERROR \n", channelId);
            return FAILED;
        }
    }

    Camera  cameraDevice(channelId);
    if(false == cameraDevice.IsOpened(channelId))
    {
        if (cameraDevice.open(channelId)) {
            ERROR_LOG("Failed to open channelId =%d.", channelId);
            return FAILED;
        }
    }

    ImageData image;
    ImageData resizedImage;
    ImageData previewImage;
    aclmdlDataset* inferenceOutput = nullptr;
    uint32_t uOutNum  = 0;
    while(1)
    {
        Result ret = SUCCESS;
        cameraDevice.Read(channelId, image);
        if (image.data == nullptr) {
            ERROR_LOG("Read image %d failed", channelId);
            return FAILED;
        }

        detect._s_work_mode = detect.EngineHan.HandleGetWorkMode();
        //preprocess image
        ret = detect.propreview(previewImage, image);
        if (ret != SUCCESS) {
            ERROR_LOG("Preprocess image %d failed, continue to read next\n",  channelId);
            continue;
        }
        //preprocess image
        ret = detect.preprocess(resizedImage, image);
        if (ret != SUCCESS) {
            ERROR_LOG("Preprocess image %d failed, continue to read next\n",  channelId);
            continue;
        }
        ret = detect.inference(inferenceOutput, resizedImage);
        if ((ret != SUCCESS) || (inferenceOutput == nullptr)) {
            ERROR_LOG("Inference model inference output data failed");
            return FAILED;
        }
        ret = detect.postprocess(image, inferenceOutput);
        if (ret != SUCCESS) {
            ERROR_LOG("Process model inference output data failed");
            return FAILED;
        }
    }

    INFO_LOG("Execute sample success");
    return SUCCESS;
}
