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
#include <sys/time.h>

#include "atlas_videocapture.h"
#include "object_detect.h"

using namespace std;

namespace {
uint32_t kModelWidth = 304;
uint32_t kModelHeight = 300;
const char* kModelPath = "../model/face_detection.om";
}

AtlasVideoCapture* OpenVideoCapture(int argc, char *argv[]) {
    if (argc > 2) {
        ATLAS_LOG_ERROR("Too many arg, only allow no arg or one arg, "
                        "which is carmera id, video file or rtsp address");
        return nullptr;
    }
    if (argc == 1) {
        return new AtlasVideoCapture();
    }

    string param = string(argv[1]);
    if (IsDigitStr(param)) {
        int cameraId = atoi(param.c_str());
        if ((cameraId < 0) || (cameraId >= CAMERA_ID_INVALID)) {
            ATLAS_LOG_ERROR("Invalid camera id arg %s, only allow %d and %d",
                            param.c_str(), CAMERA_ID_0, CAMERA_ID_1);
            return nullptr;
        }
        return new AtlasVideoCapture(cameraId);
    } else if (IsRtspAddr(param)) {
        return new AtlasVideoCapture(param);
    } else if (IsVideoFile(param)) {
        if (!IsPathExist(param)) {
            ATLAS_LOG_ERROR("The %s is inaccessible", param.c_str());
            return nullptr;
        }
        return new AtlasVideoCapture(param);
    } else {
        ATLAS_LOG_ERROR("Invalid param. The arg should be accessible rtsp,"
                        " video file or camera id, or not input arg");
    }

    return nullptr;
}

int main(int argc, char *argv[]) {  
    ObjectDetect detect(kModelPath, kModelWidth, kModelHeight);

    if (ATLAS_OK != detect.Init()) {
        ATLAS_LOG_ERROR("Object detect init failed");
        return ATLAS_ERROR;
    }

    AtlasVideoCapture* cap = OpenVideoCapture(argc, argv);
    if (cap == nullptr) return ATLAS_ERROR;

    if(!cap->IsOpened()) {
        delete cap;
        ATLAS_LOG_ERROR("Failed to open video");
        return ATLAS_ERROR;
    }

    uint32_t frame_width = cap->Get(FRAME_WIDTH);
    uint32_t frame_height = cap->Get(FRAME_HEIGHT);
    detect.Set(frame_width, frame_height);

    int i = 0;
    while(1) {
        ImageData image;
        AtlasError ret = cap->Read(image);
        if (ret != ATLAS_OK) {
            ATLAS_LOG_ERROR("Read frame failed, return %d", ret);
            break;
        }

        ret = detect.Process(image);
        if (ret != ATLAS_OK) {
            ATLAS_LOG_ERROR("Inference image failed, return %d", ret);
            break;
        }
    }
    delete cap;

    ATLAS_LOG_INFO("Execute sample success");

    return ATLAS_OK;
}
