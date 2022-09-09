/**
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2022. All rights reserved.
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
#include <cstdlib>
#include <dirent.h>
#include <sys/time.h>
#include "acllite/AclLiteVideoProc.h"
#include "video_encoder.h"

using namespace std;

AclLiteVideoProc* OpenVideoCapture(int argc, char *argv[])
{
    int paramNum = 2;
    int paramInputPath = 1;
    if (argc > paramNum) {
        ACLLITE_LOG_ERROR("Too many arg, only allow no arg or one arg, "
                        "which is carmera id, video file or rtsp address");
        return nullptr;
    }
    if (argc == paramInputPath) {
        return new AclLiteVideoProc();
    }

    string param = string(argv[paramInputPath]);
    if (IsDigitStr(param)) {
        int cameraId = atoi(param.c_str());
        if ((cameraId < 0) || (cameraId >= CAMERA_ID_INVALID)) {
            ACLLITE_LOG_ERROR("Invalid camera id arg %s, only allow %d and %d",
                              param.c_str(), CAMERA_ID_0, CAMERA_ID_1);
            return nullptr;
        }
        return new AclLiteVideoProc(cameraId);
    } else if (IsRtspAddr(param)) {
        return new AclLiteVideoProc(param);
    } else if (IsVideoFile(param)) {
        if (!IsPathExist(param)) {
            ACLLITE_LOG_ERROR("The %s is inaccessible", param.c_str());
            return nullptr;
        }
        return new AclLiteVideoProc(param);
    } else {
        ACLLITE_LOG_ERROR("Invalid param. The arg should be accessible rtsp,"
                        " video file or camera id, or not input arg");
    }

    return nullptr;
}

int main(int argc, char *argv[])
{
    VideoEncoder videoEncode;
    AclLiteVideoProc* cap = OpenVideoCapture(argc, argv);
    if (cap == nullptr) return ACLLITE_ERROR;

    if (!cap->IsOpened()) {
        delete cap;
        ACLLITE_LOG_ERROR("Failed to open video");
        return ACLLITE_ERROR;
    }

    uint32_t frame_width = cap->Get(FRAME_WIDTH);
    uint32_t frame_height = cap->Get(FRAME_HEIGHT);
    if (ACLLITE_OK != videoEncode.Init(frame_width, frame_height)) {
        ACLLITE_LOG_ERROR("videoEncode init failed");
        return ACLLITE_ERROR;
    }

    int i = 0;
    while (1) {
        ImageData image;
        AclLiteError ret = cap->Read(image);
        if (ret != ACLLITE_OK) {
            break;
        }
        ret = videoEncode.VideoEncode(image);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("process failed, return %d", ret);
            break;
        }
    }
    ACLLITE_LOG_INFO("Execute sample success");
    return ACLLITE_OK;
}
