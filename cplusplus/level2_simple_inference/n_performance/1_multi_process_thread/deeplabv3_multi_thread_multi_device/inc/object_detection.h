/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
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
*/

#ifndef DEEPLABV3_MULTI_THREAD_MULTI_DEVICE_INC_OBJECT_DETECTION_H
#define DEEPLABV3_MULTI_THREAD_MULTI_DEVICE_INC_OBJECT_DETECTION_H

#pragma once

#include <iostream>
#include <mutex>
#include <memory>
#include <vector>
#include <unistd.h>
#include <sys/timeb.h>
#include <time.h>
#include "AclLiteType.h"
#include "AclLiteModel.h"
#include "AclLiteImageProc.h"
#include<opencv2/opencv.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"
#define RGBF32_CHAN_SIZE(width, height) ((width) * (height) * 4)

using namespace std;

namespace {
const int MSG_APP_START = 1;
const int MSG_READ_FRAME = 2;
const int MSG_PREPROC_DATA = 3;
const int MSG_INFER_OUTPUT = 4;
const int MSG_DECODE_FINISH = 5;
const int MSG_VIDEO_ENCODE = 6;
const int MSG_ENCODE_FINISH = 7;
const int MSG_PREPROC_END = 8;
const int MSG_APP_EXIT = 10;
const std::vector<std::string> g_inferName = {"inference_0", "inference_1",
    "inference_2", "inference_3"};
const std::vector<std::string> g_postprocName = {"postprocess_0", "postprocess_1",
    "postprocess_2", "postprocess_3", "postprocess_4", "postprocess_5",
    "postprocess_6", "postprocess_7", "postprocess_8", "postprocess_9",
    "postprocess_10", "postprocess_11"};
}

struct PreprocDataMsg {
    int inferThreadId;
    int postprocThreadId;
    int isLastFrame;
    int frameNum;
    ImageData resizedMat;
    string imageFileName;
};

struct InferOutputMsg {
    vector<InferenceOutput> inferData;
    int isLastFrame;
    int frameNum;
    string imageFileName;
};

#endif