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

* File sample_process.h
* Description: handle acl resource
*/
#pragma once

//#ifndef __TOOLS_H__
//#define __TOOLS_H__

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
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/highgui/highgui.hpp"

#ifndef __ACLLITETYPE_H__
#define __ACLLITETYPE_H__
#define RGBF32_CHAN_SIZE(width, height) ((width) * (height) * 4)

using namespace std;

namespace {
const int MSG_APP_START = 1;
const int MSG_READ_FRAME = 2;
const int MSG_DETECT_PREPROC_DATA = 3;
const int MSG_DETECT_INFER_OUTPUT = 4;
const int MSG_DETECT_POSTPROC_DATA = 5;
const int MSG_CLASSIFY_PREPROC_DATA = 6;
const int MSG_CLASSIFY_INFER_OUTPUT = 7;
const int MSG_ENCODE_FINISH = 8;
const int MSG_PRESENT_AGENT_DISPLAY = 9;
const int MSG_APP_EXIT = 10;
const string kPresentAgentDisplayName = "PresentAgentDisplay";
const std::vector<std::string> kDetectPreName = {"detectPre_0", "detectPre_1", "detectPre_2", "detectPre_3", "detectPre_4",
                                                 "detectPre_5", "detectPre_6", "detectPre_7", "detectPre_8", "detectPre_9",
                                                 "detectPre_10", "detectPre_11", "detectPre_12", "detectPre_13", "detectPre_14",
                                                 "detectPre_15", "detectPre_16", "detectPre_17", "detectPre_18", "detectPre_19",
                                                 "detectPre_20", "detectPre_21", "detectPre_22"};
const std::vector<std::string> kInferName = {"inference_0", "inference_1", "inference_2", "inference_3"};
const std::vector<std::string> kDetectPostName = {"detectPost_0","detectPost_1", "detectPost_2","detectPost_3", "detectPost_4",
                                                 "detectPost_5", "detectPost_6", "detectPost_7", "detectPost_8", "detectPost_9",
                                                 "detectPost_10", "detectPost_11", "detectPost_12", "detectPost_13", "detectPost_14",
                                                 "detectPost_15", "detectPost_16", "detectPost_17", "detectPost_18", "detectPost_19",
                                                 "detectPost_20", "detectPost_21", "detectPost_22"};
const std::vector<std::string> kClassifyPreName = {"classifyPre_0", "classifyPre_1", "classifyPre_2", "classifyPre_3", "classifyPre_4",
                                                 "classifyPre_5", "classifyPre_6", "classifyPre_7", "classifyPre_8", "classifyPre_9",
                                                 "classifyPre_10", "classifyPre_11", "classifyPre_12", "classifyPre_13", "classifyPre_14",
                                                 "classifyPre_15", "classifyPre_16", "classifyPre_17", "classifyPre_18", "classifyPre_19",
                                                 "classifyPre_20", "classifyPre_21", "classifyPre_22"};
const std::vector<std::string> kClassifyPostName = {"classifyPost_0","classifyPost_1", "classifyPost_2","classifyPost_3", "classifyPost_4",
                                                 "classifyPost_5", "classifyPost_6", "classifyPost_7", "classifyPost_8", "classifyPost_9",
                                                 "classifyPost_10", "classifyPost_11", "classifyPost_12", "classifyPost_13", "classifyPost_14",
                                                 "classifyPost_15", "classifyPost_16", "classifyPost_17", "classifyPost_18", "classifyPost_19",
                                                 "classifyPost_20", "classifyPost_21", "classifyPost_22"};
}

struct Rectangle {
    cv::Point lt;  // left top
    cv::Point rb;  // right bottom
};

struct CarInfo {
    ImageData cropedImgs;  // cropped image from original image
    ImageData resizedImgs;  //resized image for inference
    Rectangle rectangle;  // recognize rectangle
    std::string detect_result;
    std::string carColor_result;
};

struct CarDetectDataMsg {
    int inferThreadId;
    int detectPostThreadId;
    int classifyPreThreadId;
    int classifyPostThreadId;
    int presentAgentDisplayThreadId;
    uint32_t deviceId;
    uint32_t channelId;
    int isLastFrame;
    int frameNum;
    ImageData imageFrame;
    ImageData resizedFrame;
    cv::Mat frame;
    vector<InferenceOutput> detectInferData;
    vector<CarInfo> carInfo;
    int flag;
    vector<InferenceOutput> classifyInferData;
};
#endif
