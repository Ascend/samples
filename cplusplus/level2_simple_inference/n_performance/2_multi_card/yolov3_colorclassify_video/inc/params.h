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
#include "AclLiteType.h"
#include "AclLiteModel.h"
#include "AclLiteImageProc.h"
#include <opencv2/opencv.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"
#include <time.h>

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
const int MSG_APP_EXIT = 10;
const std::vector<std::string> kDetectPreName = {"detectPre_0", "detectPre_1"};
const std::vector<std::string> kInferName = {"inference_0", "inference_1"};
const std::vector<std::string> kDetectPostName = {"detectPost_0","detectPost_1"};
const std::vector<std::string> kClassifyPreName = {"classifyPre_0", "classifyPre_1"};
const std::vector<std::string> kClassifyPostName = {"classifyPost_0","classifyPost_1"};
}

struct Rectangle {
    cv::Point lt;  // left top
    cv::Point rb;  // right bottom
};

struct CarInfo {
    ImageData cropedImgs;  // cropped image from original image
    ImageData resizedImgs;  //resized image for inference
    Rectangle rectangle;  
    std::string detect_result;
    std::string carColor_result;
};

struct CarDetectDataMsg {
    int inferThreadId;
    int detectPostThreadId;
    int classifyPreThreadId;
    int classifyPostThreadId;
    uint32_t channelId;
    int isLastFrame;
    int frameNum;

    ImageData imageFrame;
    ImageData resizedMat;
    cv::Mat frame;

    vector<InferenceOutput> detectInferData;
    vector<CarInfo> carInfo;
    int flag;
    vector<InferenceOutput> classifyInferData;
};
#endif
