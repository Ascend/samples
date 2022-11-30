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

#ifndef YOLOV3_CARCOLOR_SAMPLE_INC_OBJECT_DETECT_H
#define YOLOV3_CARCOLOR_SAMPLE_INC_OBJECT_DETECT_H

#pragma once
#include <memory>
#include<opencv2/core/core.hpp>
#include "acl/acl.h"
#include "AclLiteModel.h"
#include "AclLiteImageProc.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc/types_c.h"
#include "params.h"

/**
* ClassifyProcess
*/
class ObjectDetect {
public:
    ObjectDetect();
    ~ObjectDetect();
    AclLiteError Init();
    AclLiteError PreProcess(ImageData& resizedImage, ImageData& srcImage, ImageData& yuvImage);
    AclLiteError Inference(std::vector<InferenceOutput>& inferenceOutput, ImageData& resizedImage);
    AclLiteError PostProcess(ImageData& image, std::vector<InferenceOutput>& inferenceOutput,
                             std::vector<CarInfo>& carData);
private:
    void DestroyResource();
    AclLiteError InitModelInput();

private:
    uint32_t g_imageInfoSize_;
    void*    g_imageInfoBuf_;
    AclLiteModel g_model_;
    const char* g_modelPath_;
    AclLiteImageProc g_dvpp_;
    aclrtRunMode g_runMode_;
    bool g_isInited_;
    bool g_isReleased_;
};

#endif