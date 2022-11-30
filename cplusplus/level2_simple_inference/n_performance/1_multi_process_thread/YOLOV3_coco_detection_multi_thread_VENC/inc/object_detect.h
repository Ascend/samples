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

#ifndef YOLOV3_COCO_DETECTION_4_THREAD_INC_OBJECT_DETECT_H
#define YOLOV3_COCO_DETECTION_4_THREAD_INC_OBJECT_DETECT_H

#pragma once
#include <memory>
#include <opencv2/opencv.hpp>
#include "acl/acl.h"
#include "acllite/AclLiteModel.h"
#include "acllite/AclLiteUtils.h"
#include "opencv2/imgproc/types_c.h"

/**
* ObjectDetect
*/
class ObjectDetect {
public:
    ObjectDetect(const char* modelPath, uint32_t modelWidth,
    uint32_t modelHeight);

    ~ObjectDetect();

    AclLiteError Init();

    AclLiteError Inference(std::vector<InferenceOutput>& inferenceOutput, cv::Mat& reiszeMat);

    void DestroyResource();

private:
    AclLiteError CreateInput();

    AclLiteModel g_model_;
    const char* g_modelPath_;
    uint32_t g_modelWidth_;
    uint32_t g_modelHeight_;
    uint32_t g_imageDataSize_;
    void*    g_imageDataBuf_;
    uint32_t g_imageInfoSize_;
    void*    g_imageInfoBuf_;
    aclrtRunMode g_runMode_;
    bool g_isInited_;
};

#endif