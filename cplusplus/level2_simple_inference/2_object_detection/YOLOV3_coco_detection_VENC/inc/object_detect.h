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

#ifndef YOLOV3_COCO_DETECTION_VENC_INC_OBJECT_DETECT_H
#define YOLOV3_COCO_DETECTION_VENC_INC_OBJECT_DETECT_H

#pragma once

#include <memory>
#include "utils.h"
#include "acl/acl.h"
#include "model_process.h"
#include "dvpp_process.h"

using namespace std;

/**
* ObjectDetect
*/
class ObjectDetect {
public:
    ObjectDetect(const char* modelPath, uint32_t modelWidth,
    uint32_t modelHeight);
    ~ObjectDetect();
    // Inference initialization
    Result Init(int imgWidth, int imgHeight);
    // Inference frame image preprocessing
    Result Preprocess(cv::Mat& frame);
    // Inference frame picture
    Result Inference(aclmdlDataset*& inferenceOutput);
    // Inference output post-processing
    Result Postprocess(cv::Mat& frame, aclmdlDataset* modelOutput);
    // Release the requested resources
    void DestroyResource();
    
private:
    // Initializes the ACL resource
    Result InitResource();
    // Loading reasoning model
    Result InitModel(const char* omModelPath);
    Result CreateModelInputdDataset();
    // Get data from model inference output aclmdlDataset to local
    void* GetInferenceOutputItem(uint32_t& itemDataSize,
    aclmdlDataset* inferenceOutput, uint32_t idx);
    Result DrawBoundBoxToImage(vector<BBox>& detectionResults, cv::Mat& origImg);

    int32_t g_deviceId_;          // Device ID, default is 0
    aclrtContext g_context_;      // add
    aclrtStream g_stream_;        // add
    DvppProcess g_dvpp_;          // add
    ModelProcess g_model_;        // Inference model instance

    const char* g_modelPath_;     // Offline model file path
    uint32_t g_modelWidth_;       // The input width required by the model
    uint32_t g_modelHeight_;      // The model requires high input
    uint32_t g_imageDataSize_;    // Model input data size
    void*    g_imageDataBuf_;     // Model input data cache
    uint32_t g_imageInfoSize_;
    void*    g_imageInfoBuf_;
    aclrtRunMode g_runMode_;      // Run mode, which is whether the current application is running on atlas200DK or AI1
    bool g_isInited_;
};

#endif