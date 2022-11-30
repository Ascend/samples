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

#ifndef YOLOV3_COCO_DETECTION_PICTURE_WITH_FREETYPE_INC_OBJECT_DETECT_H
#define YOLOV3_COCO_DETECTION_PICTURE_WITH_FREETYPE_INC_OBJECT_DETECT_H

#pragma once
#include <memory>
#include "utils.h"
#include "acl/acl.h"
#include "model_process.h"
#include "dvpp_process.h"

using namespace std;

/**
* ClassifyProcess
*/
class ObjectDetect {
public:
    ObjectDetect(const char* modelPath,
                 uint32_t modelWidth, uint32_t modelHeight);
    ~ObjectDetect();

    Result Init();
    Result Preprocess(ImageData& resizedImage, ImageData& srcImage, ImageData& oriImage);
    Result Inference(aclmdlDataset*& inferenceOutput, ImageData& resizedImage);
    Result Postprocess(ImageData& image, aclmdlDataset* modelOutput,
                       const string& origImagePath);
private:
    Result InitResource();
    Result InitModel(const char* omModelPath);
    Result CreateImageInfoBuffer();
    void* GetInferenceOutputItem(uint32_t& itemDataSize,
                                 aclmdlDataset* inferenceOutput,
                                 uint32_t idx);
    void DrawBoundBoxToImage(ImageData& image,
                             vector<BBox>& detectionResults, const string& origImagePath);
    void DestroyResource();

private:
    int32_t g_deviceId_;
    aclrtContext g_context_;
    aclrtStream g_stream_;
    uint32_t g_imageInfoSize_;
    void* g_imageInfoBuf_;
    ModelProcess g_model_;

    const char* g_modelPath_;
    uint32_t g_modelWidth_;
    uint32_t g_modelHeight_;
    uint32_t inputDataSize_;
    DvppProcess g_dvpp_;
    aclrtRunMode g_runMode_;

    bool g_isInited_;
    bool g_isDeviceSet_;
};

#endif