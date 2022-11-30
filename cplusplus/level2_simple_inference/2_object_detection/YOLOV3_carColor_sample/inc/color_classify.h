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

#ifndef YOLOV3_CARCOLOR_SAMPLE_INC_COLOR_CLASSIFY_H
#define YOLOV3_CARCOLOR_SAMPLE_INC_COLOR_CLASSIFY_H

#pragma once
#include <memory>
#include "acl/acl.h"
#include "AclLiteModel.h"
#include "AclLiteImageProc.h"
#include "object_detect.h"

/**
* ClassifyProcess
*/
class ColorClassify {
public:
    ColorClassify();
    ~ColorClassify();
    AclLiteError Init();
    AclLiteError PreProcess(ImageData& srcImage, std::vector<CarInfo> &carImgs, int& flag);
    AclLiteError Inference(std::vector<CarInfo> &carImgs, std::vector<InferenceOutput>& inferenceOutput);
    AclLiteError PostProcess(std::vector<InferenceOutput>& inferenceOutput,
                             std::vector<CarInfo>& carInfo, const std::string& origImagePath);
private:
    AclLiteError InitModelInput();
    AclLiteError Crop(std::vector<CarInfo> &carImgs, ImageData &orgImg);
    AclLiteError Resize(std::vector<CarInfo> &carImgs);
    int CopyOneBatchImages(uint8_t* buffer, uint32_t bufferSize,
                            std::vector<CarInfo> &carImgs, int batchIdx);
    int CopyImageData(uint8_t *buffer, uint32_t bufferSize, ImageData& image);
    void DrawResult(std::vector<CarInfo>& carInfo, const std::string& origImagePath);
    void DestroyResource();

private:
    AclLiteModel g_model_;
    const char* g_modelPath_;
    AclLiteImageProc g_dvpp_;
    aclrtRunMode g_runMode_;
    bool g_isInited_;
    bool g_isReleased_;

    int32_t g_batchSize_;
    uint32_t g_inputSize_;
    uint8_t* g_inputBuf_;
};

#endif