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
#include "acl/acl.h"
#include <memory>
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
    AclLiteError Preprocess(ImageData& srcImage, std::vector<CarInfo> &carImgs, int& flag);
    AclLiteError Inference(std::vector<CarInfo> &carImgs, std::vector<InferenceOutput>& inferenceOutput);
    AclLiteError Postprocess(std::vector<InferenceOutput>& inferenceOutput,
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
    AclLiteModel model_;
    const char* modelPath_;
    AclLiteImageProc dvpp_;
    aclrtRunMode runMode_;
    bool isInited_;
    bool isReleased_;

    int32_t batchSize_;
    uint32_t inputSize_;
    uint8_t* inputBuf_;
};