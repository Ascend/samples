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
#include "utils.h"
#include "acl/acl.h"
#include "model_process.h"
#include <memory>

/**
* SuperResolutionProcess
*/
class SuperResolutionProcess {
public:
    SuperResolutionProcess(uint8_t modelType);
    ~SuperResolutionProcess();

    Result init();
    Result preprocess(const std::string& imageFile);
    Result inference(aclmdlDataset*& inferenceOutput);
    Result postprocess(const std::string& origImageFile,
                       aclmdlDataset* modelOutput);
    Result init_model(const char* omModelPath);
    void destroy_model();

private:
    Result init_resource();
    void* get_inference_output_item(uint32_t& itemDataSize,
                                 aclmdlDataset* inferenceOutput);
    void save_image(const std::string& origImageFile, cv::Mat& imageBicubic, cv::Mat& imageSR);
    void destroy_resource();

private:
    int32_t deviceId_;
    aclrtContext context_;
    aclrtStream stream_;
    ModelProcess model_;

    const char* modelPath_;
    uint32_t modelWidth_;
    uint32_t modelHeight_;
    uint32_t outputWidth_;
    uint32_t outputHeight_;
    uint8_t modelType_;
    uint8_t upScale_;
    uint32_t inputDataSize_;
    void*    inputBuf_;
    aclrtRunMode runMode_;

    bool isInited_;
};

