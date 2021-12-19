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
#include <memory>
#include "acl/acl.h"
#include "acllite/AclLiteModel.h"
#include "acllite/AclLiteUtils.h"
#include <opencv2/opencv.hpp>

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

    AclLiteModel model_;
    const char* modelPath_;
    uint32_t modelWidth_;
    uint32_t modelHeight_;
    uint32_t imageDataSize_;
    void*    imageDataBuf_;
    uint32_t imageInfoSize_;
    void*    imageInfoBuf_;
    aclrtRunMode runMode_;
    bool isInited_;
};
