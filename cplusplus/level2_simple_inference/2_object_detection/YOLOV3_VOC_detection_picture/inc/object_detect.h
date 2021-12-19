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
#include "acllite/AclLiteModel.h"
#include "acllite/AclLiteImageProc.h"

/**
* ClassifyProcess
*/

struct BoundingBox {
    uint32_t lt_x;
    uint32_t lt_y;
    uint32_t rb_x;
    uint32_t rb_y;
    uint32_t attribute;
    float score;
};

class ObjectDetect {
public:
    ObjectDetect();
    ~ObjectDetect();
    AclLiteError Init();
    AclLiteError Preprocess(ImageData& resizedImage, ImageData& srcImage, aclrtRunMode RunMode);
    AclLiteError Inference(std::vector<InferenceOutput>& inferenceOutput, ImageData& resizedImage);
    AclLiteError Postprocess(ImageData& image, std::vector<InferenceOutput>& modelOutput,
                       const std::string& origImagePath);
private:
    void DrawBoundBoxToImage(std::vector<BBox>& detectionResults,
                             const std::string& origImagePath);
    void DestroyResource();

private:
    uint32_t imageInfoSize_;
    AclLiteModel model_;
    const char* modelPath_;
    uint32_t inputDataSize_;
    AclLiteImageProc dvpp_;
    aclrtRunMode runMode_;
    bool isInited_;
    bool isReleased_;
};
