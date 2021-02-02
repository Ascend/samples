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

#include <iostream>
#include <mutex>
#include <unistd.h>
#include "presenter/agent/presenter_channel.h"
#include "presenter/agent/presenter_types.h"

#include "acl/acl.h"
#include "atlasutil/atlas_model.h"
#include "atlasutil/dvpp_process.h"

class FaceDetect {
public:
    FaceDetect();
    ~FaceDetect();

    AtlasError Init();
    AtlasError Process(ImageData& image);
    void DestroyResource();

private:
    AtlasError Inference(std::vector<InferenceOutput>& inferOutputs,
                         ImageData& resizedImage);
    void PostProcess(std::vector<ascend::presenter::DetectionResult>& detectResults, 
                     uint32_t imageWidth, uint32_t imageHeight,
                     std::vector<InferenceOutput>& modelOutput);
    AtlasError SendImage(ImageData& jpegImage,
                         std::vector<ascend::presenter::DetectionResult>& detRes);
                             
private:
    AtlasModel model_;
    DvppProcess dvpp_;    
    ascend::presenter::Channel* presenterChannel_;

    bool isInited_;
    bool isReleased_;
};

