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
#include "acllite/AclLiteModel.h"
#include "acllite/AclLiteImageProc.h"

class ObjectDetect {
public:
    ObjectDetect();
    ~ObjectDetect();

    AclLiteError Init();
    AclLiteError Process(ImageData& image);
    void DestroyResource();

private:
    AclLiteError Inference(std::vector<InferenceOutput>& inferOutputs,
                         ImageData& resizedImage);
    void PostProcess(std::vector<ascend::presenter::DetectionResult>& detectResults, 
                     uint32_t imageWidth, uint32_t imageHeight,
                     std::vector<InferenceOutput>& modelOutput);
    AclLiteError SendImage(ImageData& jpegImage,
                         std::vector<ascend::presenter::DetectionResult>& detRes);
                             
private:
    AclLiteModel model_;
    AclLiteImageProc dvpp_;
    ascend::presenter::Channel* presenterChannel_;

    bool isInited_;
    bool isReleased_;
    uint32_t imageInfoSize_;
    void* imageInfoBuf_;
    aclrtRunMode runMode_;
};

