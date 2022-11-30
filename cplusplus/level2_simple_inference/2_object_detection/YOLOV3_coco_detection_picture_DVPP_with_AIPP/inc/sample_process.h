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

#ifndef YOLOV3_COCO_DETECTION_PICTURE_DVPP_WITH_AIPP_INC_SAMPLE_PROCESS_H
#define YOLOV3_COCO_DETECTION_PICTURE_DVPP_WITH_AIPP_INC_SAMPLE_PROCESS_H

#pragma once
#include "utils.h"
#include "acl/acl.h"

class SampleProcess {
public:
    /**
    * @brief Constructor
    */
    SampleProcess();

    /**
    * @brief Destructor
    */
    virtual ~SampleProcess();

    /**
    * @brief init reousce
    * @return result
    */
    Result InitResource();

    /**
    * @brief sample process
    * @return result
    */
    Result Process();

    void DrawBoundBoxToImage(std::vector<BBox>& detectionResults, const std::string& origImagePath);

    void* GetInferenceOutputItem(uint32_t& itemDataSize, const aclmdlDataset* inferenceOutput, uint32_t idx);

    Result Postprocess(const aclmdlDataset* modelOutput, PicDesc &picDesc, int modelWidth, int modelHeight);

private:
    void DestroyResource();
    aclrtRunMode g_runMode_;
    int32_t g_deviceId_;
    aclrtContext g_context_;
    aclrtStream g_stream_;
};

#endif