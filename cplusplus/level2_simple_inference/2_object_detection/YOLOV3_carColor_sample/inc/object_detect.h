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
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc/types_c.h"
#include<opencv2/core/core.hpp>
#include "params.h"

/**
* ClassifyProcess
*/
class ObjectDetect {
public:
    ObjectDetect();
    ~ObjectDetect();
    AclLiteError Init();
    AclLiteError Preprocess(ImageData& resizedImage, ImageData& srcImage, ImageData& yuvImage);
    AclLiteError Inference(std::vector<InferenceOutput>& inferenceOutput, ImageData& resizedImage);
    AclLiteError Postprocess(ImageData& image, std::vector<InferenceOutput>& inferenceOutput,
                             std::vector<CarInfo>& carData);
    // void DrawCarBoxToImage(std::vector<CarInfo>& detectionResults,
    //                          const std::string& origImagePath);
private:
    void DestroyResource();
    AclLiteError InitModelInput();

private:
    uint32_t imageInfoSize_;
    void*    imageInfoBuf_;
    AclLiteModel model_;
    const char* modelPath_;
    AclLiteImageProc dvpp_;
    aclrtRunMode runMode_;
    bool isInited_;
    bool isReleased_;
};
