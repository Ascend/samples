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
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"
#include "AclLiteModel.h"
#include "AclLiteImageProc.h"

using namespace std;

/**
* ClassifyProcess
*/
class ObjectDetect {
public:
    ObjectDetect(const char* modelPath, 
                 uint32_t modelWidth, uint32_t modelHeight);
    ~ObjectDetect();

    AclLiteError init();
    AclLiteError preprocess_cv(const string& imageFile);
    AclLiteError preprocess(ImageData& resizedImage, ImageData& srcImage);
    AclLiteError inference(vector<InferenceOutput>& inferenceOutput, ImageData& resizedImage);
    AclLiteError postprocess(vector<InferenceOutput>& inferenceOutput, const string& imageFile, const ImageData& resizedImage);
private:
    AclLiteError init_resource();
    AclLiteError create_input(size_t inputDataSize);
    void save_image(const string& origImageFile, cv::Mat& image);
    void destroy_resource();

private:
    int32_t deviceId_;
    aclrtContext context_;
    aclrtStream stream_;
    uint32_t imageInfoSize_;
    void* imageInfoBuf_;
    AclLiteModel model_;

    const char* modelPath_;
    uint32_t modelWidth_;
    uint32_t modelHeight_;
    uint32_t imgOrignWidth_;
    uint32_t imgOrignHeight_;
    uint32_t inputDataSize_;
    AclLiteImageProc dvpp_;
    aclrtRunMode runMode_;

    bool isInited_;
    bool isDeviceSet_;
};

