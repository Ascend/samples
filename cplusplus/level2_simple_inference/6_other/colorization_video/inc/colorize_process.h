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
#include <vector>
#include <memory>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"
#include "acl/acl.h"
#include "acllite/AclLiteModel.h"
#include "presenter/agent/presenter_channel.h"

/**
* ColorizeProcess
*/
class ColorizeProcess {
public:
    ColorizeProcess(const char* modelPath, uint32_t modelWidth, uint32_t modelHeight);
    ~ColorizeProcess();

    AclLiteError Init();
    AclLiteError Preprocess(cv::Mat& frame);
    AclLiteError Inference(std::vector<InferenceOutput>& inferOutputs);
    AclLiteError Postprocess(cv::Mat& frame, std::vector<InferenceOutput>& modelOutput);
    
private:
    AclLiteError InitResource();
    AclLiteError CreateInput();
    AclLiteError OpenPresenterChannel();

    void ConstructClassifyResult(std::vector<ascend::presenter::DetectionResult>& result,
                                 int classIdx, float score);
    void EncodeImage(std::vector<uint8_t>& encodeImg, cv::Mat& origImg);
    AclLiteError SendImage(cv::Mat& image);
    void DestroyResource();

private:
    int32_t deviceId_;
    aclrtContext context_;
    aclrtStream stream_;
    AclLiteModel model_;

    const char* modelPath_;
    uint32_t modelWidth_;
    uint32_t modelHeight_;
    uint32_t inputDataSize_;
    void*    inputBuf_;
    aclrtRunMode runMode_;

    ascend::presenter::Channel* channel_;
    bool isInited_;
    std::shared_ptr<ascend::presenter::Channel> chan_;

};

