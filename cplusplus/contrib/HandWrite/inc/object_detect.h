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
#include <iostream>
#include <mutex>
#include <unistd.h>
#include "presenter/agent/presenter_channel.h"
#include "presenter/agent/presenter_types.h"

#include "acl/acl.h"
#include "model_process.h"
#include "dvpp_process.h"

//add
#include<fstream>
#include<string>
#include<sstream>


using namespace std;
using namespace ascend::presenter;
/**
* ClassifyProcess
*/
class ObjectDetect {
public:
    ObjectDetect(const char* modelPath, 
                 uint32_t modelWidth, uint32_t modelHeight);
    ~ObjectDetect();

    Result Init();
    ImageResults Inference(ImageData& resizedImage, vector<string>& dict);
    Result Postprocess(ImageData& image, ImageResults& modelOutput);

    cv::Mat PreProcess(cv::Mat src);
    std::vector<cv::Rect> detect(cv::Mat img);
    string GetCharacterByLine(int n);

    void EncodeImage(vector<uint8_t>& encodeImg, cv::Mat& origImg);

    Result SendImage(Channel* channel, cv::Mat& jpegImage, vector<DetectionResult>& detRes);
private:
    Result InitResource();
    Result InitModel(const char* omModelPath);
    Result CreateImageInfoBuffer();
    void* GetInferenceOutputItem(uint32_t& itemDataSize,
                                 aclmdlDataset* inferenceOutput,
                                 uint32_t idx);
    void DrowBoundBoxToImage(vector<BBox>& detectionResults,
                             const string& origImagePath);
    void DestroyResource();
private:
    int32_t deviceId_;
    aclrtContext context_;
    aclrtStream stream_;
    uint32_t imageInfoSize_;
    void* imageInfoBuf_;
    ModelProcess model_;

    const char* modelPath_;
    uint32_t modelWidth_;
    uint32_t modelHeight_;
    uint32_t inputDataSize_;
    DvppProcess dvpp_;
    aclrtRunMode runMode_;

    bool isInited_;
    std::shared_ptr<Channel> chan_;
  //  std::shared_ptr<Channel> chan_;
};

