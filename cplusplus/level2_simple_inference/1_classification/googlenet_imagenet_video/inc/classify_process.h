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
#include "presenter/agent/presenter_channel.h"

using namespace std;
using namespace ascend::presenter;

/**
* ClassifyProcess
*/
class ClassifyProcess {
public:
    ClassifyProcess(const char* modelPath, uint32_t modelWidth, uint32_t modelHeight);
    ~ClassifyProcess();
    //init
    Result Init();
    //data frame preprocess
    Result Preprocess(cv::Mat& frame);
    //data inference
    Result Inference(aclmdlDataset*& inferenceOutput);
    //data postprocess
    Result Postprocess(cv::Mat& frame, aclmdlDataset* modelOutput);
    
private:
    //init presentagent channnel
    Result OpenPresentAgentChannel();
    //init acl resource
    Result InitResource();
    //load model
    Result InitModel(const char* omModelPath);
    //establish connection with present server
    Result OpenPresenterChannel();
    //get data from output dataset
    void* GetInferenceOutputItem(uint32_t& itemDataSize,
                                 aclmdlDataset* inferenceOutput);
    //analyse data & construct dst structure
    void ConstructClassifyResult(vector<DetectionResult>& result,
                                 int classIdx, float score);
    //turn frame into stream
    void EncodeImage(vector<uint8_t>& encodeImg, cv::Mat& origImg);
    Result SendImage(std::vector<DetectionResult>& detectionResults,
                     cv::Mat& frame);
    //free resource
    void DestroyResource();

private:
    int32_t deviceId_;  //default 0
    ModelProcess model_; //model instance

    const char* modelPath_; //offline model file path
    uint32_t modelWidth_;   //width model needed
    uint32_t modelHeight_;  //height model needed
    uint32_t inputDataSize_; //model input data size
    void*    inputBuf_;      //model input data
    aclrtRunMode runMode_;   

    Channel* channel_;  //presenter server channel
    bool isInited_;     //init flag
};

