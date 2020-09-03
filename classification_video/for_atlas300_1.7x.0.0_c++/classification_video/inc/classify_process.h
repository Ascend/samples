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
#include "ascenddk/presenter/agent/presenter_channel.h"

using namespace std;
using namespace ascend::presenter;

/**
* ClassifyProcess
*/
class ClassifyProcess {
public:
    ClassifyProcess(const char* modelPath, uint32_t modelWidth, uint32_t modelHeight);
    ~ClassifyProcess();
    //Inference initialization
    Result Init();
    //Inference frame picture preprocessing
    Result Preprocess(cv::Mat& frame);
    //Inference frame picture
    Result Inference(aclmdlDataset*& inferenceOutput);
    //Inference output post-processing
    Result Postprocess(cv::Mat& frame, aclmdlDataset* modelOutput);
    
private:
    //Initialize acl resources
    Result InitResource();
    //Load the inference model
    Result InitModel(const char* omModelPath);
    //Establish a connection with the presenter server
    Result OpenPresenterChannel();
    //Obtain data from model inference output aclmdlDataset to local
    void* GetInferenceOutputItem(uint32_t& itemDataSize,
                                 aclmdlDataset* inferenceOutput);
    //Use the parsed inference data to construct the inference result data structure sent to the presenter server
    void ConstructClassifyResult(vector<DetectionResult>& result,
                                 int classIdx, float score);
    //Serialize the frame image into a data stream
    void EncodeImage(vector<uint8_t>& encodeImg, cv::Mat& origImg);
    Result SendImage(std::vector<DetectionResult>& detectionResults,
                     cv::Mat& frame);
    //Release requested resources
    void DestroyResource();

private:
    int32_t deviceId_;  //Device id, default is 0
    ModelProcess model_; //Inference model example

    const char* modelPath_; //Offline model file path
    uint32_t modelWidth_;   //Input width required by the model
    uint32_t modelHeight_;  //Model requires high input
    uint32_t inputDataSize_; //Model input data size
    void*    inputBuf_;      //Model input data cache
    aclrtRunMode runMode_;   //Operating mode, that is, the current application is running on atlas200dk or AI1

    Channel* channel_;  //Channel connected to the presenter server
    bool isInited_;     //Initialization flag to prevent the inference instance from being initialized multiple times
};

