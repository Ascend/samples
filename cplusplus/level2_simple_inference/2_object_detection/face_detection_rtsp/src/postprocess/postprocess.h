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
#include "atlas_error.h"
#include "dvpp_process.h"
#include "atlas_thread.h"

using namespace std;
using namespace ascend::presenter;

class Postprocess: public AtlasThread {
public:
    Postprocess(const string& configFile, bool display = false);
    ~Postprocess();

    AtlasError Init();
    AtlasError Process(int msgId, shared_ptr<void> data);

    void SetDisplay() { display_ = true; }    

private:
    AtlasError AnalyzeInferenceOutput(vector<DetectionResult>& detectResults,
                                      uint32_t imageWidth, uint32_t imageHeight,
                                      vector<InferenceOutput>& modelOutput);
    AtlasError InferOutputProcess(shared_ptr<InferOutputMsg> data);
    AtlasError SendImage(Channel* channel,
                         ImageData& jpegImage,vector<DetectionResult>& detRes);
    void PrintDetectResults(vector<DetectionResult>& detectResults, 
                            uint32_t channelId);

    void DestroyResource();
private:
    bool display_; 
    string   configFile_;
    Channel* presenterChan_;
    
};

