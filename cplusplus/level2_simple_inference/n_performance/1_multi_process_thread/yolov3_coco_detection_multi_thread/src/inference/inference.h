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
#include "acl/acl.h"
#include "AclLiteModel.h"
#include "AclLiteImageProc.h"
#include "AclLiteThread.h"
#include "object_detection.h"

using namespace std;

/**
* ClassifyProcess
*/
class Inference : public AclLiteThread {
public:
    Inference(const string& modelPath,
              uint32_t modelWidth, uint32_t modelHeight);
    ~Inference();

    AclLiteError Init();
    AclLiteError Process(int msgId, shared_ptr<void> data);
private:
    AclLiteError CreateImageInfoBuffer();
    AclLiteError FrameImageProcess(shared_ptr<PreprocDataMsg> preprocData);
    AclLiteError Execute(vector<InferenceOutput>& inferenceOutput,
                       ImageData& image);

    void DestroyResource();
private:   
    AclLiteModel model_;
    uint32_t modelWidth_;
    uint32_t modelHeight_;
    uint32_t imageInfoSize_;
    void*    imageInfoBuf_;
    aclrtRunMode runMode_;
};

