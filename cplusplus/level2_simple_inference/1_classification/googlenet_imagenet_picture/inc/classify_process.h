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

using namespace std;

/**
* ClassifyProcess
*/
class ClassifyProcess {
public:
    ClassifyProcess(const char* modelPath, uint32_t modelWidth, uint32_t modelHeight);
    ~ClassifyProcess();

    Result Init();
    Result Preprocess(const string& imageFile);
    Result Inference(aclmdlDataset*& inferenceOutput);
    Result Postprocess(const string& origImageFile,
                       aclmdlDataset* modelOutput);
private:
    Result InitResource();
    Result InitModel(const char* omModelPath);

    void* GetInferenceOutputItem(uint32_t& itemDataSize,
                                 aclmdlDataset* inferenceOutput);
    void LabelClassToImage(int classIdx, const string& imagePath);
    void DestroyResource();

private:
    int32_t deviceId_;
    aclrtContext context_;
    aclrtStream stream_;
    ModelProcess model_;

    const char* modelPath_;
    uint32_t modelWidth_;
    uint32_t modelHeight_;
    uint32_t inputDataSize_;
    void*    inputBuf_;
    aclrtRunMode runMode_;

    bool isInited_;
};

