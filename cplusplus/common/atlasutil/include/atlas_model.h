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

* File model_process.h
* Description: handle model process
*/
#pragma once
#include <iostream>
#include "atlas_utils.h"
#include "acl/acl.h"

class AtlasModel {
public:
    AtlasModel();
    AtlasModel(const std::string& modelPath);
    AtlasModel(void *modelAddr, size_t modelSize);

    ~AtlasModel();

    AtlasError Init();
    AtlasError Init(const std::string& modelPath);
    AtlasError Init(void *modelAddr, size_t modelSize);

    void DestroyResource();
    AtlasError CreateInput(void *input, uint32_t input1size);
    AtlasError CreateInput(void *input1, uint32_t input1size,
                       void* input2, uint32_t input2size);
    AtlasError CreateInput(std::vector<DataInfo>& inputData);
    AtlasError Execute(std::vector<InferenceOutput>& inferOutputs, 
                       void *data, uint32_t size);
    AtlasError Execute(std::vector<InferenceOutput>& inferOutputs);
    size_t GetModelInputSize(int index);
    void DestroyInput();

private:
    AtlasError LoadModelFromFile(const std::string& modelPath);
    AtlasError LoadModelFromMem();
    AtlasError CreateDesc();
    AtlasError CreateOutput();
    AtlasError AddDatasetBuffer(aclmdlDataset* dataset, 
                                void* buffer, uint32_t bufferSize);
    AtlasError GetOutputItem(InferenceOutput& out,
                             uint32_t idx);
    void Unload();    
    void DestroyDesc();
    void DestroyOutput();
    
private:
    std::string modelPath_;
    bool loadFlag_;  // model load flag
    uint32_t modelId_;
    void *modelMemPtr_;
    size_t modelMemSize_;
    void *modelWorkPtr_;
    size_t modelWorkSize_;
    void *modelWeightPtr_;
    size_t modelWeightSize_;
    aclmdlDesc *modelDesc_;
    aclmdlDataset *input_;
    aclmdlDataset *output_;
    aclrtRunMode runMode_;
    size_t outputsNum_;
    bool isReleased_;
};

