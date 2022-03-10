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
#include "AclLiteUtils.h"
#include "acl/acl.h"

class AclLiteModel {
public:
    AclLiteModel(const std::string& modelPath);

    ~AclLiteModel();

    AclLiteError Init();
    void DestroyResource();
    AclLiteError CreateInput(void *input, uint32_t inputsize);
    AclLiteError CreateInput(void *input1, uint32_t input1size,
                             void* input2, uint32_t input2size);
    AclLiteError CreateInput(std::vector<DataInfo>& inputData);
    AclLiteError Execute(std::vector<InferenceOutput>& inferOutputs, 
                         void *data, uint32_t size);
    AclLiteError Execute(std::vector<InferenceOutput>& inferOutputs);
    size_t GetModelInputSize(int index);
    void DestroyInput();

private:
    AclLiteError LoadModelFromFile(const std::string& modelPath);
    AclLiteError SetDesc();
    AclLiteError CreateOutput();
    AclLiteError AddDatasetBuffer(aclmdlDataset* dataset, 
                                  void* buffer, uint32_t bufferSize);
    AclLiteError GetOutputItem(InferenceOutput& out,
                               uint32_t idx);
    void Unload();    
    void DestroyDesc();
    void DestroyOutput();
    
private:
    bool loadFlag_;    // model load flag
    bool isReleased_;    // model release flag
    uint32_t modelId_;    //modelid
    size_t outputsNum_;
    size_t modelMemSize_;
    size_t modelWorkSize_;
    size_t modelWeightSize_;
    aclrtRunMode runMode_;
    void *modelMemPtr_;
    void *modelWorkPtr_;
    void *modelWeightPtr_;
    aclmdlDesc *modelDesc_;
    aclmdlDataset *input_;    //input dataset
    aclmdlDataset *output_;    //output dataset
    std::string modelPath_;    //model path
};