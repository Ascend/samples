/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
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
*/

#ifndef YOLOV3_CARCOLOR_SAMPLE_INC_ACLLITEMODEL_H
#define YOLOV3_CARCOLOR_SAMPLE_INC_ACLLITEMODEL_H

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
    AclLiteError CreateInput(void *input, uint32_t inputSize);
    AclLiteError CreateInput(void *input1, uint32_t input1Size,
                             void* input2, uint32_t input2Size);
    AclLiteError CreateInput(std::vector<DataInfo>& inputData);
    AclLiteError Execute(std::vector<InferenceOutput>& inferOutputs,
                         void *data, uint32_t size);
    AclLiteError Execute(std::vector<InferenceOutput>& inferOutputs);
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
    bool g_loadFlag_;    // model load flag
    bool g_isReleased_;    // model release flag
    uint32_t g_modelId_;    // modelid
    size_t g_outputsNum_;
    size_t g_modelMemSize_;
    size_t g_modelWorkSize_;
    size_t g_modelWeightSize_;
    aclrtRunMode g_runMode_;
    void *g_modelMemPtr_;
    void *g_modelWorkPtr_;
    void *g_modelWeightPtr_;
    aclmdlDesc *g_modelDesc_;
    aclmdlDataset *g_input_;    // input dataset
    aclmdlDataset *g_output_;    // output dataset
    std::string g_modelPath_;    // model path
};

#endif