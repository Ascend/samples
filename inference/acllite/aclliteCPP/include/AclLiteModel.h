/**
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2022. All rights reserved.
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
#ifndef ACLLITE_MODEL_H
#define ACLLITE_MODEL_H
#pragma once
#include <iostream>
#include "AclLiteUtils.h"
#include "acl/acl.h"

class AclLiteModel {
public:
    AclLiteModel();
    AclLiteModel(const std::string& modelPath);
    AclLiteModel(void *modelAddr, size_t modelSize);

    ~AclLiteModel();

    AclLiteError Init();
    AclLiteError Init(const std::string& modelPath);
    AclLiteError Init(void *modelAddr, size_t modelSize);
    void DestroyResource();
    /**
    * @brief create model input (scenario: model with one input)
    * @param [in]: input: model input data
    * @param [in]: inputsize: Model input data size
    * @return AclLiteError ACLLITE_OK: Created successfully
    * Other: Failed to create
    */
    AclLiteError CreateInput(void *input, uint32_t inputsize);
    /**
    * @brief Create model inputs (scenario: model with two inputs)
    * @param [in]: input1: the first input data of the model
    * @param [in]: input1size: The size of the first input data of the model
    * @param [in]: input2: the second input data of the model
    * @param [in]: input2size: The second input data size of the model
    * @return AclLiteError ACLLITE_OK: Created successfully
    * Other: Failed to create
    */
    AclLiteError CreateInput(void *input1, uint32_t input1size,
                             void* input2, uint32_t input2size);
    /**
    * @brief Create Model Inputs (Scenario: Model with Multiple Inputs)
    * @param [in]: inputData: model input data vector
    * @return AclLiteError ACLLITE_OK: Created successfully
    * Other: Failed to create
    */
    AclLiteError CreateInput(std::vector<DataInfo>& inputData);
    /**
    * @brief Execute model inference.
    * This interface is for the scenario where the model has only one input.
    * The second and third parameters are used to construct the model input
    * before sending it to inference.
    * It supports the dynamic batch feature and is disabled by default.
    * @param [in]: inferOutputs: model inference results
    * @param [in]: data: model input data
    * @param [in]: size: model input data size
    * @param [in]: batchsize: The number of batches for
    * a single inference of the dynamic batch model
    * @return AclLiteError ACLLITE_OK: Inference successfully
    * Other: Inference failed
    */
    AclLiteError Execute(std::vector<InferenceOutput>& inferOutputs,
                         void *data, uint32_t size, uint32_t batchsize = 0);
    /**
    * @brief Execute model inference.
    * @param [in]: inferOutputs: model inference results
    * @return AclLiteError ACLLITE_OK: Inference successfully
    * Other: Inference failed
    */
    AclLiteError Execute(std::vector<InferenceOutput>& inferOutputs);
    /**
    * @brief Get the model input data size
    * @param [in]: index: index. The mark is the first input of the model.
    * The index starts from 0
    * @return Return value model input data size:
    * >0: Getting succeeded
    * Other: Failed to get
    */
    size_t GetModelInputSize(int index);
    AclLiteError GetModelOutputInfo(std::vector<ModelOutputInfo>& modelOutputInfo);
    void DestroyInput();

private:
    int SetDynamicBatchSize(uint64_t batchSize);
    AclLiteError LoadModelFromFile(const std::string& modelPath);
    AclLiteError LoadModelFromMem();
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
    uint32_t modelId_;    // modelid
    size_t outputsNum_;
    size_t modelMemSize_;
    size_t modelWorkSize_;
    size_t modelWeightSize_;
    aclrtRunMode runMode_;
    void *modelMemPtr_;
    void *modelWorkPtr_;
    void *modelWeightPtr_;
    aclmdlDesc *modelDesc_;
    aclmdlDataset *input_;    // input dataset
    aclmdlDataset *output_;    // output dataset
    std::string modelPath_;    // model path
};
#endif