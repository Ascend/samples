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

#ifndef YOLOV3_COCO_DETECTION_DYNAMIC_AIPP_INC_MODEL_PROCESS_H
#define YOLOV3_COCO_DETECTION_DYNAMIC_AIPP_INC_MODEL_PROCESS_H

#pragma once
#include <iostream>
#include "utils.h"
#include "acl/acl.h"

/**
* ModelProcess
*/
class ModelProcess {
public:
    /**
    * @brief Constructor
    */
    ModelProcess();

    /**
    * @brief Destructor
    */
    ~ModelProcess();

    /**
    * @brief load model from file with mem
    * @param [in] modelPath: model path
    * @return result
    */
    Result LoadModelFromFileWithMem(const char *modelPath);

    /**
    * @brief release all acl resource
    */
    void DestroyResource();

    /**
    * @brief unload model
    */
    void Unload();

    /**
    * @brief create model desc
    * @return result
    */
    Result CreateDesc();

    /**
    * @brief destroy desc
    */
    void DestroyDesc();

    /**
    * @brief Add input buffer to input dataset
    * @brief [in] dataset: dst dataset
    * @brief [in] buffer: src buffer
    * @brief [in] bufferSize: src bufferSize
    * @return result
    */
    Result AddDatasetBuffer(aclmdlDataset *dataset,
                            void* buffer, uint32_t bufferSize);

    /**
    * @brief create aipptensor
    * @brief [in] batch_size: batch number
    * @return aipp tensor
    */
    aclmdlAIPP* SetAIPPTensor(uint64_t batch_size);

    /**
    * @brief create aipptensor for opencv
    * @brief [in] batch_size: batch number
    * @return aipp tensor for opencv
    */
    aclmdlAIPP* SetAIPPTensorOpenCV(uint64_t batch_size);

    /**
    * @brief create model input
    * @param [in] inputDataBuffer: input buffer
    * @param [in] bufferSize: input buffer size
    * @return result
    */
    Result CreateInput(void *input1, uint32_t input1Size,
                       void* input2, uint32_t input2Size);

    /**
    * @brief create model input for opencv pic
    * @param [in] inputDataBuffer: input buffer
    * @param [in] bufferSize: input buffer size
    * @return result
    */
    Result CreateInputOpenCV(void* input1, uint32_t input1Size,
                             void* input2, uint32_t input2Size);

    /**
    * @brief destroy input resource
    */
    void DestroyInput();

    /**
    * @brief create output buffer
    * @return result
    */
    Result CreateOutput();

    /**
    * @brief destroy output resource
    */
    void DestroyOutput();

    /**
    * @brief model execute
    * @return result
    */
    Result Execute();

    /**
    * @brief get model output data
    * @return output dataset
    */
    aclmdlDataset *GetModelOutputData();

private:
    bool g_loadFlag_;  // model load flag
    uint32_t g_modelId_;
    void *g_modelMemPtr_;
    size_t g_modelMemSize_;
    void *g_modelWeightPtr_;
    size_t g_modelWeightSize_;
    aclmdlDesc *g_modelDesc_;
    aclmdlDataset *g_input_;
    aclmdlDataset *g_output_;
    void *g_inputAIPP_;
    uint32_t g_inputAIPPSize_;
    size_t g_aipp_index_;
    bool g_isReleased_;
};

#endif