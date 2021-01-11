/**
* @file model_process.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#pragma once
#include <iostream>
#include "utils.h"
#include "acl/acl.h"

class ModelProcess {
public:
    /**
    * @brief Constructor
    */
    ModelProcess();

    /**
    * @brief Destructor
    */
    virtual ~ModelProcess();

    /**
    * @brief load model
    * @param [in] modelPath: model path
    * @return result
    */
    Result LoadModel(const char *modelPath);

    /**
    * @brief unload model
    */
    void UnloadModel();

    /**
    * @brief create model desc
    * @return result
    */
    Result CreateModelDesc();

    /**
    * @brief destroy desc
    */
    void DestroyModelDesc();

    /**
    * @get input size by index
    * @param [in] index: input index
    * @param [out] inputSize: input size of index
    * @return result
    */
    Result GetInputSizeByIndex(const size_t index, size_t &inputSize);

    /**
    * @brief create model input
    * @param [in] inputDataBuffer: input buffer
    * @param [in] bufferSize: input buffer size
    * @return result
    */
    Result CreateInput(void *inputDataBuffer, size_t bufferSize);

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
    * @brief dump model output result to file
    */
    void DumpModelOutputResult();

    /**
    * @brief print model output result
    */
    void OutputModelResult();

private:
    uint32_t modelId_;
    size_t modelWorkSize_; // model work memory buffer size
    size_t modelWeightSize_; // model weight memory buffer size
    void *modelWorkPtr_; // model work memory buffer
    void *modelWeightPtr_; // model weight memory buffer
    bool loadFlag_;  // model load flag
    aclmdlDesc *modelDesc_;
    aclmdlDataset *input_;
    aclmdlDataset *output_;
};

