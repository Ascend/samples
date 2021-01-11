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
    * @brief initialize
    * @param [in] deviceId: id of the used device
    * @return process result
    */
    Result Init(int deviceId);

    /**
    * @brief destroy resource
    */
    void Finalize();

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
    * @brief get model output data
    * @return output dataset
    */
    const aclmdlDataset *GetModelOutputData();

    /**
    * @brief get model intput width and height
    * @return output dataset
    */
    Result GetModelInputWH(int &width, int &height);

private:
    uint32_t modelId_;
    bool loadFlag_;  // model load flag
    aclmdlDesc *modelDesc_;
    aclmdlDataset *input_;
    aclmdlDataset *output_;

    int deviceId_;
    aclrtContext context_;
    bool initFlag_;
};

