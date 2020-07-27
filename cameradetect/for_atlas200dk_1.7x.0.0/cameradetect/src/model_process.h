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

#include "acl/acl.h"
#include "atlas_utils_common.h"
/**
* ModelProcess
*/
class ModelProcess {
public:
    /**
    * @brief Constructor
    */
    ModelProcess(const char* omModelPath);

    /**
    * @brief Destructor
    */
    ~ModelProcess();
    int Init();

    /**
    * @brief load model from file with mem
    * @param [in] modelPath: model path
    * @return result
    */
    int LoadModelFromFileWithMem();

    /**
    * @brief unload model
    */
    void Unload();

    /**
    * @brief create model desc
    * @return result
    */
    int CreateDesc();

    /**
    * @brief destroy desc
    */
    void DestroyDesc();

    /**
    * @brief create model input
    * @param [in] inputDataBuffer: input buffer
    * @param [in] bufferSize: input buffer size
    * @return result
    */
    int CreateInput(std::vector<DataBuffer>& inferenceInput);

    /**
    * @brief destroy input resource
    */
    void DestroyInput();

    /**
    * @brief create output buffer
    * @return result
    */
    int CreateOutput();

    /**
    * @brief destroy output resource
    */
    void DestroyOutput();

    /**
    * @brief model execute
    * @return result
    */
    int Execute(std::vector<DataBuffer>& inferenceOutput, 
                std::vector<DataBuffer>& inferenceInput);

    /**
    * @brief get model output result
    */
    void OutputModelResult(std::vector<DataBuffer>& inferenceOutput);

private:
    std::string modelPath_;
    std::string aclConfigPath_;

    uint32_t modelId_;
    size_t modelMemSize_;
    size_t modelWeightSize_;
    void *modelMemPtr_;
    void *modelWeightPtr_;
    bool loadFlag_;  // model load flag
    aclmdlDesc *modelDesc_;
    aclmdlDataset *input_;
    aclmdlDataset *output_;

    bool isInited_;
    bool isDevice_;
};

