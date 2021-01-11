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
    ~ModelProcess();

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
    Result CreateDesc();

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
     * @brief set dynamic size
     * @param [in] dynamicInfo: dynmaic information
     * @return result
     */
    Result SetDynamicSize(const DynamicInfo &dynamicInfo);

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
     * @brief print model desc info
     * @param [in] dynamicType: dynamic type
     */
    void PrintModelDescInfo(DynamicType dynamicType);

    /**
     * @brief print model current output dims info
     */
    void PrintModelCurOutputDims();

private:
    /**
     * @brief set dynamic batch size
     * @param [in] batchSize: batch size
     * @return result
     */
    Result SetDynamicBatchSize(uint64_t batchSize);

    /**
     * @brief set dynamic HW size
     * @param [in] height: input height
     * @param [in] width: input width
     * @return result
     */
    Result SetDynamicHWSize(uint64_t height, uint64_t width);

    /**
     * @brief print model dynamic batch info
     */
    void PrintDynamicBatchInfo();

    /**
     * @brief print model dynamic hw info
     */
    void PrintDynamicHWInfo();

    uint32_t modelId_;
    void *modelWorkPtr_;
    size_t modelWorkSize_;
    void *modelWeightPtr_;
    size_t modelWeightSize_;
    bool loadFlag_;  // model load flag
    aclmdlDesc *modelDesc_;
    aclmdlDataset *input_;
    aclmdlDataset *output_;
};

