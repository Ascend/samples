/**
* @file model_process.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#ifndef MODEL_PROCESS_H_
#define MODEL_PROCESS_H_

#include <iostream>
#include <vector>
#include "utils.h"
#include "acl/acl.h"

class ModelProcess {
public:
    /**
    * @brief Constructor
    */
    ModelProcess(aclrtStream &stream);

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
    * @brief model async execute
    * @return result
    */
    Result ExecuteAsync();

    /**
    * @brief output model execute result
    * @param [in] output: ouput dataset
    */
    static void OutputModelResult(aclmdlDataset *output);

    /**
    * @brief dump model output result to file
    * @param [in] output: ouput dataset
    */
    static void DumpModelOutputResult(aclmdlDataset *output);

    /**
    * @brief synchronize stream
    */
    Result SynchronizeStream();

    /**
    * @brief callback function
    * @param [in] arg: callback function parameter
    * @return result
    */
    static void CallBackFunc(void *arg);

    /**
    * @brief init memory pool
    */
    Result InitMemPool();

    /**
    * @brief destroy memory pool
    */
    void DestroyMemPool();

private:
    uint32_t modelId_;
    size_t modelWorkSize_; // model work memory buffer size
    size_t modelWeightSize_; // model weight memory buffer size
    void *modelWorkPtr_; // model work memory buffer
    void *modelWeightPtr_; // model weight memory buffer
    bool loadFlag_;  // model load flag
    aclmdlDesc *modelDesc_;
    aclrtStream stream_;
};

#endif // MODEL_PROCESS_H_