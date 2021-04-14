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
    Result load_model_from_file_with_mem(const char *modelPath);

    /**
    * @brief unload model
    */
    void unload();

    /**
    * @brief create model desc
    * @return result
    */
    Result create_desc();

    /**
    * @brief destroy desc
    */
    void destroy_desc();

    /**
    * @brief create model input
    * @param [in] inputDataBuffer: input buffer
    * @param [in] bufferSize: input buffer size
    * @return result
    */
    Result create_input(void *inputDataBuffer, size_t bufferSize);

    /**
    * @brief destroy input resource
    */
    void destroy_input();

    /**
    * @brief create output buffer
    * @return result
    */
    Result create_output();

    /**
    * @brief destroy output resource
    */
    void destroy_output();

    /**
    * @brief model execute
    * @return result
    */
    Result execute();

    /**
    * @brief get model output data
    * @return output dataset
    */
    aclmdlDataset *get_model_output_data();

private:
    bool loadFlag_;  // model load flag
    uint32_t modelId_;
    void *modelMemPtr_;
    size_t modelMemSize_;
    void *modelWeightPtr_;
    size_t modelWeightSize_;
    aclmdlDesc *modelDesc_;
    aclmdlDataset *input_;
    aclmdlDataset *output_;
};

