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

#ifndef YOLOV4_COCO_DETECTION_PICTURE_INC_MODEL_PROCESS_H
#define YOLOV4_COCO_DETECTION_PICTURE_INC_MODEL_PROCESS_H

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
    uint32_t g_modelId_;
    bool g_loadFlag_;  // model load flag
    aclmdlDesc *g_modelDesc_;
    aclmdlDataset *g_input_;
    aclmdlDataset *g_output_;
    int g_deviceId_;
    aclrtContext g_context_;
    bool g_initFlag_;
};

#endif