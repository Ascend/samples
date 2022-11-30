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

#ifndef RESNET50_IMAGENET_DYNAMIC_DIMS_INC_MODEL_PROCESS_H
#define RESNET50_IMAGENET_DYNAMIC_DIMS_INC_MODEL_PROCESS_H

#include <iostream>
#include "utils.h"
#include "acl/acl.h"

class ModelProcess {
public:
    ModelProcess();

    virtual ~ModelProcess();

    Result GetInputSizeByIndex(const size_t index, size_t &inputSize);

    Result ModelSetDynamicInfo(int dims0, int dims1, int dims2, int dims3);

    void GetRunMode(aclrtRunMode runMode);

    Result LoadModel(const char *modelPath);

    void UnloadModel();

    Result CreateModelDesc();

    void DestroyModelDesc();

    Result CreateInput(void *inputDataBuffer, size_t bufferSize);

    void DestroyInput();

    Result CreateOutput();

    void DestroyOutput();

    Result Execute();

    void OutputModelResultSoftMax(int classNum, int batchSize);

    void OutputModelResult();

private:
    bool isDevice_;
    uint32_t modelId_;
    size_t modelWorkSize_;
    size_t modelWeightSize_;
    void *modelWorkPtr_;
    void *modelWeightPtr_;
    bool loadFlag_;
    aclmdlDesc *modelDesc_;
    aclmdlDataset *input_;
    aclmdlDataset *output_;
    aclmdlIODims currentDims_;
};
#endif
