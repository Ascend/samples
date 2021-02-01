/*
 * Copyright(C) 2020. Huawei Technologies Co.,Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MODELPROCSS_H
#define MODELPROCSS_H

#include <cstdio>
#include <vector>
#include <unordered_map>
#include <mutex>
#include "acl/acl.h"
#include "CommonDataType/CommonDataType.h"
#include "Log/Log.h"
#include "ErrorCode/ErrorCode.h"

// Class of model inference
class ModelProcess {
public:

    // Construct a new Model process object for model in the device
    ModelProcess(const int deviceId, const std::string& modelName) : deviceId_(deviceId), modelName_(modelName){};
    ModelProcess();
    ~ModelProcess();

    int init(std::string modelPath);
    int deinit();

    APP_ERROR InputBufferWithSizeMalloc(aclrtMemMallocPolicy policy = ACL_MEM_MALLOC_HUGE_FIRST);
    APP_ERROR OutputBufferWithSizeMalloc(aclrtMemMallocPolicy policy = ACL_MEM_MALLOC_HUGE_FIRST);
    int ModelInference(std::vector<void *> &inputBufs, std::vector<size_t> &inputSizes, std::vector<void *> &ouputBufs,
                       std::vector<size_t> &outputSizes, size_t dynamicBatchSize = 0);
    int ModelInferDynamicHW(const std::vector<void *> &inputBufs, const std::vector<size_t> &inputSizes,
                            const std::vector<void *> &ouputBufs, const std::vector<size_t> &outputSizes);
    aclmdlDesc *GetModelDesc();
    size_t GetModelNumInputs();
    size_t GetModelNumOutputs();
    size_t GetModelInputSizeByIndex(const size_t &i);
    size_t GetModelOutputSizeByIndex(const size_t &i);
    void ReleaseModelBuffer(std::vector<void *> &modelBuffers) const;
    void SetModelWH(uint32_t width, uint32_t height);

    std::vector<void *> inputBuffers_ = {};
    std::vector<size_t> inputSizes_ = {};
    std::vector<void *> outputBuffers_ = {};
    std::vector<size_t> outputSizes_ = {};

private:
    aclmdlDataset *CreateAndFillDataset(const std::vector<void *> &bufs, const std::vector<size_t> &sizes);
    void DestroyDataset(aclmdlDataset *dataset);

    std::mutex mtx_ = {};
    int deviceId_ = 0; // Device used
    std::string modelName_ = "";
    uint32_t modelId_ = 0; // Id of import model
    uint32_t modelWidth_ = 0;
    uint32_t modelHeight_ = 0;
    void *modelDevPtr_ = nullptr;
    size_t modelDevPtrSize_ = 0;
    void *weightDevPtr_ = nullptr;
    size_t weightDevPtrSize_ = 0;
    aclrtContext contextModel_ = nullptr;
    std::shared_ptr<aclmdlDesc> modelDesc_ = nullptr;
    bool isDeInit_ = false;
};

#endif
