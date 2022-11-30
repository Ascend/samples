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

#include <iostream>
#include "model_process.h"
#include "utils.h"
using namespace std;

ModelProcess::ModelProcess()
    : g_loadFlag_(false), g_modelId_(0), g_modelWorkPtr_(nullptr), g_modelWorkSize_(0),
      g_modelWeightPtr_(nullptr), g_modelWeightSize_(0), g_modelDesc_(nullptr),
      g_input_(nullptr), g_output_(nullptr)
{
}

ModelProcess::~ModelProcess()
{
    UnloadModel();
    DestroyDesc();
    DestroyInput();
    DestroyOutput();
}

Result ModelProcess::LoadModel(const char *modelPath)
{
    if (g_loadFlag_) {
        ERROR_LOG("model has already been loaded");
        return FAILED;
    }

    aclError ret = aclmdlQuerySize(modelPath, &g_modelWorkSize_, &g_modelWeightSize_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("query model failed, model file is %s, errorCode is %d", modelPath, static_cast<int32_t>(ret));
        return FAILED;
    }

    // using ACL_MEM_MALLOC_HUGE_FIRST to malloc memory, huge memory is preferred to use
    // and huge memory can improve performance.
    ret = aclrtMalloc(&g_modelWorkPtr_, g_modelWorkSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("malloc buffer for work failed, require size is %zu, errorCode is %d",
            g_modelWorkSize_, static_cast<int32_t>(ret));
        return FAILED;
    }

    // using ACL_MEM_MALLOC_HUGE_FIRST to malloc memory, huge memory is preferred to use
    // and huge memory can improve performance.
    ret = aclrtMalloc(&g_modelWeightPtr_, g_modelWeightSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("malloc buffer for weight failed, require size is %zu, errorCode is %d",
            g_modelWeightSize_, static_cast<int32_t>(ret));
        return FAILED;
    }

    ret = aclmdlLoadFromFileWithMem(modelPath, &g_modelId_, g_modelWorkPtr_,
        g_modelWorkSize_, g_modelWeightPtr_, g_modelWeightSize_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("load model from file failed, model file is %s, errorCode is %d",
            modelPath, static_cast<int32_t>(ret));
        return FAILED;
    }

    g_loadFlag_ = true;
    INFO_LOG("load model %s success", modelPath);
    return SUCCESS;
}

Result ModelProcess::CreateDesc()
{
    g_modelDesc_ = aclmdlCreateDesc();
    if (g_modelDesc_ == nullptr) {
        ERROR_LOG("create model description failed");
        return FAILED;
    }

    aclError ret = aclmdlGetDesc(g_modelDesc_, g_modelId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("get model description failed");
        return FAILED;
    }

    INFO_LOG("create model description success");

    return SUCCESS;
}

void ModelProcess::DestroyDesc()
{
    if (g_modelDesc_ != nullptr) {
        (void)aclmdlDestroyDesc(g_modelDesc_);
        g_modelDesc_ = nullptr;
    }
    INFO_LOG("destroy model description success");
}

Result ModelProcess::CreateInput(void *input1, size_t input1Size, void *input2, size_t input2Size)
{
    vector<DataInfo> inputData = {{input1, input1Size}, {input2, input2Size}};

    uint32_t dataNum = aclmdlGetNumInputs(g_modelDesc_);
    if (dataNum == 0) {
        ERROR_LOG("Create input failed for no input data");
        return FAILED;
    }

    // om used in this sample has only one input
    if (g_modelDesc_ == nullptr) {
        ERROR_LOG("no model description, create input failed");
        return FAILED;
    }

    g_input_ = aclmdlCreateDataset();
    if (g_input_ == nullptr) {
        ERROR_LOG("can't create dataset, create input failed");
        return FAILED;
    }

    for (uint32_t i = 0; i < inputData.size(); i++) {
        size_t modelInputSize = aclmdlGetInputSizeByIndex(g_modelDesc_, i);
        if (modelInputSize != inputData[i].size) {
            ERROR_LOG("Input size verify failed "
                      "input[%d] size: %ld, provide size : %d",
                      i, modelInputSize, inputData[i].size);
        }

        aclDataBuffer *dataBuf = aclCreateDataBuffer(inputData[i].data, inputData[i].size);
        if (dataBuf == nullptr) {
            ERROR_LOG("can't create data buffer, create input failed");
            return FAILED;
        }

        aclError ret = aclmdlAddDatasetBuffer(g_input_, dataBuf);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("add input dataset buffer failed, errorCode is %d", static_cast<int32_t>(ret));
            (void)aclDestroyDataBuffer(dataBuf);
            dataBuf = nullptr;
            return FAILED;
        }
    }
    INFO_LOG("create model input success");
    return SUCCESS;
}

void ModelProcess::DestroyInput()
{
    if (g_input_ == nullptr) {
        return;
    }

    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(g_input_); ++i) {
        aclDataBuffer *dataBuffer = aclmdlGetDatasetBuffer(g_input_, i);
        (void)aclDestroyDataBuffer(dataBuffer);
    }
    (void)aclmdlDestroyDataset(g_input_);
    g_input_ = nullptr;
    INFO_LOG("destroy model input success");
}

Result ModelProcess::CreateOutput()
{
    if (g_modelDesc_ == nullptr) {
        ERROR_LOG("no model description, create ouput failed");
        return FAILED;
    }

    g_output_ = aclmdlCreateDataset();
    if (g_output_ == nullptr) {
        ERROR_LOG("can't create dataset, create output failed");
        return FAILED;
    }

    size_t outputSize = aclmdlGetNumOutputs(g_modelDesc_);
    for (size_t i = 0; i < outputSize; ++i) {
        size_t modelOutputSize = aclmdlGetOutputSizeByIndex(g_modelDesc_, i);

        void *outputBuffer = nullptr;
        aclError ret = aclrtMalloc(&outputBuffer, modelOutputSize, ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("can't malloc buffer, create output failed, size is %zu, errorCode is %d",
                modelOutputSize, static_cast<int32_t>(ret));
            return FAILED;
        }

        aclDataBuffer *outputData = aclCreateDataBuffer(outputBuffer, modelOutputSize);
        if (outputData == nullptr) {
            ERROR_LOG("can't create data buffer, create output failed");
            (void)aclrtFree(outputBuffer);
            return FAILED;
        }

        ret = aclmdlAddDatasetBuffer(g_output_, outputData);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("can't add data buffer, create output failed, errorCode is %d", static_cast<int32_t>(ret));
            (void)aclrtFree(outputBuffer);
            (void)aclDestroyDataBuffer(outputData);
            return FAILED;
        }
    }

    INFO_LOG("create model output success");
    return SUCCESS;
}

void ModelProcess::DestroyOutput()
{
    if (g_output_ == nullptr) {
        return;
    }

    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(g_output_); ++i) {
        aclDataBuffer *dataBuffer = aclmdlGetDatasetBuffer(g_output_, i);
        void* data = aclGetDataBufferAddr(dataBuffer);
        (void)aclrtFree(data);
        (void)aclDestroyDataBuffer(dataBuffer);
    }

    (void)aclmdlDestroyDataset(g_output_);
    g_output_ = nullptr;
    INFO_LOG("destroy model output success");
}

Result ModelProcess::Execute()
{
    aclError ret = aclmdlExecute(g_modelId_, g_input_, g_output_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("execute model failed, modelId is %u, errorCode is %d", g_modelId_, static_cast<int32_t>(ret));
        return FAILED;
    }

    INFO_LOG("model execute success, modelId is %u", g_modelId_);
    return SUCCESS;
}

void ModelProcess::UnloadModel()
{
    if (!g_loadFlag_) {
        WARN_LOG("no model had been loaded, unload failed");
        return;
    }

    aclError ret = aclmdlUnload(g_modelId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("unload model failed, modelId is %u, errorCode is %d", g_modelId_, static_cast<int32_t>(ret));
    }

    if (g_modelDesc_ != nullptr) {
        (void)aclmdlDestroyDesc(g_modelDesc_);
        g_modelDesc_ = nullptr;
    }

    if (g_modelWorkPtr_ != nullptr) {
        aclrtFree(g_modelWorkPtr_);
        g_modelWorkPtr_ = nullptr;
        g_modelWorkSize_ = 0;
    }

    if (g_modelWeightPtr_ != nullptr) {
        aclrtFree(g_modelWeightPtr_);
        g_modelWeightPtr_ = nullptr;
        g_modelWeightSize_ = 0;
    }

    g_loadFlag_ = false;
    INFO_LOG("unload model success, modelId is %u", g_modelId_);
}

const aclmdlDataset *ModelProcess::GetModelOutputData()
{
    return g_output_;
}

Result ModelProcess::GetModelInputWH(int &width, int &height)
{
    if (g_modelDesc_ == nullptr) {
        ERROR_LOG("no model description, get input hw failed");
        return FAILED;
    }
    // format of om used in this app is NHWC, dimsCout is 4
    // dims[0] is N, dims[1] is H, dims[2] is W dims[3] is C
    aclmdlIODims dims;
    // om used in this app has only one input
    aclError ret = aclmdlGetInputDims(g_modelDesc_, 0, &dims);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("get model input dims failed, modelId is %u errorCode is %d", g_modelId_, static_cast<int32_t>(ret));
        return FAILED;
    }
    if (dims.dimCount != 4) {
        ERROR_LOG("invalid dimsCount %zu, get input hw failed", dims.dimCount);
        return FAILED;
    }
    width = dims.dims[2];
    height = dims.dims[1];

    INFO_LOG("model input width %d, input height %d", width, height);

    return SUCCESS;
}