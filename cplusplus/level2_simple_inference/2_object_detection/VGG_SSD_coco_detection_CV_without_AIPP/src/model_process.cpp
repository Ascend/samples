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

ModelProcess::ModelProcess():g_loadFlag(false), g_modelId(0), g_modelDesc(nullptr), g_input(nullptr), g_output(nullptr)
{
}

ModelProcess::~ModelProcess()
{
    Unload();
    DestroyDesc();
    DestroyInput();
    DestroyOutput();
}

Result ModelProcess::LoadModelFromFile(const char *modelPath)
{
    if (g_loadFlag) {
        ERROR_LOG("has already loaded a model");
        return FAILED;
    }

    // load model and get modelID.
    aclError ret = aclmdlLoadFromFile(modelPath, &g_modelId);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("load model from file failed, model file is %s", modelPath);
        return FAILED;
    }

    g_loadFlag = true;
    INFO_LOG("load model %s success", modelPath);
    return SUCCESS;
}

Result ModelProcess::CreateDesc()
{
    g_modelDesc = aclmdlCreateDesc();
    if (g_modelDesc == nullptr) {
        ERROR_LOG("create model description failed");
        return FAILED;
    }
    // get modelDesc(model description) by modelID
    aclError ret = aclmdlGetDesc(g_modelDesc, g_modelId);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("get model description failed");
        return FAILED;
    }

    INFO_LOG("create model description success");
    return SUCCESS;
}

void ModelProcess::DestroyDesc()
{
    if (g_modelDesc != nullptr) {
        (void)aclmdlDestroyDesc(g_modelDesc);
        g_modelDesc = nullptr;
    }
}

Result ModelProcess::CreateInput(void *inputDataBuffer, size_t bufferSize)
{
    g_input = aclmdlCreateDataset();
    if (g_input == nullptr) {
        ERROR_LOG("can't create dataset, create input failed");
        return FAILED;
    }

    aclDataBuffer* inputData = aclCreateDataBuffer(inputDataBuffer, bufferSize);
    if (inputData == nullptr) {
        ERROR_LOG("can't create data buffer, create input failed");
        return FAILED;
    }

    aclError ret = aclmdlAddDatasetBuffer(g_input, inputData);

    if (inputData == nullptr) {
        ERROR_LOG("can't add data buffer, create input failed");
        aclDestroyDataBuffer(inputData);
        inputData = nullptr;
        return FAILED;
    }
    return SUCCESS;
}

void ModelProcess::DestroyInput()
{
    if (g_input == nullptr) {
        return;
    }

    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(g_input); ++i) {
        aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(g_input, i);
        aclDestroyDataBuffer(dataBuffer);
    }
    aclmdlDestroyDataset(g_input);
    g_input = nullptr;
}

Result ModelProcess::CreateOutput()
{
    if (g_modelDesc == nullptr) {
        ERROR_LOG("no model description, create ouput failed");
        return FAILED;
    }

    g_output = aclmdlCreateDataset();
    if (g_output == nullptr) {
        ERROR_LOG("can't create dataset, create output failed");
        return FAILED;
    }
    size_t outputSize = aclmdlGetNumOutputs(g_modelDesc);

    for (size_t i = 0; i < outputSize; ++i) {
        size_t buffer_size = aclmdlGetOutputSizeByIndex(g_modelDesc, i);

        void *outputBuffer = nullptr;
        aclError ret = aclrtMalloc(&outputBuffer, buffer_size, ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("can't malloc buffer, size is %zu, create output failed", buffer_size);
            return FAILED;
        }

        aclDataBuffer* outputData = aclCreateDataBuffer(outputBuffer, buffer_size);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("can't create data buffer, create output failed");
            aclrtFree(outputBuffer);
            return FAILED;
        }

        ret = aclmdlAddDatasetBuffer(g_output, outputData);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("can't add data buffer, create output failed");
            aclrtFree(outputBuffer);
            aclDestroyDataBuffer(outputData);
            return FAILED;
        }
    }

    INFO_LOG("create model output success");
    return SUCCESS;
}

void ModelProcess::DestroyOutput()
{
    if (g_output == nullptr) {
        return;
    }

    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(g_output); ++i) {
        aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(g_output, i);
        void* data = aclGetDataBufferAddr(dataBuffer);
        (void)aclrtFree(data);
        (void)aclDestroyDataBuffer(dataBuffer);
    }

    (void)aclmdlDestroyDataset(g_output);
    g_output = nullptr;
}

Result ModelProcess::Execute()
{
    aclError ret = aclmdlExecute(g_modelId, g_input, g_output);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("execute model failed, modelId is %u", g_modelId);
        return FAILED;
    }

    INFO_LOG("model execute success");
    return SUCCESS;
}

void ModelProcess::Unload()
{
    if (!g_loadFlag) {
        WARN_LOG("no model had been loaded, unload failed");
        return;
    }

    aclError ret = aclmdlUnload(g_modelId);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("unload model failed, modelId is %u", g_modelId);
    }

    if (g_modelDesc != nullptr) {
        (void)aclmdlDestroyDesc(g_modelDesc);
        g_modelDesc = nullptr;
    }

    g_loadFlag = false;
    INFO_LOG("unload model success, modelId is %u", g_modelId);
}

aclmdlDataset *ModelProcess::GetModelOutputData()
{
    return g_output;
}
aclmdlDesc *ModelProcess::GetModelDesc()
{
    return g_modelDesc;
}
