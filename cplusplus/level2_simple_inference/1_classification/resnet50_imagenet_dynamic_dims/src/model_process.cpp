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
#include <fstream>
#include <map>
#include <sstream>
#include <algorithm>
#include <functional>
#include "utils.h"
#include "model_process.h"

using namespace std;
namespace {
    const int DIM_COUNT = 4;
    const int DIM_0 = 0;
    const int DIM_1 = 1;
    const int DIM_2 = 2;
    const int DIM_3 = 3;
}
ModelProcess::ModelProcess() : g_modelId(0), g_modelWorkSize(0), g_modelWeightSize(0), g_modelWorkPtr(nullptr),
                               g_modelWeightPtr(nullptr), g_loadFlag(false), g_modelDesc(nullptr), g_input(nullptr), g_output(nullptr)
{
}

ModelProcess::~ModelProcess()
{
    UnloadModel();
    DestroyModelDesc();
    DestroyInput();
    DestroyOutput();
}
void ModelProcess::GetRunMode(aclrtRunMode runMode)
{
    g_isDevice = (runMode == ACL_DEVICE);
}

Result ModelProcess::LoadModel(const char *modelPath)
{
    if (g_loadFlag) {
        ERROR_LOG("model has already been loaded");
        return FAILED;
    }
    aclError ret = aclmdlQuerySize(modelPath, &g_modelWorkSize, &g_modelWeightSize);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("query model failed, model file is %s, errorCode is %d",
                  modelPath, static_cast<int32_t>(ret));
        return FAILED;
    }
    ret = aclrtMalloc(&g_modelWorkPtr, g_modelWorkSize, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("malloc buffer for work failed, require size is %zu, errorCode is %d",
                  g_modelWorkSize, static_cast<int32_t>(ret));
        return FAILED;
    }
    ret = aclrtMalloc(&g_modelWeightPtr, g_modelWeightSize, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("malloc buffer for weight failed, require size is %zu, errorCode is %d",
                  g_modelWeightSize, static_cast<int32_t>(ret));
        return FAILED;
    }
    ret = aclmdlLoadFromFileWithMem(modelPath, &g_modelId, g_modelWorkPtr,
                                    g_modelWorkSize, g_modelWeightPtr, g_modelWeightSize);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("load model from file failed, model file is %s, errorCode is %d",
                  modelPath, static_cast<int32_t>(ret));
        return FAILED;
    }

    g_loadFlag = true;
    INFO_LOG("load model %s success", modelPath);
    return SUCCESS;
}

Result ModelProcess::CreateModelDesc()
{
    g_modelDesc = aclmdlCreateDesc();
    if (g_modelDesc == nullptr) {
        ERROR_LOG("create model description failed");
        return FAILED;
    }

    aclError ret = aclmdlGetDesc(g_modelDesc, g_modelId);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("get model description failed, modelId is %u, errorCode is %d",
                  g_modelId, static_cast<int32_t>(ret));
        return FAILED;
    }

    INFO_LOG("create model description success");

    return SUCCESS;
}

void ModelProcess::DestroyModelDesc()
{
    if (g_modelDesc != nullptr) {
        (void)aclmdlDestroyDesc(g_modelDesc);
        g_modelDesc = nullptr;
    }
    INFO_LOG("destroy model description success");
}

Result ModelProcess::ModelSetDynamicInfo(int dims0, int dims1, int dims2, int dims3)
{
    size_t index;
    aclError ret = aclmdlGetInputIndexByName(g_modelDesc, ACL_DYNAMIC_TENSOR_NAME, &index);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("get input index by name[%s] failed, errorCode = %d.",
                  ACL_DYNAMIC_TENSOR_NAME, static_cast<int32_t>(ret));
        return FAILED;
    }
    g_currentDims.dimCount = DIM_COUNT;
    g_currentDims.dims[DIM_0] = dims0;
    g_currentDims.dims[DIM_1] = dims1;
    g_currentDims.dims[DIM_2] = dims2;
    g_currentDims.dims[DIM_3] = dims3;
    ret = aclmdlSetInputDynamicDims(g_modelId, g_input, index, &g_currentDims);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("set dynamic dims failed, errorCode = %d.", static_cast<int32_t>(ret));
        return FAILED;
    }
    return SUCCESS;
}
Result ModelProcess::Execute()
{
    aclError ret = aclmdlExecute(g_modelId, g_input, g_output);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("execute model failed, modelId is %u, errorCode is %d",
                  g_modelId, static_cast<int32_t>(ret));
        return FAILED;
    }

    INFO_LOG("model execute success");
    return SUCCESS;
}

Result ModelProcess::CreateInput(void *inputDataBuffer, size_t bufferSize)
{
    uint32_t dataNum = aclmdlGetNumInputs(g_modelDesc);
    if (g_modelDesc == nullptr) {
        ERROR_LOG("no model description, create input failed");
        return FAILED;
    }
    g_input = aclmdlCreateDataset();
    if (g_input == nullptr) {
        ERROR_LOG("can't create dataset, create input failed");
        return FAILED;
    }
    aclDataBuffer *inputData = aclCreateDataBuffer(inputDataBuffer, bufferSize);
    if (inputData == nullptr) {
        ERROR_LOG("can't create data buffer, create input failed");
        return FAILED;
    }
    aclError ret = aclmdlAddDatasetBuffer(g_input, inputData);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("add input dataset buffer failed, errorCode is %d", static_cast<int32_t>(ret));
        (void)aclDestroyDataBuffer(inputData);
        inputData = nullptr;
        return FAILED;
    }

    size_t dynamicIdx = 0;
    ret = aclmdlGetInputIndexByName(g_modelDesc, ACL_DYNAMIC_TENSOR_NAME, &dynamicIdx);
    if ((ret == ACL_SUCCESS) && (dynamicIdx == (dataNum - 1))) {
        size_t dataLen = aclmdlGetInputSizeByIndex(g_modelDesc, dynamicIdx);
        void *data = nullptr;
        ret = aclrtMalloc(&data, dataLen, ACL_MEM_MALLOC_HUGE_FIRST);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("malloc device memory failed, errorCode = %d.", static_cast<int32_t>(ret));
            return FAILED;
        }
        aclDataBuffer *dataBuf = aclCreateDataBuffer(data, dataLen);
        if (dataBuf == nullptr) {
            ERROR_LOG("Create data buffer error");
            return FAILED;
        }
        aclError ret = aclmdlAddDatasetBuffer(g_input, dataBuf);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("Add dataset buffer error %d", ret);
            ret = aclDestroyDataBuffer(dataBuf);
            if (ret != ACL_SUCCESS) {
                ERROR_LOG("Destroy dataset buffer error %d", ret);
            }
            dataBuf = nullptr;
            return FAILED;
        }
    }

    INFO_LOG("create model input success");
    return SUCCESS;
}
Result ModelProcess::GetInputSizeByIndex(const size_t index, size_t &inputSize)
{
    if (g_modelDesc == nullptr) {
        ERROR_LOG("no model description, create input failed");
        return FAILED;
    }
    inputSize = aclmdlGetInputSizeByIndex(g_modelDesc, index);
    return SUCCESS;
}
void ModelProcess::DestroyInput()
{
    if (g_input == nullptr) {
        return;
    }

    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(g_input); ++i) {
        aclDataBuffer *dataBuffer = aclmdlGetDatasetBuffer(g_input, i);
        (void)aclDestroyDataBuffer(dataBuffer);
    }
    (void)aclmdlDestroyDataset(g_input);
    g_input = nullptr;
    INFO_LOG("destroy model input success");
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
        size_t modelOutputSize = aclmdlGetOutputSizeByIndex(g_modelDesc, i);

        void *outputBuffer = nullptr;
        aclError ret = aclrtMalloc(&outputBuffer, modelOutputSize, ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("can't malloc buffer, size is %zu, create output failed, errorCode is %d",
                      modelOutputSize, static_cast<int32_t>(ret));
            return FAILED;
        }

        aclDataBuffer *outputData = aclCreateDataBuffer(outputBuffer, modelOutputSize);
        if (outputData == nullptr) {
            ERROR_LOG("can't create data buffer, create output failed");
            (void)aclrtFree(outputBuffer);
            return FAILED;
        }

        ret = aclmdlAddDatasetBuffer(g_output, outputData);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("can't add data buffer, create output failed, errorCode is %d",
                      static_cast<int32_t>(ret));
            (void)aclrtFree(outputBuffer);
            (void)aclDestroyDataBuffer(outputData);
            return FAILED;
        }
    }

    INFO_LOG("create model output success");

    return SUCCESS;
}

void ModelProcess::OutputModelResult()
{
    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(g_output); ++i) {
        aclDataBuffer *dataBuffer = aclmdlGetDatasetBuffer(g_output, i);
        void *data = aclGetDataBufferAddr(dataBuffer);
        uint32_t len = aclGetDataBufferSizeV2(dataBuffer);

        void *outHostData = nullptr;
        aclError ret = ACL_SUCCESS;
        float *outData = nullptr;
        if (!g_isDevice) {
            aclError ret = aclrtMallocHost(&outHostData, len);
            if (ret != ACL_SUCCESS) {
                ERROR_LOG("aclrtMallocHost failed, malloc len[%u], errorCode[%d]",
                          len, static_cast<int32_t>(ret));
                return;
            }

            // if app is running in host, need copy model output data from device to host
            ret = aclrtMemcpy(outHostData, len, data, len, ACL_MEMCPY_DEVICE_TO_HOST);
            if (ret != ACL_SUCCESS) {
                ERROR_LOG("aclrtMemcpy failed, errorCode[%d]", static_cast<int32_t>(ret));
                (void)aclrtFreeHost(outHostData);
                return;
            }
            INFO_LOG("aclrtMallocHost--------success------------");
            outData = reinterpret_cast<float *>(outHostData);
        } else {
            outData = reinterpret_cast<float *>(data);
        }
        int classNum = 1000;
        for (int i = 0; i < (len / sizeof(float) / classNum); i++) {
            map<float, unsigned int, greater<float>> resultMap;
            for (unsigned int j = 0; j < classNum; ++j) {
                resultMap[*outData] = j;
                outData++;
            }
            INFO_LOG("seq = %d---------------------", i);
            int cnt = 0;
            for (auto it = resultMap.begin(); it != resultMap.end(); ++it) {
                // print top 5
                if (++cnt > 5) {
                    break;
                }
                INFO_LOG("top %d: index[%d] value[%lf] cnt= %d", cnt, it->second, it->first, cnt);
            }
        }
        if (!g_isDevice) {
            ret = aclrtFreeHost(outHostData);
            if (ret != ACL_SUCCESS) {
                ERROR_LOG("aclrtFreeHost failed, errorCode[%d]", static_cast<int32_t>(ret));
                return;
            }
        }
    }

    INFO_LOG("output data success");
    return;
}

void ModelProcess::DestroyOutput()
{
    if (g_output == nullptr) {
        return;
    }

    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(g_output); ++i) {
        aclDataBuffer *dataBuffer = aclmdlGetDatasetBuffer(g_output, i);
        void *data = aclGetDataBufferAddr(dataBuffer);
        (void)aclrtFree(data);
        (void)aclDestroyDataBuffer(dataBuffer);
    }

    (void)aclmdlDestroyDataset(g_output);
    g_output = nullptr;
    INFO_LOG("destroy model output success");
}

void ModelProcess::UnloadModel()
{
    if (!g_loadFlag) {
        WARN_LOG("no model had been loaded, unload failed");
        return;
    }

    aclError ret = aclmdlUnload(g_modelId);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("unload model failed, modelId is %u, errorCode is %d",
                  g_modelId, static_cast<int32_t>(ret));
    }

    if (g_modelDesc != nullptr) {
        (void)aclmdlDestroyDesc(g_modelDesc);
        g_modelDesc = nullptr;
    }

    if (g_modelWorkPtr != nullptr) {
        (void)aclrtFree(g_modelWorkPtr);
        g_modelWorkPtr = nullptr;
        g_modelWorkSize = 0;
    }

    if (g_modelWeightPtr != nullptr) {
        (void)aclrtFree(g_modelWeightPtr);
        g_modelWeightPtr = nullptr;
        g_modelWeightSize = 0;
    }

    g_loadFlag = false;
    INFO_LOG("unload model success, modelId is %u", g_modelId);
    g_modelId = 0;
}
