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
#include "acl/ops/acl_dvpp.h"

using namespace std;

ModelProcess::ModelProcess()
    : g_loadFlag_(false), g_modelId_(0), g_modelDesc_(nullptr),
      g_input_(nullptr), g_output_(nullptr),
      g_deviceId_(-1), g_initFlag_(false) {
}

ModelProcess::~ModelProcess()
{
    Finalize();
}

Result ModelProcess::Init(int deviceId)
{
    // acl initialize
    aclError ret = aclInit(nullptr);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclInit failed, errorCode is %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    g_initFlag_ = true;

    // open device
    ret = aclrtSetDevice(deviceId);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclrtSetDevice failed, errorCode is %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    g_deviceId_ = deviceId;

    // create context
    ret = aclrtCreateContext(&g_context_, g_deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclrtCreateContext failed, errorCode is %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    // check running mode
    aclrtRunMode runMode;
    ret = aclrtGetRunMode(&runMode);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclrtGetRunMode failed, errorCode is %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    bool isDeviceSide = (runMode == ACL_DEVICE);
    RunStatus::SetDeviceStatus(isDeviceSide);

    INFO_LOG("init model process success.");
    return SUCCESS;
}

void ModelProcess::Finalize()
{
    UnloadModel();
    DestroyInput();
    DestroyOutput();

    if (g_context_ != nullptr) {
        aclrtDestroyContext(g_context_);
        g_context_ = nullptr;
    }

    if (g_deviceId_ >= 0) {
        (void) aclrtResetDevice(g_deviceId_);
        g_deviceId_ = -1;
    }

    if (g_initFlag_) {
        (void) aclFinalize();
        g_initFlag_ = false;
    }
    INFO_LOG("finalize model process success.");
}

Result ModelProcess::LoadModel(const char *modelPath)
{
    if (g_loadFlag_) {
        ERROR_LOG("model has already been loaded");
        return FAILED;
    }

    aclError ret = aclmdlLoadFromFile(modelPath, &g_modelId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("load model from file failed, model file is %s, errorCode is %d",
                  modelPath, static_cast<int32_t>(ret));
        return FAILED;
    }

    g_modelDesc_ = aclmdlCreateDesc();
    if (g_modelDesc_ == nullptr) {
        ERROR_LOG("create model description failed");
        (void) aclmdlUnload(g_modelId_);
        return FAILED;
    }

    ret = aclmdlGetDesc(g_modelDesc_, g_modelId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("get model description failed");
        (void) aclmdlDestroyDesc(g_modelDesc_);
        g_modelDesc_ = nullptr;
        (void) aclmdlUnload(g_modelId_);
        return FAILED;
    }

    g_loadFlag_ = true;
    INFO_LOG("load model %s success", modelPath);
    return SUCCESS;
}

void ModelProcess::UnloadModel()
{
    if (!g_loadFlag_) {
        return;
    }

    aclError ret = aclmdlUnload(g_modelId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("unload model failed, modelId is %u, errorCode is %d", g_modelId_, static_cast<int32_t>(ret));
    }

    if (g_modelDesc_ != nullptr) {
        (void) aclmdlDestroyDesc(g_modelDesc_);
        g_modelDesc_ = nullptr;
    }

    g_loadFlag_ = false;
    INFO_LOG("unload model success, modelId is %u", g_modelId_);
}

Result ModelProcess::CreateInput(void *inputDataBuffer, size_t bufferSize)
{
    // om used in this sample has only one input
    if (g_modelDesc_ == nullptr) {
        ERROR_LOG("no model description, create input failed");
        (void) acldvppFree(inputDataBuffer);
        return FAILED;
    }
    size_t modelInputSize = aclmdlGetInputSizeByIndex(g_modelDesc_, 0);
    if (bufferSize != modelInputSize) {
        ERROR_LOG("input image size[%zu] is not equal to model input size[%zu]", bufferSize, modelInputSize);
        (void) acldvppFree(inputDataBuffer);
        return FAILED;
    }

    g_input_ = aclmdlCreateDataset();
    if (g_input_ == nullptr) {
        ERROR_LOG("can't create dataset, create input failed");
        (void) acldvppFree(inputDataBuffer);
        return FAILED;
    }

    aclDataBuffer *inputData = aclCreateDataBuffer(inputDataBuffer, bufferSize);
    if (inputData == nullptr) {
        ERROR_LOG("can't create data buffer, create input failed");
        (void) acldvppFree(inputDataBuffer);
        return FAILED;
    }

    aclError ret = aclmdlAddDatasetBuffer(g_input_, inputData);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("add input dataset buffer failed, errorCode is %d", static_cast<int32_t>(ret));
        (void) aclDestroyDataBuffer(inputData);
        inputData = nullptr;
        (void) acldvppFree(inputDataBuffer);
        return FAILED;
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
        void *data = aclGetDataBufferAddr(dataBuffer);
        (void) acldvppFree(data);
        (void) aclDestroyDataBuffer(dataBuffer);
    }
    (void) aclmdlDestroyDataset(g_input_);
    g_input_ = nullptr;
    INFO_LOG("destroy model input success");
}

Result ModelProcess::CreateOutput()
{
    if (g_modelDesc_ == nullptr) {
        ERROR_LOG("no model description, create output failed");
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
            (void) aclrtFree(outputBuffer);
            return FAILED;
        }

        ret = aclmdlAddDatasetBuffer(g_output_, outputData);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("can't add data buffer, create output failed, errorCode is %d", static_cast<int32_t>(ret));
            (void) aclrtFree(outputBuffer);
            (void) aclDestroyDataBuffer(outputData);
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
        void *data = aclGetDataBufferAddr(dataBuffer);
        (void) aclrtFree(data);
        (void) aclDestroyDataBuffer(dataBuffer);
    }

    (void) aclmdlDestroyDataset(g_output_);
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

const aclmdlDataset *ModelProcess::GetModelOutputData() {
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
        ERROR_LOG("get model input dims failed, errorCode is %d", static_cast<int32_t>(ret));
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