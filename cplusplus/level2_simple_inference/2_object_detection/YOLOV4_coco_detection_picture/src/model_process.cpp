/**
* @file model_process.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include "model_process.h"
#include <iostream>
#include "utils.h"
#include "acl/ops/acl_dvpp.h"

using namespace std;

ModelProcess::ModelProcess() : loadFlag_(false), modelId_(0), modelDesc_(nullptr), input_(nullptr), output_(nullptr),
                               deviceId_(-1), initFlag_(false) {
}

ModelProcess::~ModelProcess() {
    Finalize();
}

Result ModelProcess::Init(int deviceId) {
    // acl initialize
    aclError ret = aclInit(nullptr);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("aclInit failed, errorCode is %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    initFlag_ = true;

    // open device
    ret = aclrtSetDevice(deviceId);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("aclrtSetDevice failed, errorCode is %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    deviceId_ = deviceId;

    // create context
    ret = aclrtCreateContext(&context_, deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("aclrtCreateContext failed, errorCode is %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    // check running mode
    aclrtRunMode runMode;
    ret = aclrtGetRunMode(&runMode);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("aclrtGetRunMode failed, errorCode is %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    bool isDeviceSide = (runMode == ACL_DEVICE);
    RunStatus::SetDeviceStatus(isDeviceSide);

    INFO_LOG("init model process success.");
    return SUCCESS;
}

void ModelProcess::Finalize() {
    UnloadModel();

    DestroyInput();
    DestroyOutput();

    if (context_ != nullptr) {
        aclrtDestroyContext(context_);
        context_ = nullptr;
    }

    if (deviceId_ >= 0) {
        (void) aclrtResetDevice(deviceId_);
        deviceId_ = -1;
    }

    if (initFlag_) {
        (void) aclFinalize();
        initFlag_ = false;
    }
    INFO_LOG("finalize model process success.");
}

Result ModelProcess::LoadModel(const char *modelPath) {
    if (loadFlag_) {
        ERROR_LOG("model has already been loaded");
        return FAILED;
    }

    aclError ret = aclmdlLoadFromFile(modelPath, &modelId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("load model from file failed, model file is %s, errorCode is %d",
                  modelPath, static_cast<int32_t>(ret));
        return FAILED;
    }

    modelDesc_ = aclmdlCreateDesc();
    if (modelDesc_ == nullptr) {
        ERROR_LOG("create model description failed");
        (void) aclmdlUnload(modelId_);
        return FAILED;
    }

    ret = aclmdlGetDesc(modelDesc_, modelId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("get model description failed");
        (void) aclmdlDestroyDesc(modelDesc_);
        modelDesc_ = nullptr;
        (void) aclmdlUnload(modelId_);
        return FAILED;
    }

    loadFlag_ = true;
    INFO_LOG("load model %s success", modelPath);
    return SUCCESS;
}

void ModelProcess::UnloadModel() {
    if (!loadFlag_) {
        return;
    }

    aclError ret = aclmdlUnload(modelId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("unload model failed, modelId is %u, errorCode is %d", modelId_, static_cast<int32_t>(ret));
    }

    if (modelDesc_ != nullptr) {
        (void) aclmdlDestroyDesc(modelDesc_);
        modelDesc_ = nullptr;
    }

    loadFlag_ = false;
    INFO_LOG("unload model success, modelId is %u", modelId_);
}

Result ModelProcess::CreateInput(void *inputDataBuffer, size_t bufferSize) {
    // om used in this sample has only one input
    if (modelDesc_ == nullptr) {
        ERROR_LOG("no model description, create input failed");
        (void) acldvppFree(inputDataBuffer);
        return FAILED;
    }
    size_t modelInputSize = aclmdlGetInputSizeByIndex(modelDesc_, 0);
    if (bufferSize != modelInputSize) {
        ERROR_LOG("input image size[%zu] is not equal to model input size[%zu]", bufferSize, modelInputSize);
        (void) acldvppFree(inputDataBuffer);
        return FAILED;
    }

    input_ = aclmdlCreateDataset();
    if (input_ == nullptr) {
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

    aclError ret = aclmdlAddDatasetBuffer(input_, inputData);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("add input dataset buffer failed, errorCode is %d", static_cast<int32_t>(ret));
        (void) aclDestroyDataBuffer(inputData);
        inputData = nullptr;
        (void) acldvppFree(inputDataBuffer);
        return FAILED;
    }
    INFO_LOG("create model input success");
    return SUCCESS;
}

void ModelProcess::DestroyInput() {
    if (input_ == nullptr) {
        return;
    }

    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(input_); ++i) {
        aclDataBuffer *dataBuffer = aclmdlGetDatasetBuffer(input_, i);
        void *data = aclGetDataBufferAddr(dataBuffer);
        (void) acldvppFree(data);
        (void) aclDestroyDataBuffer(dataBuffer);
    }
    (void) aclmdlDestroyDataset(input_);
    input_ = nullptr;
    INFO_LOG("destroy model input success");
}

Result ModelProcess::CreateOutput() {
    if (modelDesc_ == nullptr) {
        ERROR_LOG("no model description, create output failed");
        return FAILED;
    }

    output_ = aclmdlCreateDataset();
    if (output_ == nullptr) {
        ERROR_LOG("can't create dataset, create output failed");
        return FAILED;
    }

    size_t outputSize = aclmdlGetNumOutputs(modelDesc_);
    for (size_t i = 0; i < outputSize; ++i) {
        size_t modelOutputSize = aclmdlGetOutputSizeByIndex(modelDesc_, i);

        void *outputBuffer = nullptr;
        aclError ret = aclrtMalloc(&outputBuffer, modelOutputSize, ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != ACL_ERROR_NONE) {
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

        ret = aclmdlAddDatasetBuffer(output_, outputData);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("can't add data buffer, create output failed, errorCode is %d", static_cast<int32_t>(ret));
            (void) aclrtFree(outputBuffer);
            (void) aclDestroyDataBuffer(outputData);
            return FAILED;
        }
    }

    INFO_LOG("create model output success");
    return SUCCESS;
}

void ModelProcess::DestroyOutput() {
    if (output_ == nullptr) {
        return;
    }

    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(output_); ++i) {
        aclDataBuffer *dataBuffer = aclmdlGetDatasetBuffer(output_, i);
        void *data = aclGetDataBufferAddr(dataBuffer);
        (void) aclrtFree(data);
        (void) aclDestroyDataBuffer(dataBuffer);
    }

    (void) aclmdlDestroyDataset(output_);
    output_ = nullptr;
    INFO_LOG("destroy model output success");
}

Result ModelProcess::Execute() {
    aclError ret = aclmdlExecute(modelId_, input_, output_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("execute model failed, modelId is %u, errorCode is %d", modelId_, static_cast<int32_t>(ret));
        return FAILED;
    }

    INFO_LOG("model execute success, modelId is %u", modelId_);
    return SUCCESS;
}

const aclmdlDataset *ModelProcess::GetModelOutputData() {
    return output_;
}

Result ModelProcess::GetModelInputWH(int &width, int &height) {
    if (modelDesc_ == nullptr) {
        ERROR_LOG("no model description, get input hw failed");
        return FAILED;
    }
    // format of om used in this app is NHWC, dimsCout is 4
    // dims[0] is N, dims[1] is H, dims[2] is W dims[3] is C
    aclmdlIODims dims;
    // om used in this app has only one input
    aclError ret = aclmdlGetInputDims(modelDesc_, 0, &dims);
    if (ret != ACL_ERROR_NONE) {
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
