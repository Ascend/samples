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
#include <map>
#include <sstream>
#include <algorithm>

using namespace std;
bool g_isDevice;

ModelProcess::ModelProcess(const char* omModelPath) 
:modelPath_(omModelPath), modelId_(0), modelMemSize_(0),
modelWeightSize_(0), modelMemPtr_(nullptr), modelWeightPtr_(nullptr), 
loadFlag_(false), modelDesc_(nullptr), input_(nullptr), output_(nullptr), isInited_(false)
{
}

ModelProcess::~ModelProcess()
{
    Unload();
    DestroyDesc();
    DestroyInput();
    DestroyOutput();
}


int ModelProcess::LoadModelFromFileWithMem()
{
    if (loadFlag_) {
        ASC_LOG_ERROR("has already loaded a model");
        return STATUS_ERROR;
    }

    aclError ret = aclmdlQuerySize(modelPath_.c_str(), &modelMemSize_, &modelWeightSize_);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("query model failed, model file is %s", modelPath_.c_str());
        return STATUS_ERROR;
    }

    ret = aclrtMalloc(&modelMemPtr_, modelMemSize_, ACL_MEM_MALLOC_NORMAL_ONLY);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("malloc buffer for mem failed, require size is %zu", modelMemSize_);
        return STATUS_ERROR;
    }

    ret = aclrtMalloc(&modelWeightPtr_, modelWeightSize_, ACL_MEM_MALLOC_NORMAL_ONLY);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("malloc buffer for weight failed, require size is %zu", modelWeightSize_);
        return STATUS_ERROR;
    }

    ret = aclmdlLoadFromFileWithMem(modelPath_.c_str(), &modelId_, modelMemPtr_,
        modelMemSize_, modelWeightPtr_, modelWeightSize_);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("load model from file failed, model file is %s", modelPath_.c_str());
        return STATUS_ERROR;
    }

    loadFlag_ = true;
    ASC_LOG_INFO("load model %s success", modelPath_.c_str());
    return STATUS_OK;
}

int ModelProcess::CreateDesc()
{
    modelDesc_ = aclmdlCreateDesc();
    if (modelDesc_ == nullptr) {
        ASC_LOG_ERROR("create model description failed");
        return STATUS_ERROR;
    }

    aclError ret = aclmdlGetDesc(modelDesc_, modelId_);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("get model description failed");
        return STATUS_ERROR;
    }

    ASC_LOG_INFO("create model description success");

    return STATUS_OK;
}

void ModelProcess::DestroyDesc()
{
    if (modelDesc_ != nullptr) {
        (void)aclmdlDestroyDesc(modelDesc_);
        modelDesc_ = nullptr;
    }
}

int ModelProcess::Init() {
    ASC_LOG_INFO("Init mode process...");
    int ret = LoadModelFromFileWithMem();
    if (ret != STATUS_OK) {
        ASC_LOG_ERROR("Init mode process failed for load model from file failed");
        return STATUS_ERROR;
    }

    ret = CreateDesc();
    if (ret != STATUS_OK) {
        ASC_LOG_ERROR("Init mode process failed for create model description failed");
        return STATUS_ERROR;
    }

    ret = CreateOutput();
    if (ret != STATUS_OK) {
        ASC_LOG_ERROR("Init mode process failed for create model output failed");
        return STATUS_ERROR;
    }

    aclrtRunMode runMode;
    ret = aclrtGetRunMode(&runMode);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("acl get run mode failed");
        return STATUS_ERROR;
    }
    isDevice_ = (runMode == ACL_DEVICE);
    ASC_LOG_INFO("Current run in %s", isDevice_?"device":"host");

    isInited_ = true;
    ASC_LOG_INFO("Init mode process success");

    return STATUS_OK;
}

int ModelProcess::CreateInput(std::vector<DataBuffer>& inferenceInput)
{
    input_ = aclmdlCreateDataset();
    if (input_ == nullptr) {
        ASC_LOG_ERROR("can't create dataset, create input failed");
        return STATUS_ERROR;
    }

    for (uint32_t i = 0; i < inferenceInput.size(); i++) {
        aclDataBuffer* inputData = aclCreateDataBuffer(inferenceInput[i].data.get(), 
                                                       inferenceInput[i].size);
        if (inputData == nullptr) {
            ASC_LOG_ERROR("can't create data buffer, create input failed");
            return STATUS_ERROR;
        }

        aclError ret = aclmdlAddDatasetBuffer(input_, inputData);
        if (ret != ACL_ERROR_NONE) {
            ASC_LOG_ERROR("add input dataset buffer failed");
            aclDestroyDataBuffer(inputData);
            inputData = nullptr;
            return STATUS_ERROR;
        }
    }

    return STATUS_OK;
}

void ModelProcess::DestroyInput()
{
    if (input_ == nullptr) {
        return;
    }

    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(input_); ++i) {
        aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(input_, i);
        aclDestroyDataBuffer(dataBuffer);
    }
    aclmdlDestroyDataset(input_);
    input_ = nullptr;
}

int ModelProcess::CreateOutput()
{
    if (modelDesc_ == nullptr) {
        ASC_LOG_ERROR("no model description, create ouput failed");
        return STATUS_ERROR;
    }

    output_ = aclmdlCreateDataset();
    if (output_ == nullptr) {
        ASC_LOG_ERROR("can't create dataset, create output failed");
        return STATUS_ERROR;
    }

    size_t outputSize = aclmdlGetNumOutputs(modelDesc_);
    for (size_t i = 0; i < outputSize; ++i) {
        size_t buffer_size = aclmdlGetOutputSizeByIndex(modelDesc_, i);

        void *outputBuffer = nullptr;
        aclError ret = aclrtMalloc(&outputBuffer, buffer_size, ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != ACL_ERROR_NONE) {
            ASC_LOG_ERROR("can't malloc buffer, size is %zu, create output failed", buffer_size);
            return STATUS_ERROR;
        }

        aclDataBuffer* outputData = aclCreateDataBuffer(outputBuffer, buffer_size);
        if (ret != ACL_ERROR_NONE) {
            ASC_LOG_ERROR("can't create data buffer, create output failed");
            aclrtFree(outputBuffer);
            return STATUS_ERROR;
        }

        ret = aclmdlAddDatasetBuffer(output_, outputData);
        if (ret != ACL_ERROR_NONE) {
            ASC_LOG_ERROR("can't add data buffer, create output failed");
            aclrtFree(outputBuffer);
            aclDestroyDataBuffer(outputData);
            return STATUS_ERROR;
        }
    }

    ASC_LOG_INFO("create model output success");
    return STATUS_OK;
}

void ModelProcess::OutputModelResult(std::vector<DataBuffer>& inferenceOutput)
{
    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(output_); ++i) {
        aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(output_, i);
        void* data = aclGetDataBufferAddr(dataBuffer);
        uint32_t len = aclGetDataBufferSize(dataBuffer);   
        DataBuffer output;

        if (isDevice_) {           
            void* resultData = CopyDataDeviceToDevice(data, len);
            output.data = shared_ptr<void>(resultData, [](void* p) { aclrtFree(p); });
        } else {
            void* resultData = CopyDataDeviceToDevice(data, len);
            output.data = shared_ptr<void>(resultData, [](void* p) { aclrtFreeHost(p); }); 
        }                                   
        output.size = len;
        inferenceOutput.emplace_back(output);
    }

    return;
}

void ModelProcess::DestroyOutput()
{
    if (output_ == nullptr) {
        return;
    }

    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(output_); ++i) {
        aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(output_, i);
        void* data = aclGetDataBufferAddr(dataBuffer);
        (void)aclrtFree(data);
        (void)aclDestroyDataBuffer(dataBuffer);
    }

    (void)aclmdlDestroyDataset(output_);
    output_ = nullptr;
}

int ModelProcess::Execute(std::vector<DataBuffer>& inferenceOutput, 
                          std::vector<DataBuffer>& inferenceInput)
{
    if (!isInited_) {
        if (STATUS_OK != Init()) {
            ASC_LOG_ERROR("mode process init failed");
            return STATUS_ERROR;
        }
    }
    if (STATUS_OK != CreateInput(inferenceInput)) {
        ASC_LOG_ERROR("model create input failed");
        return STATUS_ERROR;
    }

    aclError ret = aclmdlExecute(modelId_, input_, output_);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("execute model failed, modelId is %u", modelId_);
        return STATUS_ERROR;
    }

    OutputModelResult(inferenceOutput);
    DestroyInput();

    ASC_LOG_INFO("model execute success");
    return STATUS_OK;
}

void ModelProcess::Unload()
{
    if (!loadFlag_) {
        ASC_LOG_ERROR("no model had been loaded, unload failed");
        return;
    }

    aclError ret = aclmdlUnload(modelId_);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("unload model failed, modelId is %u", modelId_);
    }

    if (modelDesc_ != nullptr) {
        (void)aclmdlDestroyDesc(modelDesc_);
        modelDesc_ = nullptr;
    }

    if (modelMemPtr_ != nullptr) {
        aclrtFree(modelMemPtr_);
        modelMemPtr_ = nullptr;
        modelMemSize_ = 0;
    }

    if (modelWeightPtr_ != nullptr) {
        aclrtFree(modelWeightPtr_);
        modelWeightPtr_ = nullptr;
        modelWeightSize_ = 0;
    }

    loadFlag_ = false;
    ASC_LOG_INFO("unload model success, modelId is %u", modelId_);
}
