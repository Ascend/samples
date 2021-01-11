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

* File model_process.cpp
* Description: handle model process
*/
#include "model_process.h"
#include <bits/stdint-uintn.h>
#include <iostream>
#include "utils.h"

#define IMAGEINFOSIZE 12
using namespace std;

ModelProcess::ModelProcess():loadFlag_(false), modelId_(0), modelDesc_(nullptr), input_(nullptr), output_(nullptr), dynamic_resolution_input_(nullptr)
{
    imageInfoSize_ = IMAGEINFOSIZE;
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
    if (loadFlag_) {
        ERROR_LOG("has already loaded a model");
        return FAILED;
    }

    // TODO:
    // load model and get modelID.
    aclError ret = aclmdlLoadFromFile(modelPath, &modelId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("load model from file failed, model file is %s", modelPath);
        return FAILED;
    }

    loadFlag_ = true;
    INFO_LOG("load model %s success", modelPath);
    return SUCCESS;
}

Result ModelProcess::CreateDesc()
{
    modelDesc_ = aclmdlCreateDesc();
    if (modelDesc_ == nullptr) {
        ERROR_LOG("create model description failed");
        return FAILED;
    }
    //TODO:
    // get modelDesc(model description) by modelID
    aclError ret = aclmdlGetDesc(modelDesc_, modelId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("get model description failed");
        return FAILED;
    }

    ret = aclmdlGetInputIndexByName(modelDesc_, ACL_DYNAMIC_TENSOR_NAME, &DynamicHWSizeIndex_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("aclmdlGetInputIndexByName failed");
        return FAILED;
    }

    DynamicHWSizeInputSize_ = aclmdlGetInputSizeByIndex(modelDesc_, DynamicHWSizeIndex_);
    if (dynamic_resolution_input_ == nullptr) {
        aclError aclRet = aclrtMalloc(&dynamic_resolution_input_, DynamicHWSizeInputSize_, ACL_MEM_MALLOC_HUGE_FIRST);
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("malloc device data input3 failed, aclRet is %d", aclRet);
            return FAILED;
        }
    }

    INFO_LOG("create model description success");
    return SUCCESS;
}

Result ModelProcess::CreateImageInfoBuffer(){
    aclError aclRet = aclrtMalloc(&imageInfoBuf_, imageInfoSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("malloc imageInfoBuf_ device data buffer failed, aclRet is %d", aclRet);
        return FAILED;
    }
}

Result ModelProcess::SetImageInfoBuffer(){


    static auto it = resizeWidthHeight.begin();
    if (it == resizeWidthHeight.end()){
        it = resizeWidthHeight.begin();
    }


    const float imageInfo[3] = {(float)(it->second), (float)(it->first),  (float)3};

    aclError ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl get run mode failed");
        return FAILED;
    }

    if (runMode_ == ACL_HOST){
        aclError aclRet = aclrtMemcpy(imageInfoBuf_, imageInfoSize_, (void *)(imageInfo), imageInfoSize_, ACL_MEMCPY_HOST_TO_DEVICE);
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("Copy data to device failed, aclRet is %d", aclRet);
            (void)aclrtFree(imageInfoBuf_);
            return FAILED;
        }
    }
    else{
        aclError aclRet = aclrtMemcpy(imageInfoBuf_, imageInfoSize_, (void *)(imageInfo), imageInfoSize_, ACL_MEMCPY_DEVICE_TO_DEVICE);
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("Copy data to device failed, aclRet is %d", aclRet);
            (void)aclrtFree(imageInfoBuf_);
            return FAILED;
        }
    }
    it++;
    return SUCCESS;
}

void ModelProcess::DestroyDesc()
{
    if (modelDesc_ != nullptr) {
        (void)aclmdlDestroyDesc(modelDesc_);
        modelDesc_ = nullptr;
    }
}

Result ModelProcess::CreateInput(void *inputDataBuffer, size_t bufferSize)
{
    size_t inputnums = aclmdlGetNumInputs(modelDesc_);

    for (int i=0; i<inputnums; i++) {
        size_t inputSize = aclmdlGetInputSizeByIndex(modelDesc_, i);
    }


    static auto it = resizeWidthHeight.begin();
    if (it == resizeWidthHeight.end()){
        it = resizeWidthHeight.begin();
    }


    // TODO:
    input_ = aclmdlCreateDataset();
    if (input_ == nullptr) {
        ERROR_LOG("can't create dataset, create input failed");
        return FAILED;
    }

    aclDataBuffer* inputData = aclCreateDataBuffer(inputDataBuffer, bufferSize);
    if (inputData == nullptr) {
        ERROR_LOG("can't create data buffer, create input failed");
        return FAILED;
    }

    aclError ret = aclmdlAddDatasetBuffer(input_, inputData);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("can't add data buffer, create input failed");
        aclDestroyDataBuffer(inputData);
        inputData = nullptr;
        return FAILED;
    }

    // TODO:
    // add model format desc

    aclDataBuffer* imageInfoData = aclCreateDataBuffer(imageInfoBuf_, imageInfoSize_);
    if (imageInfoData == nullptr) {
        ERROR_LOG("can't create data buffer, create input failed");
        return FAILED;
    }

    ret = aclmdlAddDatasetBuffer(input_, imageInfoData);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("can't add data buffer, create input failed");
        aclDestroyDataBuffer(imageInfoData);
        imageInfoData = nullptr;
        return FAILED;
    }

    aclDataBuffer* inputData3 = aclCreateDataBuffer(dynamic_resolution_input_, DynamicHWSizeInputSize_);
    if (inputData3 == nullptr) {
        ERROR_LOG("can't create data inputData3, create input failed");
        return FAILED;
    }

    ret = aclmdlAddDatasetBuffer(input_, inputData3);
    if (inputData3 == nullptr) {
        ERROR_LOG("can't add data buffer, create input failed");
        aclDestroyDataBuffer(inputData3);
        inputData3 = nullptr;
        return FAILED;
    }
    ret = aclmdlSetDynamicHWSize(modelId_, input_, DynamicHWSizeIndex_, it->second, it->first);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("aclmdlSetDynamicHWSize failed, aclRet is %d", ret);
        return FAILED;
    }
    it++;

    return SUCCESS;
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
//    if (dynamic_resolution_input_ != nullptr) {
//        aclrtFree(dynamic_resolution_input_);
//        dynamic_resolution_input_ = nullptr;
//    }
}


Result ModelProcess::CreateOutput()
{
    if (modelDesc_ == nullptr) {
        ERROR_LOG("no model description, create ouput failed");
        return FAILED;
    }

    //TODO:
    output_ = aclmdlCreateDataset();
    if (output_ == nullptr) {
        ERROR_LOG("can't create dataset, create output failed");
        return FAILED;
    }
    size_t outputSize = aclmdlGetNumOutputs(modelDesc_);

    for (size_t i = 0; i < outputSize; ++i) {
        //TODO:
        size_t buffer_size = aclmdlGetOutputSizeByIndex(modelDesc_, i);

        void *outputBuffer = nullptr;
        aclError ret = aclrtMalloc(&outputBuffer, buffer_size, ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("can't malloc buffer, size is %zu, create output failed", buffer_size);
            return FAILED;
        }

        aclDataBuffer* outputData = aclCreateDataBuffer(outputBuffer, buffer_size);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("can't create data buffer, create output failed");
            aclrtFree(outputBuffer);
            return FAILED;
        }

        ret = aclmdlAddDatasetBuffer(output_, outputData);
        if (ret != ACL_ERROR_NONE) {
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

    if (dynamic_resolution_input_ != nullptr) {
        aclrtFree(dynamic_resolution_input_);
        dynamic_resolution_input_ = nullptr;
    }

    if (imageInfoBuf_ != nullptr) {
        aclrtFree(imageInfoBuf_);
        imageInfoBuf_ = nullptr;
    }
}

Result ModelProcess::Execute()
{
    //TODO:
    aclError ret = aclmdlExecute(modelId_, input_, output_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("execute model failed, modelId is %u", modelId_);
        return FAILED;
    }

    INFO_LOG("model execute success");
    return SUCCESS;
}

void ModelProcess::Unload()
{

    if (!loadFlag_) {
        WARN_LOG("no model had been loaded, unload failed");
        return;
    }

    //TODO:
    aclError ret = aclmdlUnload(modelId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("unload model failed, modelId is %u", modelId_);
    }

    if (modelDesc_ != nullptr) {
        (void)aclmdlDestroyDesc(modelDesc_);
        modelDesc_ = nullptr;
    }

    loadFlag_ = false;
    INFO_LOG("unload model success, modelId is %u", modelId_);
}

aclmdlDataset *ModelProcess::GetModelOutputData()
{
    return output_;
}


