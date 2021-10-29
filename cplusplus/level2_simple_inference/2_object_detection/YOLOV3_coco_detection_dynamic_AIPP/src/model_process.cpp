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
#include <iostream>
#include <vector>
#include "utils.h"
using namespace std;

ModelProcess::ModelProcess():loadFlag_(false), modelId_(0), modelMemPtr_(nullptr), modelMemSize_(0),
modelWeightPtr_(nullptr),modelWeightSize_(0), modelDesc_(nullptr), input_(nullptr), output_(nullptr),
isReleased_(false){

}

ModelProcess::~ModelProcess(){
    DestroyResource();
}

void ModelProcess::DestroyResource(){
    if (isReleased_) {return;}
    Unload();
    DestroyDesc();
    DestroyInput();
    DestroyOutput();
    isReleased_ = true;
}

Result ModelProcess::LoadModelFromFileWithMem(const char *modelPath){
    if (loadFlag_) {
        ERROR_LOG("has already loaded a model");
        return FAILED;
    }

    aclError ret = aclmdlQuerySize(modelPath, &modelMemSize_, &modelWeightSize_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("query model failed, model file is %s", modelPath);
        return FAILED;
    }

    ret = aclrtMalloc(&modelMemPtr_, modelMemSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("malloc buffer for mem failed, require size is %zu", modelMemSize_);
        return FAILED;
    }

    ret = aclrtMalloc(&modelWeightPtr_, modelWeightSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("malloc buffer for weight failed, require size is %zu", modelWeightSize_);
        return FAILED;
    }

    ret = aclmdlLoadFromFileWithMem(modelPath, &modelId_, modelMemPtr_,
    modelMemSize_, modelWeightPtr_, modelWeightSize_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("load model from file failed, model file is %s", modelPath);
        return FAILED;
    }

    loadFlag_ = true;
    INFO_LOG("load model %s success", modelPath);
    return SUCCESS;
}

Result ModelProcess::CreateDesc(){
    modelDesc_ = aclmdlCreateDesc();
    if (modelDesc_ == nullptr) {
        ERROR_LOG("create model description failed");
        return FAILED;
    }

    aclError ret = aclmdlGetDesc(modelDesc_, modelId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("get model description failed");
        return FAILED;
    }

    ret = aclmdlGetInputIndexByName(modelDesc_, ACL_DYNAMIC_AIPP_NAME, &aipp_index_);
    if (ret != 0) {
        ERROR_LOG("call aclmdlGetInputIndexByName failed");
        return FAILED;
    }
    inputAIPPSize_ = aclmdlGetInputSizeByIndex(modelDesc_, aipp_index_);
    ret = aclrtMalloc(&inputAIPP_, inputAIPPSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("malloc device data input3 failed, result is %d", ret);
        return FAILED;
    }

    INFO_LOG("create model description success");
    return SUCCESS;
}

void ModelProcess::DestroyDesc(){
    if (modelDesc_ != nullptr) {
        (void)aclmdlDestroyDesc(modelDesc_);
        modelDesc_ = nullptr;
    }
}

Result ModelProcess::AddDatasetBuffer(aclmdlDataset *dataset, 
                                      void* buffer, uint32_t bufferSize) {
    aclDataBuffer* dataBuf = aclCreateDataBuffer(buffer, bufferSize);
    if (dataBuf == nullptr) {
        ERROR_LOG("Create data buffer error");
        return FAILED;
    }

    aclError ret = aclmdlAddDatasetBuffer(dataset, dataBuf);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("Add dataset buffer error %d", ret);
        aclDestroyDataBuffer(dataBuf);
        return FAILED;
    }

    return SUCCESS;
}

aclmdlAIPP* ModelProcess::SetAIPPTensor(uint64_t batch_size){
    aclmdlAIPP* aippParamTensor = aclmdlCreateAIPP(batch_size);
    if (aippParamTensor == nullptr) {
        ERROR_LOG("call aclmdlCreateAIPP failed");
    }

    aclError ret = aclmdlSetAIPPInputFormat(aippParamTensor, ACL_YUV420SP_U8);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPInputFormat failed");
    }

    ret = aclmdlSetAIPPCscParams(aippParamTensor, 1, 298, 516, 0, 298, -100, -208, 298, 0, 409, 0, 0, 0, 16, 128, 128);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPCscParams failed");
    }

    ret = aclmdlSetAIPPRbuvSwapSwitch(aippParamTensor, 0);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPRbuvSwapSwitch failed");
    }

    ret = aclmdlSetAIPPAxSwapSwitch(aippParamTensor, 0);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPAxSwapSwitch failed");
    }

    ret = aclmdlSetAIPPSrcImageSize(aippParamTensor, 416, 416);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPSrcImageSize failed");
    }

    ret = aclmdlSetAIPPScfParams(aippParamTensor, 0, 0, 0, 0, 0, 0);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPScfParams failed");
    }

    ret = aclmdlSetAIPPCropParams(aippParamTensor, 0, 0, 0, 0, 0, 0);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPCropParams failed");
    }

    ret = aclmdlSetAIPPPaddingParams(aippParamTensor, 0, 0, 0, 0, 0, 0);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPPaddingParams failed");
    }

    ret = aclmdlSetAIPPDtcPixelMean(aippParamTensor, 0, 0, 0, 0, 0);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPDtcPixelMean failed");
    }

    ret = aclmdlSetAIPPDtcPixelMin(aippParamTensor, 0.0, 0.0, 0.0, 0.0, 0);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPDtcPixelMin failed");
    }

    ret = aclmdlSetAIPPPixelVarReci(aippParamTensor, 0.003921568627451, 0.003921568627451, 0.003921568627451, 1.0, 0);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPPixelVarReci failed");
    }
    return aippParamTensor;
}

aclmdlAIPP* ModelProcess::SetAIPPTensorOpenCV(uint64_t batch_size){
    aclmdlAIPP* aippParamTensor = aclmdlCreateAIPP(batch_size);
    if (aippParamTensor == nullptr) {
        ERROR_LOG("call aclmdlCreateAIPP failed");
    }

    aclError ret = aclmdlSetAIPPSrcImageSize(aippParamTensor, 416, 416);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPSrcImageSize failed");
    }

    ret = aclmdlSetAIPPInputFormat(aippParamTensor, ACL_RGB888_U8);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPInputFormat failed");
    }

    ret = aclmdlSetAIPPCscParams(aippParamTensor, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPCscParams failed");
    }

    ret = aclmdlSetAIPPRbuvSwapSwitch(aippParamTensor, 1);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPRbuvSwapSwitch failed");
    }

    ret = aclmdlSetAIPPAxSwapSwitch(aippParamTensor, 0);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPAxSwapSwitch failed");
    }

    ret = aclmdlSetAIPPScfParams(aippParamTensor, 0, 0, 0, 0, 0, 0);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPScfParams failed");
    }

    ret = aclmdlSetAIPPCropParams(aippParamTensor, 0, 0, 0, 0, 0, 0);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPCropParams failed");
    }

    ret = aclmdlSetAIPPPaddingParams(aippParamTensor, 0, 0, 0, 0, 0, 0);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPPaddingParams failed");
    }

    ret = aclmdlSetAIPPDtcPixelMean(aippParamTensor, 0, 0, 0, 0, 0);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPDtcPixelMean failed");
    }

    ret = aclmdlSetAIPPDtcPixelMin(aippParamTensor, 0.0, 0.0, 0.0, 0.0, 0);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPDtcPixelMin failed");
    }

    ret = aclmdlSetAIPPPixelVarReci(aippParamTensor, 0.003921568627451, 0.003921568627451, 0.003921568627451, 1.0, 0);
    if (ret != 0) {
        ERROR_LOG("call aclmdlSetAIPPPixelVarReci failed");
    }
    return aippParamTensor;
}

Result ModelProcess::CreateInput(void* input1, uint32_t input1size,
                                 void* input2, uint32_t input2size){
    vector<DataInfo> inputData = {{input1, input1size},{input2, input2size},
                                  {inputAIPP_, inputAIPPSize_}};
    uint32_t dataNum = aclmdlGetNumInputs(modelDesc_);
    if (dataNum == 0) {
        ERROR_LOG("Create input failed for no input data");
        return FAILED;
    }
    if (dataNum != inputData.size()) {
        ERROR_LOG("Create input failed for wrong input nums");
        return FAILED;
    }

    input_ = aclmdlCreateDataset();
    if (input_ == nullptr) {
        ERROR_LOG("Create input failed for create dataset failed");
        return FAILED;
    }

    for (uint32_t i = 0; i < inputData.size(); i++) {
        size_t modelInputSize = aclmdlGetInputSizeByIndex(modelDesc_, i);
        if (modelInputSize != inputData[i].size) {
            WARNNING_LOG("Input size verify failed "
                         "input[%d] size: %d, provide size : %d", 
                         i, modelInputSize, inputData[i].size);
                        }
        Result ret = AddDatasetBuffer(input_, 
                                      inputData[i].data,
                                      inputData[i].size);
        if (ret != SUCCESS) {
            ERROR_LOG("Create input failed for "
                      "add dataset buffer error %d", ret);
            return FAILED;
            }
    }
    uint64_t batch_num = 1 ;
    aclmdlAIPP* aippParamTensor = SetAIPPTensor(batch_num);
    aclError aclret = aclmdlSetInputAIPP(modelId_, input_, aipp_index_, aippParamTensor);
    if (aclret != ACL_ERROR_NONE) {
        ERROR_LOG("call aclmdlSetInputAIPP failed");
        return FAILED;
    }

    aclret = aclmdlDestroyAIPP(aippParamTensor);
    if (aclret != ACL_ERROR_NONE) {
        ERROR_LOG("call aclmdlDestroyAIPP failed");
        return FAILED;
    }
    return SUCCESS;
}

Result ModelProcess::CreateInputOpenCV(void* input1, uint32_t input1size,
                                 void* input2, uint32_t input2size){
    vector<DataInfo> inputData = {{input1, input1size},{input2, input2size},
                                  {inputAIPP_, inputAIPPSize_}};
    uint32_t dataNum = aclmdlGetNumInputs(modelDesc_);
    if (dataNum == 0) {
        ERROR_LOG("Create input failed for no input data");
        return FAILED;
    }
    if (dataNum != inputData.size()) {
        ERROR_LOG("Create input failed for wrong input nums");
        return FAILED;
    }

    input_ = aclmdlCreateDataset();
    if (input_ == nullptr) {
        ERROR_LOG("Create input failed for create dataset failed");
        return FAILED;
    }

    for (uint32_t i = 0; i < inputData.size(); i++) {
        size_t modelInputSize = aclmdlGetInputSizeByIndex(modelDesc_, i);
        if (modelInputSize != inputData[i].size) {
            WARNNING_LOG("Input size verify failed "
                         "input[%d] size: %d, provide size : %d", 
                         i, modelInputSize, inputData[i].size);
            }
        Result ret = AddDatasetBuffer(input_, 
                                      inputData[i].data,
                                      inputData[i].size);
        if (ret != SUCCESS) {
            ERROR_LOG("Create input failed for "
                      "add dataset buffer error %d", ret);
            return FAILED;
            }
    }
    uint64_t batch_num = 1 ;
    aclmdlAIPP* aippParamTensor = SetAIPPTensorOpenCV(batch_num);
    aclError aclret = aclmdlSetInputAIPP(modelId_, input_, aipp_index_, aippParamTensor);
    if (aclret != ACL_ERROR_NONE) {
        ERROR_LOG("call aclmdlSetInputAIPP failed");
        return FAILED;
    }

    aclret = aclmdlDestroyAIPP(aippParamTensor);
    if (aclret != ACL_ERROR_NONE) {
        ERROR_LOG("call aclmdlDestroyAIPP failed");
        return FAILED;
    }
    return SUCCESS;
}

void ModelProcess::DestroyInput(){
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

Result ModelProcess::CreateOutput(){
    if (modelDesc_ == nullptr) {
        ERROR_LOG("no model description, create ouput failed");
        return FAILED;
    }

    output_ = aclmdlCreateDataset();
    if (output_ == nullptr) {
        ERROR_LOG("can't create dataset, create output failed");
        return FAILED;
    }

    size_t outputSize = aclmdlGetNumOutputs(modelDesc_);
    for (size_t i = 0; i < outputSize; ++i) {
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

void ModelProcess::DestroyOutput(){
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

Result ModelProcess::Execute(){
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
        WARNNING_LOG("no model had been loaded, unload failed");
        return;
    }

    aclError ret = aclmdlUnload(modelId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("unload model failed, modelId is %u", modelId_);
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
    INFO_LOG("unload model success, modelId is %u", modelId_);
}

aclmdlDataset *ModelProcess::GetModelOutputData(){
    return output_;
}


