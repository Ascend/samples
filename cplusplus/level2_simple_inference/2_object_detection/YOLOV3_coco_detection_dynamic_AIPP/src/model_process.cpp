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
#include <vector>
#include "model_process.h"
#include "utils.h"
using namespace std;

ModelProcess::ModelProcess()
    : g_loadFlag_(false), g_modelId_(0), g_modelMemPtr_(nullptr), g_modelMemSize_(0),
      g_modelWeightPtr_(nullptr), g_modelWeightSize_(0), g_modelDesc_(nullptr),
      g_input_(nullptr), g_output_(nullptr),
      g_isReleased_(false)
{
}

ModelProcess::~ModelProcess()
{
    DestroyResource();
}

void ModelProcess::DestroyResource()
{
    if (g_isReleased_) {return;}
    Unload();
    DestroyDesc();
    DestroyInput();
    DestroyOutput();
    g_isReleased_ = true;
}

Result ModelProcess::LoadModelFromFileWithMem(const char *modelPath)
{
    if (g_loadFlag_) {
        ERROR_LOG("has already loaded a model");
        return FAILED;
    }

    aclError ret = aclmdlQuerySize(modelPath, &g_modelMemSize_, &g_modelWeightSize_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("query model failed, model file is %s", modelPath);
        return FAILED;
    }

    ret = aclrtMalloc(&g_modelMemPtr_, g_modelMemSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("malloc buffer for mem failed, require size is %zu", g_modelMemSize_);
        return FAILED;
    }

    ret = aclrtMalloc(&g_modelWeightPtr_, g_modelWeightSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("malloc buffer for weight failed, require size is %zu", g_modelWeightSize_);
        return FAILED;
    }

    ret = aclmdlLoadFromFileWithMem(modelPath, &g_modelId_, g_modelMemPtr_,
    g_modelMemSize_, g_modelWeightPtr_, g_modelWeightSize_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("load model from file failed, model file is %s", modelPath);
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

    ret = aclmdlGetInputIndexByName(g_modelDesc_, ACL_DYNAMIC_AIPP_NAME, &g_aipp_index_);
    if (ret != 0) {
        ERROR_LOG("call aclmdlGetInputIndexByName failed");
        return FAILED;
    }
    g_inputAIPPSize_ = aclmdlGetInputSizeByIndex(g_modelDesc_, g_aipp_index_);
    ret = aclrtMalloc(&g_inputAIPP_, g_inputAIPPSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("malloc device data input3 failed, result is %d", ret);
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
}

Result ModelProcess::AddDatasetBuffer(aclmdlDataset *dataset,
                                      void* buffer, uint32_t bufferSize)
{
    aclDataBuffer* dataBuf = aclCreateDataBuffer(buffer, bufferSize);
    if (dataBuf == nullptr) {
        ERROR_LOG("Create data buffer error");
        return FAILED;
    }

    aclError ret = aclmdlAddDatasetBuffer(dataset, dataBuf);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("Add dataset buffer error %d", ret);
        aclDestroyDataBuffer(dataBuf);
        return FAILED;
    }

    return SUCCESS;
}

aclmdlAIPP* ModelProcess::SetAIPPTensor(uint64_t batch_size)
{
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

aclmdlAIPP* ModelProcess::SetAIPPTensorOpenCV(uint64_t batch_size)
{
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

Result ModelProcess::CreateInput(void* input1, uint32_t input1Size,
                                 void* input2, uint32_t input2Size)
{
    vector<DataInfo> inputData = {{input1, input1Size}, {input2, input2Size},
                                  {g_inputAIPP_, g_inputAIPPSize_}};
    uint32_t dataNum = aclmdlGetNumInputs(g_modelDesc_);
    if (dataNum == 0) {
        ERROR_LOG("Create input failed for no input data");
        return FAILED;
    }
    if (dataNum != inputData.size()) {
        ERROR_LOG("Create input failed for wrong input nums");
        return FAILED;
    }

    g_input_ = aclmdlCreateDataset();
    if (g_input_ == nullptr) {
        ERROR_LOG("Create input failed for create dataset failed");
        return FAILED;
    }

    for (uint32_t i = 0; i < inputData.size(); i++) {
        size_t modelInputSize = aclmdlGetInputSizeByIndex(g_modelDesc_, i);
        if (modelInputSize != inputData[i].size) {
            WARNNING_LOG("Input size verify failed "
                         "input[%d] size: %d, provide size : %d",
                         i, modelInputSize, inputData[i].size);
                        }
        Result ret = AddDatasetBuffer(g_input_,
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
    aclError aclret = aclmdlSetInputAIPP(g_modelId_, g_input_, g_aipp_index_, aippParamTensor);
    if (aclret != ACL_SUCCESS) {
        ERROR_LOG("call aclmdlSetInputAIPP failed");
        return FAILED;
    }

    aclret = aclmdlDestroyAIPP(aippParamTensor);
    if (aclret != ACL_SUCCESS) {
        ERROR_LOG("call aclmdlDestroyAIPP failed");
        return FAILED;
    }
    return SUCCESS;
}

Result ModelProcess::CreateInputOpenCV(void* input1, uint32_t input1Size,
                                       void* input2, uint32_t input2Size)
{
    vector<DataInfo> inputData = {{input1, input1Size}, {input2, input2Size},
                                  {g_inputAIPP_, g_inputAIPPSize_}};
    uint32_t dataNum = aclmdlGetNumInputs(g_modelDesc_);
    if (dataNum == 0) {
        ERROR_LOG("Create input failed for no input data");
        return FAILED;
    }
    if (dataNum != inputData.size()) {
        ERROR_LOG("Create input failed for wrong input nums");
        return FAILED;
    }

    g_input_ = aclmdlCreateDataset();
    if (g_input_ == nullptr) {
        ERROR_LOG("Create input failed for create dataset failed");
        return FAILED;
    }

    for (uint32_t i = 0; i < inputData.size(); i++) {
        size_t modelInputSize = aclmdlGetInputSizeByIndex(g_modelDesc_, i);
        if (modelInputSize != inputData[i].size) {
            WARNNING_LOG("Input size verify failed "
                         "input[%d] size: %d, provide size : %d",
                         i, modelInputSize, inputData[i].size);
            }
        Result ret = AddDatasetBuffer(g_input_,
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
    aclError aclret = aclmdlSetInputAIPP(g_modelId_, g_input_, g_aipp_index_, aippParamTensor);
    if (aclret != ACL_SUCCESS) {
        ERROR_LOG("call aclmdlSetInputAIPP failed");
        return FAILED;
    }

    aclret = aclmdlDestroyAIPP(aippParamTensor);
    if (aclret != ACL_SUCCESS) {
        ERROR_LOG("call aclmdlDestroyAIPP failed");
        return FAILED;
    }
    return SUCCESS;
}

void ModelProcess::DestroyInput()
{
    if (g_input_ == nullptr) {
        return;
    }

    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(g_input_); ++i) {
        aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(g_input_, i);
        aclDestroyDataBuffer(dataBuffer);
    }
    aclmdlDestroyDataset(g_input_);
    g_input_ = nullptr;
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
        size_t buffer_size = aclmdlGetOutputSizeByIndex(g_modelDesc_, i);

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

        ret = aclmdlAddDatasetBuffer(g_output_, outputData);
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
    if (g_output_ == nullptr) {
        return;
    }

    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(g_output_); ++i) {
        aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(g_output_, i);
        void* data = aclGetDataBufferAddr(dataBuffer);
        (void)aclrtFree(data);
        (void)aclDestroyDataBuffer(dataBuffer);
    }

    (void)aclmdlDestroyDataset(g_output_);
    g_output_ = nullptr;
}

Result ModelProcess::Execute()
{
    aclError ret = aclmdlExecute(g_modelId_, g_input_, g_output_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("execute model failed, modelId is %u", g_modelId_);
        return FAILED;
    }

    INFO_LOG("model execute success");
    return SUCCESS;
}

void ModelProcess::Unload()
{
    if (!g_loadFlag_) {
        WARNNING_LOG("no model had been loaded, unload failed");
        return;
    }

    aclError ret = aclmdlUnload(g_modelId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("unload model failed, modelId is %u", g_modelId_);
    }

    if (g_modelDesc_ != nullptr) {
        (void)aclmdlDestroyDesc(g_modelDesc_);
        g_modelDesc_ = nullptr;
    }

    if (g_modelMemPtr_ != nullptr) {
        aclrtFree(g_modelMemPtr_);
        g_modelMemPtr_ = nullptr;
        g_modelMemSize_ = 0;
    }

    if (g_modelWeightPtr_ != nullptr) {
        aclrtFree(g_modelWeightPtr_);
        g_modelWeightPtr_ = nullptr;
        g_modelWeightSize_ = 0;
    }

    g_loadFlag_ = false;
    INFO_LOG("unload model success, modelId is %u", g_modelId_);
}

aclmdlDataset *ModelProcess::GetModelOutputData()
{
    return g_output_;
}
