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
#include "atlas_model.h"
#include <iostream>
#include "atlas_utils.h"
using namespace std;

AtlasModel::AtlasModel()
:modelPath_(""), loadFlag_(false), modelId_(0), modelMemPtr_(nullptr),
modelMemSize_(0),modelWeightPtr_(nullptr),modelWeightSize_(0),
modelDesc_(nullptr), input_(nullptr), output_(nullptr), outputsNum_(0),
isReleased_(false) {
}

AtlasModel::AtlasModel(const string& modelPath)
:modelPath_(modelPath), loadFlag_(false), modelId_(0), modelMemPtr_(nullptr),
modelMemSize_(0),modelWeightPtr_(nullptr),modelWeightSize_(0), 
modelDesc_(nullptr), input_(nullptr), output_(nullptr), outputsNum_(0), 
isReleased_(false) {
}

AtlasModel::~AtlasModel() {
    DestroyResource();
}

void AtlasModel::DestroyResource() {
    if (isReleased_)
        return;
    
    Unload();
    DestroyDesc();
    DestroyInput();
    DestroyOutput();
    isReleased_ = true;
}


AtlasError AtlasModel::Init() {
    aclError aclRet = aclrtGetRunMode(&runMode_);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("acl get run mode failed");
        return ATLAS_ERROR_GET_RUM_MODE;
    } 

    AtlasError ret = LoadModelFromFile(modelPath_);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Load model from file failed, error: %d", ret);
        return ret;
    }

    ret = CreateDesc();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("execute CreateDesc failed");
        return ret;
    }

    ret = CreateOutput();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("execute CreateOutput failed");
        return ret;
    }

    ATLAS_LOG_INFO("Init model %s success", modelPath_.c_str());

    return ATLAS_OK;
}

AtlasError AtlasModel::Init(const string& modelPath) {
    modelPath_.assign(modelPath.c_str());
    return Init();
}

AtlasError AtlasModel::LoadModelFromFile(const string& modelPath) {
    if (loadFlag_) {
        ATLAS_LOG_ERROR("%s is loaded already", modelPath.c_str());
        return ATLAS_ERROR_LOAD_MODEL_REPEATED;
    }

    aclError ret = aclmdlLoadFromFile(modelPath.c_str(), &modelId_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Load model(%s) from file return %d", 
                        modelPath.c_str(), ret);
        return ATLAS_ERROR_LOAD_MODEL;
    }

    loadFlag_ = true;
    ATLAS_LOG_INFO("Load model %s success", modelPath.c_str());

    return ATLAS_OK;
}

AtlasError AtlasModel::CreateDesc() {
    modelDesc_ = aclmdlCreateDesc();
    if (modelDesc_ == nullptr) {
        ATLAS_LOG_ERROR("Create model(%s) description failed", 
                        modelPath_.c_str());
        return ATLAS_ERROR_CREATE_MODEL_DESC;
    }

    aclError ret = aclmdlGetDesc(modelDesc_, modelId_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Get model(%s) description failed", 
                        modelPath_.c_str());
        return ATLAS_ERROR_GET_MODEL_DESC;
    }

    ATLAS_LOG_INFO("Create model description success");
    return ATLAS_OK;
}

void AtlasModel::DestroyDesc()
{
    if (modelDesc_ != nullptr) {
        (void)aclmdlDestroyDesc(modelDesc_);
        modelDesc_ = nullptr;
    }
}

AtlasError AtlasModel::CreateInput(void *input, uint32_t size) {
    vector<DataInfo> inputData = {{input, size}};
    return CreateInput(inputData);
}

AtlasError AtlasModel::CreateInput(void *input1, uint32_t input1size, 
                                 void* input2, uint32_t input2size) {
    vector<DataInfo> inputData = {{input1, input1size}, {input2, input2size}};
    return CreateInput(inputData);
}

AtlasError AtlasModel::CreateInput(vector<DataInfo>& inputData) {
    uint32_t dataNum = inputData.size();

    if (dataNum == 0) {
        ATLAS_LOG_ERROR("Create input failed for no input data");
        return ATLAS_ERROR_INVALID_ARGS;
    }

    input_ = aclmdlCreateDataset();
    if (input_ == nullptr) {
        ATLAS_LOG_ERROR("Create input failed for create dataset failed");
        return ATLAS_ERROR_CREATE_DATASET;
    }

    for (uint32_t i = 0; i < inputData.size(); i++) {
        AtlasError atlRet = AddDatasetBuffer(input_, 
                                             inputData[i].data,
                                             inputData[i].size);
        if (atlRet != ATLAS_OK) {
            ATLAS_LOG_ERROR("Create input failed for "
                            "add dataset buffer error %d", atlRet);
            return ATLAS_ERROR_ADD_DATASET_BUFFER;
        }
    }

    return ATLAS_OK;
}

AtlasError AtlasModel::CreateOutput()
{
    if (modelDesc_ == nullptr) {
        ATLAS_LOG_ERROR("Create output failed for no model(%s) description",
                        modelPath_.c_str());
        return ATLAS_ERROR_NO_MODEL_DESC;
    }

    output_ = aclmdlCreateDataset();
    if (output_ == nullptr) {
        ATLAS_LOG_ERROR("Create output failed for create dataset error");
        return ATLAS_ERROR_CREATE_DATASET;
    }

    outputsNum_ = aclmdlGetNumOutputs(modelDesc_);
    for (size_t i = 0; i < outputsNum_; ++i) {
        size_t bufSize = aclmdlGetOutputSizeByIndex(modelDesc_, i);

        void *outputBuffer = nullptr;
        aclError ret = aclrtMalloc(&outputBuffer, bufSize, 
                                   ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != ACL_ERROR_NONE) {
            ATLAS_LOG_ERROR("Create output failed for malloc "
                            "device failed, size %d", (int)bufSize);
            return ATLAS_ERROR_MALLOC_DEVICE;
        }

        AtlasError atlRet = AddDatasetBuffer(output_, outputBuffer, bufSize);
        if (atlRet != ACL_ERROR_NONE) {
            ATLAS_LOG_ERROR("Create output failed for "
                            "add dataset buffer error %d", atlRet);
            aclrtFree(outputBuffer);
            return ATLAS_ERROR_ADD_DATASET_BUFFER;
        }
    }

    ATLAS_LOG_INFO("Create model(%s) output success", modelPath_.c_str());
    return ATLAS_OK;
}

AtlasError AtlasModel::AddDatasetBuffer(aclmdlDataset *dataset, 
                                        void* buffer, uint32_t bufferSize) {
    aclDataBuffer* dataBuf = aclCreateDataBuffer(buffer, bufferSize);
    if (dataBuf == nullptr) {
        ATLAS_LOG_ERROR("Create data buffer error");
        return ATLAS_ERROR_CREATE_DATA_BUFFER;
    }

    aclError ret = aclmdlAddDatasetBuffer(dataset, dataBuf);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Add dataset buffer error %d", ret);
        aclDestroyDataBuffer(dataBuf);
        return ATLAS_ERROR_ADD_DATASET_BUFFER;
    }

    return ATLAS_OK;
}

AtlasError AtlasModel::Execute(vector<InferenceOutput>& inferOutputs) {
    aclError ret = aclmdlExecute(modelId_, input_, output_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Execute model(%s) error:%d", modelPath_.c_str(), ret);
        return ATLAS_ERROR_EXECUTE_MODEL;
    }

    for (uint32_t i = 0; i < outputsNum_; i++) {
        InferenceOutput out;
        AtlasError ret = GetOutputItem(out, i); 
        if (ret != ATLAS_OK) {
            ATLAS_LOG_ERROR("Get the %dth interference output failed, "
                            "error: %d", i, ret);
            return ret;
        }
        inferOutputs.push_back(out);
    }

    return ATLAS_OK;
}

AtlasError AtlasModel::GetOutputItem(InferenceOutput& out,
                                     uint32_t idx) {
    aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(output_, idx);
    if (dataBuffer == nullptr) {
        ATLAS_LOG_ERROR("Get the %dth dataset buffer from model "
        "inference output failed", idx);
        return ATLAS_ERROR_GET_DATASET_BUFFER;
    }

    void* dataBufferDev = aclGetDataBufferAddr(dataBuffer);
    if (dataBufferDev == nullptr) {
        ATLAS_LOG_ERROR("Get the %dth dataset buffer address "
        "from model inference output failed", idx);
        return ATLAS_ERROR_GET_DATA_BUFFER_ADDR;
    }

    size_t bufferSize = aclGetDataBufferSize(dataBuffer);
    if (bufferSize == 0) {
        ATLAS_LOG_ERROR("The %dth dataset buffer size of "
        "model inference output is 0", idx);
        return ATLAS_ERROR_GET_DATA_BUFFER_SIZE;
    }

    void* data = CopyDataToHost(dataBufferDev, bufferSize, 
                                runMode_, MEMORY_NORMAL);
    if (data == nullptr) {
        ATLAS_LOG_ERROR("Copy inference output to host failed");
        return ATLAS_ERROR_COPY_DATA;
    }

    out.data = SHARED_PRT_U8_BUF(data);
    out.size = bufferSize;

    return ATLAS_OK;
}

void AtlasModel::DestroyInput()
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

void AtlasModel::DestroyOutput()
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

void AtlasModel::Unload()
{
    if (!loadFlag_) {
        ATLAS_LOG_INFO("Model(%s) had not been loaded or unload already", 
                       modelPath_.c_str());
        return;
    }

    aclError ret = aclmdlUnload(modelId_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Unload model(%s) error:%d", modelPath_.c_str(), ret);
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
    ATLAS_LOG_INFO("Unload model %s success", modelPath_.c_str());
}


