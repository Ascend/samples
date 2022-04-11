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
#include "AclLiteModel.h"
#include <iostream>
#include "AclLiteUtils.h"
using namespace std;

AclLiteModel::AclLiteModel(const string& modelPath):
  loadFlag_(false),
  isReleased_(false),
  modelId_(0),
  outputsNum_(0),
  modelMemSize_(0),
  modelWorkSize_(0), 
  modelWeightSize_(0),
  modelMemPtr_(nullptr),
  modelWorkPtr_(nullptr),
  modelWeightPtr_(nullptr),
  modelDesc_(nullptr), 
  input_(nullptr),
  output_(nullptr),
  modelPath_(modelPath) {
}

AclLiteModel::~AclLiteModel() {
    DestroyResource();
}

void AclLiteModel::DestroyResource() {
    if (isReleased_) {
        return;
    }
    
    Unload();
    DestroyDesc();
    DestroyInput();
    DestroyOutput();
    isReleased_ = true;
}

AclLiteError AclLiteModel::Init() {
    aclError aclRet = aclrtGetRunMode(&runMode_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acl get run mode failed");
        return ACLLITE_ERROR_GET_RUM_MODE;
    } 

    AclLiteError ret = LoadModelFromFile(modelPath_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Load model %s from file failed", modelPath_.c_str());
        return ret;
    }

    ret = SetDesc();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("execute SetDesc failed");
        return ret;
    }

    ret = CreateOutput();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("execute CreateOutput failed");
        return ret;
    }

    ACLLITE_LOG_INFO("Init model %s success", modelPath_.c_str());

    return ACLLITE_OK;
}

AclLiteError AclLiteModel::LoadModelFromFile(const string& modelPath) {
    if (loadFlag_) {
        ACLLITE_LOG_ERROR("%s is loaded already", modelPath.c_str());
        return ACLLITE_ERROR_LOAD_MODEL_REPEATED;
    }

    aclError ret = aclmdlLoadFromFile(modelPath.c_str(), &modelId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Load model(%s) from file return %d", 
                          modelPath.c_str(), ret);
        return ACLLITE_ERROR_LOAD_MODEL;
    }

    loadFlag_ = true;
    ACLLITE_LOG_INFO("Load model %s success", modelPath.c_str());

    return ACLLITE_OK;
}

AclLiteError AclLiteModel::SetDesc() {
    modelDesc_ = aclmdlCreateDesc();
    if (modelDesc_ == nullptr) {
        ACLLITE_LOG_ERROR("Create model(%s) description failed", 
                          modelPath_.c_str());
        return ACLLITE_ERROR_CREATE_MODEL_DESC;
    }

    aclError ret = aclmdlGetDesc(modelDesc_, modelId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Get model(%s) description failed", 
                          modelPath_.c_str());
        return ACLLITE_ERROR_GET_MODEL_DESC;
    }

    ACLLITE_LOG_INFO("Create model description success");
    return ACLLITE_OK;
}

void AclLiteModel::DestroyDesc()
{
    if (modelDesc_ != nullptr) {
        (void)aclmdlDestroyDesc(modelDesc_);
        modelDesc_ = nullptr;
    }
}

AclLiteError AclLiteModel::CreateInput(void *input, uint32_t size) {
    vector<DataInfo> inputData = {{input, size}};
    return CreateInput(inputData);
}

AclLiteError AclLiteModel::CreateInput(void *input1, uint32_t input1size, 
                                 void* input2, uint32_t input2size) {
    vector<DataInfo> inputData = {{input1, input1size}, {input2, input2size}};
    return CreateInput(inputData);
}

AclLiteError AclLiteModel::CreateInput(vector<DataInfo>& inputData) {
    uint32_t dataNum = aclmdlGetNumInputs(modelDesc_);

    if (dataNum == 0) {
        ACLLITE_LOG_ERROR("Create input failed for no input data");
        return ACLLITE_ERROR_INVALID_ARGS;
    }
    if (dataNum != inputData.size()) {
        ACLLITE_LOG_ERROR("Create input failed for wrong input nums");
        return ACLLITE_ERROR_INVALID_ARGS;
    }

    input_ = aclmdlCreateDataset();
    if (input_ == nullptr) {
        ACLLITE_LOG_ERROR("Create input failed for create dataset failed");
        return ACLLITE_ERROR_CREATE_DATASET;
    }

    for (uint32_t i = 0; i < inputData.size(); i++) {
        size_t modelInputSize = aclmdlGetInputSizeByIndex(modelDesc_, i);
        if (modelInputSize != inputData[i].size) {
            ACLLITE_LOG_WARNING("Input size verify failed "
                                "input[%d] size: %ld, provide size : %d", 
                                i, modelInputSize, inputData[i].size);
        }
        AclLiteError atlRet = AddDatasetBuffer(input_, 
                                             inputData[i].data,
                                             inputData[i].size);
        if (atlRet != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Create input failed for "
                              "add dataset buffer error %d", atlRet);
            return ACLLITE_ERROR_ADD_DATASET_BUFFER;
        }
    }

    return ACLLITE_OK;
}

AclLiteError AclLiteModel::CreateOutput()
{
    if (modelDesc_ == nullptr) {
        ACLLITE_LOG_ERROR("Create output failed for no model(%s) description",
                          modelPath_.c_str());
        return ACLLITE_ERROR_NO_MODEL_DESC;
    }

    output_ = aclmdlCreateDataset();
    if (output_ == nullptr) {
        ACLLITE_LOG_ERROR("Create output failed for create dataset error");
        return ACLLITE_ERROR_CREATE_DATASET;
    }

    outputsNum_ = aclmdlGetNumOutputs(modelDesc_);
    for (size_t i = 0; i < outputsNum_; ++i) {
        size_t bufSize = aclmdlGetOutputSizeByIndex(modelDesc_, i);

        void *outputBuffer = nullptr;
        aclError ret = aclrtMalloc(&outputBuffer, bufSize, 
                                   ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("Create output failed for malloc "
                              "device failed, size %d", (int)bufSize);
            return ACLLITE_ERROR_MALLOC_DEVICE;
        }

        AclLiteError atlRet = AddDatasetBuffer(output_, outputBuffer, bufSize);
        if (atlRet != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Create output failed for "
                              "add dataset buffer error %d", atlRet);
            aclrtFree(outputBuffer);
            return ACLLITE_ERROR_ADD_DATASET_BUFFER;
        }
    }

    ACLLITE_LOG_INFO("Create model(%s) output success", modelPath_.c_str());
    return ACLLITE_OK;
}

AclLiteError AclLiteModel::AddDatasetBuffer(aclmdlDataset *dataset, 
                                        void* buffer, uint32_t bufferSize) {
    aclDataBuffer* dataBuf = aclCreateDataBuffer(buffer, bufferSize);
    if (dataBuf == nullptr) {
        ACLLITE_LOG_ERROR("Create data buffer error");
        return ACLLITE_ERROR_CREATE_DATA_BUFFER;
    }

    aclError ret = aclmdlAddDatasetBuffer(dataset, dataBuf);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Add dataset buffer error %d", ret);
        ret = aclDestroyDataBuffer(dataBuf);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("Destroy dataset buffer error %d", ret);
        }
        dataBuf = nullptr;
        return ACLLITE_ERROR_ADD_DATASET_BUFFER;
    }

    return ACLLITE_OK;
}

AclLiteError AclLiteModel::Execute(vector<InferenceOutput>& inferOutputs, 
                               void *data, uint32_t size) {
    AclLiteError ret = CreateInput(data, size);                                        
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed");
        return ret;
    }

    ret = Execute(inferOutputs);
    if (ret != ACLLITE_OK) {
        DestroyInput();
        ACLLITE_LOG_ERROR("Execute model inference failed");
        return ret;
    }
    DestroyInput();

    return ACLLITE_OK;
}

AclLiteError AclLiteModel::Execute(vector<InferenceOutput>& inferOutputs) {
    aclError ret = aclmdlExecute(modelId_, input_, output_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Execute model(%s) error:%d", modelPath_.c_str(), ret);
        return ACLLITE_ERROR_EXECUTE_MODEL;
    }

    for (uint32_t i = 0; i < outputsNum_; i++) {
        InferenceOutput out;
        AclLiteError ret = GetOutputItem(out, i); 
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Get the %dth interference output failed, "
                              "error: %d", i, ret);
            return ret;
        }
        inferOutputs.push_back(out);
    }

    return ACLLITE_OK;
}

AclLiteError AclLiteModel::GetOutputItem(InferenceOutput& out,
                                     uint32_t idx) {
    aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(output_, idx);
    if (dataBuffer == nullptr) {
        ACLLITE_LOG_ERROR("Get the %dth dataset buffer from model "
                          "inference output failed", idx);
        return ACLLITE_ERROR_GET_DATASET_BUFFER;
    }

    void* dataBufferDev = aclGetDataBufferAddr(dataBuffer);
    if (dataBufferDev == nullptr) {
        ACLLITE_LOG_ERROR("Get the %dth dataset buffer address "
                          "from model inference output failed", idx);
        return ACLLITE_ERROR_GET_DATA_BUFFER_ADDR;
    }

    size_t bufferSize = aclGetDataBufferSize(dataBuffer);
    if (bufferSize == 0) {
        ACLLITE_LOG_ERROR("The %dth dataset buffer size of "
                          "model inference output is 0", idx);
        return ACLLITE_ERROR_GET_DATA_BUFFER_SIZE;
    }

    void* data = CopyDataToHost(dataBufferDev, bufferSize, 
                                runMode_, MEMORY_NORMAL);
    if (data == nullptr) {
        ACLLITE_LOG_ERROR("Copy inference output to host failed");
        return ACLLITE_ERROR_COPY_DATA;
    }

    out.data = SHARED_PTR_U8_BUF(data);
    out.size = bufferSize;

    return ACLLITE_OK;
}

void AclLiteModel::DestroyInput()
{
    if (input_ == nullptr) {
        return;
    }

    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(input_); ++i) {
        aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(input_, i);
        aclDestroyDataBuffer(dataBuffer);
        dataBuffer = nullptr;
    }
    aclmdlDestroyDataset(input_);
    input_ = nullptr;
}

size_t AclLiteModel::GetModelInputSize(int index){
    size_t modelInputSize = aclmdlGetInputSizeByIndex(modelDesc_, index);
    return modelInputSize;
}

void AclLiteModel::DestroyOutput()
{
    if (output_ == nullptr) {
        return;
    }

    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(output_); ++i) {
        aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(output_, i);
        void* data = aclGetDataBufferAddr(dataBuffer);
        (void)aclrtFree(data);
        (void)aclDestroyDataBuffer(dataBuffer);
        dataBuffer = nullptr;
    }

    (void)aclmdlDestroyDataset(output_);
    output_ = nullptr;
}

void AclLiteModel::Unload()
{
    if (!loadFlag_) {
        ACLLITE_LOG_INFO("Model(%s) had not been loaded or unload already", 
                         modelPath_.c_str());
        return;
    }

    aclError ret = aclmdlUnload(modelId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Unload model(%s) error:%d", modelPath_.c_str(), ret);
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
    ACLLITE_LOG_INFO("Unload model %s success", modelPath_.c_str());
}