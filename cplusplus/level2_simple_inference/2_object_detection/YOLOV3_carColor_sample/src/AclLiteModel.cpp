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
#include "AclLiteModel.h"
#include "AclLiteUtils.h"
using namespace std;

AclLiteModel::AclLiteModel(const string& modelPath)
    : g_loadFlag_(false),
      g_isReleased_(false),
      g_modelId_(0),
      g_outputsNum_(0),
      g_modelMemSize_(0),
      g_modelWorkSize_(0),
      g_modelWeightSize_(0),
      g_modelMemPtr_(nullptr),
      g_modelWorkPtr_(nullptr),
      g_modelWeightPtr_(nullptr),
      g_modelDesc_(nullptr),
      g_input_(nullptr),
      g_output_(nullptr),
      g_modelPath_(modelPath)
{
}

AclLiteModel::~AclLiteModel()
{
    DestroyResource();
}

void AclLiteModel::DestroyResource()
{
    if (g_isReleased_) {
        return;
    }
    
    Unload();
    DestroyDesc();
    DestroyInput();
    DestroyOutput();
    g_isReleased_ = true;
}

AclLiteError AclLiteModel::Init()
{
    aclError aclRet = aclrtGetRunMode(&g_runMode_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acl get run mode failed");
        return ACLLITE_ERROR_GET_RUM_MODE;
    } 

    AclLiteError ret = LoadModelFromFile(g_modelPath_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Load model %s from file failed", g_modelPath_.c_str());
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

    ACLLITE_LOG_INFO("Init model %s success", g_modelPath_.c_str());

    return ACLLITE_OK;
}

AclLiteError AclLiteModel::LoadModelFromFile(const string& modelPath)
{
    if (g_loadFlag_) {
        ACLLITE_LOG_ERROR("%s is loaded already", modelPath.c_str());
        return ACLLITE_ERROR_LOAD_MODEL_REPEATED;
    }

    aclError ret = aclmdlLoadFromFile(modelPath.c_str(), &g_modelId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Load model(%s) from file return %d",
                          modelPath.c_str(), ret);
        return ACLLITE_ERROR_LOAD_MODEL;
    }

    g_loadFlag_ = true;
    ACLLITE_LOG_INFO("Load model %s success", modelPath.c_str());

    return ACLLITE_OK;
}

AclLiteError AclLiteModel::SetDesc()
{
    g_modelDesc_ = aclmdlCreateDesc();
    if (g_modelDesc_ == nullptr) {
        ACLLITE_LOG_ERROR("Create model(%s) description failed",
                          g_modelPath_.c_str());
        return ACLLITE_ERROR_CREATE_MODEL_DESC;
    }

    aclError ret = aclmdlGetDesc(g_modelDesc_, g_modelId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Get model(%s) description failed",
                          g_modelPath_.c_str());
        return ACLLITE_ERROR_GET_MODEL_DESC;
    }

    ACLLITE_LOG_INFO("Create model description success");
    return ACLLITE_OK;
}

void AclLiteModel::DestroyDesc()
{
    if (g_modelDesc_ != nullptr) {
        (void)aclmdlDestroyDesc(g_modelDesc_);
        g_modelDesc_ = nullptr;
    }
}

AclLiteError AclLiteModel::CreateInput(void *input, uint32_t size)
{
    vector<DataInfo> inputData = {{input, size}};
    return CreateInput(inputData);
}

AclLiteError AclLiteModel::CreateInput(void *input1, uint32_t input1Size,
                                 void* input2, uint32_t input2Size)
{
    vector<DataInfo> inputData = {{input1, input1Size}, {input2, input2Size}};
    return CreateInput(inputData);
}

AclLiteError AclLiteModel::CreateInput(vector<DataInfo>& inputData) {
    uint32_t dataNum = aclmdlGetNumInputs(g_modelDesc_);

    if (dataNum == 0) {
        ACLLITE_LOG_ERROR("Create input failed for no input data");
        return ACLLITE_ERROR_INVALID_ARGS;
    }
    if (dataNum != inputData.size()) {
        ACLLITE_LOG_ERROR("Create input failed for wrong input nums");
        return ACLLITE_ERROR_INVALID_ARGS;
    }

    g_input_ = aclmdlCreateDataset();
    if (g_input_ == nullptr) {
        ACLLITE_LOG_ERROR("Create input failed for create dataset failed");
        return ACLLITE_ERROR_CREATE_DATASET;
    }

    for (uint32_t i = 0; i < inputData.size(); i++) {
        size_t modelInputSize = aclmdlGetInputSizeByIndex(g_modelDesc_, i);
        if (modelInputSize != inputData[i].size) {
            ACLLITE_LOG_WARNING("Input size verify failed "
                                "input[%d] size: %ld, provide size : %d",
                                i, modelInputSize, inputData[i].size);
        }
        AclLiteError atlRet = AddDatasetBuffer(g_input_, 
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
    if (g_modelDesc_ == nullptr) {
        ACLLITE_LOG_ERROR("Create output failed for no model(%s) description",
                          g_modelPath_.c_str());
        return ACLLITE_ERROR_NO_MODEL_DESC;
    }

    g_output_ = aclmdlCreateDataset();
    if (g_output_ == nullptr) {
        ACLLITE_LOG_ERROR("Create output failed for create dataset error");
        return ACLLITE_ERROR_CREATE_DATASET;
    }

    g_outputsNum_ = aclmdlGetNumOutputs(g_modelDesc_);
    for (size_t i = 0; i < g_outputsNum_; ++i) {
        size_t bufSize = aclmdlGetOutputSizeByIndex(g_modelDesc_, i);

        void *outputBuffer = nullptr;
        aclError ret = aclrtMalloc(&outputBuffer, bufSize, 
                                   ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("Create output failed for malloc "
                              "device failed, size %d", (int)bufSize);
            return ACLLITE_ERROR_MALLOC_DEVICE;
        }

        AclLiteError atlRet = AddDatasetBuffer(g_output_, outputBuffer, bufSize);
        if (atlRet != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Create output failed for "
                              "add dataset buffer error %d", atlRet);
            aclrtFree(outputBuffer);
            return ACLLITE_ERROR_ADD_DATASET_BUFFER;
        }
    }

    ACLLITE_LOG_INFO("Create model(%s) output success", g_modelPath_.c_str());
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
    aclError ret = aclmdlExecute(g_modelId_, g_input_, g_output_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Execute model(%s) error:%d", g_modelPath_.c_str(), ret);
        return ACLLITE_ERROR_EXECUTE_MODEL;
    }

    for (uint32_t i = 0; i < g_outputsNum_; i++) {
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
    aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(g_output_, idx);
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
                                g_runMode_, MEMORY_NORMAL);
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
    if (g_input_ == nullptr) {
        return;
    }

    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(g_input_); ++i) {
        aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(g_input_, i);
        aclDestroyDataBuffer(dataBuffer);
        dataBuffer = nullptr;
    }
    aclmdlDestroyDataset(g_input_);
    g_input_ = nullptr;
}

void AclLiteModel::DestroyOutput()
{
    if (g_output_ == nullptr) {
        return;
    }

    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(g_output_); ++i) {
        aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(g_output_, i);
        void* data = aclGetDataBufferAddr(dataBuffer);
        (void)aclrtFree(data);
        (void)aclDestroyDataBuffer(dataBuffer);
        dataBuffer = nullptr;
    }

    (void)aclmdlDestroyDataset(g_output_);
    g_output_ = nullptr;
}

void AclLiteModel::Unload()
{
    if (!g_loadFlag_) {
        ACLLITE_LOG_INFO("Model(%s) had not been loaded or unload already", 
                         g_modelPath_.c_str());
        return;
    }

    aclError ret = aclmdlUnload(g_modelId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Unload model(%s) error:%d", g_modelPath_.c_str(), ret);
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
    ACLLITE_LOG_INFO("Unload model %s success", g_modelPath_.c_str());
}