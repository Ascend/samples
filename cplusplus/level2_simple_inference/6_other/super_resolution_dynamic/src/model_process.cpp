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
#include "utils.h"
using namespace std;

ModelProcess::ModelProcess():loadFlag_(false), modelId_(0), modelMemPtr_(nullptr), modelMemSize_(0),
modelWeightPtr_(nullptr),modelWeightSize_(0), modelDesc_(nullptr), input_(nullptr), output_(nullptr) {

}

ModelProcess::~ModelProcess() {
}

Result ModelProcess::load_model_from_file_with_mem(const char *modelPath) {
    if (loadFlag_) {
        ERROR_LOG("has already loaded a model");
        return FAILED;
    }

    aclError ret = aclmdlQuerySize(modelPath, &modelMemSize_, &modelWeightSize_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("query model failed, model file is %s", modelPath);
        return FAILED;
    }

    ret = aclrtMalloc(&modelMemPtr_, modelMemSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("malloc buffer for mem failed, require size is %zu", modelMemSize_);
        return FAILED;
    }

    ret = aclrtMalloc(&modelWeightPtr_, modelWeightSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("malloc buffer for weight failed, require size is %zu", modelWeightSize_);
        return FAILED;
    }

    ret = aclmdlLoadFromFileWithMem(modelPath, &modelId_, modelMemPtr_,
        modelMemSize_, modelWeightPtr_, modelWeightSize_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("load model from file failed, model file is %s", modelPath);
        return FAILED;
    }

    loadFlag_ = true;
    INFO_LOG("load model %s success", modelPath);
    return SUCCESS;
}

Result ModelProcess::create_desc() {
    modelDesc_ = aclmdlCreateDesc();
    if (modelDesc_ == nullptr) {
        ERROR_LOG("create model description failed");
        return FAILED;
    }

    aclError ret = aclmdlGetDesc(modelDesc_, modelId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("get model description failed");
        return FAILED;
    }

    INFO_LOG("create model description success");
    return SUCCESS;
}

void ModelProcess::destroy_desc() {
    if (modelDesc_ != nullptr) {
        (void)aclmdlDestroyDesc(modelDesc_);
        modelDesc_ = nullptr;
    }
}

Result ModelProcess::create_input(void *inputDataBuffer, size_t bufferSize) {
    input_ = aclmdlCreateDataset();
    if (input_ == nullptr) {
        ERROR_LOG("can't create dataset, create input failed");
        return FAILED;
    }

    for (size_t index = 0; index < aclmdlGetNumInputs(modelDesc_); ++index) {
        const char *name = aclmdlGetInputNameByIndex(modelDesc_, index);
        if (name == nullptr) {
            ERROR_LOG("get input name failed, index = %zu.", index);
            return FAILED;
        }
        size_t inputLen = aclmdlGetInputSizeByIndex(modelDesc_, index);
        if (strcmp(name, ACL_DYNAMIC_TENSOR_NAME) == 0) {
            void *data = nullptr;
            auto ret = aclrtMalloc(&data, inputLen, ACL_MEM_MALLOC_HUGE_FIRST);
            if (ret != ACL_SUCCESS) {
                ERROR_LOG("malloc device memory failed, errorCode = %d.", static_cast<int32_t>(ret));
                return FAILED;
            }

            aclDataBuffer *dataBuffer = aclCreateDataBuffer(data, inputLen);
            if (dataBuffer == nullptr) {
                ERROR_LOG("create data buffer failed.");
                (void)aclrtFree(data);
                data = nullptr;
                return FAILED;
            }

            ret = aclmdlAddDatasetBuffer(input_, dataBuffer);
            if (ret != ACL_SUCCESS) {
                ERROR_LOG("add user input dataset buffer failed, errorCode = %d.", static_cast<int32_t>(ret));
                (void)aclrtFree(data);
                data = nullptr;
                (void)aclDestroyDataBuffer(dataBuffer);
                dataBuffer = nullptr;
                return FAILED;
            }
        } else {
            aclDataBuffer *inputData = aclCreateDataBuffer(inputDataBuffer, bufferSize);
            if (inputData == nullptr) {
                ERROR_LOG("can't create data buffer, create input failed.");
                (void)aclrtFree(inputDataBuffer);
                inputDataBuffer = nullptr;
                return FAILED;
            }

            aclError ret = aclmdlAddDatasetBuffer(input_, inputData);
            if (ret != ACL_SUCCESS) {
                ERROR_LOG("add input dataset buffer failed, errorCode = %d.", static_cast<int32_t>(ret));
                aclDestroyDataBuffer(inputData);
                inputData = nullptr;
                (void)aclrtFree(inputDataBuffer);
                inputDataBuffer = nullptr;
                return FAILED;
            }
        }
    }
    INFO_LOG("create model input success.");
    return SUCCESS;
}

void ModelProcess::destroy_input() {
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

Result ModelProcess::create_output() {
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

        ret = aclmdlAddDatasetBuffer(output_, outputData);
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

void ModelProcess::destroy_output() {
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

Result ModelProcess::set_dynamic_hw_size(uint64_t height, uint64_t width)
{
    size_t index = 0;
    aclError ret = aclmdlGetInputIndexByName(modelDesc_, ACL_DYNAMIC_TENSOR_NAME, &index);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("get input index by name[%s] failed, errorCode = %d.",
            ACL_DYNAMIC_TENSOR_NAME, static_cast<int32_t>(ret));
        return FAILED;
    }

    ret = aclmdlSetDynamicHWSize(modelId_, input_, index, height, width);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("set dynamic hw[%lu, %lu] failed, errorCode = %d.",
            height, width, static_cast<int32_t>(ret));
        return FAILED;
    }
    return SUCCESS;
}

Result ModelProcess::execute() {
    aclError ret = aclmdlExecute(modelId_, input_, output_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("execute model failed, modelId is %u", modelId_);
        return FAILED;
    }

    INFO_LOG("model execute success");
    return SUCCESS;
}

void ModelProcess::unload() {
    if (!loadFlag_) {
        WARN_LOG("no model had been loaded, unload failed");
        return;
    }

    aclError ret = aclmdlUnload(modelId_);
    if (ret != ACL_SUCCESS) {
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

aclmdlDataset *ModelProcess::get_model_output_data() {
    return output_;
}
