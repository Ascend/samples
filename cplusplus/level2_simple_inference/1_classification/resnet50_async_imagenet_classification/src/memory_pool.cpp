/**
* @file memory_pool.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include "memory_pool.h"

using namespace std;
extern size_t g_memoryPoolSize;

Result MemoryPool::Init(aclmdlDesc *modelDesc)
{
    string testFile[] = {
        "../data/dog1_1024_683.bin",
        "../data/dog2_1024_683.bin"
    };

    for (size_t i = 0; i < g_memoryPoolSize; ++i) {
        size_t index = i % (sizeof(testFile) / sizeof(testFile[0]));
        // model process
        uint32_t devBufferSize;
        void *picDevBuffer = nullptr;
        // copy image data to device buffer
        Result ret = Utils::GetDeviceBufferOfFile(testFile[index], picDevBuffer, devBufferSize);
         if (ret != SUCCESS) {
            ERROR_LOG("get pic device buffer failed, index is %zu", index);
            return FAILED;
        }

        aclmdlDataset *input = nullptr;
        ret = CreateInput(picDevBuffer, devBufferSize, input, modelDesc);
        if (ret != SUCCESS) {
            ERROR_LOG("execute CreateInput failed");
            aclrtFree(picDevBuffer);
            return FAILED;
        }
        aclmdlDataset *output = nullptr;
        ret = CreateOutput(output, modelDesc);
        if (ret != SUCCESS) {
            DestroyDataset(input);
            ERROR_LOG("execute CreateOutput failed");
            return FAILED;
        }
        {
            std::lock_guard<std::recursive_mutex> lk(freePoolMutex_);
            freeMemoryPool_[input] = output;
        }
    }
    return SUCCESS;
}

Result MemoryPool::CreateInput(void *inputDataBuffer, size_t bufferSize, aclmdlDataset *&input, aclmdlDesc *modelDesc)
{
    // om used in this sample has only one input
    if (modelDesc == nullptr) {
        ERROR_LOG("no model description, create input failed");
        return FAILED;
    }
    size_t modelInputSize = aclmdlGetInputSizeByIndex(modelDesc, 0);
    if (bufferSize != modelInputSize) {
        ERROR_LOG("input image size[%zu] is not equal to model input size[%zu]", bufferSize, modelInputSize);
        return FAILED;
    }

    input = aclmdlCreateDataset();
    if (input == nullptr) {
        ERROR_LOG("can't create dataset, create input failed");
        return FAILED;
    }

    aclDataBuffer* inputData = aclCreateDataBuffer(inputDataBuffer, bufferSize);
    if (inputData == nullptr) {
        ERROR_LOG("can't create data buffer, create input failed");
        (void)aclmdlDestroyDataset(input);
        input = nullptr;
        return FAILED;
    }

    aclError ret = aclmdlAddDatasetBuffer(input, inputData);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("add input dataset buffer failed, errorCode is %d", static_cast<int32_t>(ret));
        (void)aclDestroyDataBuffer(inputData);
        inputData = nullptr;
        (void)aclmdlDestroyDataset(input);
        input = nullptr;
        return FAILED;
    }

    return SUCCESS;
}

Result MemoryPool::CreateOutput(aclmdlDataset *&output, aclmdlDesc *modelDesc)
{
    if (modelDesc == nullptr) {
        ERROR_LOG("no model description, create ouput failed");
        return FAILED;
    }
    output = aclmdlCreateDataset();
    if (output == nullptr) {
        ERROR_LOG("can't create dataset, create output failed");
        return FAILED;
    }

    size_t outputSize = aclmdlGetNumOutputs(modelDesc);
    for (size_t i = 0; i < outputSize; ++i) {
        size_t modelOutputSize = aclmdlGetOutputSizeByIndex(modelDesc, i);

        void *outputBuffer = nullptr;
        aclError ret = aclrtMalloc(&outputBuffer, modelOutputSize, ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("can't malloc buffer, create output failed, size is %zu, errorCode is %d",
                modelOutputSize, static_cast<int32_t>(ret));
            DestroyDataset(output);
            return FAILED;
        }

        aclDataBuffer* outputData = aclCreateDataBuffer(outputBuffer, modelOutputSize);
        if (outputData == nullptr) {
            ERROR_LOG("can't create data buffer, create output failed");
            DestroyDataset(output);
            (void)aclrtFree(outputBuffer);
            return FAILED;
        }
        ret = aclmdlAddDatasetBuffer(output, outputData);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("can't add data buffer, create output failed");
            DestroyDataset(output);
            (void)aclrtFree(outputBuffer);
            (void)aclDestroyDataBuffer(outputData);
            return FAILED;
        }
    }

    return SUCCESS;
}

void MemoryPool::DestroyDataset(aclmdlDataset *dataset)
{
    if (dataset == nullptr) {
        return;
    }

    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(dataset); ++i) {
        aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(dataset, i);
        void *data = aclGetDataBufferAddr(dataBuffer);
        (void)aclrtFree(data);
        (void)aclDestroyDataBuffer(dataBuffer);
    }
    (void)aclmdlDestroyDataset(dataset);
    dataset = nullptr;
}

void MemoryPool::Destroy()
{
    std::lock_guard<std::recursive_mutex> lk(freePoolMutex_);
    for (auto &it : freeMemoryPool_) {
        DestroyDataset(it.first);
        DestroyDataset(it.second);
    }
    freeMemoryPool_.clear();
}

Result MemoryPool::MallocMemory(aclmdlDataset *&input, aclmdlDataset *&output)
{
    std::lock_guard<std::recursive_mutex> lk(freePoolMutex_);
    if (!freeMemoryPool_.empty()) {
        auto it = freeMemoryPool_.begin();
        input = it->first;
        output = it->second;
        freeMemoryPool_.erase(it);
        return SUCCESS;
    }
    WARN_LOG("no free memory");
    return FAILED;
}

void MemoryPool::FreeMemory(aclmdlDataset *input, aclmdlDataset *output)
{
    std::lock_guard<std::recursive_mutex> lk(freePoolMutex_);
    freeMemoryPool_[input] = output;
}
