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
#include <functional>
#include "utils.h"
#include "memory_pool.h"


using namespace std;
extern bool g_isDevice;
extern size_t g_executeTimes;
extern size_t g_callbackInterval;

ModelProcess::ModelProcess(aclrtStream &stream) :modelId_(0), modelWorkSize_(0), modelWeightSize_(0),
    modelWorkPtr_(nullptr), modelWeightPtr_(nullptr), loadFlag_(false), modelDesc_(nullptr)
{
    stream_ = stream;
}

ModelProcess::~ModelProcess()
{
    UnloadModel();
    DestroyModelDesc();
    DestroyMemPool();
}

Result ModelProcess::InitMemPool()
{
    MemoryPool *memPool = MemoryPool::Instance();
    return memPool->Init(modelDesc_);
}

void ModelProcess::DestroyMemPool()
{
    MemoryPool *memPool = MemoryPool::Instance();
    memPool->Destroy();
}

Result ModelProcess::LoadModel(const char *modelPath)
{
    if (loadFlag_) {
        ERROR_LOG("model has already been loaded");
        return FAILED;
    }

    aclError ret = aclmdlQuerySize(modelPath, &modelWorkSize_, &modelWeightSize_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("query model failed, model file is %s, errorCode is %d",
            modelPath, static_cast<int32_t>(ret));
        return FAILED;
    }

    // using ACL_MEM_MALLOC_HUGE_FIRST to malloc memory, huge memory is preferred to use
    // and huge memory can improve performance.
    ret = aclrtMalloc(&modelWorkPtr_, modelWorkSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("malloc buffer for work failed, require size is %zu, errorCode is %d",
            modelWorkSize_, static_cast<int32_t>(ret));
        return FAILED;
    }

    // using ACL_MEM_MALLOC_HUGE_FIRST to malloc memory, huge memory is preferred to use
    // and huge memory can improve performance.
    ret = aclrtMalloc(&modelWeightPtr_, modelWeightSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("malloc buffer for weight failed, require size is %zu, errorCode is %d",
            modelWeightSize_, static_cast<int32_t>(ret));
        return FAILED;
    }

    ret = aclmdlLoadFromFileWithMem(modelPath, &modelId_, modelWorkPtr_,
        modelWorkSize_, modelWeightPtr_, modelWeightSize_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("load model from file failed, model file is %s, errorCode is %d",
            modelPath, static_cast<int32_t>(ret));
        return FAILED;
    }

    loadFlag_ = true;
    INFO_LOG("load model %s success", modelPath);
    return SUCCESS;
}

Result ModelProcess::CreateModelDesc()
{
    modelDesc_ = aclmdlCreateDesc();
    if (modelDesc_ == nullptr) {
        ERROR_LOG("create model description failed");
        return FAILED;
    }

    aclError ret = aclmdlGetDesc(modelDesc_, modelId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("get model description failed, modelId is %u, errorCode is %d",
            modelId_, static_cast<int32_t>(ret));
        return FAILED;
    }

    INFO_LOG("create model description success");

    return SUCCESS;
}

void ModelProcess::DestroyModelDesc()
{
    if (modelDesc_ != nullptr) {
        (void)aclmdlDestroyDesc(modelDesc_);
        modelDesc_ = nullptr;
    }
    INFO_LOG("destroy model description success");
}

void ModelProcess::DumpModelOutputResult(aclmdlDataset *output)
{
    stringstream ss;
    size_t outputNum = aclmdlGetDatasetNumBuffers(output);
    static int executeNum = 0;
    for (size_t i = 0; i < outputNum; ++i) {
        ss << "output" << ++executeNum << "_" << i << ".bin";
        string outputFileName = ss.str();
        FILE *outputFile = fopen(outputFileName.c_str(), "wb");
        if (outputFile != nullptr) {
            // get model output data
            aclDataBuffer *dataBuffer = aclmdlGetDatasetBuffer(output, i);
            void *data = aclGetDataBufferAddr(dataBuffer);
            uint32_t len = aclGetDataBufferSize(dataBuffer);

            void *outHostData = nullptr;
            aclError ret = ACL_ERROR_NONE;
            if (!g_isDevice) {
                ret = aclrtMallocHost(&outHostData, len);
                if (ret != ACL_ERROR_NONE) {
                    ERROR_LOG("aclrtMallocHost failed, malloc len[%u], errorCode[%d]",
                        len, static_cast<int32_t>(ret));
                    fclose(outputFile);
                    return;
                }
                // if app is running in host, need copy model output data from device to host
                ret = aclrtMemcpy(outHostData, len, data, len, ACL_MEMCPY_DEVICE_TO_HOST);
                if (ret != ACL_ERROR_NONE) {
                    ERROR_LOG("aclrtMemcpy failed, errorCode[%d]", static_cast<int32_t>(ret));
                    (void)aclrtFreeHost(outHostData);
                    fclose(outputFile);
                    return;
                }

                fwrite(outHostData, len, sizeof(char), outputFile);

                ret = aclrtFreeHost(outHostData);
                if (ret != ACL_ERROR_NONE) {
                    ERROR_LOG("aclrtFreeHost failed, errorCode[%d]", static_cast<int32_t>(ret));
                    fclose(outputFile);
                    return;
                }
            } else {
                // if app is running in host, write model output data into result file
                fwrite(data, len, sizeof(char), outputFile);
            }
            fclose(outputFile);
        } else {
            ERROR_LOG("create output file [%s] failed", outputFileName.c_str());
            return;
        }
    }

    INFO_LOG("dump data success");
    return;
}


void ModelProcess::OutputModelResult(aclmdlDataset *output)
{
    for (size_t i = 0; i < aclmdlGetDatasetNumBuffers(output); ++i) {
        // get model output data
        aclDataBuffer *dataBuffer = aclmdlGetDatasetBuffer(output, i);
        void *data = aclGetDataBufferAddr(dataBuffer);
        uint32_t len = aclGetDataBufferSize(dataBuffer);

        void *outHostData = nullptr;
        aclError ret = ACL_ERROR_NONE;
        float *outData = nullptr;
        if (!g_isDevice) {
            aclError ret = aclrtMallocHost(&outHostData, len);
            if (ret != ACL_ERROR_NONE) {
                ERROR_LOG("aclrtMallocHost failed, malloc len[%u], errorCode[%d]",
                    len, static_cast<int32_t>(ret));
                return;
            }

            // if app is running in host, need copy model output data from device to host
            ret = aclrtMemcpy(outHostData, len, data, len, ACL_MEMCPY_DEVICE_TO_HOST);
            if (ret != ACL_ERROR_NONE) {
                ERROR_LOG("aclrtMemcpy failed, errorCode[%d]", static_cast<int32_t>(ret));
                (void)aclrtFreeHost(outHostData);
                return;
            }

            outData = reinterpret_cast<float *>(outHostData);
        } else {
            outData = reinterpret_cast<float *>(data);
        }
        map<float, unsigned int, greater<float>> resultMap;
        for (unsigned int j = 0; j < len / sizeof(float); ++j) {
            resultMap[*outData] = j;
            outData++;
        }

        int cnt = 0;
        for (auto it = resultMap.begin(); it != resultMap.end(); ++it) {
            // print top 1
            if (++cnt > 1) {
                break;
            }

            INFO_LOG("top %d: index[%d] value[%lf]", cnt, it->second, it->first);
        }
        if (!g_isDevice) {
            ret = aclrtFreeHost(outHostData);
            if (ret != ACL_ERROR_NONE) {
                ERROR_LOG("aclrtFreeHost failed, ret[%d]", ret);
                return;
            }
        }
    }

    INFO_LOG("output data success");
    return;
}

void ModelProcess::CallBackFunc(void *arg)
{
    std::map<aclmdlDataset *, aclmdlDataset *> *dataMap =
        (std::map<aclmdlDataset *, aclmdlDataset *> *)arg;

    MemoryPool *memPool = MemoryPool::Instance();

    for (auto& data : *dataMap) {
        // OutputModelResult prints the top 1 confidence value with index.
        // If want to dump output result to file in the current directory,
        // use function DumpModelOutputResult.
        ModelProcess::OutputModelResult(data.second);
        memPool->FreeMemory(data.first, data.second);
    }

    delete dataMap;
}

Result ModelProcess::ExecuteAsync()
{
    bool isCallback = (g_callbackInterval != 0);
    std::map<aclmdlDataset *, aclmdlDataset *> *dataMap = nullptr;
    aclmdlDataset *input = nullptr;
    aclmdlDataset *output = nullptr;
    MemoryPool *memPool = MemoryPool::Instance();
    for (size_t cnt = 0; cnt < g_executeTimes; ++cnt) {
        if (memPool->MallocMemory(input, output) != SUCCESS) {
            ERROR_LOG("get free memory failed");
            return FAILED;
        }
        aclError ret = aclmdlExecuteAsync(modelId_, input, output, stream_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("execute model async failed, modelId is %u, errorCode is %d",
                modelId_, static_cast<int32_t>(ret));
            memPool->FreeMemory(input, output);
            return FAILED;
        }

        if (isCallback) {
            if (dataMap == nullptr) {
                dataMap = new std::map<aclmdlDataset *, aclmdlDataset *>;
                if (dataMap == nullptr) {
                    ERROR_LOG("malloc for map failed, modelId is %u", modelId_);
                    memPool->FreeMemory(input, output);
                    return FAILED;
                }
            }
            // input and output of per model async execute are saved into map dataMap
            (*dataMap)[input] = output;
            if (((cnt+1) % g_callbackInterval) == 0) {
                // launch callback is to process all output data of model async execute
                // which are prior to this callback
                ret = aclrtLaunchCallback(CallBackFunc, (void *)dataMap, ACL_CALLBACK_BLOCK, stream_);
                if (ret != ACL_ERROR_NONE) {
                    ERROR_LOG("launch callback failed, index=%zu, errorCode is %d",
                        cnt, static_cast<int32_t>(ret));
                    memPool->FreeMemory(input, output);
                    delete dataMap;
                    return FAILED;
                }
                dataMap = nullptr;
            }

        }
    }

    if (dataMap != nullptr) {
        delete dataMap;
    }

    return SUCCESS;
}

Result ModelProcess::SynchronizeStream()
{
    aclError ret = aclrtSynchronizeStream(stream_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("aclrtSynchronizeStream failed, errorCode is %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    return SUCCESS;
}

void ModelProcess::UnloadModel()
{
    if (!loadFlag_) {
        WARN_LOG("no model had been loaded, unload failed");
        return;
    }

    aclError ret = aclmdlUnload(modelId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("unload model failed, modelId is %u, errorCode is %d",
            modelId_, static_cast<int32_t>(ret));
    }

    if (modelDesc_ != nullptr) {
        (void)aclmdlDestroyDesc(modelDesc_);
        modelDesc_ = nullptr;
    }

    if (modelWorkPtr_ != nullptr) {
        (void)aclrtFree(modelWorkPtr_);
        modelWorkPtr_ = nullptr;
        modelWorkSize_ = 0;
    }

    if (modelWeightPtr_ != nullptr) {
        (void)aclrtFree(modelWeightPtr_);
        modelWeightPtr_ = nullptr;
        modelWeightSize_ = 0;
    }

    loadFlag_ = false;
    INFO_LOG("unload model success, modelId is %u", modelId_);
    modelId_ = 0;
}
