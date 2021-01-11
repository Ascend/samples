/**
* @file memory_pool.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#ifndef MEMORY_POOL_H_
#define MEMORY_POOL_H_

#include <iostream>
#include <vector>
#include <mutex>
#include <map>
#include "utils.h"
#include "acl/acl.h"

class MemoryPool {
public:
    /**
    * @brief Constructor
    */
    MemoryPool() {}

    /**
    * @brief Destructor
    */
    virtual ~MemoryPool() {}

    /**
    * @brief get memory pool Instance
    */
    static MemoryPool* Instance()
    {
        static MemoryPool memPool;
        return &memPool;
    }

    /**
    * @brief init memory pool
    * @param [in] modelDesc: model description
    */
    Result Init(aclmdlDesc *modelDesc);

    /**
    * @brief destroy memory pool
    */
    void Destroy();

    /**
    * @brief malloc memory
    * @param [out] intput: intput dataset
    * @param [out] output: output dataset
    */
    Result MallocMemory(aclmdlDataset *&input, aclmdlDataset *&output);

    /**
    * @brief create model input
    * @param [in] inputDataBuffer: input buffer
    * @param [in] bufferSize: input buffer size
    * @param [out] input: input dataset
    * @param [in] modelDesc: model description
    * @return result
    */
    Result CreateInput(void *inputDataBuffer, size_t bufferSize, aclmdlDataset *&input, aclmdlDesc *modelDesc);

    /**
    * @brief create output buffer
    * @param [out] output: output dataset
    * @param [in] modelDesc: model description
    * @return result
    */
    Result CreateOutput(aclmdlDataset *&output, aclmdlDesc *modelDesc);

    /**
    * @brief free memory
    * @param [in] intput: intput dataset
    * @param [in] output: output dataset
    */
    void FreeMemory(aclmdlDataset *input, aclmdlDataset *output);

    /**
    * @brief destroy dataset
    * @param [in] dataset: dataset
    */
    static void DestroyDataset(aclmdlDataset *dataset);

private:
    std::recursive_mutex freePoolMutex_;
    std::map<aclmdlDataset *, aclmdlDataset *> freeMemoryPool_;
};

#endif // MEMORY_POOL_H_
