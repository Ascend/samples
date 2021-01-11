/**
* @file singleOp_process.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#pragma once
#include "utils.h"
#include "acl/acl.h"

class SingleOpProcess {
public:
    /**
    * @brief Constructor
    * @param [in] stream: stream
    */
    explicit SingleOpProcess(aclrtStream &stream);

    /**
    * @brief Destructor
    */
    ~SingleOpProcess() = default;

    /**
    * @brief SingleOp init
    * @param [in] inputShape: input shape
    * @return result
    */
    Result Init(const int64_t inputShape);

    /**
    * @brief SingleOp input init
    * @param [in] dataSet: input
    * @return result
    */
    Result InitInput(const aclmdlDataset *dataSet);

    /**
    * @brief SingleOp process
    * @return result
    */
    Result Process();

    /**
    * @brief print Category result
    */
    void PrintResult();

    /**
    * @brief destry resource
    */
    void destroyResource();

private:
    // run cast
    Result RunSigleOpCast();

    // run ArgMaxD
    Result RunSigleOpArgMaxD();

    aclrtStream stream_;
    aclDataBuffer *inputBuffer_[1];
    aclDataBuffer *outputBufferCast_[1];
    aclDataBuffer *outputBufferArgMaxD_[1];
    size_t tensorSizeCast_;
    size_t tensorSizeArgMaxD_;
    void *devBufferCast_;
    void *devBufferArgMaxD_;
    int64_t inputShape_;
};

