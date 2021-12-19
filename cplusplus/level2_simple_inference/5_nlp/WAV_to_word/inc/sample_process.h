/**
* @file sample_process.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#pragma once
#include "acllite/AclLiteUtils.h"
#include "acllite/AclLiteModel.h"
#include "acl/acl.h"
#include <memory>

/**
* SampleProcess
*/
class SampleProcess {
public:
    /**
    * @brief Constructor
    */
    SampleProcess();

    /**
    * @brief Destructor
    */
    ~SampleProcess();

    /**
    * @brief init reousce
    * @return result
    */
    AclLiteError InitResource();

    /**
    * @brief sample process
    * @return result
    */
    AclLiteError Process();

    /**
    * @brief dump model output result to file
    */
    AclLiteError DumpModelOutputResult(std::vector<InferenceOutput>& modelOutput);

private:
    void DestroyResource();
    AclLiteError CreateInput();

    int32_t deviceId_;
    aclrtContext context_;
    aclrtStream stream_;
    uint32_t inputDataSize_;
    void*    inputBuf_;
    AclLiteModel model_;
    aclrtRunMode runMode_;
};

