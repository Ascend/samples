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
#include "atlasutil/atlas_utils.h"
#include "atlasutil/atlas_model.h"
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
    AtlasError InitResource();

    /**
    * @brief sample process
    * @return result
    */
    AtlasError Process();

    /**
    * @brief dump model output result to file
    */
    AtlasError DumpModelOutputResult(std::vector<InferenceOutput>& modelOutput);

private:
    void DestroyResource();
    AtlasError CreateInput();

    int32_t deviceId_;
    aclrtContext context_;
    aclrtStream stream_;
    uint32_t inputDataSize_;
    void*    inputBuf_;
    AtlasModel model_;
    aclrtRunMode runMode_;
};

