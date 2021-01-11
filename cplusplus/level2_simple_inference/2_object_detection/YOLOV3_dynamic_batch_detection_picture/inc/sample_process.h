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

#include <map>
#include "model_process.h"
#include "utils.h"
#include "acl/acl.h"

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
    Result InitResource();

    /**
     * @brief check and fill dynamic parameter
     * @param [in] argc: parameter number
     * @param [in] argv: parameter array
     * @param [out] dynamicInfo: dynamic information
     * @return result
     */
    Result CheckAndFillDynamicPara(int argc, char **argv, DynamicInfo &dynamicInfo);

    /**
     * @brief sample process
     * @param [out] dynamicInfo: dynamic information
     * @return result
     */
    Result Process(const DynamicInfo &dynamicInfo);

private:
    Result SelectDynamicInputPara(DynamicType dynamicType, std::string &omModelPath,
        std::map<uint64_t, std::string> &testFile);

    void DestroyResource();

    int32_t deviceId_;
    aclrtContext context_;
    aclrtStream stream_;
};

