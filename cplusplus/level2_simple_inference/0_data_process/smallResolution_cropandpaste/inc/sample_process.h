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
#include "utils.h"

class SampleProcess {
public:
    /**
    * @brief Constructor
    */
    SampleProcess();

    /**
    * @brief Destructor
    */
    virtual ~SampleProcess();

    /**
    * @brief init reousce
    * @return result
    */
    Result InitResource();

    /**
    * @brief get input parameter
    * @return result
    */
    Result GetInputOption(int argc, char **argv);

    /**
    * @brief vpc (crop and resize) process
    * @return result
    */
    Result VpcProcess();

private:
    /**
    * @brief destroy resource
    */
    void DestroyResource();

    /**
    * @brief get input parameter
    * @return result
    */
    Result GetInputOptionSecond(int32_t c);

    int32_t deviceId_;
    aclrtContext context_;
    aclrtStream stream_;
};

