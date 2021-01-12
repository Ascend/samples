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
#include "dvpp_process.h"
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
    virtual ~SampleProcess();

    /**
    *@brief set deviceid
    */
    void SetDeviceId(int deviceId);

    /**
    * @brief init reousce
    * @return result
    */
    Result InitResource();

    /**
    * @brief crop picture
    * @return result
    */
    Result BatchCropProcess();

    void DestroyResource();

private:
    std::vector<InputArea> BatchInputCrop();
    int32_t deviceId_;
    aclrtContext context_;
    aclrtStream stream_;
};
