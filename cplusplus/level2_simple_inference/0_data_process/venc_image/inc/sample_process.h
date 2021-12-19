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
#include <thread>
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
    virtual ~SampleProcess();

    /**
    * @brief init reousce
    * @return result
    */
    Result InitResource();

    /**
    * @brief venc process
    * @return result
    */
    Result DoVencProcess();

private:
    /**
    * @brief destroy resource
    * @return result
    */
    void DestroyResource();

    int32_t deviceId_;
    aclrtContext context_;
    std::thread thread_;
    char *outFolder_;
    // 1：YUV420 semi-planner(nv12); 2：YVU420 semi-planner(nv21)
    acldvppPixelFormat format_;
    /* 
    * 0：H265 main level
    * 1：H264 baseline level
    * 2：H264 main level
    * 3：H264 high level
    */
    acldvppStreamFormat enType_;
};

