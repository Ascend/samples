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
    * @brief decode, vpc and infer sample process
    * @param [in] dvpptype: dvpp type
    * @return result
    */
    Result JpegdProcess(DvppType dvpptype);

    /**
    * @brief encode sample process
    * @param [in] dvpptype: dvpp type
    * @return result
    */
    Result JpegeProcess(DvppType dvpptype);

    /**
    * @brief resize 8k sample process
    * @param [in] dvpptype: dvpp type
    * @return result
    */
    Result Resize8kProcess(DvppType dvpptype);

private:
    /**
    * @brief destroy resource
    */
    void DestroyResource();

    int32_t deviceId_;
    aclrtContext context_;
    aclrtStream stream_;
};

