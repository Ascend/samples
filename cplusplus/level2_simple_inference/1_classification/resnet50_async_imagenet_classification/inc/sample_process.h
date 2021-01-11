/**
* @file sample_process.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#ifndef SAMPLE_PROCESS_H_
#define SAMPLE_PROCESS_H_

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
    * @brief sample process
    * @return result
    */
    Result Process();

    /**
    * @brief callback process
    * @return result
    */
    static void ProcessCallback(void *arg);
public:
    static aclrtContext context_;
    static aclrtStream stream_;
private:
    void DestroyResource();

    int32_t deviceId_;

};

#endif // SAMPLE_PROCESS_H_
