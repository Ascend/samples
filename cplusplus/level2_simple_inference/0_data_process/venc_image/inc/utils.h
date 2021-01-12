/**
* @file utils.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#pragma once
#include <iostream>
#include <fstream>
#include <cstring>
#include <vector>
#include <sys/types.h>
#include <sys/stat.h>
#include <map>
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

#define INFO_LOG(fmt, ...) fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__)
#define WARN_LOG(fmt, ...) fprintf(stdout, "[WARN]  " fmt "\n", ##__VA_ARGS__)
#define ERROR_LOG(fmt, ...) fprintf(stdout, "[ERROR] " fmt "\n", ##__VA_ARGS__)

typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
} Result;

class RunStatus {
public:
    static void SetDeviceStatus(bool isDevice)
    {
        isDevice_ = isDevice;
    }
    static bool GetDeviceStatus()
    {
        return isDevice_;
    }
private:
    RunStatus() = default;
    ~RunStatus() = default;
    static bool isDevice_;
};

class Utils {
public:
    /**
    * @brief write venc result to file
    * @param [in] fileFp: file stream
    * @param [in] buffer of input data
    * @param [in] dataSize: size of data
    * @return success or fail
    */
    static bool WriteToFile(FILE *fileFp, void *dataDev, uint32_t dataSize);

    /**
    * @brief copy image to device memory
    * @param [in] fileName: file name
    * @param [out] buffer of input data
    * @param [out] dataSize: size of data
    * @return success or fail
    */
    static bool ReadFileToDeviceMem(const char *fileName, void *&dataDev, uint32_t &dataSize);

    /**
    * @brief check fold, if not exist, create it
    * @param [in] fileName: fold to check
    * @return result
    */
    static Result CheckAndCreateFolder(const char* foldName);
};
