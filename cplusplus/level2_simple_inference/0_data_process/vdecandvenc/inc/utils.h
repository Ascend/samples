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
#include <unistd.h>
#include <dirent.h>
#include <fstream>
#include <cstring>
#include <vector>
#include <sys/types.h>
#include <sys/stat.h>
#include <map>
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

#define INFO_LOG(fmt, args...) fprintf(stdout, "[INFO]  " fmt "\n", ##args)
#define WARN_LOG(fmt, args...) fprintf(stdout, "[WARN]  " fmt "\n", ##args)
#define ERROR_LOG(fmt, args...) fprintf(stdout, "[ERROR] " fmt "\n", ##args)

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
    RunStatus() {}
    ~RunStatus() {}
    static bool isDevice_;
};

/**
 * Utils
 */
class Utils {
public:
    /**
    * @brief get width and height of video
    * @param [in] fileFp: file stream
    * @param [in] buffer of input data
    * @param [in] dataSize: size of data
    * @return success or fail
    */
    static bool WriteToFile(FILE *fileFp, void *dataDev, uint32_t dataSize);

    /**
    * @brief check fold, if not exist, create it
    * @param [in] fileName: fold to check
    * @return result
    */
    static Result CheckFolder(const char* foldName);
};

