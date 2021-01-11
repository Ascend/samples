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
#include <string>
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

typedef struct PicDesc {
    std::string picName;
    int width;
    int height;
} PicDesc;

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
    * @brief write device memory to file
    * @param [in] fileName: file name
    * @param [in] buffer of input data
    * @param [in] dataSize: size of data
    * @return success or fail
    */
    static bool WriteDeviceMemoryToFile(const char *fileName, void *dataDev, uint32_t dataSize);

    /**
    * @brief read file to device memory
    * @param [in] fileName: file name
    * @param [out] buffer of input data
    * @param [out] dataSize: size of data
    * @return success or fail
    */
    static bool ReadFileToDeviceMem(const char *fileName, void *&dataDev, uint32_t &dataSize);

    /**
    * @brief pull model output data to file
    * @param [in] modelOutput: model output dataset
    * @param [in] fileName: file name
    * @return result
    */
    static Result PullModelOutputData(aclmdlDataset *modelOutput, const char *fileName);

    /**
    * @brief save model output data to dst file
    * @param [in] srcfileName: src file name
    * @param [in] dstfileName: dst file name
    * @return result
    */
    static Result SaveModelOutputData(const char *srcfileName, const char *dstfileName);

    /**
    * @brief save dvpp output data
    * @param [in] fileName: file name
    * @param [in] devPtr: dvpp output data device addr
    * @param [in] dataSize: dvpp output data size
    * @return result
    */
    static Result SaveDvppOutputData(const char *fileName, const void *devPtr, uint32_t dataSize);

    /**
    * @brief check file if exist
    * @param [in] fileName: file to check
    * @return result
    */
    static Result CheckFile(const char* fileName);

    /**
    * @brief check fold, if not exist, create it
    * @param [in] fileName: fold to check
    * @return result
    */
    static Result CheckAndCreateFolder(const char *foldName);

    /**
    * @brief read file of a dir
    * @param [in] fileName: folder
    * @return fileList
    */
    static std::vector<std::string> ReadDir(const char *folder);

    /**
    * @brief remove dir
    * @param [in] fileName: folder
    * @return fileList
    */
    static void RemoveDir(const char* outFolder_);

    /**
    * @brief program waiting
    * @param [in] wating time: seconds
    * @return void
    */
    static void SleepTime(unsigned int seconds);
};

