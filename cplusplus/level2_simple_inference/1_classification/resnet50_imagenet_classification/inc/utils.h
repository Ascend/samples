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
#include <string>
#include <iostream>
#include "acl/acl.h"

#define INFO_LOG(fmt, ...) fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__)
#define WARN_LOG(fmt, ...) fprintf(stdout, "[WARN]  " fmt "\n", ##__VA_ARGS__)
#define ERROR_LOG(fmt, ...)fprintf(stdout, "[ERROR] " fmt "\n", ##__VA_ARGS__)

typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
} Result;

class Utils {
public:
    /**
    * @brief create buffer of file
    * @param [in] fileName: file name
    * @param [out] inputBuff: input data buffer
    * @param [out] fileSize: size of file
    * @return result
    */
    static Result ReadBinFile(const std::string &fileName, void *&inputBuff, uint32_t &fileSize);

    /**
    * @brief create buffer of file
    * @param [in] fileName: file name
    * @param [out] picDevBuffer: input data device buffer which need to be memcpy
    * @param [out] inputBuffSize: size of inputBuff
    * @return result
    */
    static Result MemcpyFileToDeviceBuffer(const std::string &fileName, void *&picDevBuffer, size_t inputBuffSize);

    /**
    * @brief Check whether the path is a file.
    * @param [in] fileName: fold to check
    * @return result
    */
    static Result CheckPathIsFile(const std::string &fileName);
};

#pragma once
