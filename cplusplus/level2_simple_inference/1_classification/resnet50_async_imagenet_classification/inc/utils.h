/**
* @file utils.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>
#include <string>

#define INFO_LOG(fmt, ...) fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__)
#define WARN_LOG(fmt, ...) fprintf(stdout, "[WARN]  " fmt "\n", ##__VA_ARGS__)
#define ERROR_LOG(fmt, ...) fprintf(stdout, "[ERROR] " fmt "\n", ##__VA_ARGS__)

typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
} Result;

class Utils {
public:
    /**
    * @brief create device buffer of file
    * @param [in] fileName: file name
    * @param [out] picDevBuffer: device memory of picture
    * @param [out] fileSize: size of file
    * @return result
    */
    static Result GetDeviceBufferOfFile(const std::string &fileName, void *&picDevBuffer, uint32_t &fileSize);

    /**
    * @brief create buffer of file
    * @param [in] fileName: file name
    * @param [out] inputBuff: input data buffer
    * @param [out] fileSize: size of file
    * @return result
    */
    static Result ReadBinFile(const std::string &fileName, void *&inputBuff, uint32_t &fileSize);

    /**
    * @brief Check whether the path is a file.
    * @param [in] fileName: fold to check
    * @return SUCCESS or FAILED
    */
    static Result CheckPathIsFile(const std::string &fileName);
};

#endif // UTILS_H_
