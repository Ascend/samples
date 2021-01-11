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

#define INFO_LOG(fmt, ...) fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__)
#define WARN_LOG(fmt, ...) fprintf(stdout, "[WARN]  " fmt "\n", ##__VA_ARGS__)
#define ERROR_LOG(fmt, ...) fprintf(stdout, "[ERROR] " fmt "\n", ##__VA_ARGS__)

typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
} Result;

typedef struct PicDesc {
    std::string picName;
    uint32_t width;
    uint32_t height;
    uint32_t jpegDecodeSize;
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
    * @brief create device buffer of pic
    * @param [in] picDesc: pic desc
    * @param [out] picDevBuffer: device memory of picture
    * @param [out] devPicBufferSize: size of pic
    * @return device buffer of pic
    */
    static Result GetDeviceBufferOfPicture(PicDesc &picDesc, void *&picDevBuffer, uint32_t &devPicBufferSize);

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
    * @return result
    */
    static Result CheckPathIsFile(const std::string &fileName);
};

