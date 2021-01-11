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
#include "acl/acl.h"

#define INFO_LOG(fmt, ...) fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__)
#define WARN_LOG(fmt, ...) fprintf(stdout, "[WARN]  " fmt "\n", ##__VA_ARGS__)
#define ERROR_LOG(fmt, ...) fprintf(stdout, "[ERROR] " fmt "\n", ##__VA_ARGS__)

typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
} Result;

typedef enum DvppType {
    VPC_RESIZE = 0,
    VPC_CROP = 1,
    VPC_CROP_AND_PASTE = 2,
    JPEG_ENCODE = 3,
    VPC_8K_RESIZE = 4
} DvppType;

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
    * @param [out] devPicBufferSize: actual pic size
    * @return device buffer of pic
    */
    static Result GetPicDevBuffer4JpegD(PicDesc &picDesc, char *&picDevBuffer, uint32_t &devPicBufferSize);

    /**
    * @brief create buffer of bin file
    * @param [in] fileName: file name
    * @param [out] inputBuff: input data buffer
    * @param [out] fileSize: actual file szie
    * @return buffer of pic
    */
    static Result ReadBinFile(const std::string &fileName, void *&inputBuff, uint32_t &fileSize);

    /**
    * @brief create device buffer of pic
    * @param [in] picDesc: pic desc
    * @param [in] PicBufferSize: aligned pic size
    * @return device buffer of pic
    */
    static void *GetPicDevBuffer(const PicDesc &picDesc, uint32_t &PicBufferSize);

    /**
    * @brief pull model output data to file
    * @param [in] modelOutput: model output dataset
    * @param [in] fileName: file name
    * @return result
    */
    static Result PullModelOutputData(aclmdlDataset *modelOutput, const char *fileName);

    /**
    * @brief save dvpp output data
    * @param [in] fileName: file name
    * @param [in] devPtr: dvpp output data device addr
    * @param [in] dataSize: dvpp output data size
    * @return result
    */
    static Result SaveDvppOutputData(const char *fileName, void *devPtr, uint32_t dataSize);

    /**
    * @brief check file if exist
    * @param [in] fileName: file to check
    * @return result
    */
    static Result CheckFile(const char *fileName);

    /**
    * @brief save model output data to dst file
    * @param [in] srcfileName: src file name
    * @param [in] dstfileName: dst file name
    * @return result
    */
    static Result SaveModelOutputData(const char *srcfileName, const char *dstfileName);

    /**
    * @brief check fold, if not exist, create it
    * @param [in] fileName: fold to check
    * @return result
    */
    static Result CheckAndCreateFolder(const char* foldName);

    /**
    * @brief program waiting
    * @param [in] wating time: seconds
    * @return void
    */
    static void SleepTime(unsigned int seconds);
};

