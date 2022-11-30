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
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

#define INFO_LOG(fmt, ...) fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__)
#define WARN_LOG(fmt, ...) fprintf(stdout, "[WARN]  " fmt "\n", ##__VA_ARGS__)
#define ERROR_LOG(fmt, ...) fprintf(stderr, "[ERROR]  " fmt "\n", ##__VA_ARGS__)

#define IMAGE_FILE_NAME_LEN 1024

typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
} Result;

typedef struct PictureDesc {
    std::string fileName = "";
    uint32_t bufferSize = 0;
    acldvppPixelFormat format = PIXEL_FORMAT_YUV_400;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t widthStride = 0;
    uint32_t heightStride = 0;
} PictureDesc;

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
    * @param [in] picDesc: picture description
    * @return device buffer of picture
    */
    static void *GetPicDevBuffer(const PictureDesc &picDesc);

    /**
    * @brief save dvpp output data
    * @param [in] fileName: file name
    * @param [in] devPtr: dvpp output data device addr
    * @param [in] dataSize: dvpp output data size
    * @return result
    */
    static Result SaveDvppOutputData(const char *fileName, void *devPtr, uint32_t dataSize);
private:
    static Result AlignSpImage(const PictureDesc &picDesc, uint8_t *inBuffer, uint8_t *imageBuffer,
        uint32_t imageBufferSize, aclrtMemcpyKind memcpyType);

    static Result AlignPackedImage(const PictureDesc &picDesc, uint8_t *inBuffer, uint8_t *imageBuffer,
        uint32_t imageBufferSize, aclrtMemcpyKind memcpyType);
};

