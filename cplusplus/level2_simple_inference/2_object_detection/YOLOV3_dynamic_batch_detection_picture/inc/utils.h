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

#define INFO_LOG(fmt, ...) fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__)
#define WARN_LOG(fmt, ...) fprintf(stdout, "[WARN]  " fmt "\n", ##__VA_ARGS__)
#define ERROR_LOG(fmt, ...) fprintf(stderr, "[ERROR]   " fmt "\n", ##__VA_ARGS__)

typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
} Result;

typedef enum DynamicType {
    DYNAMIC_BATCH = 0,
    DYNAMIC_HW = 1
} DynamicType;

typedef struct DynamicInfo {
    DynamicType dynamicType = DYNAMIC_BATCH;
    uint32_t imageW = 0;
    uint32_t imageH = 0;
    uint64_t dynamicArr[2] = {0};
} DynamicInfo;

typedef struct ImageMemoryInfo {
    size_t imageDataSize = 0;
    size_t imageInfoSize = 0;
    void *imageDataBuf = nullptr;
    void *imageInfoBuf = nullptr;
} ImageMemoryInfo;

class Utils {
public:
    /**
     * @brief create device buffer of file
     * @param [in] fileName: file name
     * @param [in] imageW: image width
     * @param [in] imageH: image height
     * @param [out] imageMemInfo: device memory info of image
     * @return reult
     */
    static Result GetDeviceBufferOfFile(const std::string &fileName, uint32_t imageW, uint32_t imageH,
        ImageMemoryInfo &imageMemInfo);

    /**
     * @brief Check whether the path is a file.
     * @param [in] fileName: fold to check
     * @return result
     */
    static Result CheckPathIsFile(const std::string &fileName);

private:
    /**
     * @brief Get image info buf.
     * @param [in] imageWidth: width of image
     * @param [in] imageHeight: height of image
     * @param [out] imageMemInfo: device memory info of image
     * @return result
     */
    static Result GetImageInfoBuffer(uint32_t imageWidth, uint32_t imageHeight, ImageMemoryInfo &imageMemInfo);

    /**
     * @brief copy data to device memoty
     * @param [in] data: source data
     * @param [in] dataSize: source data size
     * @param [in] policy: copy type
     * @return dst data buffer
     */
    static void *CopyDataToDevice(void *data, uint32_t dataSize, aclrtMemcpyKind policy);

    /**
     * @brief create buffer of file
     * @param [in] fileName: file name
     * @param [out] fileSize: size of file
     * @return buffer of pic
     */
    static void *ReadBinFile(const std::string &fileName, uint32_t &fileSize);
};

#pragma once
