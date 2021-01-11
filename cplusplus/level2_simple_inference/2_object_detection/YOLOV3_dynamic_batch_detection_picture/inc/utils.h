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

#define INFO_LOG(fmt, ...) fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__)
#define WARN_LOG(fmt, ...) fprintf(stdout, "[WARN]  " fmt "\n", ##__VA_ARGS__)
#define ERROR_LOG(fmt, ...) fprintf(stdout, "[ERROR]  " fmt "\n", ##__VA_ARGS__)

typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
} Result;

typedef enum DynamicType {
    DYNAMIC_BATCH = 0,
    DYNAMIC_HW = 1
} DynamicType;

typedef struct DynamicInfo {
    uint64_t dynamicArr[2] = {0};
    DynamicType dynamicType = DYNAMIC_BATCH;
} DynamicInfo;

class Utils {
public:
    /**
     * @brief create device buffer of file
     * @param [in] fileName: file name
     * @param [out] fileSize: size of file
     * @return device buffer of file
     */
    static void *GetDeviceBufferOfFile(std::string &fileName, uint32_t &fileSize);

    /**
     * @brief create buffer of file
     * @param [in] fileName: file name
     * @param [out] fileSize: size of file
     * @return buffer of pic
     */
    static void *ReadBinFile(std::string &fileName, uint32_t &fileSize);

    /**
     * @brief Check whether the path is a file.
     * @param [in] fileName: fold to check
     * @return result
     */
    static Result CheckPathIsFile(const std::string &fileName);
};

#pragma once
