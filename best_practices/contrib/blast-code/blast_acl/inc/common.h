/**
* @file common.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#ifndef COMMON_H
#define COMMON_H

#include <cstdio>
#include <string>
#include <vector>
#include <iostream>
#include <iomanip>

#include "acl/acl.h"

extern aclrtStream g_stream;

#define SUCCESS 0
#define FAILED 1

#define INFO_LOG(fmt, args...) fprintf(stdout, "[INFO]  " fmt "\n", ##args)
#define WARN_LOG(fmt, args...) fprintf(stdout, "[WARN]  " fmt "\n", ##args)
#define ERROR_LOG(fmt, args...) fprintf(stdout, "[ERROR]  " fmt "\n", ##args)
#define ACL_ERROR_LOG(fmt, args...) fprintf(stdout, "[ACL_ERROR]  " fmt "\n", ##args)

#define RETURN_IF_NOT_SUCCESS(expr, format, action)  \
    do {                             \
        if ((expr) != ACL_SUCCESS) { \
            ERROR_LOG(format);       \
            action;                  \
        }                            \
    } while (0)

#define IF_NOT_SUCCESS_RETURN_FALSE(expr, format, action)  \
    do {                             \
        if ((expr) != ACL_SUCCESS) { \
            ERROR_LOG(format);       \
            action;                  \
            return false;            \
        }                            \
    } while (0)

#define IF_NOT_SUCCESS_RETURN_FALSE_WITH_ARGS(expr, format, args, action)  \
    do {                             \
        if ((expr) != ACL_SUCCESS) { \
            ERROR_LOG(format, args); \
            action;                  \
            return false;            \
        }                            \
    } while (0)

/**
 * @brief Read data from file
 * @param [in] filePath: file path
 * @param [out] fileSize: file size
 * @return read result
 */
bool ReadFile(const std::string &filePath, size_t &fileSize, void *buffer, size_t bufferSize);

/**
 * @brief Write data to file
 * @param [in] filePath: file path
 * @param [in] buffer: data to write to file
 * @param [in] size: size to write
 * @return write result
 */
bool WriteFile(const std::string &filePath, const void *buffer, size_t size);

#endif // COMMON_H
