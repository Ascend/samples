/**
* Copyright 2020 Huawei Technologies Co., Ltd
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at

* http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

* File utils.h
* Description: handle file operations
*/
#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include <mutex>
#include <unistd.h>
#include <vector>
#include <string>
#include <map>
#include "acl/acl.h"

extern "C" { 

/**
 * @brief calculate YUVSP420 image size
 * @param [in] width:  image width
 * @param [in] height: image height
 * @return bytes size of image
 */
#define YUV420SP_SIZE(width, height) ((width) * (height) * 3 / 2)

/**
 * @brief Write acl error level log to host log
 * @param [in] fmt: the input format string
 * @return none
 */
#define ACLLITE_LOG_ERROR(fmt, ...) \
    do{aclAppLog(ACL_ERROR, __FUNCTION__, __FILE__, __LINE__, fmt, ##__VA_ARGS__); \
      fprintf(stdout, "[ERROR]  " fmt "\n", ##__VA_ARGS__);}while(0)

/**
 * @brief Write acl info level log to host log
 * @param [in] fmt: the input format string
 * @return none
 */
#define ACLLITE_LOG_INFO(fmt, ...) \
    do{aclAppLog(ACL_INFO, __FUNCTION__, __FILE__, __LINE__, fmt, ##__VA_ARGS__); \
    fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__);}while(0)

/**
 * @brief Write acl debug level log to host log
 * @param [in] fmt: the input format string
 * @return none
 */
#define ACLLITE_LOG_DEBUG(fmt, ...) \
    do{aclAppLog(ACL_DEBUG, __FUNCTION__, __FILE__, __LINE__, fmt, ##__VA_ARGS__); \
    fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__);}while(0)
}