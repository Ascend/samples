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

#include "atlas_error.h"
#include "atlas_type.h"

using namespace std;

extern "C" { 

/**
 * @brief calculate RGB 24bits image size
 * @param [in] width:  image width
 * @param [in] height: image height
 * @return bytes size of image
 */
#define RGBU8_IMAGE_SIZE(width, height) ((width) * (height) * 3)

/**
 * @brief calculate YUVSP420 image size
 * @param [in] width:  image width
 * @param [in] height: image height
 * @return bytes size of image
 */
#define YUV420SP_SIZE(width, height) ((width) * (height) * 3 / 2)

/**
 * @brief generate shared pointer of dvpp memory
 * @param [in] buf: memory pointer, malloc by acldvppMalloc
 * @return shared pointer of input buffer
 */
#define SHARED_PRT_DVPP_BUF(buf) (shared_ptr<uint8_t>((uint8_t *)(buf), [](uint8_t* p) { acldvppFree(p); }))

/**
 * @brief generate shared pointer of memory
 * @param [in] buf memory pointer, malloc by new
 * @return shared pointer of input buffer
 */
#define SHARED_PRT_U8_BUF(buf) (shared_ptr<uint8_t>((uint8_t *)(buf), [](uint8_t* p) { delete[](p); }))

/**
 * @brief calculate aligned number
 * @param [in] num: the original number that to aligned
 * @param [in] align: the align factor 
 * @return the number after aligned
 */
#define ALIGN_UP(num, align) (((num) + (align) - 1) & ~((align) - 1))

/**
 * @brief calculate number align with 2
 * @param [in] num: the original number that to aligned
 * @return the number after aligned
 */
#define ALIGN_UP2(num) ALIGN_UP(num, 2)

/**
 * @brief calculate number align with 16
 * @param [in] num: the original number that to aligned
 * @return the number after aligned
 */
#define ALIGN_UP16(num) ALIGN_UP(num, 16)

/**
 * @brief calculate number align with 128
 * @param [in] num: the original number that to aligned
 * @return the number after aligned
 */
#define ALIGN_UP128(num) ALIGN_UP(num, 128)

/**
 * @brief Write acl error level log to host log
 * @param [in] fmt: the input format string
 * @return none
 */
#define ATLAS_LOG_ERROR(fmt, ...) \
    do{aclAppLog(ACL_ERROR, __FUNCTION__, __FILE__, __LINE__, fmt, ##__VA_ARGS__); \
      fprintf(stdout, "[ERROR]  " fmt "\n", ##__VA_ARGS__);}while(0)

/**
 * @brief Write acl info level log to host log
 * @param [in] fmt: the input format string
 * @return none
 */
#define ATLAS_LOG_INFO(fmt, ...) \
    do{aclAppLog(ACL_INFO, __FUNCTION__, __FILE__, __LINE__, fmt, ##__VA_ARGS__); \
    fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__);}while(0)

/**
 * @brief Write acl debug level log to host log
 * @param [in] fmt: the input format string
 * @return none
 */
#define ATLAS_LOG_DEBUG(fmt, ...) \
    do{aclAppLog(ACL_DEBUG, __FUNCTION__, __FILE__, __LINE__, fmt, ##__VA_ARGS__); \
    fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__);}while(0)

/**
 * @brief Recognize the string is a accessable directory or not
 * @param [in] path: the input string
 * @return bool  true: is directory; false: not directory
 */
bool IsDirectory(const string &path);

/**
 * @brief Copy data to device
 * @param [in] data: The data to be copy
 * @param [in] size: The data bytes size
 * @param [in] curRunMode: The run mode, get by aclrtGetRunMode,
 *                         Atlas200DK is ACL_DEVICE, Atlas300 is ACL_HOST
 * @param [in] memType: The dest memory type:MEMORY_NORMAL(in Atlas200DK),
 *                      MEMORY_DEVICE, MEMORY_DVPP  
 * @return void* The dest memory pointer
 */
void* CopyDataToDevice(void* data, uint32_t size, 
                       aclrtRunMode curRunMode, MemoryType memType);
/**
 * @brief Copy data to host
 * @param [in] data: The data to be copy
 * @param [in] size: The data bytes size
 * @param [in] curRunMode: The run mode, get by aclrtGetRunMode,
 *                         Atlas200DK is ACL_DEVICE, Atlas300 is ACL_HOST
 * @param [in] memType: The dest memory type:MEMORY_NORMAL, MEMORY_HOST
 * @return void* The dest memory pointer
 */
void* CopyDataToHost(void* data, uint32_t size, 
                     aclrtRunMode curRunMode, MemoryType memType);
/**
 * @brief Copy data to memory
 * @param [in] data: The data to be copy
 * @param [in] size: The data bytes size
 * @param [in] policy: the kind of sync,
 *                   typedef enum aclrtMemcpyKind {
 *                       ACL_MEMCPY_HOST_TO_HOST, // Host内的内存复制
 *                       ACL_MEMCPY_HOST_TO_DEVICE, // Host到Device的内存复制
 *                       ACL_MEMCPY_DEVICE_TO_HOST, // Device到Host的内存复制
 *                       ACL_MEMCPY_DEVICE_TO_DEVICE, // Device内的内存复制
 *                       } aclrtMemcpyKind;
 * @param [in] memType: The dest memory type
 * @return void* The dest memory pointer
 */
void* CopyData(void* data, uint32_t size, 
               aclrtMemcpyKind policy, MemoryType memType);


/**
 * @brief Get all files from file list string
 * @param [in] path: files list string, seperate by ',', 
 *                   the element could be file path or directory 
 * @param [in] fileName: The data bytes size
 * @return AtlasError ATLAS_OK: read success 
 *                    others: read failed
 */  
void GetAllFiles(const string &path, vector<string> &file_vec);

/**
 * @brief Save data to binary file
 * @param [in] filename: binary file name with path
 * @param [in] data: binary data
 * @param [in] size: bytes size of data
 * @return AtlasError ATLAS_OK: read success 
 *                    others: read failed
 */
void SaveBinFile(const string& filename, void* data, uint32_t size);

/**
 * @brief Save data to binary file
 * @param [in] filename: binary file name with path
 * @param [in] data: binary data
 * @param [in] size: bytes size of data
 * @return AtlasError ATLAS_OK: read success 
 *                    others: read failed
 */
AtlasError ReadBinFile(const string& filename, void*& data, uint32_t& size);

bool ReadConfig(map<string, string>& config,  const char* configFile);
void PrintConfig(const map<string, string> & m);

bool IsVideoFile(const string& str);

bool IsRtspAddr(const string &str);

}