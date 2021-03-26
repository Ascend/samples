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

#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

#include "atlas_error.h"
#include "atlas_type.h"

/**
 * @brief calculate RGB 24bits image size
 * @param [in]: width:  image width
 * @param [in]: height: image height
 * @return bytes size of image
 */
#define RGBU8_IMAGE_SIZE(width, height) ((width) * (height) * 3)

/**
 * @brief calculate RGB C3F32 image size
 * @param [in]: width:  image width
 * @param [in]: height: image height
 * @return bytes size of image
 */
#define RGBF32_IMAGE_SIZE(width, height) ((width) * (height) * 3 * sizeof(float))

/**
 * @brief calculate YUVSP420 image size
 * @param [in]: width:  image width
 * @param [in]: height: image height
 * @return bytes size of image
 */
#define YUV420SP_SIZE(width, height) ((width) * (height) * 3 / 2)

/**
 * @brief calculate YUVSP420 nv12 load to opencv mat height paramter
 * @param [in]: height: yuv image height
 * @return bytes size of image
 */
#define YUV420SP_CV_MAT_HEIGHT(height) ((height) * 3 / 2)

/**
 * @brief generate shared pointer of dvpp memory
 * @param [in]: buf: memory pointer, malloc by acldvppMalloc
 * @return shared pointer of input buffer
 */
#define SHARED_PRT_DVPP_BUF(buf) (shared_ptr<uint8_t>((uint8_t *)(buf), [](uint8_t* p) { acldvppFree(p); }))

/**
 * @brief generate shared pointer of device memory
 * @param [in]: buf: memory pointer, malloc by acldvppMalloc
 * @return shared pointer of input buffer
 */
#define SHARED_PRT_DEV_BUF(buf) (shared_ptr<uint8_t>((uint8_t *)(buf), [](uint8_t* p) { aclrtFree(p); }))

/**
 * @brief generate shared pointer of memory
 * @param [in]: buf memory pointer, malloc by new
 * @return shared pointer of input buffer
 */
#define SHARED_PRT_U8_BUF(buf) (shared_ptr<uint8_t>((uint8_t *)(buf), [](uint8_t* p) { delete[](p); }))

/**
 * @brief calculate aligned number
 * @param [in]: num: the original number that to aligned
 * @param [in]: align: the align factor 
 * @return the number after aligned
 */
#define ALIGN_UP(num, align) (((num) + (align) - 1) & ~((align) - 1))

/**
 * @brief calculate number align with 2
 * @param [in]: num: the original number that to aligned
 * @return the number after aligned
 */
#define ALIGN_UP2(num) ALIGN_UP(num, 2)

/**
 * @brief calculate number align with 16
 * @param [in]: num: the original number that to aligned
 * @return the number after aligned
 */
#define ALIGN_UP16(num) ALIGN_UP(num, 16)

/**
 * @brief calculate number align with 128
 * @param [in]: num: the original number that to aligned
 * @return the number after aligned
 */
#define ALIGN_UP128(num) ALIGN_UP(num, 128)

/**
 * @brief calculate elements num of array
 * @param [in]: array: the array variable
 * @return elements num of array
 */
#define SIZEOF_ARRAY(array)  (sizeof(array)/sizeof(array[0]))

/**
 * @brief Write acl error level log to host log
 * @param [in]: fmt: the input format string
 * @return none
 */
#define ATLAS_LOG_ERROR(fmt, ...) \
    do{aclAppLog(ACL_ERROR, __FUNCTION__, __FILE__, __LINE__, fmt, ##__VA_ARGS__); \
      fprintf(stdout, "[ERROR]  " fmt "\n", ##__VA_ARGS__);}while(0)

/**
 * @brief Write acl info level log to host log
 * @param [in]: fmt: the input format string
 * @return none
 */
#define ATLAS_LOG_INFO(fmt, ...) \
    do{aclAppLog(ACL_INFO, __FUNCTION__, __FILE__, __LINE__, fmt, ##__VA_ARGS__); \
    fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__);}while(0)

/**
 * @brief Write acl debug level log to host log
 * @param [in]: fmt: the input format string
 * @return none
 */
#define ATLAS_LOG_DEBUG(fmt, ...) \
    do{aclAppLog(ACL_DEBUG, __FUNCTION__, __FILE__, __LINE__, fmt, ##__VA_ARGS__); \
    fprintf(stdout, "[INFO]  " fmt "\n", ##__VA_ARGS__);}while(0)

/**
 * @brief Recognize the string is a accessable directory or not
 * @param [in]: path: the input string
 * @return bool  true: is directory; false: not directory
 */
bool IsDirectory(const std::string &path);

/**
 * @brief Copy data to device
 * @param [in]: data: The data to copy
 * @param [in]: size: The data bytes size
 * @param [in]: curRunMode: The run mode, get by aclrtGetRunMode,
 *                         Atlas200DK is ACL_DEVICE, Atlas300 is ACL_HOST
 * @param [in]: memType: The dest memory type:MEMORY_NORMAL(in Atlas200DK),
 *                      MEMORY_DEVICE, MEMORY_DVPP  
 * @return void* The dest memory pointer
 */
void* CopyDataToDevice(const void* data, uint32_t size, 
                       aclrtRunMode curRunMode, MemoryType memType);

/**
 * @brief Copy data to device buffer
 * @param [in]: dest: The device buffer
 * @param [in]: destSize: The device buffer size
 * @param [in]: src: The data to copy
 * @param [in]: srcSize: The data bytes size
 * @param [in]: curRunMode: The run mode, get by aclrtGetRunMode,
 *                         Atlas200DK is ACL_DEVICE, Atlas300 is ACL_HOST
 * @return AtlasError ATLAS_OK: copy success 
 *                    others: copy failed
 */
AtlasError CopyDataToDeviceEx(void* dest, uint32_t destSize, 
                                 const void* src, uint32_t srcSize, 
                                 aclrtRunMode runMode);

/**
 * @brief Copy data to host
 * @param [in]: data: The data to be copy
 * @param [in]: size: The data bytes size
 * @param [in]: curRunMode: The run mode, get by aclrtGetRunMode,
 *                         Atlas200DK is ACL_DEVICE, Atlas300 is ACL_HOST
 * @param [in]: memType: The dest memory type:MEMORY_NORMAL, MEMORY_HOST
 * @return void* The dest memory pointer
 */
void* CopyDataToHost(const void* data, uint32_t size, 
                     aclrtRunMode curRunMode, MemoryType memType);

/**
 * @brief Copy data to host buffer
 * @param [in]: dest: The host buffer
 * @param [in]: destSize: The host buffer size
 * @param [in]: src: The data to copy
 * @param [in]: srcSize: The data bytes size
 * @param [in]: curRunMode: The run mode, get by aclrtGetRunMode,
 *                         Atlas200DK is ACL_DEVICE, Atlas300 is ACL_HOST
 * @return AtlasError ATLAS_OK: copy success 
 *                    others: copy failed
 */
AtlasError CopyDataToHostEx(void* dest, uint32_t destSize, 
                            const void* src, uint32_t srcSize, 
                            aclrtRunMode runMode);

/**
 * @brief Copy data to memory
 * @param [in]: data: The data to be copy
 * @param [in]: size: The data bytes size
 * @param [in]: policy: the kind of sync,
 *                   typedef enum aclrtMemcpyKind {
 *                       ACL_MEMCPY_HOST_TO_HOST, // Host内的内存复制
 *                       ACL_MEMCPY_HOST_TO_DEVICE, // Host到Device的内存复制
 *                       ACL_MEMCPY_DEVICE_TO_HOST, // Device到Host的内存复制
 *                       ACL_MEMCPY_DEVICE_TO_DEVICE, // Device内的内存复制
 *                       } aclrtMemcpyKind;
 * @param [in]: memType: The dest memory type
 * @return void* The dest memory pointer
 */
void* CopyData(const void* data, uint32_t size, 
               aclrtMemcpyKind policy, MemoryType memType);

/**
 * @brief Read jpeg image file. Only support baseline, not support progressive
 * @param [out]: image: image data read from file.          
 * @param [in]: fileName: The data bytes size
 * @return AtlasError ATLAS_OK: read success 
 *                    others: read failed
 */  
AtlasError ReadJpeg(ImageData& image, const std::string& fileName);

/**
 * @brief Get all files from file list string
 * @param [in]: pathList: files list string, seperate by ',', 
 *                   the element could be file path or directory 
 * @param [in]: fileVec: The data bytes size
 * @return AtlasError ATLAS_OK: read success 
 *                    others: read failed
 */  
void GetAllFiles(const std::string &pathList, 
                 std::vector<std::string> &fileVec);

/**
 * @brief Save data to binary file
 * @param [in]: filename: binary file name with path
 * @param [in]: data: binary data
 * @param [in]: size: bytes size of data
 * @return AtlasError ATLAS_OK: read success 
 *                    others: read failed
 */
void SaveBinFile(const std::string& filename, const void* data, uint32_t size);

/**
 * @brief Read binary file to buffer
 * @param [in]: filename: binary file name with path
 * @param [in]: data: buffer 
 * @param [in]: size: buffer size
 * @return AtlasError ATLAS_OK: read success 
 *                    others: read failed
 */
AtlasError ReadBinFile(const std::string& filename, 
                       void*& data, uint32_t& size);

/**
 * @brief Copy image to memory that malloc by new
 * @param [out]: destImage: The image after copy
 * @param [in]: srcImage: The image to copy 
 * @param [in]: curRunMode: The run mode, get by aclrtGetRunMode,
 *                          Atlas200DK is ACL_DEVICE, Atlas300 is ACL_HOST
 * @return AtlasError ATLAS_OK: read success 
 *                    others: read failed
 */                       
AtlasError CopyImageToLocal(ImageData& destImage,
                            ImageData& srcImage, aclrtRunMode curRunMode);

/**
 * @brief Copy image to acl device
 * @param [out]: destImage: The image after copy
 * @param [in]: srcImage: The image to copy 
 * @param [in]: curRunMode: The run mode, get by aclrtGetRunMode,
 *                          Atlas200DK is ACL_DEVICE, Atlas300 is ACL_HOST
 * @param [in]: memType: memory type, dvpp is MEMORY_DVPP, 
 *                       device is MEMPRY_DEVICE 
 * @return AtlasError ATLAS_OK: read success 
 *                    others: read failed
 */  
AtlasError CopyImageToDevice(ImageData& destImage, ImageData& srcImage, 
                             aclrtRunMode curRunMode, MemoryType memType);

/**
 * @brief Match ip address string as <1-255>.<0-255>.<0-255>.<0-255>:<port>
 * @param [in]: addrStr: Ip address string 
 * @return bool true: The input string match success
 *              false: is not match
 */ 
bool IsIpAddrWithPort(const std::string& addrStr);

/**
 * @brief Split ip address string <1-255>.<0-255>.<0-255>.<0-255>:<port> to 
 *        ip and port
 * @param [out]: ip: Ip address <1-255>.<0-255>.<0-255>.<0-255> 
 * @param [out]: port: port string 
 * @param [in]: addr: Ip address string 
 * @return None
 */ 
void ParseIpAddr(std::string& ip, std::string& port, const std::string& addr);

/**
 * @brief Judge input string is mp4 file path
 * @param [in]: path: file path 
 * @return bool true: input string is mp4 file path
 *              false: is not mp4 file path
 */ 
bool IsVideoFile(const std::string& path);

/**
 * @brief Judge input string is rtsp addr link rtsp://
 * @param [in]: str: input string 
 * @return bool true: input string is rtsp address
 *              false: is not rtsp address
 */ 
bool IsRtspAddr(const std::string &str);

/**
 * @brief Judge input string is digit string
 * @param [in]: str: input string 
 * @return bool true: input string is digit string
 *              false: is not rtsp address
 */ 
bool IsDigitStr(const std::string& str);

/**
 * @brief Test file path is exist or not
 * @param [in]: path: file path 
 * @return bool true: file path is exist
 *              false: is not exist
 */ 
bool IsPathExist(const std::string &path);
