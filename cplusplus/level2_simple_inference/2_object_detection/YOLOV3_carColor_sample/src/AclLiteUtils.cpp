/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
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
*/

#include <map>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <cstring>
#include <dirent.h>
#include <regex>
#include <vector>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "AclLiteUtils.h"
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

using namespace std;

#define COMMENT_CHAR '#'
#define EQUALS_CHAR  '='
#define BLANK_SPACE_CHAR ' '
#define TABLE_CHAR '\t'

namespace {
const std::string g_ImagePathSeparator = ",";
const int g_StatSuccess = 0;
const std::string g_FileSperator = "/";
const std::string g_PathSeparator = "/";
}

bool IsPathExist(const string &path)
{
    ifstream file(path);
    if (!file) {
        return false;
    }
    return true;
}

bool IsDirectory(const string &path)
{
    // get path stat
    struct stat buf;
    if (stat(path.c_str(), &buf) != g_StatSuccess) {
        return false;
    }

    // check
    return S_ISDIR(buf.st_mode);
}

void SplitPath(const string &path, vector<string> &pathVec)
{
    char *charPath = const_cast<char*>(path.c_str());
    const char *charSplit = g_ImagePathSeparator.c_str();
    char *imageFile = strtok(charPath, charSplit);
    while (imageFile) {
        pathVec.emplace_back(imageFile);
        imageFile = strtok(nullptr, charSplit);
    }
}

void GetPathFiles(const string &path, vector<string> &fileVec)
{
    struct dirent *direntPtr = nullptr;
    DIR *dir = nullptr;
    if (IsDirectory(path)) {
        dir = opendir(path.c_str());
        while ((direntPtr = readdir(dir)) != nullptr) {
            // skip . and ..
            if (direntPtr->d_name[0] == '.') {
            continue;
            }

            // file path
            string fullPath = path + g_PathSeparator + direntPtr->d_name;
            // directory need recursion
            if (IsDirectory(fullPath)) {
                GetPathFiles(fullPath, fileVec);
            } else {
                // put file
                fileVec.emplace_back(fullPath);
            }
        }
    } else {
        fileVec.emplace_back(path);
    }
}

void GetAllFiles(const string &pathList, vector<string> &fileVec)
{
    // split file path
    vector<string> pathVec;
    SplitPath(pathList, pathVec);

    for (string everyPath : pathVec) {
        // check path exist or not
        if (!IsPathExist(pathList)) {
            ACLLITE_LOG_ERROR("Failed to deal path=%s. Reason: not exist or can not access.",
                everyPath.c_str());
            continue;
        }
        // get files in path and sub-path
        GetPathFiles(everyPath, fileVec);
    }
}

void* MallocMemory(uint32_t dataSize, MemoryType memType)
{
    void* buffer = nullptr;
    aclError aclRet = ACL_SUCCESS;
    
    switch (memType) {
        case MEMORY_NORMAL:
            buffer = new uint8_t[dataSize]; 
            break;
        case MEMORY_HOST:
            aclRet = aclrtMallocHost(&buffer, dataSize);
            break;
        case MEMORY_DEVICE:
            aclRet = aclrtMalloc(&buffer, dataSize, ACL_MEM_MALLOC_HUGE_FIRST);
            break;
        case MEMORY_DVPP:
            aclRet = acldvppMalloc(&buffer, dataSize);
            break;
        default:
            ACLLITE_LOG_ERROR("Invalid memory type %d", memType);
            aclRet = ACL_ERROR_INVALID_PARAM;
            break;
    }

    if ((aclRet != ACL_SUCCESS) || (buffer == nullptr)) {
        ACLLITE_LOG_ERROR("Malloc memory failed, type: %d, errorno:%d",
                        memType, aclRet);
        return nullptr;
    }

    return buffer;
}

void FreeMemory(void* mem, MemoryType memType)
{
    switch(memType){
        case MEMORY_NORMAL:
            delete[]((uint8_t *)mem);
            break;
        case MEMORY_HOST:
            aclrtFreeHost(mem);
            break;
        case MEMORY_DEVICE:
            aclrtFree(mem);
            break;
        case MEMORY_DVPP:
            acldvppFree(mem);
            break;
        default:
            ACLLITE_LOG_ERROR("Invalid memory type %d", memType);
            break;
    }
}

aclrtMemcpyKind GetCopyPolicy(aclrtRunMode srcDev, 
                              CopyDirection direct, MemoryType memType) {
    aclrtMemcpyKind policy = ACL_MEMCPY_HOST_TO_HOST;

    if (direct == TO_DEVICE) {
        if (srcDev == ACL_HOST)
            policy = ACL_MEMCPY_HOST_TO_DEVICE;
        else
            policy = ACL_MEMCPY_DEVICE_TO_DEVICE;
    } else {//TO_HOST
        if (srcDev == ACL_HOST) 
            policy = ACL_MEMCPY_DEVICE_TO_HOST; 
        else
            policy = ACL_MEMCPY_DEVICE_TO_DEVICE;       
    }

    return policy;
}

void* CopyDataToDevice(const void* data, uint32_t size, 
                       aclrtRunMode curRunMode, MemoryType memType) {                        
    if ((data == nullptr) || (size == 0) || 
        ((curRunMode != ACL_HOST) && (curRunMode != ACL_DEVICE)) ||
        (memType >= MEMORY_INVALID_TYPE) || (memType == MEMORY_HOST)) {
        ACLLITE_LOG_ERROR("Copy data args invalid, data %p, "
                          "size %d, src dev %d, memory type %d",
                          data, size, curRunMode, memType);
        return nullptr;
    }

    aclrtMemcpyKind policy = GetCopyPolicy(curRunMode, TO_DEVICE, memType);

    return CopyData(data, size, policy, memType);
}

AclLiteError CopyDataToDeviceEx(void* dest, uint32_t destSize, 
                                const void* src, uint32_t srcSize,
                                aclrtRunMode runMode)
{
    aclrtMemcpyKind policy = ACL_MEMCPY_HOST_TO_DEVICE;
    if (runMode == ACL_DEVICE) {
        policy = ACL_MEMCPY_DEVICE_TO_DEVICE;
    }

    aclError aclRet = aclrtMemcpy(dest, destSize, src, srcSize, policy);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Copy data to device failed, aclRet is %d", aclRet);
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

void* CopyDataToHost(const void* data, uint32_t size,
                     aclrtRunMode curRunMode, MemoryType memType)
{                        
    if ((data == nullptr) || (size == 0) ||
        ((curRunMode != ACL_HOST) && (curRunMode != ACL_DEVICE)) ||
        ((memType != MEMORY_HOST) && (memType != MEMORY_NORMAL))) {
        ACLLITE_LOG_ERROR("Copy data args invalid, data %p, "
                          "size %d, src dev %d, memory type %d",
                          data, size, curRunMode, memType);
        return nullptr;
    }

    aclrtMemcpyKind policy = GetCopyPolicy(curRunMode, TO_HOST, memType);

    return CopyData(data, size, policy, memType);
}

void* CopyData(const void* data, uint32_t size,
               aclrtMemcpyKind policy, MemoryType memType)
{
    void* buffer = MallocMemory(size, memType);
    if (buffer == nullptr) {
        return nullptr;
    }

    aclError aclRet = aclrtMemcpy(buffer, size, data, size, policy);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Copy data to device failed, aclRet is %d", aclRet);
        FreeMemory(buffer, memType);
        return nullptr;
    }

    return buffer;
}

AclLiteError CopyImageToDevice(ImageData& destImage, ImageData& srcImage,
                               aclrtRunMode curRunMode, MemoryType memType)
{
    void* data = CopyDataToDevice(srcImage.data.get(), srcImage.size,
                                  curRunMode, memType);
    if (data == nullptr) {
        return ACLLITE_ERROR_COPY_DATA;
    }

    destImage.format = srcImage.format;
    destImage.width = srcImage.width;
    destImage.height = srcImage.height;
    destImage.size = srcImage.size;
    destImage.alignWidth = srcImage.alignWidth;
    destImage.alignHeight = srcImage.alignHeight;
    
    if (memType == MEMORY_DEVICE) {
        destImage.data = SHARED_PTR_DEV_BUF(data);
        } else {
        destImage.data = SHARED_PTR_DVPP_BUF(data);
    }

    return ACLLITE_OK;
}

AclLiteError ReadBinFile(const string& fileName, void*& data, uint32_t& size)
{
    struct stat sBuf;
    int fileStatus = stat(fileName.data(), &sBuf);
    if (fileStatus == -1) {
        ACLLITE_LOG_ERROR("failed to get file");
        return ACLLITE_ERROR_ACCESS_FILE;
    }
    if (S_ISREG(sBuf.st_mode) == 0) {
        ACLLITE_LOG_ERROR("%s is not a file, please enter a file",
                          fileName.c_str());
        return ACLLITE_ERROR_INVALID_FILE;
        }
    std::ifstream binFile(fileName, std::ifstream::binary);
    if (binFile.is_open() == false) {
        ACLLITE_LOG_ERROR("open file %s failed", fileName.c_str());
        return ACLLITE_ERROR_OPEN_FILE;
        }

    binFile.seekg(0, binFile.end);
    uint32_t binFileBufferLen = binFile.tellg();
    if (binFileBufferLen == 0) {
        ACLLITE_LOG_ERROR("binfile is empty, filename is %s", fileName.c_str());
        binFile.close();
        return ACLLITE_ERROR_INVALID_FILE;
    }

    binFile.seekg(0, binFile.beg);

    uint8_t* binFileBufferData = new(std::nothrow) uint8_t[binFileBufferLen];
    if (binFileBufferData == nullptr) {
        ACLLITE_LOG_ERROR("malloc binFileBufferData failed");
        binFile.close();
        return ACLLITE_ERROR_MALLOC;
    }
    binFile.read((char *)binFileBufferData, binFileBufferLen);
    binFile.close();

    data = binFileBufferData;
    size = binFileBufferLen;

    return ACLLITE_OK;
}

AclLiteError ReadJpeg(ImageData& image, const std::string& fileName)
{
    uint32_t size = 0;
    void* buf = nullptr;
    
    ReadBinFile(fileName, buf, size);

    int32_t ch = 0;
    acldvppJpegGetImageInfo(buf, size,
                            &(image.width), &(image.height), &ch);
    if (image.width == 0 || image.height == 0) {
        ACLLITE_LOG_ERROR("unsupported format, only Baseline JPEG");
        return ACLLITE_ERROR;
    }
    image.data.reset((uint8_t *)buf, [](uint8_t* p) { 
        delete[](p);
    }
                    );
    image.size = size;

    return ACLLITE_OK;
}
