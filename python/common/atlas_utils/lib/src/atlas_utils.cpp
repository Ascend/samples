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

* File utils.cpp
* Description: handle file operations
*/
#include "atlas_utils.h"
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
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

using namespace std;

extern "C" { 

namespace {
    const std::string kImagePathSeparator = ",";
    const int kStatSuccess = 0;
    const std::string kFileSperator = "/";
    const std::string kPathSeparator = "/";
    // output image prefix
    const std::string kOutputFilePrefix = "out_";

    const string kRegexIpAddr =
        "^(1\\d{2}|2[0-4]\\d|25[0-5]|[1-9]\\d|[0-9])\\."
            "(1\\d{2}|2[0-4]\\d|25[0-5]|[1-9]\\d|\\d)\\."
            "(1\\d{2}|2[0-4]\\d|25[0-5]|[1-9]\\d|\\d)\\."
            "(1\\d{2}|2[0-4]\\d|25[0-5]|[1-9]\\d|\\d)"
            ":([1-9]|[1-9]\\d|[1-9]\\d{2}|[1-9]\\d{3}|[1-5]\\d{4}|"
            "6[0-4]\\d{3}|65[0-4]\\d{2}|655[0-2]\\d|6553[0-5])$";

    const uint32_t kScorePrecision = 3;
    // bounding box line solid
    const uint32_t kLineSolid = 2;
    const double kFontScale = 0.5;
    const uint32_t kLabelOffset = 11;
	
	// regex for verify video file name
const string kRegexVideoFile = "^.+\\.(mp4|h264|h265)$";

// regex for verify RTSP rtsp://
const string kRegexRtsp = "^rtsp://.*";
}

bool IsVideoFile(const string& str) {
    regex regexVideoFile(kRegexVideoFile.c_str());
    return regex_match(str, regexVideoFile);
}

bool IsRtspAddr(const string &str) {
    regex regexRtspAddress(kRegexRtsp.c_str());

    return regex_match(str, regexRtspAddress);
}

bool IsIpAddrWithPort(const string& addrStr) {
    regex regexIpAddr(kRegexIpAddr.c_str());

    return regex_match(addrStr, regexIpAddr);
}

void ParseIpAddr(string& ip, string& port, const string& addr)
{
    string::size_type pos = addr.find(":");
    
    ip.assign(addr.substr(0, pos));
    port.assign(addr.substr(pos + 1));
}

bool IsDirectory(const string &path) {
    // get path stat
    struct stat buf;
    if (stat(path.c_str(), &buf) != kStatSuccess) {
        return false;
    }

    // check
    return S_ISDIR(buf.st_mode);
}

bool IsPathExist(const string &path) {
    ifstream file(path);
    if (!file) {
        return false;
    }
    return true;
}

void SplitPath(const string &path, vector<string> &pathVec) {
    char *charPath = const_cast<char*>(path.c_str());
    const char *charSplit = kImagePathSeparator.c_str();
    char *imageFile = strtok(charPath, charSplit);
    while (imageFile) {
        pathVec.emplace_back(imageFile);
        imageFile = strtok(nullptr, charSplit);
    }
}

void GetPathFiles(const string &path, vector<string> &fileVec) {
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
            string fullPath = path + kPathSeparator + direntPtr->d_name;
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

void GetAllFiles(const string &path, vector<string> &fileVec) {
    // split file path
    vector<string> pathVec;
    SplitPath(path, pathVec);

    for (string everyPath : pathVec) {
        // check path exist or not
        if (!IsPathExist(path)) {
            ATLAS_LOG_ERROR("Failed to deal path=%s. Reason: not exist or can not access.",
                everyPath.c_str());
            continue;
        }
        // get files in path and sub-path
        GetPathFiles(everyPath, fileVec);
    }
}

void* MallocMemory(uint32_t dataSize, MemoryType memType) {
    void* buffer = nullptr;
    aclError aclRet = ACL_ERROR_NONE;
    
    switch(memType){
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
            ATLAS_LOG_ERROR("Invalid memory type %d", memType);
            aclRet = ACL_ERROR_INVALID_PARAM;
            break;
    }

    if ((aclRet != ACL_ERROR_NONE) || (buffer == nullptr)) {
        ATLAS_LOG_ERROR("Malloc memory failed, type: %d, errorno:%d",
                        memType, aclRet);
        return nullptr;
    }

    return buffer;    
}

void FreeMemory(void* mem, MemoryType memType) {
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
            ATLAS_LOG_ERROR("Invalid memory type %d", memType);
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

void* CopyDataToDevice(void* data, uint32_t size, 
                       aclrtRunMode curRunMode, MemoryType memType) {                        
    if ((data == nullptr) || (size == 0) || 
        ((curRunMode != ACL_HOST) && (curRunMode != ACL_DEVICE)) ||
        (memType >= MEMORY_INVALID_TYPE) || (memType == MEMORY_HOST)) {
        ATLAS_LOG_ERROR("Copy data args invalid, data %p, "
                        "size %d, src dev %d, memory type %d",
                        data, size, curRunMode, memType);
        return nullptr;
    }

    aclrtMemcpyKind policy = GetCopyPolicy(curRunMode, TO_DEVICE, memType);

    return CopyData(data, size, policy, memType);
}

void* CopyDataToHost(void* data, uint32_t size, 
                     aclrtRunMode curRunMode, MemoryType memType) {                        
    if ((data == nullptr) || (size == 0) || 
        ((curRunMode != ACL_HOST) && (curRunMode != ACL_DEVICE)) ||
        ((memType != MEMORY_HOST) && (memType != MEMORY_NORMAL))) {
        ATLAS_LOG_ERROR("Copy data args invalid, data %p, "
                        "size %d, src dev %d, memory type %d",
                        data, size, curRunMode, memType);
        return nullptr;
    }

    aclrtMemcpyKind policy = GetCopyPolicy(curRunMode, TO_HOST, memType);

    return CopyData(data, size, policy, memType);
}

void* CopyData(void* data, uint32_t size, 
               aclrtMemcpyKind policy, MemoryType memType) {
    void* buffer = MallocMemory(size, memType);
    if (buffer == nullptr) return nullptr;

    aclError aclRet = aclrtMemcpy(buffer, size, data, size, policy);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Copy data to device failed, aclRet is %d", aclRet);
        FreeMemory(buffer, memType);
        return nullptr;
    }

    return buffer;
}

int CopyDataBufToLocal(DataBuf& dest, DataBuf& src, int runMode) {
    dest.data = (uint8_t *)CopyDataToHost(src.data, src.size, 
                                          (aclrtRunMode)runMode, MEMORY_NORMAL);
    if (dest.data == nullptr) {
        ATLAS_LOG_ERROR("New malloc memory failed");
        return ATLAS_ERROR;
    }

    dest.size = src.size;

    return ATLAS_OK;
}

void ReleaseDataBuf(DataBuf& dataBuf) {
    delete[](dataBuf.data);
    dataBuf.data = nullptr;
    dataBuf.size = 0;
}

AtlasError ReadBinFile(const string& fileName, void*& data, uint32_t& size) {
    struct stat sBuf;
    int fileStatus = stat(fileName.data(), &sBuf);
    if (fileStatus == -1) {
        ATLAS_LOG_ERROR("failed to get file");
        return ATLAS_ERROR_ACCESS_FILE;
    }
    if (S_ISREG(sBuf.st_mode) == 0) {
        ATLAS_LOG_ERROR("%s is not a file, please enter a file", 
                      fileName.c_str());
        return ATLAS_ERROR_INVALID_FILE;
    }
    std::ifstream binFile(fileName, std::ifstream::binary);
    if (binFile.is_open() == false) {
        ATLAS_LOG_ERROR("open file %s failed", fileName.c_str());
        return ATLAS_ERROR_OPEN_FILE;
    }

    binFile.seekg(0, binFile.end);
    uint32_t binFileBufferLen = binFile.tellg();
    if (binFileBufferLen == 0) {
        ATLAS_LOG_ERROR("binfile is empty, filename is %s", fileName.c_str());
        binFile.close();
        return ATLAS_ERROR_INVALID_FILE;
    }

    binFile.seekg(0, binFile.beg);

    uint8_t* binFileBufferData = new(std::nothrow) uint8_t[binFileBufferLen];
    if (binFileBufferData == nullptr) {
        ATLAS_LOG_ERROR("malloc binFileBufferData failed");
        binFile.close();
        return ATLAS_ERROR_MALLOC;
    }
    binFile.read((char *)binFileBufferData, binFileBufferLen);
    binFile.close();

    data = binFileBufferData;
    size = binFileBufferLen;

    return ATLAS_OK;
}



void SaveBinFile(const string& filename, void* data, uint32_t size) {
    FILE *outFileFp = fopen(filename.c_str(), "wb+");
    if (outFileFp == nullptr) {
        ATLAS_LOG_ERROR("Save file %s failed for open error", filename.c_str());
        return;
    }
    fwrite(data, 1, size, outFileFp);

    fflush(outFileFp);
    fclose(outFileFp);
}

void AclLog(int level, const char *func, 
            const char *file, uint32_t line, const char* logStr){
    aclAppLog((aclLogLevel)level, func, file, line, "%s\n", logStr);
}

}