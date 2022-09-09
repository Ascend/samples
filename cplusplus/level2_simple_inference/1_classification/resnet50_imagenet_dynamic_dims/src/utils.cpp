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
#include <iostream>
#include <iostream>
#include <fstream>
#include <cstring>
#if defined(_MSC_VER)
#include <windows.h>
#else
#include <sys/stat.h>
#endif
#include "utils.h"
#include "acl/acl.h"
using namespace std;
namespace {
    const std::string g_imagePathSeparator = ",";
    const int STAT_SUCCESS = 0;
    const std::string g_pathSeparator = "/";
}
Result Utils::ReadBinFile(const std::string &fileName, void *&inputBuff, uint32_t &fileSize)
{
    if (CheckPathIsFile(fileName) == FAILED) {
        ERROR_LOG("%s is not a file", fileName.c_str());
        return FAILED;
    }

    std::ifstream binFile(fileName, std::ifstream::binary);
    if (binFile.is_open() == false) {
        ERROR_LOG("open file %s failed", fileName.c_str());
        return FAILED;
    }

    binFile.seekg(0, binFile.end);
    uint32_t binFileBufferLen = binFile.tellg();
    if (binFileBufferLen == 0) {
        ERROR_LOG("binfile is empty, filename is %s", fileName.c_str());
        binFile.close();
        return FAILED;
    }
    binFile.seekg(0, binFile.beg);

    binFile.read(static_cast<char *>(inputBuff), binFileBufferLen);
    binFile.close();
    fileSize = binFileBufferLen;
    return SUCCESS;
}

void* Utils::MemcpyToDeviceBuffer(const void *data, uint32_t size, aclrtRunMode runMode)
{
    if ((data == nullptr) || (size == 0)) {
        ERROR_LOG("Copy data args invalid, data %p, size %d", data, size);
        return nullptr;
    }
    aclrtMemcpyKind policy = ACL_MEMCPY_HOST_TO_DEVICE;
    if (runMode == ACL_DEVICE) {
        policy = ACL_MEMCPY_DEVICE_TO_DEVICE;
    }
    void *buffer = nullptr;
    aclError aclRet = aclrtMalloc(&buffer, size, ACL_MEM_MALLOC_HUGE_FIRST);
    if ((aclRet != ACL_SUCCESS) || (buffer == nullptr)) {
        ERROR_LOG("Malloc memory failed, errorno:%d", aclRet);
        return nullptr;
    }
    aclRet = aclrtMemcpy(buffer, size, data, size, policy);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("Copy data to device failed, aclRet is %d", aclRet);
        return nullptr;
    }
    return buffer;
}

void Utils::GetAllFiles(const std::string &pathList, std::vector<string> &fileVec)
{
    vector<string> pathVec;
    SplitPath(pathList, pathVec);

    for (string everyPath : pathVec) {
        // check path exist or not
        if (!IsPathExist(pathList)) {
            ERROR_LOG("Failed to deal path=%s. Reason: not exist or can not access.",
                      everyPath.c_str());
            continue;
        }
        // get files in path and sub-path
        GetPathFiles(everyPath, fileVec);
    }
}

bool Utils::IsPathExist(const std::string &path)
{
    ifstream file(path);
    if (!file) {
        return false;
    }
    return true;
}

void Utils::SplitPath(const std::string &path, std::vector<std::string> &pathVec)
{
    char *charPath = const_cast<char *>(path.c_str());
    const char *charSplit = g_imagePathSeparator.c_str();
    char *imageFile = strtok(charPath, charSplit);
    while (imageFile) {
        pathVec.emplace_back(imageFile);
        imageFile = strtok(nullptr, charSplit);
    }
}

void Utils::GetPathFiles(const std::string &path, std::vector<std::string> &fileVec)
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
            string fullPath = path + g_pathSeparator + direntPtr->d_name;
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

bool Utils::IsDirectory(const std::string &path)
{
    // get path stat
    struct stat buf;
    if (stat(path.c_str(), &buf) != STAT_SUCCESS) {
        return false;
    }
    // check
    return S_ISDIR(buf.st_mode);
}

Result Utils::CheckPathIsFile(const std::string &fileName)
{
#if defined(_MSC_VER)
    DWORD bRet = GetFileAttributes((LPCSTR)fileName.c_str());
    if (bRet == FILE_ATTRIBUTE_DIRECTORY) {
        ERROR_LOG("%s is not a file, please enter a file", fileName.c_str());
        return FAILED;
    }
#else
    struct stat sBuf;
    int fileStatus = stat(fileName.data(), &sBuf);
    if (fileStatus == -1) {
        ERROR_LOG("failed to get file");
        return FAILED;
    }
    if (S_ISREG(sBuf.st_mode) == 0) {
        ERROR_LOG("%s is not a file, please enter a file", fileName.c_str());
        return FAILED;
    }
#endif
    return SUCCESS;
}