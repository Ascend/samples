/**
* @file utils.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include "utils.h"
#include <fstream>
#if defined(_MSC_VER)
#include <windows.h>
#else
#include <sys/stat.h>
#endif

#include "acl/acl.h"

extern bool g_isDevice;

void *Utils::GetDeviceBufferOfFile(std::string &fileName, uint32_t &fileSize)
{
    uint32_t inputBuffSize = 0;
    void *inputBuff = Utils::ReadBinFile(fileName, inputBuffSize);
    if (inputBuff == nullptr) {
        return nullptr;
    }
    if (!g_isDevice) {
        void *inBufferDev = nullptr;
        aclError ret = aclrtMalloc(&inBufferDev, inputBuffSize, ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("malloc device buffer failed. size is %u, errorCode = %d.",
                inputBuffSize, static_cast<int32_t>(ret));
            aclrtFreeHost(inputBuff);
            inputBuff = nullptr;
            return nullptr;
        }

        ret = aclrtMemcpy(inBufferDev, inputBuffSize, inputBuff, inputBuffSize, ACL_MEMCPY_HOST_TO_DEVICE);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("memcpy failed. device buffer size is %u, input host buffer size is %u, errorCode = %d.",
                inputBuffSize, inputBuffSize, static_cast<int32_t>(ret));
            (void)aclrtFree(inBufferDev);
            inBufferDev = nullptr;
            (void)aclrtFreeHost(inputBuff);
            inputBuff = nullptr;
            return nullptr;
        }
        (void)aclrtFreeHost(inputBuff);
        inputBuff = nullptr;
        fileSize = inputBuffSize;
        return inBufferDev;
    } else {
        fileSize = inputBuffSize;
        return inputBuff;
    }
}

void *Utils::ReadBinFile(std::string &fileName, uint32_t &fileSize)
{
    if (CheckPathIsFile(fileName) == FAILED) {
        ERROR_LOG("%s is not a file", fileName.c_str());
        return nullptr;
    }

    std::ifstream binFile(fileName, std::ifstream::binary);
    if (binFile.is_open() == false) {
        ERROR_LOG("open file %s failed.", fileName.c_str());
        return nullptr;
    }

    binFile.seekg(0, binFile.end);
    uint32_t binFileBufferLen = binFile.tellg();
    if (binFileBufferLen == 0) {
        ERROR_LOG("binfile is empty, filename is %s.", fileName.c_str());
        binFile.close();
        return nullptr;
    }
    binFile.seekg(0, binFile.beg);

    void *binFileBufferData = nullptr;
    aclError ret = ACL_ERROR_NONE;
    if (!g_isDevice) {
        ret = aclrtMallocHost(&binFileBufferData, binFileBufferLen);
        if (binFileBufferData == nullptr) {
            ERROR_LOG("malloc binFileBufferData failed, errorCode = %d.", static_cast<int32_t>(ret));
            binFile.close();
            return nullptr;
        }
    } else {
        ret = aclrtMalloc(&binFileBufferData, binFileBufferLen, ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("malloc device buffer failed. size is %u, errorCode = %d.",
                binFileBufferLen, static_cast<int32_t>(ret));
            binFile.close();
            return nullptr;
        }
    }
    binFile.read(static_cast<char *>(binFileBufferData), binFileBufferLen);
    binFile.close();
    fileSize = binFileBufferLen;
    return binFileBufferData;
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

