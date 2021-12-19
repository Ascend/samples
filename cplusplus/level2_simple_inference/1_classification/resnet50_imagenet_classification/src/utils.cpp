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
#include <iostream>
#include <fstream>
#include <cstring>
#if defined(_MSC_VER)
#include <windows.h>
#else
#include <sys/stat.h>
#endif
#include "acl/acl.h"

extern bool g_isDevice;

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

    aclError ret = ACL_SUCCESS;
    if (!g_isDevice) { // app is running in host
        ret = aclrtMallocHost(&inputBuff, binFileBufferLen);
        if (inputBuff == nullptr) {
            ERROR_LOG("malloc binFileBufferData failed, binFileBufferLen is %u, errorCode is %d",
                binFileBufferLen, static_cast<int32_t>(ret));
            binFile.close();
            return FAILED;
        }
    } else { // app is running in device
        ret = aclrtMalloc(&inputBuff, binFileBufferLen, ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("malloc device buffer failed. size is %u, errorCode is %d",
                binFileBufferLen, static_cast<int32_t>(ret));
            binFile.close();
            return FAILED;
        }
    }
    binFile.read(static_cast<char *>(inputBuff), binFileBufferLen);
    binFile.close();
    fileSize = binFileBufferLen;
    return SUCCESS;
}

Result Utils::MemcpyFileToDeviceBuffer(const std::string &fileName, void *&picDevBuffer, size_t inputBuffSize)
{
    void *inputBuff = nullptr;
    uint32_t fileSize = 0;
    auto ret = Utils::ReadBinFile(fileName, inputBuff, fileSize);
    if (ret != SUCCESS) {
        ERROR_LOG("read bin file failed, file name is %s", fileName.c_str());
        return FAILED;
    }
    if (inputBuffSize != static_cast<size_t>(fileSize)) {
        ERROR_LOG("input image size[%u] is not equal to model input size[%zu]", fileSize, inputBuffSize);
        if (!g_isDevice) {
            (void)aclrtFreeHost(inputBuff);
        } else {
            (void)aclrtFree(inputBuff);
        }
        return FAILED;
    }
    if (!g_isDevice) {
        // if app is running in host, need copy data from host to device
        aclError aclRet = aclrtMemcpy(picDevBuffer, inputBuffSize, inputBuff, inputBuffSize, ACL_MEMCPY_HOST_TO_DEVICE);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("memcpy failed. buffer size is %zu, errorCode is %d", inputBuffSize, static_cast<int32_t>(aclRet));
            (void)aclrtFreeHost(inputBuff);
            return FAILED;
        }
        (void)aclrtFreeHost(inputBuff);
    } else { // app is running in device
        aclError aclRet = aclrtMemcpy(picDevBuffer, inputBuffSize, inputBuff, inputBuffSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("memcpy d2d failed. buffer size is %zu, errorCode is %d", inputBuffSize, static_cast<int32_t>(aclRet));
            (void)aclrtFree(inputBuff);
            return FAILED;
        }
        (void)aclrtFree(inputBuff);
    }
    return SUCCESS;
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