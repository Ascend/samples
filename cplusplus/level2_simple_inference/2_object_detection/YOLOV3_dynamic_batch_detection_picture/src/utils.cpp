/**
* @file utils.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include <fstream>
#include "utils.h"

#if defined(_MSC_VER)
#include <windows.h>
#else
#include <sys/stat.h>
#endif

extern bool g_isDevice;

void *Utils::CopyDataToDevice(void *data, uint32_t dataSize, aclrtMemcpyKind policy)
{
    void *buffer = nullptr;
    aclError aclRet = aclrtMalloc(&buffer, dataSize, ACL_MEM_MALLOC_NORMAL_ONLY);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("malloc device data buffer failed, aclRet is %d", aclRet);
        return nullptr;
    }

    aclRet = aclrtMemcpy(buffer, dataSize, data, dataSize, policy);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("Copy data to device failed, aclRet is %d", aclRet);
        (void)aclrtFree(buffer);
        return nullptr;
    }

    return buffer;
}

Result Utils::GetImageInfoBuffer(uint32_t imageWidth, uint32_t imageHeight, ImageMemoryInfo &imageMemInfo)
{
    void *imageInfoBuf = nullptr;
    float imageInfo[32];
    for (size_t i = 0; i < 8; i++) {
        imageInfo[i*4 + 0] = (float)imageWidth;
        imageInfo[i*4 + 1] = (float)imageHeight;
        imageInfo[i*4 + 2] = (float)imageWidth;
        imageInfo[i*4 + 3] = (float)imageHeight;
    }

    size_t imageInfoSize = sizeof(imageInfo);
    if (!g_isDevice) {
        imageInfoBuf = CopyDataToDevice((void *)imageInfo, imageInfoSize, ACL_MEMCPY_HOST_TO_DEVICE);
    } else {
        imageInfoBuf = CopyDataToDevice((void *)imageInfo, imageInfoSize, ACL_MEMCPY_DEVICE_TO_DEVICE);
    }

    if (imageInfoBuf == nullptr) {
        return FAILED;
    }

    imageMemInfo.imageInfoBuf = imageInfoBuf;
    imageMemInfo.imageInfoSize = imageInfoSize;

    return SUCCESS;
}

Result Utils::GetDeviceBufferOfFile(const std::string &fileName, uint32_t imageW, uint32_t imageH,
    ImageMemoryInfo &imageMemInfo)
{
    uint32_t inputBuffSize = 0;
    void *inputBuff = Utils::ReadBinFile(fileName, inputBuffSize);
    if (inputBuff == nullptr) {
        return FAILED;
    }

    Result retVal = GetImageInfoBuffer(imageW, imageH, imageMemInfo);
    if (retVal != SUCCESS) {
        ERROR_LOG("malloc image info buffer failed. errorCode = %d.", static_cast<int32_t>(retVal));
        if (!g_isDevice) {
            (void)aclrtFreeHost(inputBuff);
        } else {
            (void)aclrtFree(inputBuff);
        }
        inputBuff = nullptr;
        return retVal;
    }
    if (!g_isDevice) {
        imageMemInfo.imageDataSize = inputBuffSize;
        aclError ret = aclrtMalloc(&imageMemInfo.imageDataBuf, imageMemInfo.imageDataSize,
            ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("malloc image data buffer failed. size is %u, errorCode = %d.",
                inputBuffSize, static_cast<int32_t>(ret));
            (void)aclrtFree(imageMemInfo.imageInfoBuf);
            imageMemInfo.imageInfoBuf = nullptr;
            imageMemInfo.imageInfoSize = 0;
            (void)aclrtFreeHost(inputBuff);
            inputBuff = nullptr;
            return FAILED;
        }

        ret = aclrtMemcpy(imageMemInfo.imageDataBuf, imageMemInfo.imageDataSize,
            inputBuff, inputBuffSize, ACL_MEMCPY_HOST_TO_DEVICE);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("memcpy failed. image data buffer size is %zu, input host buffer size is %u, errorCode = %d.",
                imageMemInfo.imageDataSize, inputBuffSize, static_cast<int32_t>(ret));
            (void)aclrtFree(imageMemInfo.imageInfoBuf);
            imageMemInfo.imageInfoBuf = nullptr;
            imageMemInfo.imageInfoSize = 0;
            (void)aclrtFree(imageMemInfo.imageDataBuf);
            imageMemInfo.imageDataBuf = nullptr;
            imageMemInfo.imageDataSize = 0;
            (void)aclrtFreeHost(inputBuff);
            inputBuff = nullptr;
            return FAILED;
        }
        (void)aclrtFreeHost(inputBuff);
        inputBuff = nullptr;
        return SUCCESS;
    } else {
        imageMemInfo.imageDataSize = inputBuffSize;
        imageMemInfo.imageDataBuf = inputBuff;
        return SUCCESS;
    }
}

void *Utils::ReadBinFile(const std::string &fileName, uint32_t &fileSize)
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
    aclError ret = ACL_SUCCESS;
    if (!g_isDevice) {
        ret = aclrtMallocHost(&binFileBufferData, binFileBufferLen);
        if (binFileBufferData == nullptr) {
            ERROR_LOG("malloc binFileBufferData failed, errorCode = %d.", static_cast<int32_t>(ret));
            binFile.close();
            return nullptr;
        }
    } else {
        ret = aclrtMalloc(&binFileBufferData, binFileBufferLen, ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != ACL_SUCCESS) {
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
