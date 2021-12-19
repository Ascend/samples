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
#include <map>
#include <cstring>
#include <memory>
#include "acl/ops/acl_dvpp.h"

Result Utils::ReadFile(uint32_t fileLen, FILE *fp, aclrtRunMode &runMode, void *&dataHost,
    aclrtMemcpyKind &copyKind)
{
    if (runMode == ACL_HOST) {
        copyKind = ACL_MEMCPY_HOST_TO_DEVICE;
        auto aclRet = aclrtMallocHost(&dataHost, fileLen);
        if (dataHost == nullptr) {
            ERROR_LOG("acl malloc host data buffer failed. dataSize = %u, errorCode = %d.",
                fileLen, static_cast<int32_t>(aclRet));
            return FAILED;
        }
    } else {
        auto aclRet = acldvppMalloc(&dataHost, fileLen);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("acl malloc dvpp data failed, dataSize = %u, errorCode = %d.",
                fileLen, static_cast<int32_t>(aclRet));
            return FAILED;
        }
    }
    size_t readSize = fread(dataHost, 1, fileLen, fp);
    if (readSize < fileLen) {
        ERROR_LOG("need read file length = %u bytes, but only %zu bytes read.",
                  fileLen, readSize);
        if (runMode == ACL_HOST) {
            (void)aclrtFreeHost(dataHost);
        } else {
            (void)acldvppFree(dataHost);
        }
        dataHost = nullptr;
        return FAILED;
    }
    return SUCCESS;
}

 void Utils::Deleter(FILE *fp)
{
    fflush(fp);
    fclose(fp);
}

Result Utils::VpcReadFileToDeviceMem(const char *fileName, void *&dataDev, uint32_t &dataSize)
{
    FILE *fp = fopen(fileName, "rb");
    if (fp == nullptr) {
        ERROR_LOG("open file = %s failed.", fileName);
        return FAILED;
    }
    std::unique_ptr<FILE, decltype(Utils::Deleter)*> freeFile(fp, Utils::Deleter);
    INFO_LOG("open file = %s success.", fileName);

    fseek(fp, 0, SEEK_END);
    long fileLenLong = ftell(fp);
    if (fileLenLong <= 0) {
        ERROR_LOG("file = %s len is invalid.", fileName);
        return FAILED;
    }
    fseek(fp, 0, SEEK_SET);
    auto fileLen = static_cast<uint32_t>(fileLenLong);
    aclrtRunMode runMode;
    // runMode is ACL_HOST which represents app is running in host
    // runMode is ACL_DEVICE which represents app is running in device
    auto aclRet = aclrtGetRunMode(&runMode);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("aclrtGetRunMode failed, errorCode = %d", static_cast<int32_t>(aclRet));
        return FAILED;
    }

    void *dataHost = nullptr;
    aclrtMemcpyKind copyKind = ACL_MEMCPY_DEVICE_TO_DEVICE;
    Result ret = ReadFile(fileLen, fp, runMode, dataHost, copyKind);
    if (ret != SUCCESS) {
        ERROR_LOG("read file = %s failed.", fileName);
        return ret;
    }

    dataSize = fileLen;
    // Malloc input device memory
    aclRet = acldvppMalloc(&dataDev, dataSize);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("acl malloc dvpp data failed, dataSize = %u, errorCode = %d.",
            dataSize, static_cast<int32_t>(aclRet));
        if (runMode == ACL_HOST) {
            (void)aclrtFreeHost(dataHost);
        } else {
            (void)acldvppFree(dataHost);
        }
        return FAILED;
    }
    // copy input to device memory
    aclRet = aclrtMemcpy(dataDev, dataSize, dataHost, fileLen, copyKind);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("acl memcpy data to dev failed, fileLen = %u, errorCode = %d.",
            fileLen, static_cast<int32_t>(aclRet));
        if (runMode == ACL_HOST) {
            (void)aclrtFreeHost(dataHost);
        } else {
            (void)acldvppFree(dataHost);
        }
        (void)acldvppFree(dataDev);
        dataDev = nullptr;
        return FAILED;
    }

    if (runMode == ACL_HOST) {
        (void)aclrtFreeHost(dataHost);
    } else {
        (void)acldvppFree(dataHost);
    }
    return SUCCESS;
}

Result Utils::WriteToFile(const char *fileName, const void *dataDev, uint32_t dataSize)
{
    aclrtRunMode runMode;
    auto aclRet = aclrtGetRunMode(&runMode);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("aclrtGetRunMode failed.");
        return FAILED;
    }

    aclrtMemcpyKind copyKind = ACL_MEMCPY_DEVICE_TO_DEVICE;
    void *dataHost = nullptr;
    if (runMode == ACL_HOST) {
        copyKind = ACL_MEMCPY_DEVICE_TO_HOST;
        aclRet = aclrtMallocHost(&dataHost, dataSize);
    } else {
        aclRet = acldvppMalloc(&dataHost, dataSize);
    }
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("acl malloc data failed, dataSize = %u, errorCode = %d.",
            dataSize, static_cast<int32_t>(aclRet));
        return FAILED;
    }

    if (dataHost == nullptr) {
        ERROR_LOG("malloc data buffer failed. dataSize = %u", dataSize);
        return FAILED;
    }

    // copy output to host memory
    aclRet = aclrtMemcpy(dataHost, dataSize, dataDev, dataSize, copyKind);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("acl memcpy data to host failed, dataSize=%u, errorCode = %d.",
            dataSize, static_cast<int32_t>(aclRet));
        if (runMode == ACL_HOST) {
            (void)aclrtFreeHost(dataHost);
        } else {
            (void)acldvppFree(dataHost);
        }
        return FAILED;
    }

    FILE *outFileFp = fopen(fileName, "wb+");
    if (outFileFp == nullptr) {
        ERROR_LOG("fopen out file %s failed, error=%s.", fileName, strerror(errno));
        if (runMode == ACL_HOST) {
            (void)aclrtFreeHost(dataHost);
        } else {
            (void)acldvppFree(dataHost);
        }
        return FAILED;
    }
    std::unique_ptr<FILE, decltype(Utils::Deleter)*> freeOutFile(outFileFp, Utils::Deleter);
    Result ret = SUCCESS;
    size_t writeRet = fwrite(dataHost, 1, dataSize, outFileFp);
    if (writeRet != dataSize) {
        ERROR_LOG("need write %u bytes to %s, but only write %zu bytes, error=%s.",
                  dataSize, fileName, writeRet, strerror(errno));
        ret = FAILED;
    }
    if (runMode == ACL_HOST) {
       (void)aclrtFreeHost(dataHost);
    } else {
        (void)acldvppFree(dataHost);
    }
    return ret;
}
