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
#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif

bool RunStatus::isDevice_ = false;

bool Utils::ReadFileToDeviceMem(const char *fileName, void *&dataDev, uint32_t &dataSize)
{
    // read data from file.
    FILE *fp = fopen(fileName, "rb");
    if (fp == nullptr) {
        ERROR_LOG("open file %s failed.", fileName);
        return false;
    }

    fseek(fp, 0, SEEK_END);
    long fileLenLong = ftell(fp);
    if (fileLenLong <= 0) {
        ERROR_LOG("file %s len is invalid.", fileName);
        fclose(fp);
        return false;
    }
    fseek(fp, 0, SEEK_SET);

    auto fileLen = static_cast<uint32_t>(fileLenLong);
    dataSize = fileLen;
    size_t readSize;
    // Malloc input device memory
    auto aclRet = acldvppMalloc(&dataDev, dataSize);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("acl malloc dvpp data failed, dataSize = %u, errorCode = %d.",
            dataSize, static_cast<int32_t>(aclRet));
        fclose(fp);
        return false;
    }

    if (!(RunStatus::GetDeviceStatus())) {
        void *dataHost = nullptr;
        auto aclRet = aclrtMallocHost(&dataHost, fileLen);
        if (dataHost == nullptr) {
            ERROR_LOG("acl malloc host data buffer failed. dataSize = %u, errorCode = %d.",
                fileLen, static_cast<int32_t>(aclRet));
            (void)acldvppFree(dataDev);
            dataDev = nullptr;
            fclose(fp);
            return false;
        }

        readSize = fread(dataHost, 1, fileLen, fp);
        if (readSize < fileLen) {
            ERROR_LOG("need read file %s %u bytes, but only %zu read.", fileName, fileLen, readSize);
            (void)aclrtFreeHost(dataHost);
            (void)acldvppFree(dataDev);
            dataDev = nullptr;
            fclose(fp);
            return false;
        }

        // copy input to device memory
        aclRet = aclrtMemcpy(dataDev, dataSize, dataHost, fileLen, ACL_MEMCPY_HOST_TO_DEVICE);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("acl memcpy data to dev failed, fileLen = %u, errorCode = %d.",
                fileLen, static_cast<int32_t>(aclRet));
            (void)aclrtFreeHost(dataHost);
            (void)acldvppFree(dataDev);
            dataDev = nullptr;
            fclose(fp);
            return false;
        }
        (void)aclrtFreeHost(dataHost);
    } else {
        readSize = fread(dataDev, 1, fileLen, fp);
        if (readSize < fileLen) {
            ERROR_LOG("need read file %s %u bytes, but only %zu read.", fileName, fileLen, readSize);
            (void)acldvppFree(dataDev);
            dataDev = nullptr;
            fclose(fp);
            return false;
        }
    }

    fclose(fp);
    return true;
}

bool Utils::WriteToFile(FILE *fileFp, void *dataDev, uint32_t dataSize)
{
    if (fileFp == nullptr) {
        ERROR_LOG("fileFp is nullptr!");
        return false;
    }

    if (dataDev == nullptr) {
        ERROR_LOG("dataDev is nullptr!");
        return false;
    }

    // copy output to host memory
    void *data = nullptr;
    aclError aclRet;
    if (!(RunStatus::GetDeviceStatus())) {
        aclRet = aclrtMallocHost(&data, dataSize);
        if (data == nullptr) {
            ERROR_LOG("malloc host data buffer failed. dataSize = %u, errorCode = %d.",
                dataSize, static_cast<int32_t>(aclRet));
            return false;
        }
        aclRet = aclrtMemcpy(data, dataSize, dataDev, dataSize, ACL_MEMCPY_DEVICE_TO_HOST);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("acl memcpy data to host failed, dataSize = %u, errorCode = %d.",
                dataSize, static_cast<int32_t>(aclRet));
            (void)aclrtFreeHost(data);
            return false;
        }
    } else {
        data = dataDev;
    }

    bool ret = true;
    size_t writeRet = fwrite(data, 1, dataSize, fileFp);
    if (writeRet != dataSize) {
        ERROR_LOG("need write %u bytes, but only write %zu bytes, error=%s.\n",
                      dataSize, writeRet, strerror(errno));
        ret = false;
    }

    if (!(RunStatus::GetDeviceStatus())) {
        (void)aclrtFreeHost(data);
    }
    fflush(fileFp);
    return ret;
}

Result Utils::CheckAndCreateFolder(const char* foldName)
{
    INFO_LOG("start check result fold:%s", foldName);
#if defined(_MSC_VER)
    DWORD ret = GetFileAttributes((LPCSTR)foldName);
    if (ret == INVALID_FILE_ATTRIBUTES) {
        BOOL flag = CreateDirectory((LPCSTR)foldName, nullptr);
        if (flag) {
            INFO_LOG("make successfully.");
        } else {
            INFO_LOG("make errorly.");
            return FAILED;
        }
    }
#else
    if (access(foldName , 0) == -1) {
        int flag = mkdir(foldName , 0777);
        if (flag == 0) {
            INFO_LOG("make directory successfully.");
        } else {
            ERROR_LOG("make directory errorly.");
            return FAILED;
        }
    }
#endif
    INFO_LOG("check result success, fold exist");
    return SUCCESS;
}

