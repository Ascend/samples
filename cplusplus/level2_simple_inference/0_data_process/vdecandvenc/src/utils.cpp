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

bool RunStatus::isDevice_ = false;

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
    aclrtRunMode runMode;
    aclrtGetRunMode(&runMode);
    aclError aclRet;
    if (runMode == ACL_HOST) {
        data = malloc(dataSize);
        if (data == nullptr) {
            ERROR_LOG("malloc host data buffer failed. dataSize=%u\n", dataSize);
            return false;
        }
        aclRet = aclrtMemcpy(data, dataSize, dataDev, dataSize, ACL_MEMCPY_DEVICE_TO_HOST);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("acl memcpy data to host failed, dataSize=%u, ret=%d.\n", dataSize, aclRet);
            free(data);
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

    if (runMode == ACL_HOST) {
        free(data);
    }
    fflush(fileFp);
    return ret;
}

Result Utils::CheckFolder(const char* foldName)
{
    INFO_LOG( "start check result fold:%s", foldName);
    if (access(foldName , 0) == -1) {
        int flag=mkdir(foldName , 0777);
        if (flag == 0) {
            INFO_LOG( "make directory successfully.");
        } else {
            ERROR_LOG( "make directory errorly.");
            return FAILED;
        }
    }
    INFO_LOG( "check result success, fold exist");
    return SUCCESS;
}
