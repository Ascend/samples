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
#include <fstream>
#include <cstring>
#include "utils.h"

#if defined(_MSC_VER)
#include <windows.h>
#else
#include <sys/stat.h>
#endif

#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"

bool RunStatus::isDevice_ = false;

Result Utils::ReadBinFile(PicDesc &picDesc, void *&inputBuff, uint32_t &fileSize)
{
    std::string fileName = picDesc.picName;
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

    aclError ret;
    if (!(RunStatus::GetDeviceStatus())) { // app is running in host
        ret = aclrtMallocHost(&inputBuff, binFileBufferLen);
        if (inputBuff == nullptr) {
            ERROR_LOG("malloc binFileBufferData failed. binFileBufferLen is %u, errorCode is %d",
                      binFileBufferLen, static_cast<int32_t>(ret));
            binFile.close();
            return FAILED;
        }
    } else { // app is running in device
        ret = acldvppMalloc(&inputBuff, binFileBufferLen);
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

    int32_t components = 0;
    ret = acldvppJpegGetImageInfo(inputBuff, binFileBufferLen, &picDesc.width, &picDesc.height, &components);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acldvppJpegGetImageInfo failed, errorCode is %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("read image file[%s] success, width = %u, height = %u",
             fileName.c_str(), picDesc.width, picDesc.height);

    return SUCCESS;
}

Result Utils::GetDeviceBufferOfPicture(PicDesc &picDesc, void *&picDevBuffer, uint32_t &devPicBufferSize)
{
    if (picDesc.picName.empty()) {
        ERROR_LOG("picture file name is empty");
        return FAILED;
    }

    uint32_t inputBuffSize = 0;
    void *inputBuff = nullptr;
    Result ret = ReadBinFile(picDesc, inputBuff, inputBuffSize);
    if (ret != SUCCESS) {
        ERROR_LOG("read bin file failed, file name is %s", picDesc.picName.c_str());
        return FAILED;
    }
    aclError aclRet = acldvppJpegGetImageInfo(inputBuff, inputBuffSize, &picDesc.width, &picDesc.height, nullptr);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("get jpeg image info failed, errorCode is %d", static_cast<int32_t>(aclRet));
        if (!(RunStatus::GetDeviceStatus())) {
            (void) aclrtFreeHost(inputBuff);
        } else {
            (void) acldvppFree(inputBuff);
        }
        return FAILED;
    }
    aclRet = acldvppJpegPredictDecSize(inputBuff, inputBuffSize, PIXEL_FORMAT_YUV_SEMIPLANAR_420,
                                       &picDesc.jpegDecodeSize);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("get jpeg decode size failed, errorCode is %d", static_cast<int32_t>(aclRet));
        if (!(RunStatus::GetDeviceStatus())) {
            (void) aclrtFreeHost(inputBuff);
        } else {
            (void) acldvppFree(inputBuff);
        }
        return FAILED;
    }

    if (!(RunStatus::GetDeviceStatus())) { // app is running in host
        aclRet = acldvppMalloc(&picDevBuffer, inputBuffSize);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("malloc device buffer failed. size is %u, errorCode is %d",
                      inputBuffSize, static_cast<int32_t>(aclRet));
            (void) aclrtFreeHost(inputBuff);
            return FAILED;
        }

        // if app is running in host, need copy data from host to device
        aclRet = aclrtMemcpy(picDevBuffer, inputBuffSize, inputBuff, inputBuffSize, ACL_MEMCPY_HOST_TO_DEVICE);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("memcpy failed. device buffer size is %u, input host buffer size is %u, errorCode is %d",
                      inputBuffSize, inputBuffSize, static_cast<int32_t>(aclRet));
            (void) acldvppFree(picDevBuffer);
            (void) aclrtFreeHost(inputBuff);
            return FAILED;
        }
        (void) aclrtFreeHost(inputBuff);
    } else { // app is running in device
        picDevBuffer = inputBuff;
    }
    devPicBufferSize = inputBuffSize;

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
