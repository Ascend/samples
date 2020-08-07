/**
 * ============================================================================
 *
 * Copyright (C) 2018, Hisilicon Technologies Co., Ltd. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1 Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *   2 Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *   3 Neither the names of the copyright holders nor the names of the
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * ============================================================================
 */
//#include "face_detection_params.h"
#include <memory>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include "acl/acl.h"
#include "acl/ops/acl_dvpp.h"
#include "atlas_utils_common.h"

using namespace std;

extern "C" {

#if 0
void* CopyDataHostToDvpp(void* data, int size) {
    void* buffer = nullptr;

    auto aclRet = acldvppMalloc(&buffer, size);
    if (aclRet != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("acl malloc dvpp data failed, dataSize=%u, ret=%d", 
                      size, aclRet);
        return nullptr;
    }
    printf("malloc dvpp memory size %d ok", size);
    // copy input to device memory
    aclRet = aclrtMemcpy(buffer, size, data, size, ACL_MEMCPY_HOST_TO_DEVICE);
    if (aclRet != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("acl memcpy data to dvpp failed, size %u, error %d", size, aclRet);
        acldvppFree(buffer);
        return nullptr;
    }
    printf("copy data to dvpp ok");

    return buffer;
}

void* CopyDataHostToDevice(void* data, int size) {
    void* buffer = nullptr;

    auto aclRet = aclrtMalloc(&buffer, size, ACL_MEM_MALLOC_NORMAL_ONLY);
    if (aclRet != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("acl malloc device memory failed, dataSize=%u, ret=%d", 
                      size, aclRet);
        return nullptr;
    }
    // copy input to device memory
    aclRet = aclrtMemcpy(buffer, size, data, size, ACL_MEMCPY_HOST_TO_DEVICE);
    if (aclRet != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("acl memcpy data to dev failed, size %u, error %d", size, aclRet);
        aclrtFree(buffer);
        return nullptr;
    }

    return buffer;
}

void* CopyDataDeviceToDevice(void* data, int size) {
    void* buffer = nullptr;

    auto aclRet = aclrtMalloc(&buffer, size, ACL_MEM_MALLOC_NORMAL_ONLY);
    if (aclRet != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("acl malloc device memory failed, dataSize=%u, ret=%d", 
                      size, aclRet);
        return nullptr;
    }
    // copy input to device memory
    aclRet = aclrtMemcpy(buffer, size, data, size, ACL_MEMCPY_DEVICE_TO_DEVICE);
    if (aclRet != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("acl memcpy data to dev failed, size %u, error %d", size, aclRet);
        aclrtFree(buffer);
        return nullptr;
    }

    return buffer;
}

void* CopyDataDeviceToHost(void* deviceData, uint32_t dataLen) {  
    void *outHostData = nullptr;

    aclError ret = aclrtMallocHost(&outHostData, dataLen);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("aclrtMallocHost failed, ret[%d]", ret);
        return nullptr;
    }

    ret = aclrtMemcpy(outHostData, dataLen, deviceData, 
                      dataLen, ACL_MEMCPY_DEVICE_TO_HOST);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("aclrtMemcpy failed, ret[%d]", ret);
        aclrtFreeHost(outHostData);
        return nullptr;
    }

    return outHostData;
}

void* CopyDataDeviceToNewBuf(void* deviceData, uint32_t dataLen) {  
    uint8_t* outHostData = new uint8_t[dataLen];

/*    aclError ret = aclrtMallocHost(&outHostData, dataLen);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("aclrtMallocHost failed, ret[%d]", ret);
        return nullptr;
    }
*/
    int ret = aclrtMemcpy(outHostData, dataLen, deviceData, 
                      dataLen, ACL_MEMCPY_DEVICE_TO_HOST);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("aclrtMemcpy failed, ret[%d]", ret);
        aclrtFreeHost(outHostData);
        return nullptr;
    }

    return (void *)outHostData;
}



void SaveBinFile(const char* filename, void* data, uint32_t size) {
    FILE *outFileFp = fopen(filename, "wb+");
    if (outFileFp == nullptr) {
        ASC_LOG_ERROR("Save file %s failed for open error", filename);
        return;
    }
    fwrite(data, 1, size, outFileFp);

    fflush(outFileFp);
    fclose(outFileFp);
}

char* ReadBinFile(const std::string& fileName, uint32_t& fileSize)
{
    std::ifstream binFile(fileName, std::ifstream::binary);
    if (binFile.is_open() == false) {
        ASC_LOG_ERROR("open file %s failed", fileName.c_str());
        return nullptr;
    }

    binFile.seekg(0, binFile.end);
    uint32_t binFileBufferLen = binFile.tellg();
    if (binFileBufferLen == 0) {
        ASC_LOG_ERROR("binfile is empty, filename is %s", fileName.c_str());
        binFile.close();
        return nullptr;
    }

    binFile.seekg(0, binFile.beg);

    char* binFileBufferData = new(std::nothrow) char[binFileBufferLen];
    if (binFileBufferData == nullptr) {
        ASC_LOG_ERROR("malloc binFileBufferData failed");
        binFile.close();
        return nullptr;
    }
    binFile.read(binFileBufferData, binFileBufferLen);
    binFile.close();
    fileSize = binFileBufferLen;
    return binFileBufferData;
}

int ReadImageFile(ImageData* image, const string& filename) {
    char* data;
    uint32_t size = 0;

    data = ReadBinFile(filename, size);
    if (data == nullptr) {
        ASC_LOG_ERROR("Read image file %s failed", filename.c_str());
        return STATUS_ERROR;
    }

    image->data = SHARED_PRT_U8_BUF(data);
    image->size = size;
    printf("read image ok, size %d", size);
    return STATUS_OK;
}
#endif
}
