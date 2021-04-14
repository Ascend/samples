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

#include <memory>
#include <mutex>
#include <unistd.h>
#include <vector>

#include "acl/acl.h"
#include "acl/acl_base.h"
#include "atlas_utils.h"

using namespace std;
extern "C" { 
int copy_data_buf_to_local(DataBuf& dest, DataBuf& src, int runMode) {
    dest.data = (uint8_t *)copy_data_device_to_local(src.data, src.size,
                                                 (aclrtRunMode)runMode);
    if (dest.data == nullptr) {
        ASC_LOG_ERROR("New malloc memory failed");
        return FAILED;
    }

    dest.size = src.size;

    return SUCCESS;
}

void release_data_buf(DataBuf& dataBuf) {
    delete[](dataBuf.data);
    dataBuf.data = nullptr;
    dataBuf.size = 0;
}

void* copy_data_device_to_local(void* deviceData, uint32_t dataSize, aclrtRunMode mode) {
    uint8_t* buffer = new uint8_t[dataSize];
    if (buffer == nullptr) {
        ASC_LOG_ERROR("New malloc memory failed");
        return nullptr;
    }
    aclrtMemcpyKind policy = (mode == ACL_HOST)? ACL_MEMCPY_DEVICE_TO_HOST :
                                                 ACL_MEMCPY_DEVICE_TO_DEVICE;

    aclError aclRet = aclrtMemcpy(buffer, dataSize, deviceData, dataSize, policy);
    if (aclRet != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Copy device data to local failed, aclRet is %d", aclRet);
        delete[](buffer);
        return nullptr;
    }

    return (void*)buffer;
}

void* MallocMemory(uint32_t dataSize, int memType) {
    void* buffer = nullptr;
    aclError aclRet;
    uint32_t size = ALIGN_UP128(dataSize);

    if (memType == MEMORY_DEVICE)
        aclRet = aclrtMalloc(&buffer, size, ACL_MEM_MALLOC_HUGE_FIRST);
    else if (memType == MEMORY_DVPP)
        aclRet = acldvppMalloc(&buffer, size);
    else {
        ASC_LOG_ERROR("Malloc memory failed for unknown memory type %d", memType);
        return nullptr;
    }
    if (aclRet != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Malloc %s memeory failed, aclRet is %d", 
                  (memType == MEMORY_DEVICE)?"device":"dvpp", aclRet);
        return nullptr;
    }

    return buffer;    
}

void FreeMemory(void* mem, int memType) {
    if (memType == MEMORY_DEVICE)
        aclrtFree(mem);
    else if (memType == MEMORY_DVPP)
        acldvppFree(mem);
    else 
        ASC_LOG_ERROR("Free memory failed for unknown memory type %d", memType);
}

void* copy_data_local_to_device(void* data, uint32_t dataSize,
                            aclrtRunMode mode, int memType) {
    void* buffer = MallocMemory(dataSize, memType);
    if (buffer == nullptr) return nullptr;

    aclrtMemcpyKind policy = (mode == ACL_HOST)? ACL_MEMCPY_HOST_TO_DEVICE : 
                                                 ACL_MEMCPY_DEVICE_TO_DEVICE;
    aclError aclRet = aclrtMemcpy(buffer, dataSize, data, dataSize, policy);
    if (aclRet != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Copy data to device failed, aclRet is %d", aclRet);
        FreeMemory(buffer, memType);
        return nullptr;
    }

    return buffer;
}

void AclLog(int level, const char *func, 
            const char *file, uint32_t line, const char* logStr){
    aclAppLog((aclLogLevel)level, func, file, line, "%s\n", logStr);
}

}

