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

#ifndef _ATLAS_UTILS_COMMON_H_
#define _ATLAS_UTILS_COMMON_H_
#include <memory>
#include <mutex>
#include <unistd.h>
#include <vector>

#include "acl/acl.h"
#include "acl/acl_base.h"
#include "acl/ops/acl_dvpp.h"

using namespace std;

#define MEMORY_DEVICE 0
#define MEMORY_DVPP   1

#define ALIGN_UP(num, align) (((num) + (align) - 1) & ~((align) - 1))
#define ALIGN_UP2(num) ALIGN_UP(num, 2)
#define ALIGN_UP16(num) ALIGN_UP(num, 16)
#define ALIGN_UP128(num) ALIGN_UP(num, 128)

#define YUV420SP_SIZE(width, height) ((width) * (height) * 3 / 2)

#define SHARED_PRT_DVPP_BUF(buf) (shared_ptr<uint8_t>((uint8_t *)(buf), [](uint8_t* p) { acldvppFree(p); }))

#define ASC_LOG_ERROR(fmt, ...) \
    do{aclAppLog(ACL_ERROR, __FUNCTION__, __FILE__, __LINE__, fmt, ##__VA_ARGS__); \
    printf(fmt"\n", ##__VA_ARGS__);}while(0)

#define ASC_LOG_INFO(fmt, ...) \
    do{aclAppLog(ACL_INFO, __FUNCTION__, __FILE__, __LINE__, fmt, ##__VA_ARGS__); \
    printf(fmt"\n", ##__VA_ARGS__);}while(0) 

#define ASC_LOG_DEBUG(fmt, ...) \
    do{aclAppLog(ACL_DEBUG, __FUNCTION__, __FILE__, __LINE__, fmt, ##__VA_ARGS__); \
    printf(fmt"\n", ##__VA_ARGS__);}while(0)

struct DataBuf {
    uint32_t size;
    uint8_t* data;
};

typedef enum Result {
    SUCCESS = 0,
    FAILED = 1
}Result;

struct ImageData {
    uint32_t format = 1; //PIXEL_FORMAT_YUV_SEMIPLANAR_420
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t alignWidth = 0;
    uint32_t alignHeight = 0;
    uint32_t size = 0;
    uint8_t* data = nullptr;
};

struct FrameData {
    bool isFinished = false;
    uint32_t frameId = 0;
    uint32_t size = 0;
    void* data = nullptr;
};

extern "C" { 
int copy_data_buf_to_local(DataBuf& dest, DataBuf& src, int runMode);
void release_data_buf(DataBuf& dataBuf);
void* copy_data_device_to_local(void* deviceData,
                           uint32_t dataSize, aclrtRunMode mode);
void* copy_data_local_to_device(void* data, uint32_t dataSize,
                            aclrtRunMode mode, int memType);
void AclLog(int level, const char *func, 
            const char *file, uint32_t line, const char* logStr);
}

#endif /* HIAI_APP_IMAGE_POOL_H_ */
