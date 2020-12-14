/**
* Copyright 2020 Huawei Technologies Co., Ltd
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

* File dvpp_process.cpp
* Description: handle dvpp process
*/

#include <iostream>
#include "acl/acl.h"
#include "dvpp_jpege.h"
using namespace std;
static int memcount = 0;
DvppJpegE::DvppJpegE(aclrtStream& stream, acldvppChannelDesc* dvppChannelDesc)
    : stream_(stream), dvppChannelDesc_(dvppChannelDesc), jpegeConfig_(nullptr), encodeOutBufferSize_(0), 
      encodeOutBufferDev_(nullptr), encodeInputDesc_(nullptr){
    jpegeConfig_ = acldvppCreateJpegeConfig();
    acldvppSetJpegeConfigLevel(jpegeConfig_, 50);

    encodeInputDesc_ = acldvppCreatePicDesc();
    if (encodeInputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc encodeInputDesc_ failed\n");
    }
}

DvppJpegE::~DvppJpegE() {
    if (jpegeConfig_ != nullptr) {
        (void)acldvppDestroyJpegeConfig(jpegeConfig_);
        jpegeConfig_ = nullptr;
    }

    if (encodeInputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(encodeInputDesc_);
        encodeInputDesc_ = nullptr;
    }
}

Result DvppJpegE::InitEncodeInputDesc(ImageData& inputImage)
{
    uint32_t alignWidth = ALIGN_UP16(inputImage.width);
    uint32_t alignHeight = ALIGN_UP2(inputImage.height);
    if (alignWidth == 0 || alignHeight == 0) {
        ERROR_LOG("InitResizeInputDesc AlignmentHelper failed. image w %d, h %d, align w%d, h%d\n",
            inputImage.width, inputImage.height, alignWidth, alignHeight);
        return FAILED;
    }
    uint32_t inputBufferSize = YUV420SP_SIZE(alignWidth, alignHeight);

    acldvppSetPicDescData(encodeInputDesc_, reinterpret_cast<void *>(inputImage.data.get()));
    acldvppSetPicDescFormat(encodeInputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(encodeInputDesc_, inputImage.width);
    acldvppSetPicDescHeight(encodeInputDesc_, inputImage.height);
    acldvppSetPicDescWidthStride(encodeInputDesc_, alignWidth);
    acldvppSetPicDescHeightStride(encodeInputDesc_, alignHeight);
    acldvppSetPicDescSize(encodeInputDesc_, inputBufferSize);
    return SUCCESS;
}

Result DvppJpegE::InitJpegEResource(ImageData& inputImage) {
    uint32_t encodeLevel = 50; // default optimal level (0-100)

    if (SUCCESS != InitEncodeInputDesc(inputImage)) {
        ERROR_LOG("Dvpp jpege init input desc failed\n");
        return FAILED;
    }

    aclError aclRet = acldvppJpegPredictEncSize(encodeInputDesc_, jpegeConfig_, &encodeOutBufferSize_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("Dvpp jpege init failed for predict memory size error(%d)\n", aclRet);
        return FAILED;
    }

    memcount ++;
    aclRet = acldvppMalloc(&encodeOutBufferDev_, encodeOutBufferSize_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("Dvpp jpege init failed for malloc dvpp memory error(%d)\n", aclRet);
        return FAILED;
    }
    
    return SUCCESS;
}
#define SHARED_PRT_JPEGE_DVPP_BUF(buf) (shared_ptr<uint8_t>((uint8_t *)(buf), [](uint8_t* p) { memcount--;  acldvppFree(p); }))
Result DvppJpegE::Process(ImageData& destJpegImage, ImageData& srcYuvImage)
{
    //INFO_LOG("dvpp Call JpegE");

    if (SUCCESS != InitJpegEResource(srcYuvImage)) {
        ERROR_LOG("Dvpp jpege failed for init error");
         return FAILED;
    }
    uint32_t  tempLen = encodeOutBufferSize_;
    aclError aclRet = acldvppJpegEncodeAsync(dvppChannelDesc_, encodeInputDesc_, 
                                             encodeOutBufferDev_, &tempLen,
                                             jpegeConfig_, stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppJpegEncodeAsync failed, aclRet = %d\n", aclRet);
        return FAILED;
    }

    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("encode aclrtSynchronizeStream failed, aclRet = %d\n", aclRet);
        return FAILED;
    }

    destJpegImage.width = srcYuvImage.width;
    destJpegImage.height = srcYuvImage.height;
    destJpegImage.size = tempLen;

    destJpegImage.data = SHARED_PRT_JPEGE_DVPP_BUF(encodeOutBufferDev_);
    return SUCCESS;
}

