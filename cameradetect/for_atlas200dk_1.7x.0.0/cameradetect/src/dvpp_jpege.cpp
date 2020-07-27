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

DvppJpegE::DvppJpegE(aclrtStream& stream, acldvppChannelDesc* dvppChannelDesc)
    : stream_(stream), dvppChannelDesc_(dvppChannelDesc), jpegeConfig_(nullptr), encodeOutBufferSize_(0), 
      encodeOutBufferDev_(nullptr), encodeInputDesc_(nullptr){
}


DvppJpegE::~DvppJpegE() {

}

int DvppJpegE::InitEncodeInputDesc(ImageData* inputImage)
{
    uint32_t alignWidth = ALIGN_UP16(inputImage->width);
    uint32_t alignHeight = ALIGN_UP2(inputImage->height);
    if (alignWidth == 0 || alignHeight == 0) {
        ASC_LOG_ERROR("InitResizeInputDesc AlignmentHelper failed. image w %d, h %d, align w%d, h%d",
            inputImage->width, inputImage->height, alignWidth, alignHeight);
        return STATUS_ERROR;
    }
    uint32_t inputBufferSize = YUV420SP_SIZE(alignWidth, alignHeight);

    encodeInputDesc_ = acldvppCreatePicDesc();
    if (encodeInputDesc_ == nullptr) {
        ASC_LOG_ERROR("acldvppCreatePicDesc encodeInputDesc_ failed");
        return STATUS_ERROR;
    }
   
/*    printf("jpeg e: image w %d, h %d, size %d, align w %d, h %d, size %d\n", 
            inputImage->width, inputImage->height, inputImage->size, ALIGN_UP16(inputImage->width), ALIGN_UP2(inputImage->height),
             ALIGN_UP16(inputImage->width)*ALIGN_UP2(inputImage->height)*3/2);
*/
    acldvppSetPicDescData(encodeInputDesc_, reinterpret_cast<void *>(inputImage->data.get()));
    acldvppSetPicDescFormat(encodeInputDesc_, static_cast<acldvppPixelFormat>(inputImage->format));
    acldvppSetPicDescWidth(encodeInputDesc_, inputImage->width);
    acldvppSetPicDescHeight(encodeInputDesc_, inputImage->height);
    acldvppSetPicDescWidthStride(encodeInputDesc_, alignWidth);
    acldvppSetPicDescHeightStride(encodeInputDesc_, alignHeight);
    acldvppSetPicDescSize(encodeInputDesc_, inputBufferSize);

    return STATUS_OK;
}

int DvppJpegE::InitJpegEResource(ImageData* inputImage) {
    uint32_t encodeLevel = 100; // default optimal level (0-100)

    if (STATUS_OK != InitEncodeInputDesc(inputImage)) {
        ASC_LOG_ERROR("Dvpp jpege init input desc failed");
        return STATUS_ERROR;  
    }
	
    jpegeConfig_ = acldvppCreateJpegeConfig();
    acldvppSetJpegeConfigLevel(jpegeConfig_, encodeLevel);

    acldvppJpegPredictEncSize(encodeInputDesc_, jpegeConfig_, &encodeOutBufferSize_);
    aclError aclRet = acldvppMalloc(&encodeOutBufferDev_, encodeOutBufferSize_);
    if (aclRet != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Dvpp jpege init failed for malloc dvpp memory error(%d)", aclRet);
        return STATUS_ERROR;
    }
    
    return STATUS_OK;
}

int DvppJpegE::Process(ImageData* destJpegImage, ImageData* srcYuvImage)
{
//    ASC_LOG_INFO("dvpp Call JpegE");

    if (STATUS_OK != InitJpegEResource(srcYuvImage)) {
         ASC_LOG_ERROR("Dvpp jpege failed for init error");
         return STATUS_ERROR;
    }

    aclError aclRet = acldvppJpegEncodeAsync(dvppChannelDesc_, encodeInputDesc_, 
                                             encodeOutBufferDev_, &encodeOutBufferSize_, 
                                             jpegeConfig_, stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("acldvppJpegEncodeAsync failed, aclRet = %d", aclRet);
        return STATUS_ERROR;
    }

    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("encode aclrtSynchronizeStream failed, aclRet = %d", aclRet);
        return STATUS_ERROR;
    }

    destJpegImage->isAligned = false;
    destJpegImage->width = srcYuvImage->width;
    destJpegImage->height = srcYuvImage->height;
    destJpegImage->size = encodeOutBufferSize_;
    destJpegImage->data = SHARED_PRT_DVPP_BUF(encodeOutBufferDev_);

    DestroyEncodeResource();
    
    return STATUS_OK;
}

void DvppJpegE::DestroyEncodeResource()
{
    if (jpegeConfig_ != nullptr) {
        (void)acldvppDestroyJpegeConfig(jpegeConfig_);
        jpegeConfig_ = nullptr;
    }

    if (encodeInputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(encodeInputDesc_);
        encodeInputDesc_ = nullptr;
    }
}
