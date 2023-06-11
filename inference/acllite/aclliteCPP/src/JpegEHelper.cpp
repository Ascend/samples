/**
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2022. All rights reserved.
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

* File AclLiteImageProc.cpp
* Description: handle dvpp process
*/

#include <iostream>
#include "acl/acl.h"
#include "JpegEHelper.h"

using namespace std;

JpegEHelper::JpegEHelper(aclrtStream& stream, acldvppChannelDesc* dvppChannelDesc)
    :stream_(stream), encodeOutBufferDev_(nullptr), jpegeConfig_(nullptr),
    encodeInputDesc_(nullptr), dvppChannelDesc_(dvppChannelDesc), encodeOutBufferSize_(0)
{
}

JpegEHelper::~JpegEHelper()
{
}

AclLiteError JpegEHelper::InitEncodeInputDesc(ImageData& inputImage)
{
    uint32_t alignWidth = inputImage.alignWidth;
    uint32_t alignHeight = inputImage.alignHeight;
    if (alignWidth == 0 || alignHeight == 0) {
        ACLLITE_LOG_ERROR("Input image width %d or height %d invalid",
                          inputImage.width, inputImage.height);
        return ACLLITE_ERROR_INVALID_ARGS;
    }
    uint32_t inputBufferSize = YUV420SP_SIZE(alignWidth, alignHeight);

    encodeInputDesc_ = acldvppCreatePicDesc();
    if (encodeInputDesc_ == nullptr) {
        ACLLITE_LOG_ERROR("Create dvpp pic desc failed");
        return ACLLITE_ERROR_CREATE_PIC_DESC;
    }
   
    acldvppSetPicDescData(encodeInputDesc_,
                          reinterpret_cast<void *>(inputImage.data.get()));
    acldvppSetPicDescFormat(encodeInputDesc_, inputImage.format);
    acldvppSetPicDescWidth(encodeInputDesc_, inputImage.width);
    acldvppSetPicDescHeight(encodeInputDesc_, inputImage.height);
    acldvppSetPicDescWidthStride(encodeInputDesc_, alignWidth);
    acldvppSetPicDescHeightStride(encodeInputDesc_, alignHeight);
    acldvppSetPicDescSize(encodeInputDesc_, inputBufferSize);

    return ACLLITE_OK;
}

AclLiteError JpegEHelper::InitJpegEResource(ImageData& inputImage)
{
    uint32_t encodeLevel = 100; // default optimal level (0-100)

    AclLiteError ret = InitEncodeInputDesc(inputImage);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Dvpp jpege init input desc failed");
        return ret;
    }

    jpegeConfig_ = acldvppCreateJpegeConfig();
    acldvppSetJpegeConfigLevel(jpegeConfig_, encodeLevel);

    acldvppJpegPredictEncSize(encodeInputDesc_, jpegeConfig_, &encodeOutBufferSize_);
    aclError aclRet = acldvppMalloc(&encodeOutBufferDev_, encodeOutBufferSize_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Malloc dvpp memory error(%d)", aclRet);
        return ACLLITE_ERROR_MALLOC_DVPP;
    }
    
    return ACLLITE_OK;
}

AclLiteError JpegEHelper::Process(ImageData& destJpegImage, ImageData& srcYuvImage)
{
    AclLiteError ret = InitJpegEResource(srcYuvImage);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Dvpp jpege failed for init error");
        return ret;
    }

    aclError aclRet = acldvppJpegEncodeAsync(dvppChannelDesc_,
                                             encodeInputDesc_,
                                             encodeOutBufferDev_,
                                             &encodeOutBufferSize_,
                                             jpegeConfig_, stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Dvpp jpege async failed, error:%d", aclRet);
        return ACLLITE_ERROR_JPEGE_ASYNC;
    }

    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Dvpp jpege sync stream failed, error:%d", aclRet);
        return ACLLITE_ERROR_SYNC_STREAM;
    }

    destJpegImage.width = srcYuvImage.width;
    destJpegImage.height = srcYuvImage.height;
    destJpegImage.size = encodeOutBufferSize_;
    destJpegImage.data.reset((uint8_t*)encodeOutBufferDev_,
                             [](uint8_t* p) { acldvppFree(p); });

    DestroyEncodeResource();
    
    return ACLLITE_OK;
}

void JpegEHelper::DestroyEncodeResource()
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