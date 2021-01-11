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
#include "dvpp_jpegd.h"

using namespace std;

DvppJpegD::DvppJpegD(aclrtStream& stream,  acldvppChannelDesc *dvppChannelDesc)
    : stream_(stream), dvppChannelDesc_(dvppChannelDesc),
      decodeOutBufferDev_(nullptr), decodeOutputDesc_(nullptr)
{
}

DvppJpegD::~DvppJpegD()
{
    DestroyDecodeResource();
}


AtlasError DvppJpegD::InitDecodeOutputDesc(ImageData& inputImage)
{
    uint32_t decodeOutWidthStride = ALIGN_UP16(inputImage.width);
    uint32_t decodeOutHeightStride = ALIGN_UP2(inputImage.height);
    if (decodeOutWidthStride == 0 || decodeOutHeightStride == 0) {
        ATLAS_LOG_ERROR("Input image width %d or height %d invalid",
                        inputImage.width, inputImage.height);
        return ATLAS_ERROR_INVALID_ARGS;
    }

    uint32_t decodeOutBufferSize = 
                YUV420SP_SIZE(decodeOutWidthStride, decodeOutHeightStride) * 2;

    aclError aclRet = acldvppMalloc(&decodeOutBufferDev_, decodeOutBufferSize);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Malloc dvpp memory failed, error:%d", aclRet);
        return ATLAS_ERROR_MALLOC_DVPP;
    }

    decodeOutputDesc_ = acldvppCreatePicDesc();
    if (decodeOutputDesc_ == nullptr) {
        ATLAS_LOG_ERROR("Create dvpp pic desc failed");
        return ATLAS_ERROR_CREATE_PIC_DESC;
    }

    acldvppSetPicDescData(decodeOutputDesc_, decodeOutBufferDev_);
    acldvppSetPicDescFormat(decodeOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(decodeOutputDesc_, inputImage.width);
    acldvppSetPicDescHeight(decodeOutputDesc_, inputImage.height);
    acldvppSetPicDescWidthStride(decodeOutputDesc_, decodeOutWidthStride);
    acldvppSetPicDescHeightStride(decodeOutputDesc_, decodeOutHeightStride);
    acldvppSetPicDescSize(decodeOutputDesc_, decodeOutBufferSize);

    return ATLAS_OK;
}

AtlasError DvppJpegD::Process(ImageData& dest, ImageData& src)
{
    int ret = InitDecodeOutputDesc(src);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("InitDecodeOutputDesc failed");
        return ret;
    }

    aclError aclRet = acldvppJpegDecodeAsync(dvppChannelDesc_, 
                                             reinterpret_cast<void *>(src.data.get()),
                                             src.size, decodeOutputDesc_, stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("acldvppJpegDecodeAsync failed, error: %d", aclRet);
        return ATLAS_ERROR_JPEGD_ASYNC;
    }

    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Sync stream failed, error: %d", aclRet);
        return ATLAS_ERROR_SYNC_STREAM;
    }

    dest.width = src.width;
    dest.height = src.height;
    dest.alignWidth = ALIGN_UP16(src.width);
    dest.alignHeight = ALIGN_UP2(src.height);
    dest.size = YUV420SP_SIZE(dest.alignWidth, dest.alignHeight);
    dest.data = SHARED_PRT_DVPP_BUF(decodeOutBufferDev_);

    return ATLAS_OK;
}

void DvppJpegD::DestroyDecodeResource()
{
    if (decodeOutputDesc_ != nullptr) {
        acldvppDestroyPicDesc(decodeOutputDesc_);
        decodeOutputDesc_ = nullptr;
    }
}
