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
#include "PngDHelper.h"

using namespace std;

PngDHelper::PngDHelper(aclrtStream& stream, acldvppChannelDesc *dvppChannelDesc)
    :stream_(stream), decodeOutBufferDev_(nullptr),
    decodeOutputDesc_(nullptr), dvppChannelDesc_(dvppChannelDesc)
{
}

PngDHelper::~PngDHelper()
{
    DestroyDecodeResource();
}

AclLiteError PngDHelper::InitDecodeOutputDesc(ImageData& inputImage)
{
    uint32_t decodeOutWidthStride = ALIGN_UP128(inputImage.width)*3;
    uint32_t decodeOutHeightStride = ALIGN_UP16(inputImage.height);
    if (decodeOutWidthStride == 0 || decodeOutHeightStride == 0) {
        ACLLITE_LOG_ERROR("Input image width %d or height %d invalid",
                          inputImage.width, inputImage.height);
        return ACLLITE_ERROR_INVALID_ARGS;
    }
 
    uint32_t decodeOutBufferSize =
    RGBU8_IMAGE_SIZE(decodeOutWidthStride, decodeOutHeightStride);

    aclError aclRet = acldvppMalloc(&decodeOutBufferDev_, decodeOutBufferSize);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Malloc dvpp memory failed, error:%d", aclRet);
        return ACLLITE_ERROR_MALLOC_DVPP;
    }

    decodeOutputDesc_ = acldvppCreatePicDesc();
    if (decodeOutputDesc_ == nullptr) {
        ACLLITE_LOG_ERROR("Create dvpp pic desc failed");
        return ACLLITE_ERROR_CREATE_PIC_DESC;
    }

    acldvppSetPicDescData(decodeOutputDesc_, decodeOutBufferDev_);
    acldvppSetPicDescFormat(decodeOutputDesc_, PIXEL_FORMAT_RGB_888);
    acldvppSetPicDescWidth(decodeOutputDesc_, inputImage.width);
    acldvppSetPicDescHeight(decodeOutputDesc_, inputImage.height);
    acldvppSetPicDescWidthStride(decodeOutputDesc_, decodeOutWidthStride);
    acldvppSetPicDescHeightStride(decodeOutputDesc_, decodeOutHeightStride);
    acldvppSetPicDescSize(decodeOutputDesc_, decodeOutBufferSize);

    return ACLLITE_OK;
}

AclLiteError PngDHelper::Process(ImageData& dest, ImageData& src)
{
    int ret = InitDecodeOutputDesc(src);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("InitDecodeOutputDesc failed");
        return ret;
    }

    aclError aclRet = acldvppPngDecodeAsync(dvppChannelDesc_,
                                            reinterpret_cast<void *>(src.data.get()),
                                            src.size, decodeOutputDesc_, stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acldvppPngDecodeAsync failed, error: %d", aclRet);
        return ACLLITE_ERROR_PNGD_ASYNC;
    }

    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Sync stream failed, error: %d", aclRet);
        return ACLLITE_ERROR_SYNC_STREAM;
    }

    dest.format = PIXEL_FORMAT_RGB_888;
    dest.width = src.width;
    dest.height = src.height;
    dest.alignWidth = ALIGN_UP128(src.width) * 3;
    dest.alignHeight = ALIGN_UP16(src.height);
    dest.size = RGBU8_IMAGE_SIZE(dest.alignWidth, dest.alignHeight);
    dest.data = SHARED_PTR_DVPP_BUF(decodeOutBufferDev_);

    return ACLLITE_OK;
}

void PngDHelper::DestroyDecodeResource()
{
    if (decodeOutputDesc_ != nullptr) {
        acldvppDestroyPicDesc(decodeOutputDesc_);
        decodeOutputDesc_ = nullptr;
    }
}