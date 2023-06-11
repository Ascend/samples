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
#include "JpegDHelper.h"

using namespace std;

JpegDHelper::JpegDHelper(aclrtStream& stream, acldvppChannelDesc *dvppChannelDesc)
    :stream_(stream), decodeOutBufferDev_(nullptr),
    decodeOutputDesc_(nullptr), dvppChannelDesc_(dvppChannelDesc),
    decodeOutWidthStride_(0), decodeOutHeightStride_(0)
{
}

JpegDHelper::~JpegDHelper()
{
    DestroyDecodeResource();
}

AclLiteError JpegDHelper::InitDecodeOutputDesc(ImageData& inputImage)
{
    auto socVersion = aclrtGetSocName();
    if (strncmp(socVersion, "Ascend310P3", sizeof("Ascend310P3") - 1) == 0) {
        decodeOutWidth_ = ALIGN_UP2(inputImage.width);
        decodeOutHeight_ = ALIGN_UP2(inputImage.height);
        decodeOutWidthStride_ = ALIGN_UP64(inputImage.width); // 64-byte alignment
        decodeOutHeightStride_ = ALIGN_UP16(inputImage.height); // 16-byte alignment
    } else {
        decodeOutWidth_ = inputImage.width;
        decodeOutHeight_ = inputImage.height;
        decodeOutWidthStride_ = ALIGN_UP128(inputImage.width); // 128-byte alignment
        decodeOutHeightStride_ = ALIGN_UP16(inputImage.height); // 16-byte alignment
    }
    if (decodeOutWidthStride_ == 0 || decodeOutHeightStride_ == 0) {
        ACLLITE_LOG_ERROR("Input image width %d or height %d invalid",
                          inputImage.width, inputImage.height);
        return ACLLITE_ERROR_INVALID_ARGS;
    }

    decodeOutBufferSize_ = YUV420SP_SIZE(decodeOutWidthStride_, decodeOutHeightStride_);

    aclError aclRet = acldvppMalloc(&decodeOutBufferDev_, decodeOutBufferSize_);
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
    acldvppSetPicDescFormat(decodeOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(decodeOutputDesc_, decodeOutWidth_);
    acldvppSetPicDescHeight(decodeOutputDesc_, decodeOutHeight_);
    acldvppSetPicDescWidthStride(decodeOutputDesc_, decodeOutWidthStride_);
    acldvppSetPicDescHeightStride(decodeOutputDesc_, decodeOutHeightStride_);
    acldvppSetPicDescSize(decodeOutputDesc_, decodeOutBufferSize_);

    return ACLLITE_OK;
}

AclLiteError JpegDHelper::Process(ImageData& dest, ImageData& src)
{
    int ret = InitDecodeOutputDesc(src);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("InitDecodeOutputDesc failed");
        return ret;
    }

    aclError aclRet = acldvppJpegDecodeAsync(dvppChannelDesc_,
                                             reinterpret_cast<void *>(src.data.get()),
                                             src.size, decodeOutputDesc_, stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acldvppJpegDecodeAsync failed, error: %d", aclRet);
        return ACLLITE_ERROR_JPEGD_ASYNC;
    }

    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Sync stream failed, error: %d", aclRet);
        return ACLLITE_ERROR_SYNC_STREAM;
    }
    dest.format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    dest.width = decodeOutWidth_;
    dest.height = decodeOutHeight_;
    dest.alignWidth = decodeOutWidthStride_;
    dest.alignHeight = decodeOutHeightStride_;
    dest.size = decodeOutBufferSize_;
    dest.data = SHARED_PTR_DVPP_BUF(decodeOutBufferDev_);

    return ACLLITE_OK;
}

void JpegDHelper::DestroyDecodeResource()
{
    if (decodeOutputDesc_ != nullptr) {
        acldvppDestroyPicDesc(decodeOutputDesc_);
        decodeOutputDesc_ = nullptr;
    }
}