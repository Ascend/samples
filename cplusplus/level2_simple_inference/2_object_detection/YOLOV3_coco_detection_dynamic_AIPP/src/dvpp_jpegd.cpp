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
#include "acl/acl.h"
#include "dvpp_jpegd.h"
#include "utils.h"
using namespace std;

DvppJpegD::DvppJpegD(aclrtStream& stream, acldvppChannelDesc *dvppChannelDesc)
    : g_stream_(stream), g_dvppChannelDesc_(dvppChannelDesc),
      g_decodeOutBufferDev_(nullptr), g_decodeOutputDesc_(nullptr)
{
}

DvppJpegD::~DvppJpegD()
{
    DestroyDecodeResource();
}

Result DvppJpegD::InitDecodeOutputDesc(ImageData& inputImage)
{
    uint32_t decodeOutWidthStride = ALIGN_UP128(inputImage.width);
    uint32_t decodeOutHeightStride = ALIGN_UP16(inputImage.height);
    if (decodeOutWidthStride == 0 || decodeOutHeightStride == 0) {
        ERROR_LOG("InitDecodeOutputDesc AlignmentHelper failed");
        return FAILED;
    }

	/* Allocate a large enough memory */
    uint32_t decodeOutBufferSize =
	    YUV420SP_SIZE(decodeOutWidthStride, decodeOutHeightStride) * 4;

    aclError aclRet = acldvppMalloc(&g_decodeOutBufferDev_, decodeOutBufferSize);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("acldvppMalloc g_decodeOutBufferDev_ failed, aclRet = %d", aclRet);
        return FAILED;
    }

    g_decodeOutputDesc_ = acldvppCreatePicDesc();
    if (g_decodeOutputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc g_decodeOutputDesc_ failed");
        return FAILED;
    }

    acldvppSetPicDescData(g_decodeOutputDesc_, g_decodeOutBufferDev_);
    acldvppSetPicDescFormat(g_decodeOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(g_decodeOutputDesc_, inputImage.width);
    acldvppSetPicDescHeight(g_decodeOutputDesc_, inputImage.height);
    acldvppSetPicDescWidthStride(g_decodeOutputDesc_, decodeOutWidthStride);
    acldvppSetPicDescHeightStride(g_decodeOutputDesc_, decodeOutHeightStride);
    acldvppSetPicDescSize(g_decodeOutputDesc_, decodeOutBufferSize);
    return SUCCESS;
}

Result DvppJpegD::Process(ImageData& dest, ImageData& src)
{
    int ret = InitDecodeOutputDesc(src);
    if (ret != SUCCESS) {
        ERROR_LOG("InitDecodeOutputDesc failed");
        return FAILED;
    }

    aclError aclRet = acldvppJpegDecodeAsync(g_dvppChannelDesc_, reinterpret_cast<void *>(src.data.get()),
        src.size, g_decodeOutputDesc_, g_stream_);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("acldvppJpegDecodeAsync failed, aclRet = %d", aclRet);
        return FAILED;
    }

    aclRet = aclrtSynchronizeStream(g_stream_);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("decode aclrtSynchronizeStream failed, aclRet = %d", aclRet);
        return FAILED;
    }

    auto socVersion = aclrtGetSocName();
    INFO_LOG("Current soc version is %s", socVersion);
    if (strncmp(socVersion, "Ascend310P3", sizeof("Ascend310P3") - 1) == 0) {
        dest.width = ALIGN_UP2(src.width);
        dest.height = ALIGN_UP2(src.height);
        dest.alignWidth = ALIGN_UP64(src.width); // 64-byte alignment
        dest.alignHeight = ALIGN_UP16(src.height); // 16-byte alignment
    } else {
        dest.width = src.width;
        dest.height = src.height;
        dest.alignWidth = ALIGN_UP128(src.width); // 128-byte alignment
        dest.alignHeight = ALIGN_UP16(src.height); // 16-byte alignment
    }
    dest.size = YUV420SP_SIZE(dest.alignWidth, dest.alignHeight);
    dest.data = SHARED_PTR_DVPP_BUF(g_decodeOutBufferDev_);
    INFO_LOG("convert image success");

    return SUCCESS;
}

void DvppJpegD::DestroyDecodeResource()
{
    if (g_decodeOutputDesc_ != nullptr) {
        acldvppDestroyPicDesc(g_decodeOutputDesc_);
        g_decodeOutputDesc_ = nullptr;
    }
}
