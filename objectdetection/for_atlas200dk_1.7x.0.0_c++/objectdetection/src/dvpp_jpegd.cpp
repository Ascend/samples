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
#include "utils.h"
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


Result DvppJpegD::InitDecodeOutputDesc(ImageData& inputImage)
{
    uint32_t decodeOutWidthStride = ALIGN_UP16(inputImage.width);
    uint32_t decodeOutHeightStride = ALIGN_UP2(inputImage.height);
    if (decodeOutWidthStride == 0 || decodeOutHeightStride == 0) {
        ERROR_LOG("InitDecodeOutputDesc AlignmentHelper failed");
        return FAILED;
    }

    /*预测接口要求aclrtMallocHost分配的内存,AI1上运行当前应用中会需要
	多次拷贝图片内存,暂不用.
    acldvppJpegPredictDecSize(inputImage.data.get(), inputImage.size,
    PIXEL_FORMAT_YUV_SEMIPLANAR_420, &decodeOutBufferSize);*/
    
	/*分配一块足够大的内存*/
    uint32_t decodeOutBufferSize = 
	    YUV420SP_SIZE(decodeOutWidthStride, decodeOutHeightStride) * 4;

    aclError aclRet = acldvppMalloc(&decodeOutBufferDev_, decodeOutBufferSize);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppMalloc decodeOutBufferDev_ failed, aclRet = %d", aclRet);
        return FAILED;
    }

    decodeOutputDesc_ = acldvppCreatePicDesc();
    if (decodeOutputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc decodeOutputDesc_ failed");
        return FAILED;
    }

    acldvppSetPicDescData(decodeOutputDesc_, decodeOutBufferDev_);
    acldvppSetPicDescFormat(decodeOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(decodeOutputDesc_, inputImage.width);
    acldvppSetPicDescHeight(decodeOutputDesc_, inputImage.height);
    acldvppSetPicDescWidthStride(decodeOutputDesc_, decodeOutWidthStride);
    acldvppSetPicDescHeightStride(decodeOutputDesc_, decodeOutHeightStride);
    acldvppSetPicDescSize(decodeOutputDesc_, decodeOutBufferSize);
    return SUCCESS;
}

Result DvppJpegD::Process(ImageData& dest, ImageData& src)
{
    printf("src image width %d, height %d, size %d\n", src.width, src.height, src.size);



    int ret = InitDecodeOutputDesc(src);
    if (ret != SUCCESS) {
        ERROR_LOG("InitDecodeOutputDesc failed");
        return FAILED;
    }

    aclError aclRet = acldvppJpegDecodeAsync(dvppChannelDesc_, reinterpret_cast<void *>(src.data.get()),
        src.size, decodeOutputDesc_, stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppJpegDecodeAsync failed, aclRet = %d", aclRet);
        return FAILED;
    }

    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("decode aclrtSynchronizeStream failed, aclRet = %d", aclRet);
        return FAILED;
    }

    dest.width = src.width;
    dest.height = src.height;
    dest.alignWidth = ALIGN_UP16(src.width);
    dest.alignHeight = ALIGN_UP2(src.height);
    dest.size = YUV420SP_SIZE(dest.alignWidth, dest.alignHeight);
    dest.data = SHARED_PRT_DVPP_BUF(decodeOutBufferDev_);
    INFO_LOG("convert image success");

    return SUCCESS;
}

void DvppJpegD::DestroyDecodeResource()
{
    if (decodeOutputDesc_ != nullptr) {
        acldvppDestroyPicDesc(decodeOutputDesc_);
        decodeOutputDesc_ = nullptr;
    }
}
