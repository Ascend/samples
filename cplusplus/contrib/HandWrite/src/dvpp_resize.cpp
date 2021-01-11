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
#include "utils.h"
#include "dvpp_resize.h"
using namespace std;



DvppResize::DvppResize(aclrtStream& stream, acldvppChannelDesc *dvppChannelDesc,
                       uint32_t width, uint32_t height)
: stream_(stream), dvppChannelDesc_(dvppChannelDesc),
resizeConfig_(nullptr),  vpcInputDesc_(nullptr), vpcOutputDesc_(nullptr),
inDevBuffer_(nullptr),vpcOutBufferDev_(nullptr),vpcOutBufferSize_(0){
    size_.width = width;
    size_.height = height;
    resizeConfig_ = acldvppCreateResizeConfig();
    if (resizeConfig_ == nullptr) {
        ERROR_LOG("Dvpp resize init failed for create config failed\n");
    }
}

DvppResize::~DvppResize()
{

    if (resizeConfig_ != nullptr) {
        (void)acldvppDestroyResizeConfig(resizeConfig_);
        resizeConfig_ = nullptr;
    }
}

Result DvppResize::InitResizeInputDesc(ImageData& inputImage)
{
    uint32_t alignWidth = ALIGN_UP16(inputImage.width);
    uint32_t alignHeight = ALIGN_UP2(inputImage.height);
    if (alignWidth == 0 || alignHeight == 0) {
        ERROR_LOG("InitResizeInputDesc AlignmentHelper failed. image w %d, h %d, align w%d, h%d\n",
         inputImage.width, inputImage.height, alignWidth, alignHeight);
        return FAILED;
    }
    uint32_t inputBufferSize = YUV420SP_SIZE(alignWidth, alignHeight);
    vpcInputDesc_ = acldvppCreatePicDesc();
    if (vpcInputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc vpcInputDesc_ failed\n");
        return FAILED;
    }
    INFO_LOG("input image width %d, height %d, al w %d h %d\n", inputImage.width, inputImage.height, alignWidth, alignHeight);
    INFO_LOG("input size %d, inputBufferSize %d\n", inputImage.size, inputBufferSize);

    acldvppSetPicDescData(vpcInputDesc_, inputImage.data.get()); //  JpegD . vpcResize
    acldvppSetPicDescFormat(vpcInputDesc_, format_);
    acldvppSetPicDescWidth(vpcInputDesc_, inputImage.width);
    acldvppSetPicDescHeight(vpcInputDesc_, inputImage.height);
    acldvppSetPicDescWidthStride(vpcInputDesc_, alignWidth);
    acldvppSetPicDescHeightStride(vpcInputDesc_, alignHeight);
    acldvppSetPicDescSize(vpcInputDesc_, inputBufferSize);
    return SUCCESS;
}

Result DvppResize::InitResizeOutputDesc()
{
    int resizeOutWidth = size_.width;
    int resizeOutHeight = size_.height;
    int resizeOutWidthStride = ALIGN_UP16(resizeOutWidth);
    int resizeOutHeightStride = ALIGN_UP2(resizeOutHeight);
    if (resizeOutWidthStride == 0 || resizeOutHeightStride == 0) {
        ERROR_LOG("InitResizeOutputDesc AlignmentHelper failed\n");
        return FAILED;
    }

    vpcOutBufferSize_ = YUV420SP_SIZE(resizeOutWidthStride, resizeOutHeightStride);
    aclError aclRet = acldvppMalloc(&vpcOutBufferDev_, vpcOutBufferSize_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppMalloc vpcOutBufferDev_ failed, aclRet = %d\n", aclRet);
        return FAILED;
    }

    vpcOutputDesc_ = acldvppCreatePicDesc();
    if (vpcOutputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc vpcOutputDesc_ failed\n");
        return FAILED;
    }
    INFO_LOG("vpcOutBufferSize_ %d\n", vpcOutBufferSize_);
    acldvppSetPicDescData(vpcOutputDesc_, vpcOutBufferDev_);
    acldvppSetPicDescFormat(vpcOutputDesc_, format_);
    acldvppSetPicDescWidth(vpcOutputDesc_, resizeOutWidth);
    acldvppSetPicDescHeight(vpcOutputDesc_, resizeOutHeight);
    acldvppSetPicDescWidthStride(vpcOutputDesc_, resizeOutWidthStride);
    acldvppSetPicDescHeightStride(vpcOutputDesc_, resizeOutHeightStride);
    acldvppSetPicDescSize(vpcOutputDesc_, vpcOutBufferSize_);

    return SUCCESS;
}

Result DvppResize::InitResizeResource(ImageData& inputImage) {
    format_ = static_cast<acldvppPixelFormat>(PIXEL_FORMAT_YUV_SEMIPLANAR_420);

    if (SUCCESS != InitResizeInputDesc(inputImage)) {
        ERROR_LOG("InitResizeInputDesc failed");
        return FAILED;
    } 

    if (SUCCESS != InitResizeOutputDesc()) {
        ERROR_LOG("InitResizeOutputDesc failed");
        return FAILED;
    } 

    return SUCCESS;  
}

Result DvppResize::Process(ImageData& resizedImage, ImageData& srcImage)
{
    if (SUCCESS != InitResizeResource(srcImage)) {
        ERROR_LOG("Dvpp resize failed for init error\n");
        return FAILED;
    }

    // resize pic
    aclError aclRet = acldvppVpcResizeAsync(dvppChannelDesc_, vpcInputDesc_,
        vpcOutputDesc_, resizeConfig_, stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppVpcResizeAsync failed, aclRet = %d\n", aclRet);
        return FAILED;
    }

    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("resize aclrtSynchronizeStream failed, aclRet = %d\n", aclRet);
        return FAILED;
    }

    resizedImage.width = size_.width;
    resizedImage.height = size_.height;
    resizedImage.alignWidth = ALIGN_UP16(size_.width);
    resizedImage.alignHeight = ALIGN_UP2(size_.height);
    resizedImage.size = vpcOutBufferSize_;
    resizedImage.data = SHARED_PRT_DVPP_BUF(vpcOutBufferDev_);
    DestroyResizeResource();

    return SUCCESS;
}

void DvppResize::DestroyResizeResource()
{
    if (vpcInputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(vpcInputDesc_);
        vpcInputDesc_ = nullptr;
    }

    if (vpcOutputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(vpcOutputDesc_);
        vpcOutputDesc_ = nullptr;
    }
}
