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
#include "atlas_utils.h"
#include "dvpp_resize.h"

using namespace std;

DvppResize::DvppResize(aclrtStream& stream, acldvppChannelDesc *dvppChannelDesc,
                       uint32_t width, uint32_t height)
: stream_(stream), dvppChannelDesc_(dvppChannelDesc),
resizeConfig_(nullptr),  vpcInputDesc_(nullptr), vpcOutputDesc_(nullptr),
inDevBuffer_(nullptr),vpcOutBufferDev_(nullptr),vpcOutBufferSize_(0) {
    size_.width = width;
    size_.height = height;
}

DvppResize::~DvppResize() {
    DestroyResizeResource();
}

AtlasError DvppResize::InitResizeInputDesc(ImageData& inputImage) {
    uint32_t alignWidth = ALIGN_UP16(inputImage.width);
    uint32_t alignHeight = ALIGN_UP2(inputImage.height);
    if (alignWidth == 0 || alignHeight == 0) {
        ATLAS_LOG_ERROR("Input image width %d or height %d invalid",
                        inputImage.width, inputImage.height);
        return ATLAS_ERROR_INVALID_ARGS;
    }

    uint32_t inputBufferSize = YUV420SP_SIZE(alignWidth, alignHeight);
    vpcInputDesc_ = acldvppCreatePicDesc();
    if (vpcInputDesc_ == nullptr) {
        ATLAS_LOG_ERROR("Create dvpp pic desc failed");
        return ATLAS_ERROR_CREATE_PIC_DESC;
    }

    acldvppSetPicDescData(vpcInputDesc_, inputImage.data.get());
    acldvppSetPicDescFormat(vpcInputDesc_, inputImage.format);
    acldvppSetPicDescWidth(vpcInputDesc_, inputImage.width);
    acldvppSetPicDescHeight(vpcInputDesc_, inputImage.height);
    acldvppSetPicDescWidthStride(vpcInputDesc_, alignWidth);
    acldvppSetPicDescHeightStride(vpcInputDesc_, alignHeight);
    acldvppSetPicDescSize(vpcInputDesc_, inputBufferSize);

    return ATLAS_OK;
}

AtlasError DvppResize::InitResizeOutputDesc()
{
    int resizeOutWidth = size_.width;
    int resizeOutHeight = size_.height;
    int resizeOutWidthStride = ALIGN_UP16(resizeOutWidth);
    int resizeOutHeightStride = ALIGN_UP2(resizeOutHeight);
    if (resizeOutWidthStride == 0 || resizeOutHeightStride == 0) {
        ATLAS_LOG_ERROR("InitResizeOutputDesc AlignmentHelper failed");
        return ATLAS_ERROR_INVALID_ARGS;
    }

    vpcOutBufferSize_ = YUV420SP_SIZE(resizeOutWidthStride, resizeOutHeightStride);
    aclError aclRet = acldvppMalloc(&vpcOutBufferDev_, vpcOutBufferSize_);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("acldvppMalloc vpcOutBufferDev_ failed, aclRet = %d", aclRet);
        return ATLAS_ERROR_MALLOC_DVPP;
    }

    vpcOutputDesc_ = acldvppCreatePicDesc();
    if (vpcOutputDesc_ == nullptr) {
        ATLAS_LOG_ERROR("acldvppCreatePicDesc vpcOutputDesc_ failed");
        return ATLAS_ERROR_CREATE_PIC_DESC;
    }

    acldvppSetPicDescData(vpcOutputDesc_, vpcOutBufferDev_);
    acldvppSetPicDescFormat(vpcOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(vpcOutputDesc_, resizeOutWidth);
    acldvppSetPicDescHeight(vpcOutputDesc_, resizeOutHeight);
    acldvppSetPicDescWidthStride(vpcOutputDesc_, resizeOutWidthStride);
    acldvppSetPicDescHeightStride(vpcOutputDesc_, resizeOutHeightStride);
    acldvppSetPicDescSize(vpcOutputDesc_, vpcOutBufferSize_);

    return ATLAS_OK;
}

AtlasError DvppResize::InitResizeResource(ImageData& inputImage) {
    resizeConfig_ = acldvppCreateResizeConfig();
    if (resizeConfig_ == nullptr) {
        ATLAS_LOG_ERROR("Dvpp resize init failed for create config failed");
        return ATLAS_ERROR_CREATE_RESIZE_CONFIG;
    }
    
    AtlasError ret = InitResizeInputDesc(inputImage);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("InitResizeInputDesc failed");
        return ret;
    } 
    
    ret = InitResizeOutputDesc();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("InitResizeOutputDesc failed");
        return ret;
    } 

    return ATLAS_OK;  
}

AtlasError DvppResize::Process(ImageData& resizedImage, ImageData& srcImage)
{
    AtlasError atlRet = InitResizeResource(srcImage);
    if (atlRet != ATLAS_OK) {
        ATLAS_LOG_ERROR("Dvpp resize failed for init error");
        return atlRet;
    }

    // resize pic
    aclError aclRet = acldvppVpcResizeAsync(dvppChannelDesc_, vpcInputDesc_,
        vpcOutputDesc_, resizeConfig_, stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("acldvppVpcResizeAsync failed, error: %d", aclRet);
        return ATLAS_ERROR_RESIZE_ASYNC;
    }

    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("resize aclrtSynchronizeStream failed, error: %d", aclRet);
        return ATLAS_ERROR_SYNC_STREAM;
    }

    resizedImage.width = size_.width;
    resizedImage.height = size_.height;
    resizedImage.alignWidth = ALIGN_UP16(size_.width);
    resizedImage.alignHeight = ALIGN_UP2(size_.height);
    resizedImage.size = vpcOutBufferSize_;
    resizedImage.data = SHARED_PRT_DVPP_BUF(vpcOutBufferDev_);

    DestroyResizeResource();

    return ATLAS_OK;
}

void DvppResize::DestroyResizeResource()
{
    if (resizeConfig_ != nullptr) {
        (void)acldvppDestroyResizeConfig(resizeConfig_);
        resizeConfig_ = nullptr;
    }

    if (vpcInputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(vpcInputDesc_);
        vpcInputDesc_ = nullptr;
    }

    if (vpcOutputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(vpcOutputDesc_);
        vpcOutputDesc_ = nullptr;
    }
}
