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

* File AclLiteImageProc.cpp
* Description: handle dvpp process
*/

#include <iostream>
#include "acl/acl.h"
#include "AclLiteUtils.h"
#include "ResizeHelper.h"

using namespace std;

ResizeHelper::ResizeHelper(aclrtStream& stream, acldvppChannelDesc *dvppChannelDesc,
                       uint32_t width, uint32_t height):
  stream_(stream),
  vpcOutBufferDev_(nullptr),
  vpcInputDesc_(nullptr),
  vpcOutputDesc_(nullptr),
  resizeConfig_(nullptr),
  dvppChannelDesc_(dvppChannelDesc),
  inDevBuffer_(nullptr),
  vpcOutBufferSize_(0) {
    size_.width = width;
    size_.height = height;
}

ResizeHelper::~ResizeHelper() {
    DestroyResizeResource();
}

AclLiteError ResizeHelper::InitResizeInputDesc(ImageData& inputImage) {
    uint32_t alignWidth = inputImage.alignWidth;
    uint32_t alignHeight = inputImage.alignHeight;
    if (alignWidth == 0 || alignHeight == 0) {
        ACLLITE_LOG_ERROR("Input image width %d or height %d invalid",
                          inputImage.width, inputImage.height);
        return ACLLITE_ERROR_INVALID_ARGS;
    }

    uint32_t inputBufferSize = YUV420SP_SIZE(alignWidth, alignHeight);
    vpcInputDesc_ = acldvppCreatePicDesc();
    if (vpcInputDesc_ == nullptr) {
        ACLLITE_LOG_ERROR("Create dvpp pic desc failed");
        return ACLLITE_ERROR_CREATE_PIC_DESC;
    }

    acldvppSetPicDescData(vpcInputDesc_, inputImage.data.get());
    acldvppSetPicDescFormat(vpcInputDesc_, inputImage.format);
    acldvppSetPicDescWidth(vpcInputDesc_, inputImage.width);
    acldvppSetPicDescHeight(vpcInputDesc_, inputImage.height);
    acldvppSetPicDescWidthStride(vpcInputDesc_, alignWidth);
    acldvppSetPicDescHeightStride(vpcInputDesc_, alignHeight);
    acldvppSetPicDescSize(vpcInputDesc_, inputBufferSize);

    return ACLLITE_OK;
}

AclLiteError ResizeHelper::InitResizeOutputDesc() {
    int resizeOutWidth = size_.width;
    int resizeOutHeight = size_.height;
    int resizeOutWidthStride = ALIGN_UP16(resizeOutWidth);
    int resizeOutHeightStride = ALIGN_UP2(resizeOutHeight);
    if (resizeOutWidthStride == 0 || resizeOutHeightStride == 0) {
        ACLLITE_LOG_ERROR("Align resize width(%d) and height(%d) failed",
                          size_.width, size_.height);
        return ACLLITE_ERROR_INVALID_ARGS;
    }

    vpcOutBufferSize_ = YUV420SP_SIZE(resizeOutWidthStride, resizeOutHeightStride);
    aclError aclRet = acldvppMalloc(&vpcOutBufferDev_, vpcOutBufferSize_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Dvpp resize malloc output buffer failed, "
                          "size %d, error %d", vpcOutBufferSize_, aclRet);
        return ACLLITE_ERROR_MALLOC_DVPP;
    }

    vpcOutputDesc_ = acldvppCreatePicDesc();
    if (vpcOutputDesc_ == nullptr) {
        ACLLITE_LOG_ERROR("acldvppCreatePicDesc vpcOutputDesc_ failed");
        return ACLLITE_ERROR_CREATE_PIC_DESC;
    }

    acldvppSetPicDescData(vpcOutputDesc_, vpcOutBufferDev_);
    acldvppSetPicDescFormat(vpcOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(vpcOutputDesc_, resizeOutWidth);
    acldvppSetPicDescHeight(vpcOutputDesc_, resizeOutHeight);
    acldvppSetPicDescWidthStride(vpcOutputDesc_, resizeOutWidthStride);
    acldvppSetPicDescHeightStride(vpcOutputDesc_, resizeOutHeightStride);
    acldvppSetPicDescSize(vpcOutputDesc_, vpcOutBufferSize_);

    return ACLLITE_OK;
}

AclLiteError ResizeHelper::InitResizeResource(ImageData& inputImage) {
    resizeConfig_ = acldvppCreateResizeConfig();
    if (resizeConfig_ == nullptr) {
        ACLLITE_LOG_ERROR("Dvpp resize init failed for create config failed");
        return ACLLITE_ERROR_CREATE_RESIZE_CONFIG;
    }
    
    AclLiteError ret = InitResizeInputDesc(inputImage);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("InitResizeInputDesc failed");
        return ret;
    } 
    
    ret = InitResizeOutputDesc();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("InitResizeOutputDesc failed");
        return ret;
    } 

    return ACLLITE_OK;  
}

AclLiteError ResizeHelper::Process(ImageData& resizedImage, ImageData& srcImage) {
    AclLiteError atlRet = InitResizeResource(srcImage);
    if (atlRet != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Dvpp resize failed for init error");
        return atlRet;
    }

    // resize pic
    aclError aclRet = acldvppVpcResizeAsync(dvppChannelDesc_, vpcInputDesc_,
                                            vpcOutputDesc_, resizeConfig_, stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acldvppVpcResizeAsync failed, error: %d", aclRet);
        return ACLLITE_ERROR_RESIZE_ASYNC;
    }

    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("resize aclrtSynchronizeStream failed, error: %d", aclRet);
        return ACLLITE_ERROR_SYNC_STREAM;
    }
    resizedImage.format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    resizedImage.width = size_.width;
    resizedImage.height = size_.height;
    resizedImage.alignWidth = ALIGN_UP16(size_.width);
    resizedImage.alignHeight = ALIGN_UP2(size_.height);
    resizedImage.size = vpcOutBufferSize_;
    resizedImage.data = SHARED_PTR_DVPP_BUF(vpcOutBufferDev_);

    DestroyResizeResource();

    return ACLLITE_OK;
}

void ResizeHelper::DestroyResizeResource() {
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
