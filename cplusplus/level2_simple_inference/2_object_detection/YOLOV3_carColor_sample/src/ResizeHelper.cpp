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
#include "AclLiteUtils.h"
#include "ResizeHelper.h"

using namespace std;

ResizeHelper::ResizeHelper(aclrtStream& stream, acldvppChannelDesc *dvppChannelDesc,
                       uint32_t width, uint32_t height)
    : g_stream_(stream),
      g_vpcOutBufferDev_(nullptr),
      g_vpcInputDesc_(nullptr),
      g_vpcOutputDesc_(nullptr),
      g_resizeConfig_(nullptr),
      g_dvppChannelDesc_(dvppChannelDesc),
      g_inDevBuffer_(nullptr),
      g_vpcOutBufferSize_(0) 
{
          g_size_.width = width;
          g_size_.height = height;
}

ResizeHelper::~ResizeHelper()
{
    DestroyResizeResource();
}

AclLiteError ResizeHelper::InitResizeInputDesc(ImageData& inputImage)
{
    uint32_t alignWidth = inputImage.alignWidth;
    uint32_t alignHeight = inputImage.alignHeight;
    if (alignWidth == 0 || alignHeight == 0) {
        ACLLITE_LOG_ERROR("Input image width %d or height %d invalid",
                          inputImage.width, inputImage.height);
        return ACLLITE_ERROR_INVALID_ARGS;
    }

    uint32_t inputBufferSize = YUV420SP_SIZE(alignWidth, alignHeight);
    g_vpcInputDesc_ = acldvppCreatePicDesc();
    if (g_vpcInputDesc_ == nullptr) {
        ACLLITE_LOG_ERROR("Create dvpp pic desc failed");
        return ACLLITE_ERROR_CREATE_PIC_DESC;
    }

    acldvppSetPicDescData(g_vpcInputDesc_, inputImage.data.get());
    acldvppSetPicDescFormat(g_vpcInputDesc_, inputImage.format);
    acldvppSetPicDescWidth(g_vpcInputDesc_, inputImage.width);
    acldvppSetPicDescHeight(g_vpcInputDesc_, inputImage.height);
    acldvppSetPicDescWidthStride(g_vpcInputDesc_, alignWidth);
    acldvppSetPicDescHeightStride(g_vpcInputDesc_, alignHeight);
    acldvppSetPicDescSize(g_vpcInputDesc_, inputBufferSize);

    return ACLLITE_OK;
}

AclLiteError ResizeHelper::InitResizeOutputDesc()
{
    int resizeOutWidth = g_size_.width;
    int resizeOutHeight = g_size_.height;
    int resizeOutWidthStride = ALIGN_UP16(resizeOutWidth);
    int resizeOutHeightStride = ALIGN_UP2(resizeOutHeight);
    if (resizeOutWidthStride == 0 || resizeOutHeightStride == 0) {
        ACLLITE_LOG_ERROR("Align resize width(%d) and height(%d) failed",
                          g_size_.width, g_size_.height);
        return ACLLITE_ERROR_INVALID_ARGS;
    }

    g_vpcOutBufferSize_ = YUV420SP_SIZE(resizeOutWidthStride, resizeOutHeightStride);
    aclError aclRet = acldvppMalloc(&g_vpcOutBufferDev_, g_vpcOutBufferSize_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Dvpp resize malloc output buffer failed, "
                          "size %d, error %d", g_vpcOutBufferSize_, aclRet);
        return ACLLITE_ERROR_MALLOC_DVPP;
    }

    g_vpcOutputDesc_ = acldvppCreatePicDesc();
    if (g_vpcOutputDesc_ == nullptr) {
        ACLLITE_LOG_ERROR("acldvppCreatePicDesc g_vpcOutputDesc_ failed");
        return ACLLITE_ERROR_CREATE_PIC_DESC;
    }

    acldvppSetPicDescData(g_vpcOutputDesc_, g_vpcOutBufferDev_);
    acldvppSetPicDescFormat(g_vpcOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(g_vpcOutputDesc_, resizeOutWidth);
    acldvppSetPicDescHeight(g_vpcOutputDesc_, resizeOutHeight);
    acldvppSetPicDescWidthStride(g_vpcOutputDesc_, resizeOutWidthStride);
    acldvppSetPicDescHeightStride(g_vpcOutputDesc_, resizeOutHeightStride);
    acldvppSetPicDescSize(g_vpcOutputDesc_, g_vpcOutBufferSize_);

    return ACLLITE_OK;
}

AclLiteError ResizeHelper::InitResizeResource(ImageData& inputImage)
{
    g_resizeConfig_ = acldvppCreateResizeConfig();
    if (g_resizeConfig_ == nullptr) {
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

AclLiteError ResizeHelper::Process(ImageData& resizedImage, ImageData& srcImage)
{
    AclLiteError atlRet = InitResizeResource(srcImage);
    if (atlRet != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Dvpp resize failed for init error");
        return atlRet;
    }

    // resize pic
    aclError aclRet = acldvppVpcResizeAsync(g_dvppChannelDesc_, g_vpcInputDesc_,
                                            g_vpcOutputDesc_, g_resizeConfig_, g_stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acldvppVpcResizeAsync failed, error: %d", aclRet);
        return ACLLITE_ERROR_RESIZE_ASYNC;
    }

    aclRet = aclrtSynchronizeStream(g_stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("resize aclrtSynchronizeStream failed, error: %d", aclRet);
        return ACLLITE_ERROR_SYNC_STREAM;
    }
    resizedImage.format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    resizedImage.width = g_size_.width;
    resizedImage.height = g_size_.height;
    resizedImage.alignWidth = ALIGN_UP16(g_size_.width);
    resizedImage.alignHeight = ALIGN_UP2(g_size_.height);
    resizedImage.size = g_vpcOutBufferSize_;
    resizedImage.data = SHARED_PTR_DVPP_BUF(g_vpcOutBufferDev_);

    DestroyResizeResource();

    return ACLLITE_OK;
}

void ResizeHelper::DestroyResizeResource()
{
    if (g_resizeConfig_ != nullptr) {
        (void)acldvppDestroyResizeConfig(g_resizeConfig_);
        g_resizeConfig_ = nullptr;
    }

    if (g_vpcInputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(g_vpcInputDesc_);
        g_vpcInputDesc_ = nullptr;
    }

    if (g_vpcOutputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(g_vpcOutputDesc_);
        g_vpcOutputDesc_ = nullptr;
    }
}
