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
#include "utils.h"
#include "dvpp_resize.h"
using namespace std;

DvppResize::DvppResize(aclrtStream& stream, acldvppChannelDesc *dvppChannelDesc,
                       uint32_t width, uint32_t height)
    : g_stream_(stream), g_dvppChannelDesc_(dvppChannelDesc),
      g_resizeConfig_(nullptr), g_vpcInputDesc_(nullptr), g_vpcOutputDesc_(nullptr),
      g_inDevBuffer_(nullptr), g_vpcOutBufferDev_(nullptr), g_vpcOutBufferSize_(0)
{
    g_size_.width = width;
    g_size_.height = height;
}

DvppResize::~DvppResize()
{
    DestroyResizeResource();
}

Result DvppResize::InitResizeInputDesc(ImageData& inputImage)
{
    uint32_t alignWidth = ALIGN_UP128(inputImage.width);
    uint32_t alignHeight = ALIGN_UP16(inputImage.height);
    if (alignWidth == 0 || alignHeight == 0) {
        ERROR_LOG("InitResizeInputDesc AlignmentHelper failed. image w %d, h %d, align w%d, h%d",
                  inputImage.width, inputImage.height, alignWidth, alignHeight);
        return FAILED;
    }
    uint32_t inputBufferSize = YUV420SP_SIZE(alignWidth, alignHeight);
    g_vpcInputDesc_ = acldvppCreatePicDesc();
    if (g_vpcInputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc g_vpcInputDesc_ failed");
        return FAILED;
    }

    acldvppSetPicDescData(g_vpcInputDesc_, inputImage.data.get());
    acldvppSetPicDescFormat(g_vpcInputDesc_, g_format_);
    acldvppSetPicDescWidth(g_vpcInputDesc_, inputImage.width);
    acldvppSetPicDescHeight(g_vpcInputDesc_, inputImage.height);
    acldvppSetPicDescWidthStride(g_vpcInputDesc_, alignWidth);
    acldvppSetPicDescHeightStride(g_vpcInputDesc_, alignHeight);
    acldvppSetPicDescSize(g_vpcInputDesc_, inputBufferSize);
    return SUCCESS;
}

Result DvppResize::InitResizeOutputDesc()
{
    int resizeOutWidth = g_size_.width;
    int resizeOutHeight = g_size_.height;
    int resizeOutWidthStride = ALIGN_UP16(resizeOutWidth);
    int resizeOutHeightStride = ALIGN_UP2(resizeOutHeight);
    if (resizeOutWidthStride == 0 || resizeOutHeightStride == 0) {
        ERROR_LOG("InitResizeOutputDesc AlignmentHelper failed");
        return FAILED;
    }

    g_vpcOutBufferSize_ = YUV420SP_SIZE(resizeOutWidthStride, resizeOutHeightStride);
    aclError aclRet = acldvppMalloc(&g_vpcOutBufferDev_, g_vpcOutBufferSize_);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("acldvppMalloc g_vpcOutBufferDev_ failed, aclRet = %d", aclRet);
        return FAILED;
    }

    g_vpcOutputDesc_ = acldvppCreatePicDesc();
    if (g_vpcOutputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc g_vpcOutputDesc_ failed");
        return FAILED;
    }

    acldvppSetPicDescData(g_vpcOutputDesc_, g_vpcOutBufferDev_);
    acldvppSetPicDescFormat(g_vpcOutputDesc_, g_format_);
    acldvppSetPicDescWidth(g_vpcOutputDesc_, resizeOutWidth);
    acldvppSetPicDescHeight(g_vpcOutputDesc_, resizeOutHeight);
    acldvppSetPicDescWidthStride(g_vpcOutputDesc_, resizeOutWidthStride);
    acldvppSetPicDescHeightStride(g_vpcOutputDesc_, resizeOutHeightStride);
    acldvppSetPicDescSize(g_vpcOutputDesc_, g_vpcOutBufferSize_);

    return SUCCESS;
}

Result DvppResize::InitResizeResource(ImageData& inputImage)
{
    g_format_ = static_cast<acldvppPixelFormat>(PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    g_resizeConfig_ = acldvppCreateResizeConfig();
    if (g_resizeConfig_ == nullptr) {
        ERROR_LOG("Dvpp resize init failed for create config failed");
        return FAILED;
    }

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
        ERROR_LOG("Dvpp resize failed for init error");
        return FAILED;
    }

    // resize pic
    aclError aclRet = acldvppVpcResizeAsync(g_dvppChannelDesc_, g_vpcInputDesc_,
        g_vpcOutputDesc_, g_resizeConfig_, g_stream_);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("acldvppVpcResizeAsync failed, aclRet = %d", aclRet);
        return FAILED;
    }

    aclRet = aclrtSynchronizeStream(g_stream_);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("resize aclrtSynchronizeStream failed, aclRet = %d", aclRet);
        return FAILED;
    }

    resizedImage.width = g_size_.width;
    resizedImage.height = g_size_.height;
    resizedImage.alignWidth = ALIGN_UP16(g_size_.width);
    resizedImage.alignHeight = ALIGN_UP2(g_size_.height);
    resizedImage.size = g_vpcOutBufferSize_;
    resizedImage.data = SHARED_PTR_DVPP_BUF(g_vpcOutBufferDev_);

    DestroyResizeResource();

    return SUCCESS;
}

void DvppResize::DestroyResizeResource()
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
