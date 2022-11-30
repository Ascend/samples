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
#include "dvpp_cropandpaste.h"
using namespace std;

DvppCropAndPaste::DvppCropAndPaste(aclrtStream& stream, acldvppChannelDesc *dvppChannelDesc,
                                   uint32_t width, uint32_t height)
    : g_stream_(stream), g_dvppChannelDesc_(dvppChannelDesc),
      g_vpcInputDesc_(nullptr), g_vpcOutputDesc_(nullptr),
      g_vpcOutBufferDev_(nullptr), g_vpcOutBufferSize_(0)
{
    g_size_.width = width;
    g_size_.height = height;
}

DvppCropAndPaste::~DvppCropAndPaste()
{
    DestroyCropAndPasteResource();
}

Result DvppCropAndPaste::InitCropAndPasteInputDesc(ImageData& inputImage)
{
    g_originalImageWidth_ = inputImage.width;
    g_originalImageHeight_ = inputImage.height;
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

    acldvppSetPicDescData(g_vpcInputDesc_, inputImage.data.get()); // JpegD . vpcResize
    acldvppSetPicDescFormat(g_vpcInputDesc_, g_format_);
    acldvppSetPicDescWidth(g_vpcInputDesc_, inputImage.width);
    acldvppSetPicDescHeight(g_vpcInputDesc_, inputImage.height);
    acldvppSetPicDescWidthStride(g_vpcInputDesc_, alignWidth);
    acldvppSetPicDescHeightStride(g_vpcInputDesc_, alignHeight);
    acldvppSetPicDescSize(g_vpcInputDesc_, inputBufferSize);
    return SUCCESS;
}

Result DvppCropAndPaste::InitCropAndPasteOutputDesc()
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

// IN/OUT Desc
Result DvppCropAndPaste::InitCropAndPasteResource(ImageData& inputImage)
{
    g_format_ = static_cast<acldvppPixelFormat>(PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    if (SUCCESS != InitCropAndPasteInputDesc(inputImage)) {
        ERROR_LOG("InitCropAndPasteInputDesc failed");
        return FAILED;
    }

    if (SUCCESS != InitCropAndPasteOutputDesc()) {
        ERROR_LOG("InitCropAndPasteOutputDesc failed");
        return FAILED;
    }

    return SUCCESS;
}

Result DvppCropAndPaste::Process(ImageData& resizedImage, ImageData& srcImage)
{
    if (SUCCESS != InitCropAndPasteResource(srcImage)) {
        ERROR_LOG("Dvpp cropandpaste failed for init error");
        return FAILED;
    }

    // must even
    uint32_t cropLeftOffset = 0;
    // must even
    uint32_t cropTopOffset = 0;
    // must odd
    uint32_t cropRightOffset = (((cropLeftOffset + g_originalImageWidth_) >> 1) << 1) -1;
    // must odd
    uint32_t cropBottomOffset = (((cropTopOffset + g_originalImageHeight_) >> 1) << 1) -1;

    g_cropArea_ = acldvppCreateRoiConfig(cropLeftOffset, cropRightOffset,
    cropTopOffset, cropBottomOffset);
    if (g_cropArea_ == nullptr) {
        ERROR_LOG("acldvppCreateRoiConfig g_cropArea_ failed");
        return FAILED;
    }

    // must even
    uint32_t pasteLeftOffset = 0;
    // must even
    uint32_t pasteTopOffset = 0;
    // must odd
    uint32_t pasteRightOffset = (((pasteLeftOffset + g_size_.width) >> 1) << 1) -1;
    // must odd
    uint32_t pasteBottomOffset = (((pasteTopOffset + g_size_.height) >> 1) << 1) -1;

    g_pasteArea_ = acldvppCreateRoiConfig(pasteLeftOffset, pasteRightOffset,
    pasteTopOffset, pasteBottomOffset);
    if (g_pasteArea_ == nullptr) {
        ERROR_LOG("acldvppCreateRoiConfig g_pasteArea_ failed");
        return FAILED;
    }

    // crop and patse pic
    aclError aclRet = acldvppVpcCropAndPasteAsync(g_dvppChannelDesc_, g_vpcInputDesc_,
    g_vpcOutputDesc_, g_cropArea_, g_pasteArea_, g_stream_);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("acldvppVpcCropAndPasteAsync failed, aclRet = %d", aclRet);
        return FAILED;
    }

    aclRet = aclrtSynchronizeStream(g_stream_);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("crop and paste aclrtSynchronizeStream failed, aclRet = %d", aclRet);
        return FAILED;
    }

    resizedImage.width = g_size_.width;
    resizedImage.height = g_size_.height;
    resizedImage.alignWidth = ALIGN_UP16(g_size_.width);
    resizedImage.alignHeight = ALIGN_UP2(g_size_.height);
    resizedImage.size = g_vpcOutBufferSize_;
    resizedImage.data = SHARED_PTR_DVPP_BUF(g_vpcOutBufferDev_);
    DestroyCropAndPasteResource();

    return SUCCESS;
}

void DvppCropAndPaste::DestroyCropAndPasteResource()
{
    if (g_cropArea_ != nullptr) {
        (void)acldvppDestroyRoiConfig(g_cropArea_);
        g_cropArea_ = nullptr;
    }

    if (g_pasteArea_ != nullptr) {
        (void)acldvppDestroyRoiConfig(g_pasteArea_);
        g_pasteArea_ = nullptr;
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
