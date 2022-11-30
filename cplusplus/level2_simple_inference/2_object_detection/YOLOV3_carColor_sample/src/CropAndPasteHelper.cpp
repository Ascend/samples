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

#include <cstring>
#include <iostream>
#include "acl/acl.h"
#include "AclLiteUtils.h"
#include "CropAndPasteHelper.h"

using namespace std;

CropAndPasteHelper::CropAndPasteHelper(aclrtStream& stream,
                                       acldvppChannelDesc *dvppChannelDesc,
                                       uint32_t ltHorz, uint32_t ltVert,
                                       uint32_t rbHorz, uint32_t rbVert)
    : g_stream_(stream),
      g_vpcOutBufferDev_(nullptr),
      g_cropArea_(nullptr),
      g_pasteArea_(nullptr),
      g_vpcInputDesc_(nullptr),
      g_vpcOutputDesc_(nullptr),
      g_dvppChannelDesc_(dvppChannelDesc),
      g_vpcOutBufferSize_(0)
{
    // Change the left top coordinate to even numver
    g_ltHorz_ = (ltHorz >> 1) << 1;
    g_ltVert_ = (ltVert >> 1) << 1;
    // Change the right bottom coordinate to odd numver
    g_rbHorz_ = ((rbHorz >> 1) << 1) - 1;
    g_rbVert_ = ((rbVert >> 1) << 1) - 1;
    // odd
    g_size_.width = g_rbHorz_ - g_ltHorz_ + 1;
    g_size_.height = g_rbVert_ - g_ltVert_ + 1;
}

CropAndPasteHelper::CropAndPasteHelper(aclrtStream& stream,
                                       acldvppChannelDesc *dvppChannelDesc,
                                       uint32_t width, uint32_t height,
                                       uint32_t ltHorz, uint32_t ltVert,
                                       uint32_t rbHorz, uint32_t rbVert)
    : g_stream_(stream),
      g_vpcOutBufferDev_(nullptr),
      g_cropArea_(nullptr),
      g_pasteArea_(nullptr),
      g_vpcInputDesc_(nullptr),
      g_vpcOutputDesc_(nullptr),
      g_dvppChannelDesc_(dvppChannelDesc),
      g_vpcOutBufferSize_(0)
{
      // Change the left top coordinate to even numver
      g_ltHorz_ = (ltHorz >> 1) << 1;
      g_ltVert_ = (ltVert >> 1) << 1;
      // Change the right bottom coordinate to odd numver
      g_rbHorz_ = ((rbHorz >> 1) << 1) - 1;
      g_rbVert_ = ((rbVert >> 1) << 1) - 1;
      g_size_.width = width;
      g_size_.height = height;
}

CropAndPasteHelper::~CropAndPasteHelper()
{
    DestroyCropAndPasteResource();
}

AclLiteError CropAndPasteHelper::InitCropAndPasteInputDesc(ImageData& inputImage)
{
    g_originalImageWidth_ = inputImage.width;
    g_originalImageHeight_ = inputImage.height;
    uint32_t alignWidth = inputImage.alignWidth;
    uint32_t alignHeight = inputImage.alignHeight;
    if (alignWidth == 0 || alignHeight == 0) {
        ACLLITE_LOG_ERROR("Invalid image parameters, width %d, height %d",
                          inputImage.width, inputImage.height);
        return ACLLITE_ERROR;
    }

    uint32_t inputBufferSize = YUV420SP_SIZE(alignWidth, alignHeight);

    g_vpcInputDesc_ = acldvppCreatePicDesc();
    if (g_vpcInputDesc_ == nullptr) {
        ACLLITE_LOG_ERROR("Dvpp crop create pic desc failed");
        return ACLLITE_ERROR;
    }

    acldvppSetPicDescData(g_vpcInputDesc_, inputImage.data.get());
    acldvppSetPicDescFormat(g_vpcInputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(g_vpcInputDesc_, inputImage.width);
    acldvppSetPicDescHeight(g_vpcInputDesc_, inputImage.height);
    acldvppSetPicDescWidthStride(g_vpcInputDesc_, alignWidth);
    acldvppSetPicDescHeightStride(g_vpcInputDesc_, alignHeight);
    acldvppSetPicDescSize(g_vpcInputDesc_, inputBufferSize);

    return ACLLITE_OK;
}

AclLiteError CropAndPasteHelper::InitCropAndPasteOutputDesc()
{
    int cropOutWidth = g_size_.width;
    int cropOutHeight = g_size_.height;
    int cropOutWidthStride = ALIGN_UP16(cropOutWidth);
    int cropOutHeightStride = ALIGN_UP2(cropOutHeight);

    if (cropOutWidthStride == 0 || cropOutHeightStride == 0) {
        ACLLITE_LOG_ERROR("Crop image align widht(%d) and height(%d) failed",
                          g_size_.width, g_size_.height);
        return ACLLITE_ERROR;
    }

    g_vpcOutBufferSize_ = YUV420SP_SIZE(cropOutWidthStride,
                                        cropOutHeightStride);
    aclError aclRet = acldvppMalloc(&g_vpcOutBufferDev_, g_vpcOutBufferSize_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Dvpp crop malloc output memory failed, crop "
                          "width %d, height %d size %d, error %d",
                          g_size_.width, g_size_.height,
                          g_vpcOutBufferSize_, aclRet);
        return ACLLITE_ERROR;
    }
    aclrtMemset(g_vpcOutBufferDev_, g_vpcOutBufferSize_, 0, g_vpcOutBufferSize_);
    g_vpcOutputDesc_ = acldvppCreatePicDesc();
    if (g_vpcOutputDesc_ == nullptr) {
        ACLLITE_LOG_ERROR("Dvpp crop create pic desc failed");
        return ACLLITE_ERROR;
    }

    acldvppSetPicDescData(g_vpcOutputDesc_, g_vpcOutBufferDev_);
    acldvppSetPicDescFormat(g_vpcOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(g_vpcOutputDesc_, cropOutWidth);
    acldvppSetPicDescHeight(g_vpcOutputDesc_, cropOutHeight);
    acldvppSetPicDescWidthStride(g_vpcOutputDesc_, cropOutWidthStride);
    acldvppSetPicDescHeightStride(g_vpcOutputDesc_, cropOutHeightStride);
    acldvppSetPicDescSize(g_vpcOutputDesc_, g_vpcOutBufferSize_);

    return ACLLITE_OK;
}

AclLiteError CropAndPasteHelper::InitCropAndPasteResource(ImageData& inputImage)
{
    if (ACLLITE_OK != InitCropAndPasteInputDesc(inputImage)) {
        ACLLITE_LOG_ERROR("Dvpp crop init input failed");
        return ACLLITE_ERROR;
    }

    if (ACLLITE_OK != InitCropAndPasteOutputDesc()) {
        ACLLITE_LOG_ERROR("Dvpp crop init output failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError CropAndPasteHelper::Process(ImageData& cropedImage, ImageData& srcImage)
{
    if (ACLLITE_OK != InitCropAndPasteResource(srcImage)) {
        ACLLITE_LOG_ERROR("Dvpp cropandpaste failed for init error");
        return ACLLITE_ERROR;
    }

    // must even
    uint32_t cropLeftOffset = g_ltHorz_;
    // must even
    uint32_t cropTopOffset = g_ltVert_;
    // must odd
    uint32_t cropRightOffset = g_rbHorz_;
    // must odd
    uint32_t cropBottomOffset = g_rbVert_;
    
    g_cropArea_ = acldvppCreateRoiConfig(cropLeftOffset, cropRightOffset,
                                         cropTopOffset, cropBottomOffset);
    if (g_cropArea_ == nullptr) {
        ACLLITE_LOG_ERROR("acldvppCreateRoiConfig g_cropArea_ failed");
        return ACLLITE_ERROR;
    }

    // must even
    uint32_t pasteLeftOffset = 0;
    // must even
    uint32_t pasteTopOffset = 0;
    // must odd
    uint32_t pasteRightOffset = g_rbHorz_ - g_ltHorz_;
    // must odd
    uint32_t pasteBottomOffset = g_rbVert_ - g_ltVert_;

    g_pasteArea_ = acldvppCreateRoiConfig(pasteLeftOffset, pasteRightOffset,
                                          pasteTopOffset, pasteBottomOffset);
    if (g_pasteArea_ == nullptr) {
        ACLLITE_LOG_ERROR("acldvppCreateRoiConfig g_pasteArea_ failed");
        return ACLLITE_ERROR;
    }

    // crop and patse pic
    aclError aclRet = acldvppVpcCropAndPasteAsync(g_dvppChannelDesc_, g_vpcInputDesc_,
                                                  g_vpcOutputDesc_, g_cropArea_,
                                                  g_pasteArea_, g_stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acldvppVpcCropAndPasteAsync failed, aclRet = %d", aclRet);
        return ACLLITE_ERROR;
    }

    aclRet = aclrtSynchronizeStream(g_stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("crop and paste aclrtSynchronizeStream failed, aclRet = %d", aclRet);
        return ACLLITE_ERROR;
    }
    cropedImage.format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    cropedImage.width = g_size_.width;
    cropedImage.height = g_size_.height;
    cropedImage.alignWidth = ALIGN_UP16(g_size_.width);
    cropedImage.alignHeight = ALIGN_UP2(g_size_.height);
    cropedImage.size = g_vpcOutBufferSize_;
    cropedImage.data = SHARED_PTR_DVPP_BUF(g_vpcOutBufferDev_);

    DestroyCropAndPasteResource();

    return ACLLITE_OK;
}

AclLiteError CropAndPasteHelper::ProcessCropPaste(ImageData& cropedImage, ImageData& srcImage)
{
    if (ACLLITE_OK != InitCropAndPasteResource(srcImage)) {
        ACLLITE_LOG_ERROR("Dvpp cropandpaste failed for init error");
        return ACLLITE_ERROR;
    }

    // must even
    uint32_t cropLeftOffset = g_ltHorz_;
    // must even
    uint32_t cropTopOffset = g_ltVert_;
    // must odd
    uint32_t cropRightOffset = g_rbHorz_;
    // must odd
    uint32_t cropBottomOffset = g_rbVert_;

    g_cropArea_ = acldvppCreateRoiConfig(cropLeftOffset, cropRightOffset,
                                         cropTopOffset, cropBottomOffset);
    if (g_cropArea_ == nullptr) {
        ACLLITE_LOG_ERROR("acldvppCreateRoiConfig g_cropArea_ failed");
        return ACLLITE_ERROR;
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
        ACLLITE_LOG_ERROR("acldvppCreateRoiConfig g_pasteArea_ failed");
        return ACLLITE_ERROR;
    }

    // crop and patse pic
    aclError aclRet = acldvppVpcCropAndPasteAsync(g_dvppChannelDesc_, g_vpcInputDesc_,
                                                  g_vpcOutputDesc_, g_cropArea_,
                                                  g_pasteArea_, g_stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acldvppVpcCropAndPasteAsync failed, aclRet = %d", aclRet);
        return ACLLITE_ERROR;
    }

    aclRet = aclrtSynchronizeStream(g_stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("crop and paste aclrtSynchronizeStream failed, aclRet = %d", aclRet);
        return ACLLITE_ERROR;
    }
    cropedImage.format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    cropedImage.width = g_size_.width;
    cropedImage.height = g_size_.height;
    cropedImage.alignWidth = ALIGN_UP16(g_size_.width);
    cropedImage.alignHeight = ALIGN_UP2(g_size_.height);
    cropedImage.size = g_vpcOutBufferSize_;
    cropedImage.data = SHARED_PTR_DVPP_BUF(g_vpcOutBufferDev_);

    DestroyCropAndPasteResource();

    return ACLLITE_OK;
}

void CropAndPasteHelper::DestroyCropAndPasteResource()
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