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
    :stream_(stream), vpcOutBufferDev_(nullptr),
    cropArea_(nullptr), pasteArea_(nullptr),
    vpcInputDesc_(nullptr), vpcOutputDesc_(nullptr),
    dvppChannelDesc_(dvppChannelDesc), vpcOutBufferSize_(0)
{
    // Change the left top coordinate to even numver
    ltHorz_ = (ltHorz >> 1) << 1;
    ltVert_ = (ltVert >> 1) << 1;
    // Change the right bottom coordinate to odd numver
    rbHorz_ = ((rbHorz >> 1) << 1) - 1;
    rbVert_ = ((rbVert >> 1) << 1) - 1;

    size_.width = rbHorz_ - ltHorz_ + 1;
    size_.height = rbVert_ - ltVert_ + 1;
}

CropAndPasteHelper::CropAndPasteHelper(aclrtStream& stream,
    acldvppChannelDesc *dvppChannelDesc,
    uint32_t width, uint32_t height,
    uint32_t ltHorz, uint32_t ltVert,
    uint32_t rbHorz, uint32_t rbVert)
    :stream_(stream), vpcOutBufferDev_(nullptr),
    cropArea_(nullptr), pasteArea_(nullptr),
    vpcInputDesc_(nullptr), vpcOutputDesc_(nullptr),
    dvppChannelDesc_(dvppChannelDesc), vpcOutBufferSize_(0)
{
    // Change the left top coordinate to even numver
    ltHorz_ = (ltHorz >> 1) << 1;
    ltVert_ = (ltVert >> 1) << 1;
    // Change the right bottom coordinate to odd numver
    rbHorz_ = ((rbHorz >> 1) << 1) - 1;
    rbVert_ = ((rbVert >> 1) << 1) - 1;

    size_.width = width;
    size_.height = height;
}

CropAndPasteHelper::~CropAndPasteHelper()
{
    DestroyCropAndPasteResource();
}

AclLiteError CropAndPasteHelper::InitCropAndPasteInputDesc(ImageData& inputImage)
{
    originalImageWidth_ = inputImage.width;
    originalImageHeight_ = inputImage.height;
    uint32_t alignWidth = inputImage.alignWidth;
    uint32_t alignHeight = inputImage.alignHeight;
    if (alignWidth == 0 || alignHeight == 0) {
        ACLLITE_LOG_ERROR("Invalid image parameters, width %d, height %d",
                          inputImage.width, inputImage.height);
        return ACLLITE_ERROR;
    }

    uint32_t inputBufferSize = 0;
    if (inputImage.format == PIXEL_FORMAT_YUV_SEMIPLANAR_420) {
        inputBufferSize = YUV420SP_SIZE(alignWidth, alignHeight);
    } else if (inputImage.format == PIXEL_FORMAT_RGB_888) {
        inputBufferSize = RGBU8_IMAGE_SIZE(alignWidth, alignHeight);
    } else {
        ACLLITE_LOG_WARNING("Dvpp only support yuv and rgb format.");
    }

    vpcInputDesc_ = acldvppCreatePicDesc();
    if (vpcInputDesc_ == nullptr) {
        ACLLITE_LOG_ERROR("Dvpp crop create pic desc failed");
        return ACLLITE_ERROR;
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

AclLiteError CropAndPasteHelper::InitCropAndPasteOutputDesc()
{
    int cropOutWidth = size_.width;
    int cropOutHeight = size_.height;
    int cropOutWidthStride = ALIGN_UP16(cropOutWidth);
    int cropOutHeightStride = ALIGN_UP2(cropOutHeight);
    if (cropOutWidthStride == 0 || cropOutHeightStride == 0) {
        ACLLITE_LOG_ERROR("Crop image align widht(%d) and height(%d) failed",
                          size_.width, size_.height);
        return ACLLITE_ERROR;
    }

    vpcOutBufferSize_ = YUV420SP_SIZE(cropOutWidthStride,
                                      cropOutHeightStride);
    aclError aclRet = acldvppMalloc(&vpcOutBufferDev_, vpcOutBufferSize_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Dvpp crop malloc output memory failed, crop "
                          "width %d, height %d size %d, error %d",
                          size_.width, size_.height,
                          vpcOutBufferSize_, aclRet);
        return ACLLITE_ERROR;
    }
    aclrtMemset(vpcOutBufferDev_, vpcOutBufferSize_, 0, vpcOutBufferSize_);
    vpcOutputDesc_ = acldvppCreatePicDesc();
    if (vpcOutputDesc_ == nullptr) {
        ACLLITE_LOG_ERROR("Dvpp crop create pic desc failed");
        return ACLLITE_ERROR;
    }
    acldvppSetPicDescData(vpcOutputDesc_, vpcOutBufferDev_);
    acldvppSetPicDescFormat(vpcOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(vpcOutputDesc_, cropOutWidth);
    acldvppSetPicDescHeight(vpcOutputDesc_, cropOutHeight);
    acldvppSetPicDescWidthStride(vpcOutputDesc_, cropOutWidthStride);
    acldvppSetPicDescHeightStride(vpcOutputDesc_, cropOutHeightStride);
    acldvppSetPicDescSize(vpcOutputDesc_, vpcOutBufferSize_);

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
    uint32_t cropLeftOffset = ltHorz_;
    // must even
    uint32_t cropTopOffset = ltVert_;
    // must odd
    uint32_t cropRightOffset = rbHorz_;
    // must odd
    uint32_t cropBottomOffset = rbVert_;

    cropArea_ = acldvppCreateRoiConfig(cropLeftOffset, cropRightOffset,
                                       cropTopOffset, cropBottomOffset);
    if (cropArea_ == nullptr) {
        ACLLITE_LOG_ERROR("acldvppCreateRoiConfig cropArea_ failed");
        return ACLLITE_ERROR;
    }

    // must even
    uint32_t pasteLeftOffset = 0;
    // must even
    uint32_t pasteTopOffset = 0;
    // must odd
    uint32_t pasteRightOffset = rbHorz_ - ltHorz_;
    // must odd
    uint32_t pasteBottomOffset = rbVert_ - ltVert_;

    pasteArea_ = acldvppCreateRoiConfig(pasteLeftOffset, pasteRightOffset,
                                        pasteTopOffset, pasteBottomOffset);
    if (pasteArea_ == nullptr) {
        ACLLITE_LOG_ERROR("acldvppCreateRoiConfig pasteArea_ failed");
        return ACLLITE_ERROR;
    }

    // crop and patse pic
    aclError aclRet = acldvppVpcCropAndPasteAsync(dvppChannelDesc_, vpcInputDesc_,
                                                  vpcOutputDesc_, cropArea_,
                                                  pasteArea_, stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acldvppVpcCropAndPasteAsync failed, aclRet = %d", aclRet);
        return ACLLITE_ERROR;
    }

    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("crop and paste aclrtSynchronizeStream failed, aclRet = %d", aclRet);
        return ACLLITE_ERROR;
    }
    cropedImage.format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    cropedImage.width = size_.width;
    cropedImage.height = size_.height;
    cropedImage.alignWidth = ALIGN_UP16(size_.width);
    cropedImage.alignHeight = ALIGN_UP2(size_.height);
    cropedImage.size = vpcOutBufferSize_;
    cropedImage.data = SHARED_PTR_DVPP_BUF(vpcOutBufferDev_);

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
    uint32_t cropLeftOffset = ltHorz_;
    // must even
    uint32_t cropTopOffset = ltVert_;
    // must odd
    uint32_t cropRightOffset = rbHorz_;
    // must odd
    uint32_t cropBottomOffset = rbVert_;

    cropArea_ = acldvppCreateRoiConfig(cropLeftOffset, cropRightOffset,
                                       cropTopOffset, cropBottomOffset);
    if (cropArea_ == nullptr) {
        ACLLITE_LOG_ERROR("acldvppCreateRoiConfig cropArea_ failed");
        return ACLLITE_ERROR;
    }

    // must even
    uint32_t pasteLeftOffset = 0;
    // must even
    uint32_t pasteTopOffset = 0;
    // must odd
    uint32_t pasteRightOffset = (((pasteLeftOffset + size_.width) >> 1) << 1) -1;
    // must odd
    uint32_t pasteBottomOffset = (((pasteTopOffset + size_.height) >> 1) << 1) -1;

    pasteArea_ = acldvppCreateRoiConfig(pasteLeftOffset, pasteRightOffset,
                                        pasteTopOffset, pasteBottomOffset);
    if (pasteArea_ == nullptr) {
        ACLLITE_LOG_ERROR("acldvppCreateRoiConfig pasteArea_ failed");
        return ACLLITE_ERROR;
    }

    // crop and patse pic
    aclError aclRet = acldvppVpcCropAndPasteAsync(dvppChannelDesc_, vpcInputDesc_,
                                                  vpcOutputDesc_, cropArea_,
                                                  pasteArea_, stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acldvppVpcCropAndPasteAsync failed, aclRet = %d", aclRet);
        return ACLLITE_ERROR;
    }

    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("crop and paste aclrtSynchronizeStream failed, aclRet = %d", aclRet);
        return ACLLITE_ERROR;
    }
    cropedImage.format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    cropedImage.width = size_.width;
    cropedImage.height = size_.height;
    cropedImage.alignWidth = ALIGN_UP16(size_.width);
    cropedImage.alignHeight = ALIGN_UP2(size_.height);
    cropedImage.size = vpcOutBufferSize_;
    cropedImage.data = SHARED_PTR_DVPP_BUF(vpcOutBufferDev_);

    DestroyCropAndPasteResource();

    return ACLLITE_OK;
}

AclLiteError CropAndPasteHelper::ProportionProcess(ImageData& resizedImage, ImageData& srcImage)
{
    if (ACLLITE_OK != InitCropAndPasteResource(srcImage)) {
        ACLLITE_LOG_ERROR("Dvpp cropandpaste failed for init error");
        return ACLLITE_ERROR;
    }

    // must even
    uint32_t cropLeftOffset = 0;
    // must even
    uint32_t cropTopOffset = 0;
    // must odd
    uint32_t cropRightOffset = (((cropLeftOffset + originalImageWidth_) >> 1) << 1) -1;
    // must odd
    uint32_t cropBottomOffset = (((cropTopOffset + originalImageHeight_) >> 1) << 1) -1;
    // data created to describe area location
    cropArea_ = acldvppCreateRoiConfig(cropLeftOffset, cropRightOffset,
                                       cropTopOffset, cropBottomOffset);
    if (cropArea_ == nullptr) {
        ACLLITE_LOG_ERROR("acldvppCreateRoiConfig cropArea_ failed");
        return ACLLITE_ERROR;
    }
    
    uint32_t pasteWidth = rbHorz_ - ltHorz_;
    uint32_t pasteHeight = rbVert_ - ltVert_;
    // set crop area:
    float rx = (float)originalImageWidth_ / (float)pasteWidth;
    float ry = (float)originalImageHeight_ / (float)pasteHeight;

    int dx = 0;
    int dy = 0;
    float r = 0.0f;
    if (rx > ry) {
        dx = 0;
        r = rx;
        dy = (pasteHeight - originalImageHeight_ / r) / 2;
    } else {
        dy = 0;
        r = ry;
        dx = (pasteWidth - originalImageWidth_ / r) / 2;
    }

    // must even
    uint32_t pasteLeftOffset = 0;
    // must even
    uint32_t pasteTopOffset = 0;
    // must odd
    uint32_t pasteRightOffset = pasteWidth - 2 * dx;
    // must odd
    uint32_t pasteBottomOffset = pasteHeight -  2 * dy;

    pasteArea_ = acldvppCreateRoiConfig(pasteLeftOffset, pasteRightOffset,
                                        pasteTopOffset, pasteBottomOffset);
    if (pasteArea_ == nullptr) {
        ACLLITE_LOG_ERROR("acldvppCreateRoiConfig pasteArea_ failed");
        return ACLLITE_ERROR;
    }

    // crop and patse pic
    aclError aclRet = acldvppVpcCropAndPasteAsync(dvppChannelDesc_, vpcInputDesc_,
                                                  vpcOutputDesc_, cropArea_,
                                                  pasteArea_, stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acldvppVpcCropAndPasteAsync failed, aclRet = %d", aclRet);
        return ACLLITE_ERROR;
    }
    // END
    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("crop and paste aclrtSynchronizeStream failed, aclRet = %d", aclRet);
        return ACLLITE_ERROR;
    }

    resizedImage.width = pasteWidth - 2 * dx;
    resizedImage.height = pasteHeight -  2 * dy;
    resizedImage.alignWidth = ALIGN_UP16(pasteWidth);
    resizedImage.alignHeight = ALIGN_UP2(pasteHeight);
    resizedImage.size = vpcOutBufferSize_;
    resizedImage.data = SHARED_PTR_DVPP_BUF(vpcOutBufferDev_);

    DestroyCropAndPasteResource();

    return ACLLITE_OK;
}

AclLiteError CropAndPasteHelper::ProportionCenterProcess(ImageData& resizedImage, ImageData& srcImage)
{
    if (ACLLITE_OK != InitCropAndPasteResource(srcImage)) {
        ACLLITE_LOG_ERROR("Dvpp cropandpaste failed for init error");
        return ACLLITE_ERROR;
    }

    // must even
    uint32_t cropLeftOffset = 0;
    // must even
    uint32_t cropTopOffset = 0;
    // must odd
    uint32_t cropRightOffset = (((cropLeftOffset + originalImageWidth_) >> 1) << 1) -1;
    // must odd
    uint32_t cropBottomOffset = (((cropTopOffset + originalImageHeight_) >> 1) << 1) -1;
    // data created to describe area location
    cropArea_ = acldvppCreateRoiConfig(cropLeftOffset, cropRightOffset,
                                       cropTopOffset, cropBottomOffset);
    if (cropArea_ == nullptr) {
        ACLLITE_LOG_ERROR("acldvppCreateRoiConfig cropArea_ failed");
        return ACLLITE_ERROR;
    }

    uint32_t pasteWidth = rbHorz_ - ltHorz_;
    uint32_t pasteHeight = rbVert_ - ltVert_;
    // set crop area:
    float rx = (float)pasteWidth / (float)originalImageWidth_;
    float ry = (float)pasteHeight / (float)originalImageHeight_;
    int dx = 0;
    int dy = 0;
    float r = 0.0f;
    if (rx > ry) {
        r = ry;
        dx = (pasteWidth - originalImageWidth_ * r) / 2;
        dy = (pasteHeight - originalImageHeight_ * r) / 2;
    } else {
        r = rx;
        dx = (pasteWidth - originalImageWidth_ * r) / 2;
        dy = (pasteHeight - originalImageHeight_ * r) / 2;
    }

    dx = (dx >> 1) << 1;
    dy = (dy >> 1) << 1;

    // must even
    uint32_t pasteLeftOffset = ALIGN_UP16(dx);
    // must even
    uint32_t pasteTopOffset = dy;
    // must odd
    uint32_t pasteRightOffset = pasteWidth - dx + pasteLeftOffset - dx;
    // must odd
    uint32_t pasteBottomOffset = pasteHeight - dy;

    pasteArea_ = acldvppCreateRoiConfig(pasteLeftOffset, pasteRightOffset,
                                        pasteTopOffset, pasteBottomOffset);
    if (pasteArea_ == nullptr) {
        ACLLITE_LOG_ERROR("acldvppCreateRoiConfig pasteArea_ failed");
        return ACLLITE_ERROR;
    }

    // crop and patse pic
    aclError aclRet = acldvppVpcCropAndPasteAsync(dvppChannelDesc_, vpcInputDesc_,
                                                  vpcOutputDesc_, cropArea_,
                                                  pasteArea_, stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acldvppVpcCropAndPasteAsync failed, aclRet = %d", aclRet);
        return ACLLITE_ERROR;
    }
    // END
    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("crop and paste aclrtSynchronizeStream failed, aclRet = %d", aclRet);
        return ACLLITE_ERROR;
    }

    resizedImage.width = pasteWidth - 2 * dx;
    resizedImage.height = pasteHeight -  2 * dy;
    resizedImage.alignWidth = ALIGN_UP16(pasteWidth);
    resizedImage.alignHeight = ALIGN_UP2(pasteHeight);
    resizedImage.size = vpcOutBufferSize_;
    resizedImage.data = SHARED_PTR_DVPP_BUF(vpcOutBufferDev_);

    DestroyCropAndPasteResource();

    return ACLLITE_OK;
}

void CropAndPasteHelper::DestroyCropAndPasteResource()
{
    if (cropArea_ != nullptr) {
        (void)acldvppDestroyRoiConfig(cropArea_);
        cropArea_ = nullptr;
    }

    if (pasteArea_ != nullptr) {
        (void)acldvppDestroyRoiConfig(pasteArea_);
        pasteArea_ = nullptr;
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