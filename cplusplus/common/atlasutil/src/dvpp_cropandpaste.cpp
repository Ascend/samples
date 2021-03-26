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
#include "dvpp_cropandpaste.h"

using namespace std;

DvppCropAndPaste::DvppCropAndPaste(aclrtStream& stream, 
                                   acldvppChannelDesc *dvppChannelDesc,
                                   uint32_t ltHorz, uint32_t ltVert,
                                   uint32_t rbHorz, uint32_t rbVert)
: stream_(stream), dvppChannelDesc_(dvppChannelDesc), vpcInputDesc_(nullptr), 
vpcOutputDesc_(nullptr), vpcOutBufferDev_(nullptr),vpcOutBufferSize_(0), 
cropArea_(nullptr), pasteArea_(nullptr){
    // Change the left top coordinate to even numver
    ltHorz_ = (ltHorz >> 1) << 1;
    ltVert_ = (ltVert >> 1) << 1;

    // Change the left top coordinate to odd numver
    rbHorz_ = ((rbHorz >> 1) << 1) - 1;
    rbVert_ = ((rbVert >> 1) << 1) - 1;

    size_.width = rbHorz_ - ltHorz_;
    size_.height = rbVert_ - ltVert_;
}

DvppCropAndPaste::~DvppCropAndPaste() {
    DestroyCropAndPasteResource();
}

AtlasError DvppCropAndPaste::InitCropAndPasteInputDesc(ImageData& inputImage) {
    uint32_t alignWidth = ALIGN_UP16(inputImage.width);
    uint32_t alignHeight = ALIGN_UP2(inputImage.height);
    if (alignWidth == 0 || alignHeight == 0) {
        ATLAS_LOG_ERROR("Invalid image parameters, width %d, height %d",
                        inputImage.width, inputImage.height);
        return ATLAS_ERROR;
    }

    uint32_t inputBufferSize = YUV420SP_SIZE(alignWidth, alignHeight);
    vpcInputDesc_ = acldvppCreatePicDesc();
    if (vpcInputDesc_ == nullptr) {
        ATLAS_LOG_ERROR("Dvpp crop create pic desc failed");
        return ATLAS_ERROR;
    }

    acldvppSetPicDescData(vpcInputDesc_, inputImage.data.get());
    acldvppSetPicDescFormat(vpcInputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(vpcInputDesc_, inputImage.width);
    acldvppSetPicDescHeight(vpcInputDesc_, inputImage.height);
    acldvppSetPicDescWidthStride(vpcInputDesc_, alignWidth);
    acldvppSetPicDescHeightStride(vpcInputDesc_, alignHeight);
    acldvppSetPicDescSize(vpcInputDesc_, inputBufferSize);

    return ATLAS_OK;
}

AtlasError DvppCropAndPaste::InitCropAndPasteOutputDesc()
{
    int cropOutWidth = size_.width;
    int cropOutHeight = size_.height;
    int cropOutWidthStride = ALIGN_UP16(cropOutWidth);
    int cropOutHeightStride = ALIGN_UP2(cropOutHeight);

    if (cropOutWidthStride == 0 || cropOutHeightStride == 0) {
        ATLAS_LOG_ERROR("Crop image align widht(%d) and height(%d) failed",
                        size_.width, size_.height);
        return ATLAS_ERROR;
    }

    vpcOutBufferSize_ = YUV420SP_SIZE(cropOutWidthStride, 
                                      cropOutHeightStride);
    aclError aclRet = acldvppMalloc(&vpcOutBufferDev_, vpcOutBufferSize_);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Dvpp crop malloc output memory failed, crop "
                        "width %d, height %d size %d, error %d", 
                        size_.width, size_.height, 
                        vpcOutBufferSize_, aclRet);
        return ATLAS_ERROR;
    }

    vpcOutputDesc_ = acldvppCreatePicDesc();
    if (vpcOutputDesc_ == nullptr) {
        ATLAS_LOG_ERROR("Dvpp crop create pic desc failed");
        return ATLAS_ERROR;
    }
    acldvppSetPicDescData(vpcOutputDesc_, vpcOutBufferDev_);
    acldvppSetPicDescFormat(vpcOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescWidth(vpcOutputDesc_, cropOutWidth);
    acldvppSetPicDescHeight(vpcOutputDesc_, cropOutHeight);
    acldvppSetPicDescWidthStride(vpcOutputDesc_, cropOutWidthStride);
    acldvppSetPicDescHeightStride(vpcOutputDesc_, cropOutHeightStride);
    acldvppSetPicDescSize(vpcOutputDesc_, vpcOutBufferSize_);

    return ATLAS_OK;
}

AtlasError DvppCropAndPaste::InitCropAndPasteResource(ImageData& inputImage) {
    if (ATLAS_OK != InitCropAndPasteInputDesc(inputImage)) {
        ATLAS_LOG_ERROR("Dvpp crop init input failed");
        return ATLAS_ERROR;
    }

    if (ATLAS_OK != InitCropAndPasteOutputDesc()) {
        ATLAS_LOG_ERROR("Dvpp crop init output failed");
        return ATLAS_ERROR;
    }

    return ATLAS_OK;
}

AtlasError DvppCropAndPaste::Process(ImageData& cropedImage, ImageData& srcImage)
{
    if (ATLAS_OK != InitCropAndPasteResource(srcImage)) {
        ATLAS_LOG_ERROR("Dvpp cropandpaste failed for init error");
        return ATLAS_ERROR;
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
        ATLAS_LOG_ERROR("acldvppCreateRoiConfig cropArea_ failed");
        return ATLAS_ERROR;
    }

    // must even
    uint32_t pasteLeftOffset = 0;
    // must even
    uint32_t pasteTopOffset = 0;
    // must odd
    uint32_t pasteRightOffset = size_.width;
    // must odd
    uint32_t pasteBottomOffset = size_.height;

    pasteArea_ = acldvppCreateRoiConfig(pasteLeftOffset, pasteRightOffset,
    pasteTopOffset, pasteBottomOffset);
    if (pasteArea_ == nullptr) {
        ATLAS_LOG_ERROR("acldvppCreateRoiConfig pasteArea_ failed");
        return ATLAS_ERROR;
    }

    // crop and patse pic
    //TODO:
    aclError aclRet = acldvppVpcCropAndPasteAsync(dvppChannelDesc_, vpcInputDesc_,
    vpcOutputDesc_, cropArea_, pasteArea_, stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("acldvppVpcCropAndPasteAsync failed, aclRet = %d", aclRet);
        return ATLAS_ERROR;
    }

    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("crop and paste aclrtSynchronizeStream failed, aclRet = %d", aclRet);
        return ATLAS_ERROR;
    }
    cropedImage.format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    cropedImage.width = ALIGN_UP16(size_.width);
    cropedImage.height = ALIGN_UP2(size_.height);
    cropedImage.alignWidth = cropedImage.width;
    cropedImage.alignHeight = cropedImage.height;
    cropedImage.size = vpcOutBufferSize_;
    cropedImage.data = SHARED_PRT_DVPP_BUF(vpcOutBufferDev_);

    DestroyCropAndPasteResource();

    return ATLAS_OK;
}

void DvppCropAndPaste::DestroyCropAndPasteResource()
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
