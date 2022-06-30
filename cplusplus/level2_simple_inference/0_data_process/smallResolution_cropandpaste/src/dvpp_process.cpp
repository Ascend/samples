/**
* @file dvpp_process.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include "dvpp_process.h"
#include <string.h>

using namespace std;

namespace {
    const uint32_t YUV422_PACKED_STRIDE_COEFFICIENT = 2;
    const uint32_t YUV444_PACKED_STRIDE_COEFFICIENT = 3;
    const uint32_t XRGB_STRIDE_COEFFICIENT = 4;
}

DvppProcess::DvppProcess(aclrtStream &stream)
    : stream_(stream), dvppChannelDesc_(nullptr),inFileName_(""), outFileName_(""), inWidth_(0), inHeight_(0),
      cropLeftOffset_(0), cropRightOffset_(0), cropTopOffset_(0), cropBottomOffset_(0), outWidth_(0), outHeight_(0),
      pasteLeftOffset_(0), pasteRightOffset_(0), pasteTopOffset_(0), pasteBottomOffset_(0), inWidthStride_(0),
      inHeightStride_(0), inFormat_(PIXEL_FORMAT_YUV_SEMIPLANAR_420), outFormat_(PIXEL_FORMAT_YUV_SEMIPLANAR_420),
      vpcInBufferDev_(nullptr), vpcOutBufferDev_(nullptr), vpcOutBufferSize_(0),vpcInputDesc_(nullptr),
      vpcOutputDesc_(nullptr), cropArea_(nullptr), pasteArea_(nullptr), widthCoeffi_(1), heightCoeffi_(1),
      wExtendDirection_(TO_LEFT), hExtendDirection_(TO_BOTTOM), vpcOutBufferDevFirst_(nullptr),
      vpcOutputDescFirst_(nullptr)
{
}

DvppProcess::~DvppProcess()
{
    DestroyResource();
    DestroyCropAndPasteResource();
}

uint32_t AlignSize(uint32_t origSize, uint32_t alignment)
{
    if (alignment == 0) {
        return 0;
    }
    uint32_t alignmentH = alignment - 1;
    return (origSize + alignmentH) / alignment * alignment;
}

Result DvppProcess::InitResource()
{
    dvppChannelDesc_ = acldvppCreateChannelDesc();
    if (dvppChannelDesc_ == nullptr) {
        ERROR_LOG("acldvppCreateChannelDesc failed");
        return FAILED;
    }

    aclError aclRet = acldvppCreateChannel(dvppChannelDesc_);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("acldvppCreateChannel failed, errorCode = %d", static_cast<int32_t>(aclRet));
        return FAILED;
    }

    INFO_LOG("dvpp init resource success");
    return SUCCESS;
}

void DvppProcess::DestroyResource()
{
    if (dvppChannelDesc_ != nullptr) {
        aclError aclRet = acldvppDestroyChannel(dvppChannelDesc_);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("acldvppDestroyChannel failed, errorCode = %d", static_cast<int32_t>(aclRet));
        }

        (void)acldvppDestroyChannelDesc(dvppChannelDesc_);
        dvppChannelDesc_ = nullptr;
    }
}

void DvppProcess::SetDvppInputPara(uint32_t inputWidth, uint32_t inputHeight,
    uint32_t inputFormat, std::string inFileName)
{
    inWidth_ = inputWidth;
    inHeight_ = inputHeight;
    inFormat_ = static_cast<acldvppPixelFormat>(inputFormat);
    inFileName_ = inFileName;
}

void DvppProcess::SetDvppCropPara(uint32_t cropLeftOffset, uint32_t cropRightOffset,
    uint32_t cropTopOffset, uint32_t cropBottomOffset)
{
    cropLeftOffset_ = cropLeftOffset;
    cropRightOffset_ = cropRightOffset;
    cropTopOffset_ = cropTopOffset;
    cropBottomOffset_ = cropBottomOffset;
}

void DvppProcess::SetDvppOutputPara(uint32_t outputWidth, uint32_t outputHeight,
    uint32_t outputFormat, std::string outFileName)
{
    outWidth_ = outputWidth;
    outHeight_ = outputHeight;
    outFormat_ = static_cast<acldvppPixelFormat>(outputFormat);
    outFileName_ = outFileName;
}

void DvppProcess::SetDvppPastePara(uint32_t pasteLeftOffset, uint32_t pasteRightOffset,
    uint32_t pasteTopOffset, uint32_t pasteBottomOffset)
{
    pasteLeftOffset_ = pasteLeftOffset;
    pasteRightOffset_ = pasteRightOffset;
    pasteTopOffset_ = pasteTopOffset;
    pasteBottomOffset_ = pasteBottomOffset;
}

Result DvppProcess::CheckParameter()
{
    uint32_t cropWidth = cropRightOffset_ - cropLeftOffset_ + 1;
    uint32_t cropHeight = cropBottomOffset_ - cropTopOffset_ + 1;
    uint32_t pasteWidth = pasteRightOffset_ - pasteLeftOffset_ + 1;
    uint32_t pasteHeight = pasteBottomOffset_ - pasteTopOffset_ + 1;
    if ((cropWidth < 4) || (cropHeight < 4)) {
        ERROR_LOG("mini crop area is [4x4], but receive [%ux%u]", cropWidth, cropHeight);
        return FAILED;
    }
    if ((pasteWidth < 10) || (pasteHeight < 6)) {
        ERROR_LOG("mini paste area is [10x6], but receive [%ux%u]", pasteWidth, pasteHeight);
        return FAILED;
    }
    if ((pasteWidth * 1024 < cropWidth) || (pasteWidth > cropWidth * 256)) {
        ERROR_LOG("resize range is [1/1024, 256], but cropWidth = %u, pasteWidth = %u", cropWidth, pasteWidth);
        return FAILED;
    }
    if ((pasteHeight * 1024 < cropHeight) || (pasteHeight > cropHeight * 256)) {
        ERROR_LOG("resize range is [1/1024, 256], but cropHeight = %u, pasteHeight = %u", cropHeight, pasteHeight);
        return FAILED;
    }
    return SUCCESS;
}

uint32_t DvppProcess::GetInputWidthStride()
{
    uint32_t inWidthStride = 0;
    switch (inFormat_) {
        case PIXEL_FORMAT_YUV_400:
        case PIXEL_FORMAT_YUV_SEMIPLANAR_420:
        case PIXEL_FORMAT_YVU_SEMIPLANAR_420:
        case PIXEL_FORMAT_YUV_SEMIPLANAR_422:
        case PIXEL_FORMAT_YVU_SEMIPLANAR_422:
        case PIXEL_FORMAT_YUV_SEMIPLANAR_440:
        case PIXEL_FORMAT_YVU_SEMIPLANAR_440:
        case PIXEL_FORMAT_YUV_SEMIPLANAR_444:
        case PIXEL_FORMAT_YVU_SEMIPLANAR_444:
            inWidthStride = AlignSize(inWidth_, 16);
            break;
        case PIXEL_FORMAT_YUYV_PACKED_422:
        case PIXEL_FORMAT_UYVY_PACKED_422:
        case PIXEL_FORMAT_YVYU_PACKED_422:
        case PIXEL_FORMAT_VYUY_PACKED_422:
            inWidthStride = AlignSize(inWidth_, 16) * YUV422_PACKED_STRIDE_COEFFICIENT;
            break;
        case PIXEL_FORMAT_YUV_PACKED_444:
        case PIXEL_FORMAT_RGB_888:
        case PIXEL_FORMAT_BGR_888:
            inWidthStride = AlignSize(inWidth_, 16) * YUV444_PACKED_STRIDE_COEFFICIENT;
            break;
        case PIXEL_FORMAT_ARGB_8888:
        case PIXEL_FORMAT_ABGR_8888:
        case PIXEL_FORMAT_RGBA_8888:
        case PIXEL_FORMAT_BGRA_8888:
            inWidthStride = AlignSize(inWidth_, 16) * XRGB_STRIDE_COEFFICIENT;
            break;
        default:
            INFO_LOG("inputFormat = %d not support, get inWidthStride failed", static_cast<int32_t>(inFormat_));
            break;
    }

    return inWidthStride;
}

void DvppProcess::CalYuv400InputBufferSize(uint32_t inWidthStride, uint32_t inHeightStride, uint32_t &inBufferSize)
{
    auto socVersion = aclrtGetSocName();
    INFO_LOG("Current soc version is %s", socVersion);
    if (strncmp(socVersion, "Ascend310P3", sizeof("Ascend310P3") - 1) == 0) {
        inBufferSize = inWidthStride * inHeightStride;
    } else {
        inBufferSize = inWidthStride * inHeightStride * 3 / 2;
    }
}

uint32_t DvppProcess::GetInputBufferSize(uint32_t inWidthStride, uint32_t inHeightStride)
{
    uint32_t inBufferSize = 0;
    switch (inFormat_) {
        case PIXEL_FORMAT_YUV_400: {
            CalYuv400InputBufferSize(inWidthStride, inHeightStride, inBufferSize);
            break;
        }
        case PIXEL_FORMAT_YUV_SEMIPLANAR_420:
        case PIXEL_FORMAT_YVU_SEMIPLANAR_420:
            inBufferSize = inWidthStride * inHeightStride * 3 / 2;
            break;
        case PIXEL_FORMAT_YUV_SEMIPLANAR_422:
        case PIXEL_FORMAT_YVU_SEMIPLANAR_422:
        case PIXEL_FORMAT_YUV_SEMIPLANAR_440:
        case PIXEL_FORMAT_YVU_SEMIPLANAR_440:
            inBufferSize = inWidthStride * inHeightStride * 2;
            break;
        case PIXEL_FORMAT_YUV_SEMIPLANAR_444:
        case PIXEL_FORMAT_YVU_SEMIPLANAR_444:
            inBufferSize = inWidthStride * inHeightStride * 3;
            break;
        case PIXEL_FORMAT_YUYV_PACKED_422:
        case PIXEL_FORMAT_UYVY_PACKED_422:
        case PIXEL_FORMAT_YVYU_PACKED_422:
        case PIXEL_FORMAT_VYUY_PACKED_422:
        case PIXEL_FORMAT_YUV_PACKED_444:
        case PIXEL_FORMAT_RGB_888:
        case PIXEL_FORMAT_BGR_888:
        case PIXEL_FORMAT_ARGB_8888:
        case PIXEL_FORMAT_ABGR_8888:
        case PIXEL_FORMAT_RGBA_8888:
        case PIXEL_FORMAT_BGRA_8888:
            inBufferSize = inWidthStride * inHeightStride;
            break;
        default:
            INFO_LOG("inputFormat = %d not support, get inBufferSize failed", static_cast<int32_t>(inFormat_));
            break;
    }

    return inBufferSize;
}


Result DvppProcess::InitCropAndPasteInputDesc()
{
    uint32_t inWidthStride = GetInputWidthStride();
    if (inWidthStride == 0) {
        return FAILED;
    }
    // inWidthStride must be larger than or equal to 32
    if (inWidthStride < 32) {
        inWidthStride = 32;
    }
    uint32_t inHeightStride = AlignSize(inHeight_, 2); // 2-byte alignment
    uint32_t inBufferSize = GetInputBufferSize(inWidthStride, inHeightStride);
    if (inBufferSize == 0) {
        return FAILED;
    }

    vpcInputDesc_ = acldvppCreatePicDesc();
    if (vpcInputDesc_ == nullptr) {
        ERROR_LOG("create input picture description failed");
        return FAILED;
    }

    PictureDesc picDesc;
    picDesc.fileName = inFileName_;
    picDesc.bufferSize = inBufferSize;
    picDesc.format = inFormat_;
    picDesc.width = inWidth_;
    picDesc.height = inHeight_;
    picDesc.widthStride = inWidthStride;
    picDesc.heightStride = inHeightStride;
    vpcInBufferDev_ = Utils::GetPicDevBuffer(picDesc);
    if (vpcInBufferDev_ == nullptr) {
        ERROR_LOG("get input picture buffer failed, file name = %s, inBufferSize = %u",
            inFileName_.c_str(), inBufferSize);
        return FAILED;
    }

    (void)acldvppSetPicDescData(vpcInputDesc_, vpcInBufferDev_);
    (void)acldvppSetPicDescFormat(vpcInputDesc_, inFormat_);
    (void)acldvppSetPicDescWidth(vpcInputDesc_, inWidth_);
    (void)acldvppSetPicDescHeight(vpcInputDesc_, inHeight_);
    (void)acldvppSetPicDescWidthStride(vpcInputDesc_, inWidthStride);
    (void)acldvppSetPicDescHeightStride(vpcInputDesc_, inHeightStride);
    (void)acldvppSetPicDescSize(vpcInputDesc_, inBufferSize);

    inWidthStride_ = (AlignSize(inWidth_, 16) < 32) ? 32 : AlignSize(inWidth_, 16);
    inHeightStride_ = inHeightStride;

    return SUCCESS;
}

Result DvppProcess::InitCropAndPasteOutputDesc()
{
    uint32_t vpcOutWidthStride = AlignSize(outWidth_, 16); // 16-byte alignment
    // vpcOutWidthStride must be larger than or equal to 32
    if (vpcOutWidthStride < 32) {
        vpcOutWidthStride = 32;
    }
    uint32_t vpcOutHeightStride = AlignSize(outHeight_, 2); // 2-byte alignment
    uint32_t sizeAlignment = 3;
    uint32_t sizeNum = 2;
    vpcOutBufferSize_ = vpcOutWidthStride * vpcOutHeightStride * sizeAlignment / sizeNum;
    aclError aclRet = acldvppMalloc(&vpcOutBufferDev_, vpcOutBufferSize_);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("malloc output image buffer failed, vpcOutBufferSize = %u, errorCode = %d",
            vpcOutBufferSize_, static_cast<int32_t>(aclRet));
        return FAILED;
    }
    vpcOutputDesc_ = acldvppCreatePicDesc();
    if (vpcOutputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc vpcOutputDesc_ failed");
        return FAILED;
    }

    (void)acldvppSetPicDescData(vpcOutputDesc_, vpcOutBufferDev_);
    (void)acldvppSetPicDescFormat(vpcOutputDesc_, outFormat_);
    (void)acldvppSetPicDescWidth(vpcOutputDesc_, outWidth_);
    (void)acldvppSetPicDescHeight(vpcOutputDesc_, outHeight_);
    (void)acldvppSetPicDescWidthStride(vpcOutputDesc_, vpcOutWidthStride);
    (void)acldvppSetPicDescHeightStride(vpcOutputDesc_, vpcOutHeightStride);
    (void)acldvppSetPicDescSize(vpcOutputDesc_, vpcOutBufferSize_);

    return SUCCESS;
}

bool DvppProcess::IsNeedSplit()
{
    CropAndPaste cropAndPasteInfo;
    cropAndPasteInfo.cropWidth = cropRightOffset_ - cropLeftOffset_ + 1;
    cropAndPasteInfo.cropHeight = cropBottomOffset_ - cropTopOffset_ + 1;
    cropAndPasteInfo.pasteWidth = pasteRightOffset_ - pasteLeftOffset_ + 1;
    cropAndPasteInfo.pasteHeight = pasteBottomOffset_ - pasteTopOffset_ + 1;

    // paste width need be in range [1/32, 16] * cropWidth
    if ((cropAndPasteInfo.pasteWidth > cropAndPasteInfo.cropWidth * 16) ||
        (cropAndPasteInfo.cropWidth > cropAndPasteInfo.pasteWidth * 32)) {
        return true;
    }

    // paste height need be in range [1/32, 16] * cropHeight
    if ((cropAndPasteInfo.pasteHeight > cropAndPasteInfo.cropHeight * 16) ||
        (cropAndPasteInfo.cropHeight > cropAndPasteInfo.pasteHeight * 32)) {
        return true;
    }

    // if cropArea smaller than [10 x 6], vpc step by step required
    if ((cropAndPasteInfo.cropWidth < 10) || (cropAndPasteInfo.cropHeight < 6)) {
        return true;
    }

    return false;
}

void DvppProcess::GetCropWidth(CropAndPaste &cropAndPasteInfo, uint32_t cropWidth, uint32_t pasteWidth)
{
    if (widthCoeffi_ == 1) { // cropArea bigger than 10x6
        if (pasteWidth > cropWidth * 16) {
            // align to 32, make sure the value is an even number after dividing by 16
            cropAndPasteInfo.cropWidth = AlignSize(pasteWidth, 32) / 16;
        } else if (pasteWidth * 32 < cropWidth) {
            // align to 64, make sure the value is an even number after dividing by 32
            cropAndPasteInfo.cropWidth = AlignSize(cropWidth, 64) / 32;
        } else {
            cropAndPasteInfo.cropWidth = cropWidth;
        }
    } else {
        if (pasteWidth > cropWidth * 16) {
            // cropWidth is even, alignment is not need
            cropAndPasteInfo.cropWidth = cropWidth * 16;
        } else if (pasteWidth * 32 < cropWidth) {
            // align to 64, make sure the value is an even number after dividing by 32
            cropAndPasteInfo.cropWidth = AlignSize(cropWidth, 64) / 32;
        } else {
            cropAndPasteInfo.cropWidth = pasteWidth;
        }
    }
    // max crop width is 4096
    cropAndPasteInfo.cropWidth =
        (cropAndPasteInfo.cropWidth > 4096) ? 4096 : cropAndPasteInfo.cropWidth;
}

void DvppProcess::GetCropHeight(CropAndPaste &cropAndPasteInfo, uint32_t cropHeight, uint32_t pasteHeight)
{
    if (heightCoeffi_ == 1) { // cropArea bigger than 10x6
        if (pasteHeight > cropHeight * 16) {
            // align to 32, make sure the value is an even number after dividing by 16
            cropAndPasteInfo.cropHeight = AlignSize(pasteHeight, 32) / 16;
        } else if (pasteHeight * 32 < cropHeight) {
            // align to 64, make sure the value is an even number after dividing by 32
            cropAndPasteInfo.cropHeight = AlignSize(cropHeight, 64) / 32;
        } else {
            cropAndPasteInfo.cropHeight = cropHeight;
        }
    } else {
        if (pasteHeight > cropHeight * 16) {
            // cropHeight is even, alignment is not need
            cropAndPasteInfo.cropHeight = cropHeight * 16;
        } else if (pasteHeight * 32 < cropHeight) {
            // align to 64, make sure the value is an even number after dividing by 32
            cropAndPasteInfo.cropHeight = AlignSize(cropHeight, 64) / 32;
        } else {
            cropAndPasteInfo.cropHeight = pasteHeight;
        }
    }
    // max crop height is 4096
    cropAndPasteInfo.cropHeight =
        (cropAndPasteInfo.cropHeight > 4096) ? 4096 : cropAndPasteInfo.cropHeight;
}

void DvppProcess::GetCropAndPasteInfo(CropAndPaste &cropAndPasteFirstInfo, CropAndPaste &cropAndPasteSecondInfo)
{
    GetCropWidth(cropAndPasteSecondInfo, cropAndPasteFirstInfo.cropWidth, cropAndPasteFirstInfo.pasteWidth);
    GetCropHeight(cropAndPasteSecondInfo, cropAndPasteFirstInfo.cropHeight, cropAndPasteFirstInfo.pasteHeight);

    cropAndPasteSecondInfo.pasteWidth = cropAndPasteFirstInfo.pasteWidth;
    cropAndPasteSecondInfo.pasteHeight = cropAndPasteFirstInfo.pasteHeight;
    cropAndPasteFirstInfo.pasteWidth = cropAndPasteSecondInfo.cropWidth;
    cropAndPasteFirstInfo.pasteHeight = cropAndPasteSecondInfo.cropHeight;
}

void DvppProcess::GetWidthExtendDirection(uint32_t extendedWidth)
{
    if ((cropLeftOffset_ + extendedWidth) <= inWidthStride_) { // extend to right
        wExtendDirection_ = TO_RIGHT;
    } else if ((cropRightOffset_ + 1) >= extendedWidth) { // extend to left
        wExtendDirection_ = TO_LEFT;
    } else {
        wExtendDirection_ = TO_LEFT_AND_RIGHT;
    }
}

void DvppProcess::GetHeightExtendDirection(uint32_t extendedHeight)
{
    if ((cropTopOffset_ + extendedHeight) <= inHeightStride_) { // extend to bottom
        hExtendDirection_ = TO_BOTTOM;
    } else if ((cropBottomOffset_ + 1) >= extendedHeight) { // extend to top
        hExtendDirection_ = TO_TOP;
    } else {
        hExtendDirection_ = TO_TOP_AND_BOTTOM;
    }
}

Result DvppProcess::BuildVpcParamterStack(std::stack<CropAndPaste> &cropAndPasteInfoStack)
{
    CropAndPaste cropAndPasteFirstInfo;
    cropAndPasteFirstInfo.cropWidth = cropRightOffset_ - cropLeftOffset_ + 1;
    cropAndPasteFirstInfo.cropHeight = cropBottomOffset_ - cropTopOffset_ + 1;
    cropAndPasteFirstInfo.pasteWidth = pasteRightOffset_ - pasteLeftOffset_ + 1;
    cropAndPasteFirstInfo.pasteHeight = pasteBottomOffset_ - pasteTopOffset_ + 1;
    // corp width must be larger than 10, or must be enlarged
    if (cropAndPasteFirstInfo.cropWidth < 10) {
        if (cropAndPasteFirstInfo.cropWidth & 0x1) { // corp width must even
            ERROR_LOG("cropWidth = %u, it should be even", cropAndPasteFirstInfo.cropWidth);
            return FAILED;
        }
        widthCoeffi_ = (cropAndPasteFirstInfo.cropWidth == 4) ? 3 : 2; // cropWidth:4, 6, 8; extend to: 12, 12 ,16
    }
    // corp height must be larger than 6, or must be enlarged
    if (cropAndPasteFirstInfo.cropHeight < 6) {
        if (cropAndPasteFirstInfo.cropHeight & 0x1) { // corp height must even
            ERROR_LOG("cropHeight = %u, it should be even", cropAndPasteFirstInfo.cropHeight);
            return FAILED;
        }
        heightCoeffi_ = 2;
    }

    uint32_t extendedWidth = cropAndPasteFirstInfo.cropWidth * widthCoeffi_;
    if (extendedWidth > inWidthStride_) {
        ERROR_LOG("extended width %u is larger than inWidthStride %u", extendedWidth, inWidthStride_);
        return FAILED;
    }
    GetWidthExtendDirection(extendedWidth);

    uint32_t extendedHeight = cropAndPasteFirstInfo.cropHeight * heightCoeffi_;
    if (extendedHeight > inHeightStride_) {
        ERROR_LOG("extended height %u is larger than inHeightStride %u", extendedHeight, inHeightStride_);
        return FAILED;
    }
    GetHeightExtendDirection(extendedHeight);

    cropAndPasteFirstInfo.cropWidth *= widthCoeffi_;
    cropAndPasteFirstInfo.cropHeight *= heightCoeffi_;
    cropAndPasteFirstInfo.pasteWidth *= widthCoeffi_;
    cropAndPasteFirstInfo.pasteHeight *= heightCoeffi_;

    CropAndPaste cropAndPasteSecondInfo;
    GetCropAndPasteInfo(cropAndPasteFirstInfo, cropAndPasteSecondInfo);

    cropAndPasteInfoStack.push(cropAndPasteSecondInfo);
    cropAndPasteInfoStack.push(cropAndPasteFirstInfo);

    return SUCCESS;
}

Result DvppProcess::ModifyFirstCropRoi(CropAndPaste cropAndPasteFirstInfo)
{
    switch (wExtendDirection_) {
        case TO_RIGHT:
            (void)acldvppSetRoiConfigRight(cropArea_, cropLeftOffset_ + cropAndPasteFirstInfo.cropWidth -1);
            break;
        case TO_LEFT:
            (void)acldvppSetRoiConfigLeft(cropArea_, cropRightOffset_ + 1 - cropAndPasteFirstInfo.cropWidth);
            break;
        case TO_LEFT_AND_RIGHT:
            (void)acldvppSetRoiConfigRight(cropArea_, inWidthStride_ - 1);
            (void)acldvppSetRoiConfigLeft(cropArea_, inWidthStride_ - cropAndPasteFirstInfo.cropWidth);
            break;
         default:
            ERROR_LOG("invalid wExtendDirection_ %d", static_cast<int32_t>(wExtendDirection_));
            return FAILED;
    }

    switch (hExtendDirection_) {
        case TO_BOTTOM:
            (void)acldvppSetRoiConfigBottom(cropArea_, cropTopOffset_ + cropAndPasteFirstInfo.cropHeight - 1);
            break;
        case TO_TOP:
            (void)acldvppSetRoiConfigTop(cropArea_, cropBottomOffset_ + 1 - cropAndPasteFirstInfo.cropHeight);
            break;
        case TO_TOP_AND_BOTTOM:
            (void)acldvppSetRoiConfigBottom(cropArea_, inHeightStride_ - 1);
            (void)acldvppSetRoiConfigTop(cropArea_, inHeightStride_ - cropAndPasteFirstInfo.cropHeight);
            break;
         default:
            ERROR_LOG("invalid hExtendDirection_ %d", static_cast<int32_t>(hExtendDirection_));
            return FAILED;
    }

    return SUCCESS;
}

Result DvppProcess::SplitProcessCropAndPasteFirst(CropAndPaste cropAndPasteFirstInfo)
{
    Result ret = ModifyFirstCropRoi(cropAndPasteFirstInfo);
    if (ret != SUCCESS) {
        ERROR_LOG("modify first crop area failed");
        return FAILED;
    }
    acldvppRoiConfig *pasteAreaFirst = acldvppCreateRoiConfig(0, cropAndPasteFirstInfo.pasteWidth - 1,
        0, cropAndPasteFirstInfo.pasteHeight - 1);
    if (pasteAreaFirst == nullptr) {
        ERROR_LOG("create first paste area failed");
        return FAILED;
    }

    uint32_t vpcOutWidthStrideFirst = AlignSize(cropAndPasteFirstInfo.pasteWidth, 16); // 16-byte alignment
    // vpcOutWidthStrideFirst must be larger than or equal to 32
    if (vpcOutWidthStrideFirst < 32) {
        vpcOutWidthStrideFirst = 32;
    }
    uint32_t vpcOutHeightStrideFirst = AlignSize(cropAndPasteFirstInfo.pasteHeight, 2); // 2-byte alignment
    uint32_t vpcOutBufferSizeFirst = vpcOutWidthStrideFirst * vpcOutHeightStrideFirst * 3 / 2; // yuv420 image size
    aclError aclRet = acldvppMalloc(&vpcOutBufferDevFirst_, vpcOutBufferSizeFirst);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("malloc first output image buffer failed, vpcOutBufferSize = %u, errorCode = %d",
            vpcOutBufferSizeFirst, static_cast<int32_t>(aclRet));
        (void)acldvppDestroyRoiConfig(pasteAreaFirst);
        return FAILED;
    }
    vpcOutputDescFirst_ = acldvppCreatePicDesc();
    if (vpcOutputDescFirst_ == nullptr) {
        ERROR_LOG("creat first vpc output description failed");
        (void)acldvppDestroyRoiConfig(pasteAreaFirst);
        (void)acldvppFree(vpcOutBufferDevFirst_);
        return FAILED;
    }

    (void)acldvppSetPicDescData(vpcOutputDescFirst_, vpcOutBufferDevFirst_);
    (void)acldvppSetPicDescFormat(vpcOutputDescFirst_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    (void)acldvppSetPicDescWidth(vpcOutputDescFirst_, cropAndPasteFirstInfo.pasteWidth);
    (void)acldvppSetPicDescHeight(vpcOutputDescFirst_, cropAndPasteFirstInfo.pasteHeight);
    (void)acldvppSetPicDescWidthStride(vpcOutputDescFirst_, vpcOutWidthStrideFirst);
    (void)acldvppSetPicDescHeightStride(vpcOutputDescFirst_, vpcOutHeightStrideFirst);
    (void)acldvppSetPicDescSize(vpcOutputDescFirst_, vpcOutBufferSizeFirst);

    aclRet = acldvppVpcCropAndPasteAsync(dvppChannelDesc_, vpcInputDesc_,
        vpcOutputDescFirst_, cropArea_, pasteAreaFirst, stream_);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("first acldvppVpcCropAndPasteAsync failed, errorCode = %d", static_cast<int32_t>(aclRet));
        (void)acldvppDestroyRoiConfig(pasteAreaFirst);
        (void)acldvppFree(vpcOutBufferDevFirst_);
        (void)acldvppDestroyPicDesc(vpcOutputDescFirst_);
        return FAILED;
    }

    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("first stream synchronize of crop and paste failed, errorCode = %d", static_cast<int32_t>(aclRet));
        (void)acldvppDestroyRoiConfig(pasteAreaFirst);
        (void)acldvppFree(vpcOutBufferDevFirst_);
        (void)acldvppDestroyPicDesc(vpcOutputDescFirst_);
        return FAILED;
    }
    (void)acldvppDestroyRoiConfig(pasteAreaFirst);

    return SUCCESS;
}

acldvppRoiConfig *DvppProcess::GetSecondCropRoi(CropAndPaste cropAndPasteFirstInfo,
    CropAndPaste cropAndPasteSecondInfo)
{
    uint32_t leftOffset = 0;
    uint32_t rightOffset = 0;
    uint32_t topOffset = 0;
    uint32_t bottomOffset = 0;

    uint32_t rightPadding = (inWidthStride_ - 1 - cropRightOffset_) *
        (cropAndPasteFirstInfo.pasteWidth / cropAndPasteFirstInfo.cropWidth);
    uint32_t bottomPadding = (inHeightStride_ - 1 - cropBottomOffset_) *
        (cropAndPasteFirstInfo.pasteHeight / cropAndPasteFirstInfo.cropHeight);
    switch (wExtendDirection_) {
        case TO_RIGHT:
            rightOffset = leftOffset + cropAndPasteSecondInfo.cropWidth / widthCoeffi_ - 1;
            break;
        case TO_LEFT:
            rightOffset = cropAndPasteSecondInfo.cropWidth - 1;
            leftOffset = rightOffset + 1 - cropAndPasteSecondInfo.cropWidth / widthCoeffi_;
            break;
        case TO_LEFT_AND_RIGHT:
            rightOffset = cropAndPasteSecondInfo.cropWidth - 1 - rightPadding;
            leftOffset = rightOffset + 1 - cropAndPasteSecondInfo.cropWidth / widthCoeffi_;
            break;
         default:
            ERROR_LOG("invalid wExtendDirection_ %d", static_cast<int32_t>(wExtendDirection_));
            return nullptr;
    }

    switch (hExtendDirection_) {
        case TO_BOTTOM:
             bottomOffset = topOffset + cropAndPasteSecondInfo.cropHeight / heightCoeffi_ - 1;
            break;
        case TO_TOP:
            bottomOffset = cropAndPasteSecondInfo.cropHeight - 1;
            topOffset = bottomOffset + 1 - cropAndPasteSecondInfo.cropHeight / heightCoeffi_;
            break;
        case TO_TOP_AND_BOTTOM:
            bottomOffset = cropAndPasteSecondInfo.cropHeight - 1 - bottomPadding;
            topOffset = bottomOffset + 1 - cropAndPasteSecondInfo.cropHeight / heightCoeffi_;
            break;
         default:
            ERROR_LOG("invalid hExtendDirection_ %d", static_cast<int32_t>(hExtendDirection_));
            return nullptr;
    }

    acldvppRoiConfig *cropAreaSecond = acldvppCreateRoiConfig(leftOffset, rightOffset, topOffset, bottomOffset);
    if (cropAreaSecond == nullptr) {
        return nullptr;
    }

    return cropAreaSecond;
}

Result DvppProcess::SplitProcessCropAndPasteSecond(CropAndPaste cropAndPasteFirstInfo,
    CropAndPaste cropAndPasteSecondInfo)
{
     acldvppRoiConfig *cropAreaSecond = GetSecondCropRoi(cropAndPasteFirstInfo, cropAndPasteSecondInfo);
     if (cropAreaSecond == nullptr) {
        ERROR_LOG("create second crop area failed");
        (void)acldvppFree(vpcOutBufferDevFirst_);
        (void)acldvppDestroyPicDesc(vpcOutputDescFirst_);
        return FAILED;
     }

    // output of first step as input of second step
    aclError aclRet = acldvppVpcCropAndPasteAsync(dvppChannelDesc_, vpcOutputDescFirst_,
        vpcOutputDesc_, cropAreaSecond, pasteArea_, stream_);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("second acldvppVpcCropAndPasteAsync failed, errorCode = %d", static_cast<int32_t>(aclRet));
        (void)acldvppDestroyRoiConfig(cropAreaSecond);
        (void)acldvppFree(vpcOutBufferDevFirst_);
        (void)acldvppDestroyPicDesc(vpcOutputDescFirst_);
        return FAILED;
    }

    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("second stream synchronize of crop and paste failed, errorCode = %d", static_cast<int32_t>(aclRet));
        (void)acldvppDestroyRoiConfig(cropAreaSecond);
        (void)acldvppFree(vpcOutBufferDevFirst_);
        (void)acldvppDestroyPicDesc(vpcOutputDescFirst_);
        return FAILED;
    }

    (void)acldvppDestroyRoiConfig(cropAreaSecond);
    (void)acldvppFree(vpcOutBufferDevFirst_);
    (void)acldvppDestroyPicDesc(vpcOutputDescFirst_);

    return SUCCESS;
}

Result DvppProcess::SplitProcessCropAndPaste()
{
    std::stack<CropAndPaste> cropAndPasteInfoStack;
    Result ret = BuildVpcParamterStack(cropAndPasteInfoStack);
    if (ret != SUCCESS) {
        ERROR_LOG("build resize paramter stack failed");
        return ret;
    }

    // first step
    CropAndPaste cropAndPasteFirstInfo = cropAndPasteInfoStack.top();
    cropAndPasteInfoStack.pop();
    ret = SplitProcessCropAndPasteFirst(cropAndPasteFirstInfo);
    if (ret != SUCCESS) {
        ERROR_LOG("SplitProcessCropAndPasteFirst failed");
        return ret;
    }

    // second step
    CropAndPaste cropAndPasteSecondInfo = cropAndPasteInfoStack.top();
    cropAndPasteInfoStack.pop();
    ret = SplitProcessCropAndPasteSecond(cropAndPasteFirstInfo, cropAndPasteSecondInfo);
    if (ret != SUCCESS) {
        ERROR_LOG("SplitProcessCropAndPasteSecond failed");
        return ret;
    }

    return SUCCESS;
}

Result DvppProcess::ProcessCropAndPaste()
{
    // crop and patse pic
    aclError aclRet = acldvppVpcCropAndPasteAsync(dvppChannelDesc_, vpcInputDesc_,
        vpcOutputDesc_, cropArea_, pasteArea_, stream_);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("acldvppVpcCropAndPasteAsync failed, errorCode = %d", static_cast<int32_t>(aclRet));
        return FAILED;
    }

    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("crop and paste aclrtSynchronizeStream failed, errorCode = %d", static_cast<int32_t>(aclRet));
        return FAILED;
    }

    return SUCCESS;
}

Result DvppProcess::InitCropAndPasteResource()
{
    cropArea_ = acldvppCreateRoiConfig(cropLeftOffset_, cropRightOffset_, cropTopOffset_, cropBottomOffset_);
    if (cropArea_ == nullptr) {
        ERROR_LOG("create crop area failed");
        return FAILED;
    }

    pasteArea_ = acldvppCreateRoiConfig(pasteLeftOffset_, pasteRightOffset_, pasteTopOffset_, pasteBottomOffset_);
    if (pasteArea_ == nullptr) {
        ERROR_LOG("create paste area failed");
        return FAILED;
    }

    Result ret = InitCropAndPasteInputDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("InitCropAndPasteInputDesc failed");
        return FAILED;
    }

    ret = InitCropAndPasteOutputDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("InitCropAndPasteOutputDesc failed");
        return FAILED;
    }

    return SUCCESS;
}

void DvppProcess::DestroyCropAndPasteResource()
{
    if (cropArea_ != nullptr) {
        (void)acldvppDestroyRoiConfig(cropArea_);
        cropArea_ = nullptr;
    }

    if (pasteArea_ != nullptr) {
        (void)acldvppDestroyRoiConfig(pasteArea_);
        pasteArea_ = nullptr;
    }

    if (vpcInBufferDev_ != nullptr) {
        (void)acldvppFree(vpcInBufferDev_);
        vpcInBufferDev_ = nullptr;
    }

    if (vpcOutBufferDev_ != nullptr) {
        (void)acldvppFree(vpcOutBufferDev_);
        vpcOutBufferDev_ = nullptr;
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

Result DvppProcess::Process()
{
    bool needSplit = IsNeedSplit();
    Result ret = InitCropAndPasteResource();
    if (ret != SUCCESS) {
        ERROR_LOG("InitCropAndPasteResource failed");
        return FAILED;
    }
    if (needSplit) { // vpc process in two steps
        INFO_LOG("call SplitProcessCropAndPaste");
        ret = SplitProcessCropAndPaste();
        if (ret != SUCCESS) {
            ERROR_LOG("SplitProcessCropAndPaste failed");
            return FAILED;
        }
    } else { // vpc process in one steps
        INFO_LOG("call ProcessCropAndPaste");
        ret = ProcessCropAndPaste();
        if (ret != SUCCESS) {
            ERROR_LOG("ProcessCropAndPaste failed");
            return FAILED;
        }
    }
    // save crop and paste result
    ret = Utils::SaveDvppOutputData(outFileName_.c_str(), vpcOutBufferDev_, vpcOutBufferSize_);
    if (ret != SUCCESS) {
        ERROR_LOG("save encode output data failed.");
        return FAILED;
    }

    INFO_LOG("vpc crop and paste success");
    return SUCCESS;
}
