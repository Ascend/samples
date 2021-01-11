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
#include <iostream>
#include <string>
#include "acl/acl.h"
#include "utils.h"
using namespace std;

DvppProcess::DvppProcess(aclrtStream &stream)
    : stream_(stream), dvppChannelDesc_(nullptr), dvppType_(VPC_RESIZE),
      cropArea_(nullptr), pasteArea_(nullptr), jpegeConfig_(nullptr), resizeConfig_(nullptr),
      decodeOutBufferDev_(nullptr), decodeOutputDesc_(nullptr), encodeOutBufferDev_(nullptr),
      encodeInputDesc_(nullptr), vpcInputDesc_(nullptr), vpcOutputDesc_(nullptr), inDevBuffer_(nullptr),
      inDevBufferSizeD_(0), inDevBufferSizeE_(0), jpegDecodeOutputSize_(0), decodeOutputWidth_(0),
      decodeOutputWidthStride_(0), decodeOutputHeight_(0), vpcInBufferDev_(nullptr), vpcOutBufferDev_(nullptr),
      vpcOutBufferSize_(0), modelInputWidth_(0), modelInputHeight_(0), jpegeInputWidth_(0), jpegeInputHeight_(0)
{
}

DvppProcess::~DvppProcess()
{
    DestroyResource();
    DestroyDvppOutputPara();
}

uint32_t AlignSize(uint32_t origSize, uint32_t alignment)
{
    if (alignment == 0) {
        return 0;
    }
    uint32_t alignmentH = alignment - 1;
    return (origSize + alignmentH) / alignment * alignment;
}

void DvppProcess::SetDvppType(DvppType dvppType)
{
    dvppType_ = dvppType;
}

Result DvppProcess::InitResource()
{
    dvppChannelDesc_ = acldvppCreateChannelDesc();
    if (dvppChannelDesc_ == nullptr) {
        ERROR_LOG("acldvppCreateChannelDesc failed");
        return FAILED;
    }

    aclError aclRet = acldvppCreateChannel(dvppChannelDesc_);
    if (aclRet != ACL_ERROR_NONE) {
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
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("acldvppDestroyChannel failed, errorCode = %d", static_cast<int32_t>(aclRet));
        }

        (void)acldvppDestroyChannelDesc(dvppChannelDesc_);
        dvppChannelDesc_ = nullptr;
    }
}

void DvppProcess::SetInput4JpegD(char *inDevBuffer, uint32_t inDevBufferSize, const PicDesc &picDesc)
{
    inDevBuffer_ = inDevBuffer;
    inDevBufferSizeD_ = inDevBufferSize;
    jpegDecodeOutputSize_ = picDesc.jpegDecodeSize;
}

void DvppProcess::GetDvppOutput(void **outputBuffer, int &outputSize)
{
    *outputBuffer = vpcOutBufferDev_;
    outputSize = vpcOutBufferSize_;
    vpcOutBufferDev_ = nullptr;
    vpcOutBufferSize_ = 0;
}

Result DvppProcess::InitDvppOutputPara(int modelInputWidth, int modelInputHeight)
{
    if ((modelInputWidth <= 0) || (modelInputHeight <= 0)) {
        ERROR_LOG("init dvpp output para invalid, modelInputWidth = %d, modelInputHeight = %d",
            modelInputWidth, modelInputHeight);
        return FAILED;
    }
    modelInputWidth_ = modelInputWidth;
    modelInputHeight_ = modelInputHeight;
    return SUCCESS;
}

void DvppProcess::DestroyDvppOutputPara()
{
    if (vpcOutBufferDev_ != nullptr) {
        (void)acldvppFree(vpcOutBufferDev_);
        vpcOutBufferDev_ = nullptr;
    }
}

Result DvppProcess::InitDecodeOutputDesc()
{
    aclError aclRet = acldvppMalloc(&decodeOutBufferDev_, jpegDecodeOutputSize_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppMalloc decodeOutBufferDev_ failed, errorCode = %d", static_cast<int32_t>(aclRet));
        return FAILED;
    }

    decodeOutputDesc_ = acldvppCreatePicDesc();
    if (decodeOutputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc decodeOutputDesc_ failed");
        return FAILED;
    }

    acldvppSetPicDescData(decodeOutputDesc_, decodeOutBufferDev_);
    acldvppSetPicDescFormat(decodeOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    acldvppSetPicDescSize(decodeOutputDesc_, jpegDecodeOutputSize_);
    return SUCCESS;
}

Result DvppProcess::ProcessDecode()
{
    Result ret = InitDecodeOutputDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("InitDecodeOutputDesc failed");
        return FAILED;
    }

    aclError aclRet = acldvppJpegDecodeAsync(dvppChannelDesc_, reinterpret_cast<void *>(inDevBuffer_),
        inDevBufferSizeD_, decodeOutputDesc_, stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppJpegDecodeAsync failed, errorCode = %d", static_cast<int32_t>(aclRet));
        return FAILED;
    }

    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("decode aclrtSynchronizeStream failed, errorCode = %d", static_cast<int32_t>(aclRet));
        return FAILED;
    }
    // get yuv image width and height
    decodeOutputWidth_ = acldvppGetPicDescWidth(decodeOutputDesc_);
    decodeOutputHeight_ = acldvppGetPicDescHeight(decodeOutputDesc_);
    decodeOutputWidthStride_ = acldvppGetPicDescWidthStride(decodeOutputDesc_);

    return SUCCESS;
}

void DvppProcess::DestroyDecodeResource()
{
    if (decodeOutputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(decodeOutputDesc_);
        decodeOutputDesc_ = nullptr;
    }
}

Result DvppProcess::InitResizeInputDesc()
{
    uint32_t heightAlignment = 16;
    uint32_t sizeAlignment = 3;
    uint32_t sizeNum = 2;
    uint32_t jpegOutWidthStride = decodeOutputWidthStride_; // 128-byte alignment on 310, 64-byte alignment on 710
    uint32_t jpegOutHeightStride = AlignSize(decodeOutputHeight_, heightAlignment); // 16-byte alignment
    if (jpegOutWidthStride == 0 || jpegOutHeightStride == 0) {
        ERROR_LOG("InitResizeInputDesc AlignSize failed");
        return FAILED;
    }
    uint32_t jpegOutBufferSize = jpegOutWidthStride * jpegOutHeightStride * sizeAlignment / sizeNum;
    vpcInputDesc_ = acldvppCreatePicDesc();
    if (vpcInputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc vpcInputDesc_ failed");
        return FAILED;
    }

    (void)acldvppSetPicDescData(vpcInputDesc_, decodeOutBufferDev_);
    (void)acldvppSetPicDescFormat(vpcInputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    (void)acldvppSetPicDescWidth(vpcInputDesc_, decodeOutputWidth_);
    (void)acldvppSetPicDescHeight(vpcInputDesc_, decodeOutputHeight_);
    (void)acldvppSetPicDescWidthStride(vpcInputDesc_, jpegOutWidthStride);
    (void)acldvppSetPicDescHeightStride(vpcInputDesc_, jpegOutHeightStride);
    (void)acldvppSetPicDescSize(vpcInputDesc_, jpegOutBufferSize);
    return SUCCESS;
}

Result DvppProcess::InitResizeOutputDesc()
{
    int widthAlignment = 16;
    int heightAlignment = 2;
    int sizeAlignment = 3;
    int sizeNum = 2;
    int resizeOutWidth = modelInputWidth_;
    int resizeOutHeight = modelInputHeight_;
    int resizeOutWidthStride = AlignSize(modelInputWidth_, widthAlignment);
    int resizeOutHeightStride = AlignSize(modelInputHeight_, heightAlignment);
    if (resizeOutWidthStride == 0 || resizeOutHeightStride == 0) {
        ERROR_LOG("InitResizeOutputDesc AlignSize failed");
        return FAILED;
    }
    vpcOutBufferSize_ = resizeOutWidthStride * resizeOutHeightStride * sizeAlignment / sizeNum;
    aclError aclRet = acldvppMalloc(&vpcOutBufferDev_, vpcOutBufferSize_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppMalloc vpcOutBufferDev_ failed, errorCode = %d", static_cast<int32_t>(aclRet));
        return FAILED;
    }

    vpcOutputDesc_ = acldvppCreatePicDesc();
    if (vpcOutputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc vpcOutputDesc_ failed");
        return FAILED;
    }

    (void)acldvppSetPicDescData(vpcOutputDesc_, vpcOutBufferDev_);
    (void)acldvppSetPicDescFormat(vpcOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    (void)acldvppSetPicDescWidth(vpcOutputDesc_, resizeOutWidth);
    (void)acldvppSetPicDescHeight(vpcOutputDesc_, resizeOutHeight);
    (void)acldvppSetPicDescWidthStride(vpcOutputDesc_, resizeOutWidthStride);
    (void)acldvppSetPicDescHeightStride(vpcOutputDesc_, resizeOutHeightStride);
    (void)acldvppSetPicDescSize(vpcOutputDesc_, vpcOutBufferSize_);
    return SUCCESS;
}

Result DvppProcess::Init8kResizeInputDesc()
{
    uint32_t inWidthStride = 8192; // 8k picture width
    uint32_t inHeightStride = 8192; // 8k picture height
    uint32_t sizeAlignment = 3;
    uint32_t sizeNum = 2;

    uint32_t inBufferSize = inWidthStride * inWidthStride * sizeAlignment / sizeNum;
    vpcInputDesc_ = acldvppCreatePicDesc();
    if (vpcInputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc vpcInputDesc_ failed");
        return FAILED;
    }
    PicDesc testPic[] = {
        { "../data/dvpp_vpc_8192x8192_nv12.yuv", 8192, 8192}
        // other yuv file
    };
    vpcInBufferDev_ = Utils::GetPicDevBuffer(testPic[0], inBufferSize);
    if (vpcInBufferDev_ == nullptr) {
        ERROR_LOG("get picDevBuffer failed, file name = %s", testPic[0].picName.c_str());
        return FAILED;
    }
    (void)acldvppSetPicDescData(vpcInputDesc_, vpcInBufferDev_); //  JpegD -> vpcResize
    (void)acldvppSetPicDescFormat(vpcInputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    (void)acldvppSetPicDescWidth(vpcInputDesc_, inWidthStride);
    (void)acldvppSetPicDescHeight(vpcInputDesc_, inHeightStride);
    (void)acldvppSetPicDescWidthStride(vpcInputDesc_, inWidthStride);
    (void)acldvppSetPicDescHeightStride(vpcInputDesc_, inHeightStride);
    (void)acldvppSetPicDescSize(vpcInputDesc_, inBufferSize);
    return SUCCESS;
}

Result DvppProcess::Init8kResizeOutputDesc()
{
    uint32_t resizeOutWidthStride = 4000; // output picture width
    uint32_t resizeOutHeightStride = 4000; // output picture height
    uint32_t sizeAlignment = 3;
    uint32_t sizeNum = 2;

    vpcOutBufferSize_ = resizeOutWidthStride * resizeOutHeightStride * sizeAlignment / sizeNum;
    aclError aclRet = acldvppMalloc(&vpcOutBufferDev_, vpcOutBufferSize_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppMalloc vpcOutBufferDev_ failed, errorCode = %d", static_cast<int32_t>(aclRet));
        return FAILED;
    }

    vpcOutputDesc_ = acldvppCreatePicDesc();
    if (vpcOutputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc vpcOutputDesc_ failed");
        return FAILED;
    }

    (void)acldvppSetPicDescData(vpcOutputDesc_, vpcOutBufferDev_);
    (void)acldvppSetPicDescFormat(vpcOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    (void)acldvppSetPicDescWidth(vpcOutputDesc_, resizeOutWidthStride);
    (void)acldvppSetPicDescHeight(vpcOutputDesc_, resizeOutHeightStride);
    (void)acldvppSetPicDescWidthStride(vpcOutputDesc_, resizeOutWidthStride);
    (void)acldvppSetPicDescHeightStride(vpcOutputDesc_, resizeOutHeightStride);
    (void)acldvppSetPicDescSize(vpcOutputDesc_, vpcOutBufferSize_);
    return SUCCESS;
}


Result DvppProcess::ProcessResize()
{
    resizeConfig_ = acldvppCreateResizeConfig();
    if (resizeConfig_ == nullptr) {
        ERROR_LOG("acldvppCreateResizeConfig failed");
        return FAILED;
    }

    Result inputRet = SUCCESS;
    Result outputRet = SUCCESS;
    if (dvppType_ == VPC_RESIZE) {
        inputRet = InitResizeInputDesc();
        outputRet = InitResizeOutputDesc();
    } else if (dvppType_ == VPC_8K_RESIZE) {
        inputRet = Init8kResizeInputDesc();
        outputRet = Init8kResizeOutputDesc();
    } else {
        ERROR_LOG("invalid dvppType_ %d", static_cast<int32_t>(dvppType_));
        return FAILED;
    }
    if ((inputRet != SUCCESS) || (outputRet != SUCCESS)) {
        ERROR_LOG("init resize input or output description failed");
        return FAILED;
    }

    // resize pic
    aclError aclRet = acldvppVpcResizeAsync(dvppChannelDesc_, vpcInputDesc_,
        vpcOutputDesc_, resizeConfig_, stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppVpcResizeAsync failed, errorCode = %d", static_cast<int32_t>(aclRet));
        return FAILED;
    }

    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("resize aclrtSynchronizeStream failed, errorCode = %d", static_cast<int32_t>(aclRet));
        return FAILED;
    }

    return SUCCESS;
}

void DvppProcess::DestroyResizeResource()
{
    if (resizeConfig_ != nullptr) {
        (void)acldvppDestroyResizeConfig(resizeConfig_);
        resizeConfig_ = nullptr;
    }

    DestroyDecodeOutBuff();

    if (vpcInputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(vpcInputDesc_);
        vpcInputDesc_ = nullptr;
    }

    if (vpcOutputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(vpcOutputDesc_);
        vpcOutputDesc_ = nullptr;
    }

    if (vpcInBufferDev_ != nullptr) {
        (void)acldvppFree(vpcInBufferDev_);
        vpcInBufferDev_ = nullptr;
    }
}

Result DvppProcess::InitCropInputDesc()
{
    uint32_t heightAlignment = 16;
    uint32_t sizeAlignment = 3;
    uint32_t sizeNum = 2;
    uint32_t jpegOutWidthStride = decodeOutputWidthStride_; // 128-byte alignment on 310, 64-byte alignment on 710
    uint32_t jpegOutHeightStride = AlignSize(decodeOutputHeight_, heightAlignment); // 16-byte alignment
    if (jpegOutWidthStride == 0 || jpegOutHeightStride == 0) {
        ERROR_LOG("InitCropInputDesc AlignSize failed");
        return FAILED;
    }
    uint32_t jpegOutBufferSize = jpegOutWidthStride * jpegOutHeightStride * sizeAlignment / sizeNum;
    vpcInputDesc_ = acldvppCreatePicDesc();
    if (vpcInputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc vpcInputDesc_ failed");
        return FAILED;
    }

    (void)acldvppSetPicDescData(vpcInputDesc_, decodeOutBufferDev_); // JpegD -> vpcCrop
    (void)acldvppSetPicDescFormat(vpcInputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    (void)acldvppSetPicDescWidth(vpcInputDesc_, decodeOutputWidth_);
    (void)acldvppSetPicDescHeight(vpcInputDesc_, decodeOutputHeight_);
    (void)acldvppSetPicDescWidthStride(vpcInputDesc_, jpegOutWidthStride);
    (void)acldvppSetPicDescHeightStride(vpcInputDesc_, jpegOutHeightStride);
    (void)acldvppSetPicDescSize(vpcInputDesc_, jpegOutBufferSize);
    return SUCCESS;
}

Result DvppProcess::InitCropOutputDesc()
{
    int sizeAlignment = 3;
    int sizeNum = 2;
    int dvppOutWidth = modelInputWidth_;
    int dvppOutHeight = modelInputHeight_;
    int dvppOutWidthStride = modelInputWidth_;
    int dvppOutHeightStride = modelInputHeight_;
    vpcOutBufferSize_ = dvppOutWidthStride * dvppOutHeightStride * sizeAlignment / sizeNum;
    aclError aclRet = acldvppMalloc(&vpcOutBufferDev_, vpcOutBufferSize_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppMalloc vpcOutBufferDev_ failed, errorCode = %d", static_cast<int32_t>(aclRet));
        return FAILED;
    }
    vpcOutputDesc_ = acldvppCreatePicDesc();
    if (vpcOutputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc vpcOutputDesc_ failed");
        return FAILED;
    }

    (void)acldvppSetPicDescData(vpcOutputDesc_, vpcOutBufferDev_);
    (void)acldvppSetPicDescFormat(vpcOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    (void)acldvppSetPicDescWidth(vpcOutputDesc_, dvppOutWidth);
    (void)acldvppSetPicDescHeight(vpcOutputDesc_, dvppOutHeight);
    (void)acldvppSetPicDescWidthStride(vpcOutputDesc_, dvppOutWidthStride);
    (void)acldvppSetPicDescHeightStride(vpcOutputDesc_, dvppOutHeightStride);
    (void)acldvppSetPicDescSize(vpcOutputDesc_, vpcOutBufferSize_);
    return SUCCESS;
}

Result DvppProcess::ProcessCrop()
{
    uint32_t midNum = 2;
    uint32_t oddNum = 1;
    uint32_t cropSizeWidth = 200;
    uint32_t cropSizeHeight = 200;
    uint32_t cropLeftOffset = 550;  // must even
    uint32_t cropRightOffset = cropLeftOffset + cropSizeWidth - oddNum;  // must odd
    uint32_t cropTopOffset = 480;  // must even
    uint32_t cropBottomOffset = cropTopOffset + cropSizeHeight - oddNum;  // must odd
    cropArea_ = acldvppCreateRoiConfig(cropLeftOffset, cropRightOffset,
        cropTopOffset, cropBottomOffset);
    if (cropArea_ == nullptr) {
        ERROR_LOG("acldvppCreateRoiConfig cropArea_ failed");
        return FAILED;
    }

    Result ret = InitCropInputDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("InitCropInputDesc failed");
        return FAILED;
    }

    ret = InitCropOutputDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("InitCropOutputDesc failed");
        return FAILED;
    }

    // crop pic
    aclError aclRet = acldvppVpcCropAsync(dvppChannelDesc_, vpcInputDesc_,
        vpcOutputDesc_, cropArea_, stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppVpcCropAsync failed, errorCode = %d", static_cast<int32_t>(aclRet));
        return FAILED;
    }

    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("crop aclrtSynchronizeStream failed, errorCode = %d", static_cast<int32_t>(aclRet));
        return FAILED;
    }

    return SUCCESS;
}

void DvppProcess::DestroyCropResource()
{
    if (cropArea_ != nullptr) {
        (void)acldvppDestroyRoiConfig(cropArea_);
        cropArea_ = nullptr;
    }

    DestroyDecodeOutBuff();

    if (vpcInputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(vpcInputDesc_);
        vpcInputDesc_ = nullptr;
    }

    if (vpcOutputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(vpcOutputDesc_);
        vpcOutputDesc_ = nullptr;
    }
}

Result DvppProcess::InitCropAndPasteInputDesc()
{
    uint32_t heightAlignment = 16;
    uint32_t sizeAlignment = 3;
    uint32_t sizeNum = 2;

    uint32_t jpegOutWidthStride = decodeOutputWidthStride_; // 128-byte alignment on 310, 64-byte alignment on 710
    uint32_t jpegOutHeightStride = AlignSize(decodeOutputHeight_, heightAlignment);
    if (jpegOutWidthStride == 0 || jpegOutHeightStride == 0) {
        ERROR_LOG("InitCropAndPasteInputDesc AlignSize failed");
        return FAILED;
    }
    uint32_t jpegOutBufferSize = jpegOutWidthStride * jpegOutHeightStride * sizeAlignment / sizeNum;
    vpcInputDesc_ = acldvppCreatePicDesc();
    if (vpcInputDesc_ == nullptr) {
        ERROR_LOG("InitResizeInputDesc vpcInputDesc_ failed");
        return FAILED;
    }

    (void)acldvppSetPicDescData(vpcInputDesc_, decodeOutBufferDev_); // JpegD -> vpcCropAndPaste
    (void)acldvppSetPicDescFormat(vpcInputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    (void)acldvppSetPicDescWidth(vpcInputDesc_, decodeOutputWidth_);
    (void)acldvppSetPicDescHeight(vpcInputDesc_, decodeOutputHeight_);
    (void)acldvppSetPicDescWidthStride(vpcInputDesc_, jpegOutWidthStride);
    (void)acldvppSetPicDescHeightStride(vpcInputDesc_, jpegOutHeightStride);
    (void)acldvppSetPicDescSize(vpcInputDesc_, jpegOutBufferSize);
    return SUCCESS;
}

Result DvppProcess::InitCropAndPasteOutputDesc()
{
    int dvppOutWidth = modelInputWidth_;
    int dvppOutHeight = modelInputHeight_;
    int dvppOutWidthStride = modelInputWidth_;
    int dvppOutHeightStride = modelInputHeight_;
    int sizeAlignment = 3;
    int sizeNum = 2;
    vpcOutBufferSize_ =
        dvppOutWidthStride * dvppOutHeightStride * sizeAlignment / sizeNum;
    aclError aclRet = acldvppMalloc(&vpcOutBufferDev_, vpcOutBufferSize_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppMalloc vpcOutBufferDev_ failed, errorCode = %d", static_cast<int32_t>(aclRet));
        return FAILED;
    }
    vpcOutputDesc_ = acldvppCreatePicDesc();
    if (vpcOutputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc vpcOutputDesc_ failed");
        return FAILED;
    }

    (void)acldvppSetPicDescData(vpcOutputDesc_, vpcOutBufferDev_);
    (void)acldvppSetPicDescFormat(vpcOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    (void)acldvppSetPicDescWidth(vpcOutputDesc_, dvppOutWidth);
    (void)acldvppSetPicDescHeight(vpcOutputDesc_, dvppOutHeight);
    (void)acldvppSetPicDescWidthStride(vpcOutputDesc_, dvppOutWidthStride);
    (void)acldvppSetPicDescHeightStride(vpcOutputDesc_, dvppOutHeightStride);
    (void)acldvppSetPicDescSize(vpcOutputDesc_, vpcOutBufferSize_);
    return SUCCESS;
}

Result DvppProcess::ProcessCropAndPaste()
{
    uint32_t midNum = 2;
    uint32_t oddNum = 1;
    uint32_t cropSizeWidth = 200;
    uint32_t cropSizeHeight = 200;
    uint32_t cropLeftOffset = 512;  // must even
    uint32_t cropRightOffset = cropLeftOffset + cropSizeWidth - oddNum;  // must odd
    uint32_t cropTopOffset = 512;  // must even
    uint32_t cropBottomOffset = cropTopOffset + cropSizeHeight - oddNum;  // must odd
    cropArea_ = acldvppCreateRoiConfig(cropLeftOffset, cropRightOffset,
        cropTopOffset, cropBottomOffset);
    if (cropArea_ == nullptr) {
        ERROR_LOG("acldvppCreateRoiConfig cropArea_ failed");
        return FAILED;
    }

    uint32_t pasteLeftOffset = 16;  // must even
    uint32_t pasteRightOffset = pasteLeftOffset + cropSizeWidth - oddNum;  // must odd
    uint32_t pasteTopOffset = 16;  // must even
    uint32_t pasteBottomOffset = pasteTopOffset + cropSizeHeight - oddNum;  // must odd
    pasteArea_ = acldvppCreateRoiConfig(pasteLeftOffset, pasteRightOffset,
        pasteTopOffset, pasteBottomOffset);
    if (pasteArea_ == nullptr) {
        ERROR_LOG("acldvppCreateRoiConfig pasteArea_ failed");
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

    // crop and patse pic
    aclError aclRet = acldvppVpcCropAndPasteAsync(dvppChannelDesc_, vpcInputDesc_,
        vpcOutputDesc_, cropArea_, pasteArea_, stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppVpcCropAndPasteAsync failed, errorCode = %d", static_cast<int32_t>(aclRet));
        return FAILED;
    }

    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("crop and paste aclrtSynchronizeStream failed, errorCode = %d", static_cast<int32_t>(aclRet));
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

    DestroyDecodeOutBuff();

    if (vpcInputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(vpcInputDesc_);
        vpcInputDesc_ = nullptr;
    }

    if (vpcOutputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(vpcOutputDesc_);
        vpcOutputDesc_ = nullptr;
    }
}

void DvppProcess::SetInput4JpegE(char *inDevBuffer, int inDevBufferSize, int inputWidth, int inputHeight)
{
    inDevBuffer_ = inDevBuffer;
    inDevBufferSizeE_ = inDevBufferSize;
    jpegeInputWidth_ = inputWidth;
    jpegeInputHeight_ = inputHeight;
}

uint32_t DvppProcess::ComputeEncodeInputSize(int inputWidth, int inputHeight)
{
    uint32_t widthAlignment = 16;
    uint32_t heightAlignment = 2;
    uint32_t sizeAlignment = 3;
    uint32_t sizeNum = 2;
    uint32_t encodeInWidthStride = AlignSize(inputWidth, widthAlignment);
    uint32_t encodeInHeightStride = AlignSize(inputHeight, heightAlignment);
    if (encodeInWidthStride == 0 || encodeInHeightStride == 0) {
        ERROR_LOG("ComputeEncodeInputSize AlignSize failed");
        return FAILED;
    }
    uint32_t encodeInBufferSize =
        encodeInWidthStride * encodeInHeightStride * sizeAlignment / sizeNum;
    return encodeInBufferSize;
}

Result DvppProcess::InitEncodeResource()
{
    uint32_t widthAlignment = 16;
    uint32_t heightAlignment = 2;
    uint32_t encodeInWidthStride = AlignSize(jpegeInputWidth_, widthAlignment);
    uint32_t encodeInHeightStride = AlignSize(jpegeInputHeight_, heightAlignment);
    if (encodeInWidthStride == 0 || encodeInHeightStride == 0) {
        ERROR_LOG("InitEncodeInputDesc AlignSize failed");
        return FAILED;
    }
    encodeInputDesc_ = acldvppCreatePicDesc();
    if (encodeInputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc encodeInputDesc_ failed");
        return FAILED;
    }

    (void)acldvppSetPicDescData(encodeInputDesc_, reinterpret_cast<void *>(inDevBuffer_));
    (void)acldvppSetPicDescFormat(encodeInputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    (void)acldvppSetPicDescWidth(encodeInputDesc_, jpegeInputWidth_);
    (void)acldvppSetPicDescHeight(encodeInputDesc_, jpegeInputHeight_);
    (void)acldvppSetPicDescWidthStride(encodeInputDesc_, encodeInWidthStride);
    (void)acldvppSetPicDescHeightStride(encodeInputDesc_, encodeInHeightStride);
    (void)acldvppSetPicDescSize(encodeInputDesc_, inDevBufferSizeE_);

    jpegeConfig_ = acldvppCreateJpegeConfig();
    uint32_t encodeLevel = 100; // default optimal level (0-100)
    (void)acldvppSetJpegeConfigLevel(jpegeConfig_, encodeLevel);

    aclError aclRet = acldvppJpegPredictEncSize(encodeInputDesc_, jpegeConfig_, &encodeOutBufferSize_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("predict encodeOutBufferSize_ failed, errorCode = %d", static_cast<int32_t>(aclRet));
        return FAILED;
    }

    aclRet = acldvppMalloc(&encodeOutBufferDev_, encodeOutBufferSize_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("malloc encodeOutBufferDev_ failed, errorCode = %d", static_cast<int32_t>(aclRet));
        return FAILED;
    }

    return SUCCESS;
}

Result DvppProcess::ProcessJpegE()
{
    std::string encodeOutFileName = "./result/jpege_output_";
    PicDesc testPic[] = {
        { "../data/wood_rabbit_1024_1068_nv12.yuv", 1024, 1068}
        // other yuv file
    };

    for (size_t index = 0; index < sizeof(testPic) / sizeof(testPic[0]); ++index) {
        INFO_LOG("start to jpege picture %s", testPic[index].picName.c_str());

        uint32_t jpegInBufferSize;
        jpegInBufferSize = ComputeEncodeInputSize(testPic[index].width, testPic[index].height);

        // get input data buffer
        char *picDevBuffer = reinterpret_cast<char *>(Utils::GetPicDevBuffer(testPic[index], jpegInBufferSize));
        if (picDevBuffer == nullptr) {
            ERROR_LOG("get picDevBuffer failed, index is %zu", index);
            return FAILED;
        }

        // set jpege input data
        SetInput4JpegE(picDevBuffer, jpegInBufferSize, testPic[index].width, testPic[index].height);
        picDevBuffer = nullptr;

        // init jpege resource
        Result ret = InitEncodeResource();
        if (ret != SUCCESS) {
            ERROR_LOG("init jpeg encode failed");
            DestroyEncodeResource();
            return FAILED;
        }

        aclError aclRet = acldvppJpegEncodeAsync(dvppChannelDesc_, encodeInputDesc_, encodeOutBufferDev_,
            &encodeOutBufferSize_, jpegeConfig_, stream_);
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("acldvppJpegEncodeAsync failed, errorCode = %d", static_cast<int32_t>(aclRet));
            DestroyEncodeResource();
            return FAILED;
        }

        aclRet = aclrtSynchronizeStream(stream_);
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("encode aclrtSynchronizeStream failed, errorCode = %d", static_cast<int32_t>(aclRet));
            DestroyEncodeResource();
            return FAILED;
        }

        // save jpege result
        encodeOutFileName = encodeOutFileName + std::to_string(index) + ".jpg";
        ret = Utils::SaveDvppOutputData(encodeOutFileName.c_str(), encodeOutBufferDev_, encodeOutBufferSize_);
        if (ret != SUCCESS) {
            ERROR_LOG("save encode output data failed.");
            DestroyEncodeResource();
            return FAILED;
        }
    }
    DestroyEncodeResource();
    return SUCCESS;
}

void DvppProcess::DestroyEncodeResource()
{
    if (jpegeConfig_ != nullptr) {
        (void)acldvppDestroyJpegeConfig(jpegeConfig_);
        jpegeConfig_ = nullptr;
    }

    if (encodeInputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(encodeInputDesc_);
        encodeInputDesc_ = nullptr;
    }

    if (inDevBuffer_ != nullptr) {
        (void)acldvppFree(inDevBuffer_);
        inDevBuffer_ = nullptr;
    }

    if (encodeOutBufferDev_ != nullptr) {
        (void)acldvppFree(encodeOutBufferDev_);
        encodeOutBufferDev_ = nullptr;
    }
}

Result DvppProcess::Process8kResize()
{
    std::string vpcOutFileName = "./result/dvpp_vpc_4000x4000_nv12.yuv";
    Result ret = ProcessResize();
    if (ret != SUCCESS) {
        ERROR_LOG("ProcessResize failed");
        DestroyResizeResource();
        return FAILED;
    }
    ret = Utils::SaveDvppOutputData(vpcOutFileName.c_str(), vpcOutBufferDev_, vpcOutBufferSize_);
    if (ret != SUCCESS) {
        ERROR_LOG("save encode output data failed.");
        DestroyResizeResource();
        return FAILED;
    }
    DestroyResizeResource();

    return SUCCESS;
}

Result DvppProcess::Process()
{
    // pic decode
    INFO_LOG("call JpegD");
    Result ret = ProcessDecode();
    if (ret != SUCCESS) {
        ERROR_LOG("ProcessDecode failed");
        DestroyDecodeOutBuff();
        DestroyDecodeResource();
        return FAILED;
    }

    DestroyDecodeResource();

    switch (dvppType_) {
        case VPC_RESIZE:
            INFO_LOG("call vpcResize");
            ret = ProcessResize();
            if (ret != SUCCESS) {
                ERROR_LOG("ProcessResize failed");
                DestroyResizeResource();
                return FAILED;
            }
            DestroyResizeResource();
            break;

        case VPC_CROP:
            INFO_LOG("call vpcCrop");
            ret = ProcessCrop();
            if (ret != SUCCESS) {
                ERROR_LOG("ProcessCrop failed");
                DestroyCropResource();
                return FAILED;
            }
            DestroyCropResource();
            break;

        case VPC_CROP_AND_PASTE:
            INFO_LOG("call vpcCropAndPaste");
            ret = ProcessCropAndPaste();
            if (ret != SUCCESS) {
                ERROR_LOG("ProcessCropAndPaste failed");
                DestroyCropAndPasteResource();
                return FAILED;
            }
            DestroyCropAndPasteResource();
            break;

        default:
            ERROR_LOG("unsupported type");
            DestroyDecodeOutBuff();
            break;
    }

    INFO_LOG("Process dvpp success");
    return SUCCESS;
}

void DvppProcess::DestroyDecodeOutBuff()
{
    if (decodeOutBufferDev_ != nullptr) {
        (void)acldvppFree(decodeOutBufferDev_);
        decodeOutBufferDev_ = nullptr;
    }
}
