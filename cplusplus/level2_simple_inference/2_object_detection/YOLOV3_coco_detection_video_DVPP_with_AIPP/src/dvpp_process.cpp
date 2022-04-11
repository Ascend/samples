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
#include "acl/acl.h"
#include "utils.h"
using namespace std;

namespace {
    uint32_t AlignSize(uint32_t origSize, uint32_t alignment)
    {
        if (alignment == 0) {
            return 0;
        }
        uint32_t alignmentH = alignment - 1;
        return (origSize + alignmentH) / alignment * alignment;
    }
}

DvppProcess::DvppProcess(aclrtStream& stream) : stream_(stream), dvppChannelDesc_(nullptr),
    resizeConfig_(nullptr), resizeInputDesc_(nullptr), resizeOutputDesc_(nullptr), 
    resizeOutBufferDev_(nullptr), resizeOutBufferSize_(0), modelInputWidth_(0), 
    modelInputHeight_(0), resizeOutWidthStride_(0), resizeOutHeightStride_(0)
{
}

DvppProcess::~DvppProcess()
{
    DestroyResource();
    DestroyOutputPara();
}

Result DvppProcess::InitResource()
{
    dvppChannelDesc_ = acldvppCreateChannelDesc();
    if (dvppChannelDesc_ == nullptr) {
        ERROR_LOG("acldvppCreateChannelDesc failed");
        return FAILED;
    }

    aclError ret = acldvppCreateChannel(dvppChannelDesc_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acldvppCreateChannelAsync failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    resizeConfig_ = acldvppCreateResizeConfig();
    if (resizeConfig_ == nullptr) {
        ERROR_LOG("acldvppCreateResizeConfig failed");
        return FAILED;
    }

    INFO_LOG("dvpp init resource success");
    return SUCCESS;
}

void DvppProcess::GetDvppOutput(void **outputBuffer, int &outputSize)
{
    if (outputBuffer == nullptr) {
        ERROR_LOG("outputBuffer is nullptr");
        return;
    }
    *outputBuffer = resizeOutBufferDev_;
    outputSize = resizeOutBufferSize_;
    resizeOutBufferDev_ = nullptr;
    resizeOutBufferSize_ = 0;
}

Result DvppProcess::InitDvppOutputPara(int modelInputWidth, int modelInputHeight)
{
    if ((modelInputWidth <= 0) || (modelInputHeight <= 0)) {
        ERROR_LOG("InitInput para invalid, modelInputWidth %d, modelInputHeight %d",
            modelInputWidth, modelInputHeight);
        return FAILED;
    }

    modelInputWidth_ = modelInputWidth;
    modelInputHeight_ = modelInputHeight;
    resizeOutWidthStride_ = AlignSize(modelInputWidth, 16); // 16-byte alignment
    resizeOutHeightStride_ = AlignSize(modelInputHeight, 2); // 2-byte alignment

    // output buffer, adjust the value based on the actual model
    resizeOutBufferSize_ = resizeOutWidthStride_ * resizeOutHeightStride_ * 3 / 2; // yuv format size
    aclError ret = acldvppMalloc(&resizeOutBufferDev_, resizeOutBufferSize_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acldvppMalloc resizeOutBuffer failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    return SUCCESS;
}

void DvppProcess::DestroyOutputPara()
{
    if (resizeOutBufferDev_ != nullptr) {
        (void)acldvppFree(resizeOutBufferDev_);
        resizeOutBufferDev_ = nullptr;
    }
}

Result DvppProcess::InitResizeInputDesc(PicDesc& srcImage)
{
    uint32_t vdecOutWidthStride = AlignSize(srcImage.width, 128); // 128-byte alignment on 310, 64-byte alignment on 710
    uint32_t vdecOutHeightStride = AlignSize(srcImage.height, 16); // 16-byte alignment
    uint32_t vdecOutBufferSize = vdecOutWidthStride * vdecOutHeightStride * 3 / 2; // yuv format size
    resizeInputDesc_ = acldvppCreatePicDesc();
    if (resizeInputDesc_ == nullptr) {
        ERROR_LOG("InitResizeInputDesc failed");
        return FAILED;
    }

    (void)acldvppSetPicDescData(resizeInputDesc_, srcImage.data.get());
    (void)acldvppSetPicDescFormat(resizeInputDesc_, PIXEL_FORMAT_YVU_SEMIPLANAR_420);
    (void)acldvppSetPicDescWidth(resizeInputDesc_, srcImage.width);
    (void)acldvppSetPicDescHeight(resizeInputDesc_, srcImage.height);
    (void)acldvppSetPicDescWidthStride(resizeInputDesc_, vdecOutWidthStride);
    (void)acldvppSetPicDescHeightStride(resizeInputDesc_, vdecOutHeightStride);
    (void)acldvppSetPicDescSize(resizeInputDesc_, vdecOutBufferSize);
    return SUCCESS;
}

Result DvppProcess::InitResizeOutputDesc()
{
    resizeOutputDesc_ = acldvppCreatePicDesc();
    if (resizeOutputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc failed");
        return FAILED;
    }

    (void)acldvppSetPicDescData(resizeOutputDesc_, resizeOutBufferDev_);
    (void)acldvppSetPicDescFormat(resizeOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    (void)acldvppSetPicDescWidth(resizeOutputDesc_, modelInputWidth_);
    (void)acldvppSetPicDescHeight(resizeOutputDesc_, modelInputHeight_);
    (void)acldvppSetPicDescWidthStride(resizeOutputDesc_, resizeOutWidthStride_);
    (void)acldvppSetPicDescHeightStride(resizeOutputDesc_, resizeOutHeightStride_);
    (void)acldvppSetPicDescSize(resizeOutputDesc_, resizeOutBufferSize_);
    return SUCCESS;
}

Result DvppProcess::ProcessResize()
{
    // resize pic size
    aclError ret = acldvppSetResizeConfigInterpolation(resizeConfig_, 0);
    ret = acldvppVpcResizeAsync(dvppChannelDesc_, resizeInputDesc_,
        resizeOutputDesc_, resizeConfig_, stream_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acldvppVpcResizeAsync failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    ret = aclrtSynchronizeStream(stream_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclrtSynchronizeStream failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    return SUCCESS;
}

void DvppProcess::DestroyResizeResource()
{
    if (resizeInputDesc_ != nullptr) {
        acldvppDestroyPicDesc(resizeInputDesc_);
        resizeInputDesc_ = nullptr;
    }

    if (resizeOutputDesc_ != nullptr) {
        acldvppDestroyPicDesc(resizeOutputDesc_);
        resizeOutputDesc_ = nullptr;
    }
}

void DvppProcess::DestroyResource()
{
    // resizeConfig_ is created in initResource
    if (resizeConfig_ != nullptr) {
        (void)acldvppDestroyResizeConfig(resizeConfig_);
        resizeConfig_ = nullptr;
    }

    if (dvppChannelDesc_ != nullptr) {
        aclError ret = acldvppDestroyChannel(dvppChannelDesc_);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("acldvppDestroyChannel failed, errorCode = %d", static_cast<int32_t>(ret));
        }

        (void)acldvppDestroyChannelDesc(dvppChannelDesc_);
        dvppChannelDesc_ = nullptr;
    }
}

Result DvppProcess::Process(PicDesc& srcImage)
{
    // pic resize
    Result ret = InitResizeInputDesc(srcImage);
    if (ret != SUCCESS) {
        ERROR_LOG("InitResizeInputDesc failed");
        DestroyResizeResource();
        return FAILED;
    }

    ret = InitResizeOutputDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("InitResizeOutputDesc failed");
        DestroyResizeResource();
        return FAILED;
    }

    ret = ProcessResize();
    if (ret != SUCCESS) {
        ERROR_LOG("ProcessResize failed");
        DestroyResizeResource();
        return FAILED;
    }

    DestroyResizeResource();

    INFO_LOG("Process dvpp success");
    return SUCCESS;
}
