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
using namespace std;

DvppProcess::DvppProcess(aclrtStream& stream) : stream_(stream),
    dvppChannelDesc_(nullptr), resizeConfig_(nullptr),
    resizeInputDesc_(nullptr), resizeOutputDesc_(nullptr),
    resizeOutBufferDev_(nullptr), picOutBufferDev_(nullptr),
    resizeInBufferSize_(0),resizeOutBufferSize_(0),
    inputWidth_(0), inputHeight_(0),modelInputWidth_(0),
    modelInputHeight_(0), format_(PIXEL_FORMAT_YUV_400)
{
}

DvppProcess::~DvppProcess()
{
    DestroyResizeResource();
    DestroyOutputPara();
    DestroyResource();
}

Result DvppProcess::InitResource()
{
    // create vpc channel description
    dvppChannelDesc_ = acldvppCreateChannelDesc();
    if (dvppChannelDesc_ == nullptr) {
        ERROR_LOG("acldvppCreateChannelDesc failed");
        return FAILED;
    }

    // create vpc channel
    aclError ret = acldvppCreateChannel(dvppChannelDesc_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppCreateChannel failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    // create vpc resize config
    resizeConfig_ = acldvppCreateResizeConfig();
    if (resizeConfig_ == nullptr) {
        ERROR_LOG("acldvppCreateResizeConfig failed");
        return FAILED;
    }

    INFO_LOG("dvpp init dvpp resource success");
    return SUCCESS;
}

void DvppProcess::SetInput(int inputWidth, int inputHeight, acldvppPixelFormat format)
{
    inputWidth_ = inputWidth;
    inputHeight_ = inputHeight;
    format_ = format;
}

void DvppProcess::GetOutput(void **outputBuffer, uint32_t &outputSize)
{
    *outputBuffer = resizeOutBufferDev_;
    outputSize = resizeOutBufferSize_;
    resizeOutBufferDev_ = nullptr;
    resizeOutBufferSize_ = 0;
}

Result DvppProcess::InitOutputPara(int modelInputWidth, int modelInputHeight)
{
    if ((modelInputWidth <= 0) || (modelInputHeight <= 0)) {
        ERROR_LOG("InitInput para invalid, modelInputWidth = %d, modelInputHeight = %d",
            modelInputWidth, modelInputHeight);
        return FAILED;
    }

    modelInputWidth_ = modelInputWidth;
    modelInputHeight_ = modelInputHeight;

    // output buffer, adjust the value based on the actual model
    int resizeOutWidth = modelInputWidth_;
    int resizeOutHeight = modelInputHeight_;
    int resizeOutWidthStride = (resizeOutWidth + 15) / 16 * 16; // 16-byte alignment
    int resizeOutHeightStride = (resizeOutHeight + 1) / 2 * 2; // 2-byte alignment
    resizeOutBufferSize_ = resizeOutWidthStride * resizeOutHeightStride * 3 / 2; // yuv format size
    aclError ret = acldvppMalloc(&resizeOutBufferDev_, resizeOutBufferSize_);
    if (ret != ACL_ERROR_NONE) {
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
    if (picOutBufferDev_ != nullptr) {
        (void)acldvppFree(picOutBufferDev_);
        picOutBufferDev_ = nullptr;
    }
}

Result DvppProcess::InitResizeInputDesc()
{
    uint32_t jpegOutWidthStride = (inputWidth_ + 15) / 16 * 16; // 16-byte alignment
    uint32_t jpegOutHeightStride = (inputHeight_ + 1) / 2 * 2; // 2-byte alignment
    uint32_t jpegOutBufferSize = jpegOutWidthStride * jpegOutHeightStride * 3 / 2; // yuv format size
    resizeInputDesc_ = acldvppCreatePicDesc();
    if (resizeInputDesc_ == nullptr) {
        ERROR_LOG("InitResizeInputDesc failed");
        return FAILED;
    }
    if (jpegOutBufferSize != resizeInBufferSize_) {
        ERROR_LOG("jpegOutBufferSize [%u] != resizeInBufferSize_ [%u]",
                   jpegOutBufferSize, resizeInBufferSize_);
        return FAILED;
    }

    (void)acldvppSetPicDescData(resizeInputDesc_, picOutBufferDev_);
    (void)acldvppSetPicDescFormat(resizeInputDesc_, format_);
    (void)acldvppSetPicDescWidth(resizeInputDesc_, inputWidth_);
    (void)acldvppSetPicDescHeight(resizeInputDesc_, inputHeight_);
    (void)acldvppSetPicDescWidthStride(resizeInputDesc_, jpegOutWidthStride);
    (void)acldvppSetPicDescHeightStride(resizeInputDesc_, jpegOutHeightStride);
    (void)acldvppSetPicDescSize(resizeInputDesc_, jpegOutBufferSize);
    return SUCCESS;
}

Result DvppProcess::InitResizeOutputDesc()
{
    // adjust based on the actual model
    int resizeOutputWidthStride = (modelInputWidth_+ 15) / 16 * 16; // 16-byte alignment
    int resizeOutputHeightStride = (modelInputHeight_ + 1) / 2 * 2; // 2-byte alignment
    resizeOutputDesc_ = acldvppCreatePicDesc();
    if (resizeOutputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc failed");
        return FAILED;
    }

    (void)acldvppSetPicDescData(resizeOutputDesc_, resizeOutBufferDev_);
    (void)acldvppSetPicDescFormat(resizeOutputDesc_, format_);
    (void)acldvppSetPicDescWidth(resizeOutputDesc_, modelInputWidth_);
    (void)acldvppSetPicDescHeight(resizeOutputDesc_, modelInputHeight_);
    (void)acldvppSetPicDescWidthStride(resizeOutputDesc_, resizeOutputWidthStride);
    (void)acldvppSetPicDescHeightStride(resizeOutputDesc_, resizeOutputHeightStride);
    (void)acldvppSetPicDescSize(resizeOutputDesc_, resizeOutBufferSize_);

    return SUCCESS;
}

Result DvppProcess::ProcessResize()
{
    // resize pic size
    aclError ret = acldvppVpcResizeAsync(dvppChannelDesc_, resizeInputDesc_,
        resizeOutputDesc_, resizeConfig_, stream_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppVpcResizeAsync failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    ret = aclrtSynchronizeStream(stream_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("aclrtSynchronizeStream failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    return SUCCESS;
}

void DvppProcess::DestroyResizeResource()
{
    if (resizeOutputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(resizeOutputDesc_);
        resizeOutputDesc_ = nullptr;
    }
    if (resizeInputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(resizeInputDesc_);
        resizeInputDesc_ = nullptr;
    }
}

void DvppProcess::DestroyResource()
{
    if (resizeConfig_ != nullptr) {
        (void)acldvppDestroyResizeConfig(resizeConfig_);
        resizeConfig_ = nullptr;
    }

    if (dvppChannelDesc_ != nullptr) {
        aclError ret = acldvppDestroyChannel(dvppChannelDesc_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("acldvppDestroyChannel failed, errorCode = %d", static_cast<int32_t>(ret));
        }
        (void)acldvppDestroyChannelDesc(dvppChannelDesc_);
        dvppChannelDesc_ = nullptr;
    }
}

Result DvppProcess::Process(void *buffer, uint32_t size)
{
    picOutBufferDev_ = buffer;
    resizeInBufferSize_ = size;
    Result ret = InitResizeInputDesc();
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