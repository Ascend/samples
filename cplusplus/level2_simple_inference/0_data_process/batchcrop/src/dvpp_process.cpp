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
#include <stdio.h>
#include <vector>
#include <string>
#include <memory>
#include <stdlib.h>
#include "utils.h"
#include "acl/acl_base.h"
#include "acl/acl_rt.h"
#include "acl/acl_op.h"
#include "acl/acl_mdl.h"
#include "acl/ops/acl_dvpp.h"
#include "acl/ops/acl_cblas.h"
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

DvppProcess::DvppProcess(aclrtStream &stream)
    : stream_(stream), dvppChannelDesc_(nullptr), inputBatchPicDesc_(nullptr),
      outputBatchPicDesc_(nullptr), outputFileName_(nullptr), inputBatchSize_(0),
      outputBatchSize_(0), inputWidthStride_(0), inputHeightStride_(0),
      outputWidth_(0), outputHeight_(0)
{
}

DvppProcess::~DvppProcess()
{
    DestroyBatchCropResource();
}

void DvppProcess::SetBatchInputCrop(const vector<InputArea> &batchInput)
{
    batchInput_ = batchInput;
}

void DvppProcess::SetOutArea(int outputWidth, int outputHeight)
{
    outputWidth_ = outputWidth;
    outputHeight_ = outputHeight;
}

void DvppProcess::SetOutputFileName(const char *outputFileName)
{
    outputFileName_ = outputFileName;
}

Result DvppProcess::SetBatchSize(int inputBatchSize, int outputBatchSize)
{
    if (inputBatchSize < 1 || outputBatchSize < 1 || outputBatchSize < inputBatchSize) {
        ERROR_LOG("inputBatchSize or outputBatchSize set error inputBatchSize = %d, outputBatchSize = %d",
            inputBatchSize, outputBatchSize);
        return FAILED;
    }
    inputBatchSize_ = inputBatchSize;
    outputBatchSize_ = outputBatchSize;
    return SUCCESS;
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
    aclError aclRet = acldvppCreateChannel(dvppChannelDesc_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppCreateChannel failed, errorCode = %d", static_cast<int32_t>(aclRet));
        return FAILED;
    }
    INFO_LOG("dvpp init resource success");
    return SUCCESS;
}

uint32_t DvppProcess::CalculateInBufferSize(uint32_t inputWidth, uint32_t inputHeight,
                                            acldvppPixelFormat inputFormat)
{
    const uint32_t widthAlignment = 16;
    const uint32_t heightAlignment = 2;
    const uint32_t sizeAlignment = 3;
    const uint32_t sizeNum = 2;
    inputWidthStride_ = AlignSize(inputWidth, widthAlignment);
    inputHeightStride_ = AlignSize(inputHeight, heightAlignment);
    if (inputWidthStride_ == 0 || inputHeightStride_ == 0) {
        ERROR_LOG("AlignSize failed");
        return FAILED;
    }
    uint32_t inputBufferSize;
    switch (inputFormat) {
        case PIXEL_FORMAT_YUV_400:
        case PIXEL_FORMAT_YUV_SEMIPLANAR_420:
        case PIXEL_FORMAT_YVU_SEMIPLANAR_420:
            inputBufferSize = inputWidthStride_ * inputHeightStride_ * sizeAlignment / sizeNum;
            break;
        case PIXEL_FORMAT_YUV_SEMIPLANAR_422:
        case PIXEL_FORMAT_YVU_SEMIPLANAR_422:
            inputBufferSize = inputWidthStride_ * inputHeightStride_ * sizeNum;
            break;
        case PIXEL_FORMAT_YUV_SEMIPLANAR_444:
        case PIXEL_FORMAT_YVU_SEMIPLANAR_444:
            inputBufferSize = inputWidthStride_ * inputHeightStride_ * sizeAlignment;
            break;
        case PIXEL_FORMAT_YUYV_PACKED_422:
        case PIXEL_FORMAT_UYVY_PACKED_422:
        case PIXEL_FORMAT_YVYU_PACKED_422:
        case PIXEL_FORMAT_VYUY_PACKED_422:
            inputWidthStride_ = (inputWidth + widthAlignment - 1) * widthAlignment * sizeNum;
            inputBufferSize = inputWidthStride_ * inputHeightStride_;
            break;
        case PIXEL_FORMAT_YUV_PACKED_444:
        case PIXEL_FORMAT_RGB_888:
        case PIXEL_FORMAT_BGR_888:
            inputWidthStride_ = inputWidthStride_ * 3;
            inputBufferSize = inputWidthStride_ * inputHeightStride_;
            break;
        case PIXEL_FORMAT_ARGB_8888:
        case PIXEL_FORMAT_ABGR_8888:
        case PIXEL_FORMAT_RGBA_8888:
        case PIXEL_FORMAT_BGRA_8888:
            inputWidthStride_ = inputWidthStride_ * 4;
            inputBufferSize = inputWidthStride_ * inputHeightStride_;
            break;
        default:
            INFO_LOG("InputFormat = %d, dvpp not support.", static_cast<int32_t>(inputFormat));
            break;
    }
    return inputBufferSize;
}

void DvppProcess::InitCropArea()
{
    Area area;
    area.cropLeftOffset = 20;
    area.cropTopOffset = 20;
    batchCropArea_.push_back(area);

    area.cropLeftOffset = 30;
    area.cropTopOffset = 200;
    batchCropArea_.push_back(area);

    area.cropLeftOffset = 100;
    area.cropTopOffset = 300;
    batchCropArea_.push_back(area);

    area.cropLeftOffset = 200;
    area.cropTopOffset = 400;
    batchCropArea_.push_back(area);

    area.cropLeftOffset = 0;
    area.cropTopOffset = 600;
    batchCropArea_.push_back(area);

    area.cropLeftOffset = 230;
    area.cropTopOffset = 350;
    batchCropArea_.push_back(area);

    area.cropLeftOffset = 600;
    area.cropTopOffset = 188;
    batchCropArea_.push_back(area);

    area.cropLeftOffset = 500;
    area.cropTopOffset = 600;
    batchCropArea_.push_back(area);
}

Result DvppProcess::WriteOutputFile() {
    for (uint32_t i = 0; i < outputBatchSize_; i++) {
        acldvppPicDesc *outputDesc =nullptr;
        void *vpcBatchOutBufferDev = nullptr;
        string outputFileName = outputFileName_ + to_string(i);
        outputDesc = acldvppGetPicDesc(outputBatchPicDesc_, i);
        if (outputDesc == nullptr) {
            ERROR_LOG("acldvppGetPicDesc %d failed.", i);
            return FAILED;
        }
        vpcBatchOutBufferDev = acldvppGetPicDescData(outputDesc);
        uint32_t outputBufferSize = acldvppGetPicDescSize(outputDesc);
        if (Utils::WriteToFile(outputFileName.c_str(), vpcBatchOutBufferDev,
                               outputBufferSize) == SUCCESS) {
            INFO_LOG("write out to file %s success.", outputFileName.c_str());
        } else {
            ERROR_LOG("write out to file %s failed.", outputFileName.c_str());
        }
    }
    return SUCCESS;
}

Result DvppProcess::InitBatchCropInputDesc()
{
    uint32_t inputBufferSize;
    inputBatchPicDesc_ = acldvppCreateBatchPicDesc(inputBatchSize_);
    if (inputBatchPicDesc_ == nullptr) {
        ERROR_LOG("InitBatchCropInputDesc inBatchPicDesc failed");
        return FAILED;
    }

    if (inputBatchSize_ > batchInput_.size()) {
        inputBatchSize_ = batchInput_.size();
    }
    for (uint32_t i = 0; i < inputBatchSize_; i++) {
        void *inputBufferDev = nullptr;
        inputBufferSize = CalculateInBufferSize(batchInput_[i].inputWidth, batchInput_[i].inputHeight,
            batchInput_[i].inputFormat);
        if (Utils::VpcReadFileToDeviceMem(batchInput_[i].inputFileName, inputBufferDev,
            inputBufferSize) == FAILED) {
            ERROR_LOG("Read file %s to device mem failed.", batchInput_[i].inputFileName);
            return FAILED;
        }
        vecInPtr_.push_back(inputBufferDev);
        acldvppPicDesc *vpcInputDesc = acldvppGetPicDesc(inputBatchPicDesc_, i);
        (void)acldvppSetPicDescData(vpcInputDesc, inputBufferDev);
        (void)acldvppSetPicDescFormat(vpcInputDesc, batchInput_[i].inputFormat);
        (void)acldvppSetPicDescWidth(vpcInputDesc, batchInput_[i].inputWidth);
        (void)acldvppSetPicDescHeight(vpcInputDesc, batchInput_[i].inputHeight);
        (void)acldvppSetPicDescWidthStride(vpcInputDesc, inputWidthStride_);
        (void)acldvppSetPicDescHeightStride(vpcInputDesc, inputHeightStride_);
        (void)acldvppSetPicDescSize(vpcInputDesc, inputBufferSize);
        INFO_LOG("set inputDesc success.");
    }
    return SUCCESS;
}

Result DvppProcess::InitBatchCropOutputDesc()
{
    const uint32_t widthAlignment = 16;
    const uint32_t heightAlignment = 16;
    const uint32_t sizeAlignment = 3;
    const uint32_t sizeNum = 2;

    uint32_t outputWidthStride = AlignSize(outputWidth_, widthAlignment);
    uint32_t outputHeightStride = AlignSize(outputHeight_, heightAlignment);
    uint32_t outputBufferSize = outputWidthStride * outputHeightStride * sizeAlignment / sizeNum;

    outputBatchPicDesc_ = acldvppCreateBatchPicDesc(outputBatchSize_);
    if (outputBatchPicDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc outBatchPicDesc failed");
        return FAILED;
    }
    acldvppPicDesc *vpcOutputDesc = nullptr;
    for (uint32_t i = 0; i < outputBatchSize_; i++) {
        void *vpcBatchOutputBufferDev = nullptr;
        auto ret = acldvppMalloc(&vpcBatchOutputBufferDev, outputBufferSize);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("acldvppMalloc failed, size = %u, errorCode = %d.",
                outputBufferSize, static_cast<int32_t>(ret));
            return FAILED;
        }
        int outputindex = i / (outputBatchSize_ / inputBatchSize_);
        vecOutPtr_.push_back(vpcBatchOutputBufferDev);
        vpcOutputDesc = acldvppGetPicDesc(outputBatchPicDesc_, i);
        (void)acldvppSetPicDescData(vpcOutputDesc, vpcBatchOutputBufferDev);
        (void)acldvppSetPicDescFormat(vpcOutputDesc, batchInput_[outputindex].outputFormat);
        (void)acldvppSetPicDescWidth(vpcOutputDesc, outputWidth_);
        (void)acldvppSetPicDescHeight(vpcOutputDesc, outputHeight_);
        (void)acldvppSetPicDescWidthStride(vpcOutputDesc, outputWidthStride);
        (void)acldvppSetPicDescHeightStride(vpcOutputDesc, outputHeightStride);
        (void)acldvppSetPicDescSize(vpcOutputDesc, outputBufferSize);
    }
    return SUCCESS;
}

Result DvppProcess::ProcessBatchCrop()
{
    INFO_LOG("ProcessBatchCrop start.");
    const uint32_t oddNum = 1;
    const uint32_t cropSizeWidth = 224;
    const uint32_t cropSizeHeight = 224;
    uint32_t cropRightOffset = 0;
    uint32_t cropBottomOffset = 0;
    InitCropArea();  // init batchCropArea crop position
    size_t length = batchCropArea_.size();
    if (length < outputBatchSize_) {
        outputBatchSize_ = length;
    }
    std::unique_ptr<acldvppRoiConfig *[]>
        cropArea(new(std::nothrow) acldvppRoiConfig *[outputBatchSize_ * sizeof(acldvppRoiConfig *)]);

    for (uint32_t i = 0; i < outputBatchSize_; i++) {
        cropRightOffset = batchCropArea_[i].cropLeftOffset + cropSizeWidth - oddNum;
        cropBottomOffset = batchCropArea_[i].cropTopOffset + cropSizeHeight - oddNum;
        cropArea[i] = acldvppCreateRoiConfig(batchCropArea_[i].cropLeftOffset, cropRightOffset,
                                             batchCropArea_[i].cropTopOffset, cropBottomOffset);
        if (cropArea[i] == nullptr) {
            ERROR_LOG("acldvppCreateRoiConfig cropArea_ failed");
            return FAILED;
        }
    }
    Result ret = InitBatchCropInputDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("InitBatchCropInputDesc failed");
        return FAILED;
    }

    ret = InitBatchCropOutputDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("InitBatchCropOutputDesc failed");
        return FAILED;
    }

    // calculate total number of crop image
    uint32_t totalNum = 0;
    std::unique_ptr<uint32_t[]> roiNums(new (std::nothrow) uint32_t[inputBatchSize_]);
    if (roiNums != nullptr){
        for (int i = 0; i < inputBatchSize_; i++) {
            // crop number of images from one source image is outputBatchSize_ / inputBatchSize_
            roiNums[i] = outputBatchSize_ / inputBatchSize_;
            totalNum += roiNums[i];
        }
    }
    // crop number of images from last source image is
    // outputBatchSize_ / inputBatchSize_ + outputBatchSize_ % inputBatchSize_
    if (outputBatchSize_ % inputBatchSize_ != 0) {
        roiNums[inputBatchSize_ - 1] = (outputBatchSize_ - totalNum) + roiNums[inputBatchSize_ - 1];
    }

    aclError aclRet = acldvppVpcBatchCropAsync(dvppChannelDesc_, inputBatchPicDesc_,
                                               roiNums.get(), inputBatchSize_,
                                               outputBatchPicDesc_, cropArea.get(), stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("acldvppVpcBatchCropAsync failed, errorCode = %d", static_cast<int32_t>(aclRet));
        return FAILED;
    }

    aclRet = aclrtSynchronizeStream(stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("crop aclrtSynchronizeStream failed, errorCode = %d", static_cast<int32_t>(aclRet));
        return FAILED;
    }
    Result writeStatus = WriteOutputFile();
    if (writeStatus == FAILED){
        return FAILED;
    }
    for (uint32_t i = 0; i < outputBatchSize_; i++) {
        if (cropArea[i] != nullptr) {
            (void)acldvppDestroyRoiConfig(cropArea[i]);
            cropArea[i] = nullptr;
        }
    }
    INFO_LOG("ProcessBatchCrop success.");
    return SUCCESS;
}

void DvppProcess::DestroyBatchCropResource()
{
    INFO_LOG("DestroyBatchCropResource start.");
    for (auto ptr : vecInPtr_) {
        if (ptr != nullptr) {
            (void)acldvppFree(ptr);
        }
    }
    vecInPtr_.clear();
    if (inputBatchPicDesc_ != nullptr) {
        (void)acldvppDestroyBatchPicDesc(inputBatchPicDesc_);
        inputBatchPicDesc_ = nullptr;
    }
    for (auto ptr : vecOutPtr_) {
        if (ptr != nullptr) {
            (void)acldvppFree(ptr);
        }
    }
    vecOutPtr_.clear();
    if (outputBatchPicDesc_ != nullptr) {
        (void)acldvppDestroyBatchPicDesc(outputBatchPicDesc_);
        outputBatchPicDesc_ = nullptr;
    }
    if (dvppChannelDesc_ != nullptr) {
        aclError aclRet = acldvppDestroyChannel(dvppChannelDesc_);
        if (aclRet != ACL_ERROR_NONE) {
            ERROR_LOG("acldvppDestroyChannel failed, errorCode = %d", static_cast<int32_t>(aclRet));
        }
        (void)acldvppDestroyChannelDesc(dvppChannelDesc_);
        dvppChannelDesc_ = nullptr;
    }
    INFO_LOG("DestroyBatchCropResource end.");
    return;
}
