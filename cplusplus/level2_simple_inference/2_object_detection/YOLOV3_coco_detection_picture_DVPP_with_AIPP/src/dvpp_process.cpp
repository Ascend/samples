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

#include "dvpp_process.h"
#include "acl/acl.h"
#include "utils.h"
using namespace std;

namespace
{
    uint32_t AlignSize(uint32_t origSize, uint32_t alignment) {
        if (alignment == 0) {
            return 0;
        }
        uint32_t alignmentH = alignment - 1;
        return (origSize + alignmentH) / alignment * alignment;
    }
}

DvppProcess::DvppProcess(aclrtStream& stream)
    : g_stream_(stream), g_dvppChannelDesc_(nullptr),
      g_resizeConfig_(nullptr), g_decodeOutDevBuffer_(nullptr), g_decodeOutputDesc_(nullptr), g_resizeInputDesc_(nullptr),
      g_resizeOutputDesc_(nullptr), g_inDevBuffer_(nullptr), g_inDevBufferSize_(0), g_jpegDecodeOutputSize_(0),
      g_decodeOutputWidth_(0), g_decodeOutputWidthStride_(0), g_decodeOutputHeight_(0), g_resizeOutBufferDev_(nullptr),
      g_resizeOutBufferSize_(0), g_modelInputWidth_(0), g_modelInputHeight_(0), g_resizeOutWidthStride_(0),
      g_resizeOutHeightStride_(0)
{
}

DvppProcess::~DvppProcess()
{
    DestroyResource();
    DestroyOutputPara();
}

Result DvppProcess::InitResource()
{
    g_dvppChannelDesc_ = acldvppCreateChannelDesc();
    if (g_dvppChannelDesc_ == nullptr) {
        ERROR_LOG("acldvppCreateChannelDesc failed");
        return FAILED;
    }

    aclError ret = acldvppCreateChannel(g_dvppChannelDesc_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acldvppCreateChannelAsync failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    g_resizeConfig_ = acldvppCreateResizeConfig();
    if (g_resizeConfig_ == nullptr) {
        ERROR_LOG("acldvppCreateResizeConfig failed");
        return FAILED;
    }

    INFO_LOG("dvpp init resource success");
    return SUCCESS;
}

void DvppProcess::SetInput(void *inDevBuffer, uint32_t inDevBufferSize, const PicDesc &picDesc)
{
    g_inDevBuffer_ = inDevBuffer;
    g_inDevBufferSize_ = inDevBufferSize;
    g_jpegDecodeOutputSize_ = picDesc.jpegDecodeSize;
}

void DvppProcess::GetDvppOutput(void **outputBuffer, int &outputSize)
{
    if (outputBuffer == nullptr) {
        ERROR_LOG("outputBuffer is nullptr");
        return;
    }
    *outputBuffer = g_resizeOutBufferDev_;
    outputSize = g_resizeOutBufferSize_;
    g_resizeOutBufferDev_ = nullptr;
    g_resizeOutBufferSize_ = 0;
}

Result DvppProcess::InitDvppOutputPara(int modelInputWidth, int modelInputHeight)
{
    if ((modelInputWidth <= 0) || (modelInputHeight <= 0)) {
        ERROR_LOG("InitInput para invalid, modelInputWidth %d, modelInputHeight %d",
                  modelInputWidth, modelInputHeight);
        return FAILED;
    }

    g_modelInputWidth_ = modelInputWidth;
    g_modelInputHeight_ = modelInputHeight;
    g_resizeOutWidthStride_ = AlignSize(modelInputWidth, 16); // 16-byte alignment
    g_resizeOutHeightStride_ = AlignSize(modelInputHeight, 2); // 2-byte alignment

    // output buffer, adjust the value based on the actual model
    g_resizeOutBufferSize_ = g_resizeOutWidthStride_ * g_resizeOutHeightStride_ * 3 / 2; // yuv format size
    aclError ret = acldvppMalloc(&g_resizeOutBufferDev_, g_resizeOutBufferSize_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acldvppMalloc resizeOutBuffer failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    return SUCCESS;
}

void DvppProcess::DestroyOutputPara()
{
    if (g_resizeOutBufferDev_ != nullptr) {
        (void)acldvppFree(g_resizeOutBufferDev_);
        g_resizeOutBufferDev_ = nullptr;
    }
}

Result DvppProcess::InitDecodeOutputDesc()
{
    aclError ret = acldvppMalloc(&g_decodeOutDevBuffer_, g_jpegDecodeOutputSize_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acldvppMalloc jpegOutBufferDev failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    g_decodeOutputDesc_ = acldvppCreatePicDesc();
    if (g_decodeOutputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc decodeOutputDesc failed");
        return FAILED;
    }

    (void)acldvppSetPicDescData(g_decodeOutputDesc_, g_decodeOutDevBuffer_);
    (void)acldvppSetPicDescFormat(g_decodeOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    (void)acldvppSetPicDescSize(g_decodeOutputDesc_, g_jpegDecodeOutputSize_);
    return SUCCESS;
}

Result DvppProcess::ProcessDecode()
{
    // decode to yuv format
    aclError ret = acldvppJpegDecodeAsync(g_dvppChannelDesc_, g_inDevBuffer_, g_inDevBufferSize_,
        g_decodeOutputDesc_, g_stream_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acldvppJpegDecodeAsync failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    ret = aclrtSynchronizeStream(g_stream_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclrtSynchronizeStream failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    // get yuv image width and height
    g_decodeOutputWidth_ = acldvppGetPicDescWidth(g_decodeOutputDesc_);
    g_decodeOutputHeight_ = acldvppGetPicDescHeight(g_decodeOutputDesc_);
    g_decodeOutputWidthStride_ = acldvppGetPicDescWidthStride(g_decodeOutputDesc_);
    return SUCCESS;
}

void DvppProcess::DestroyDecodeResource()
{
    if (g_decodeOutputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(g_decodeOutputDesc_);
        g_decodeOutputDesc_ = nullptr;
    }
}

Result DvppProcess::InitResizeInputDesc()
{
    uint32_t jpegOutWidthStride = g_decodeOutputWidthStride_; // 128-byte alignment on 310, 64-byte alignment on 310P
    uint32_t jpegOutHeightStride = AlignSize(g_decodeOutputHeight_, 16); // 16-byte alignment
    uint32_t jpegOutBufferSize = jpegOutWidthStride * jpegOutHeightStride * 3 / 2; // yuv format size
    g_resizeInputDesc_ = acldvppCreatePicDesc();
    if (g_resizeInputDesc_ == nullptr) {
        ERROR_LOG("InitResizeInputDesc failed");
        return FAILED;
    }

    (void)acldvppSetPicDescData(g_resizeInputDesc_, g_decodeOutDevBuffer_);
    (void)acldvppSetPicDescFormat(g_resizeInputDesc_, PIXEL_FORMAT_YVU_SEMIPLANAR_420);
    (void)acldvppSetPicDescWidth(g_resizeInputDesc_, g_decodeOutputWidth_);
    (void)acldvppSetPicDescHeight(g_resizeInputDesc_, g_decodeOutputHeight_);
    (void)acldvppSetPicDescWidthStride(g_resizeInputDesc_, jpegOutWidthStride);
    (void)acldvppSetPicDescHeightStride(g_resizeInputDesc_, jpegOutHeightStride);
    (void)acldvppSetPicDescSize(g_resizeInputDesc_, jpegOutBufferSize);
    return SUCCESS;
}

Result DvppProcess::InitResizeOutputDesc()
{
    g_resizeOutputDesc_ = acldvppCreatePicDesc();
    if (g_resizeOutputDesc_ == nullptr) {
        ERROR_LOG("acldvppCreatePicDesc failed");
        return FAILED;
    }

    (void)acldvppSetPicDescData(g_resizeOutputDesc_, g_resizeOutBufferDev_);
    (void)acldvppSetPicDescFormat(g_resizeOutputDesc_, PIXEL_FORMAT_YUV_SEMIPLANAR_420);
    (void)acldvppSetPicDescWidth(g_resizeOutputDesc_, g_modelInputWidth_);
    (void)acldvppSetPicDescHeight(g_resizeOutputDesc_, g_modelInputHeight_);
    (void)acldvppSetPicDescWidthStride(g_resizeOutputDesc_, g_resizeOutWidthStride_);
    (void)acldvppSetPicDescHeightStride(g_resizeOutputDesc_, g_resizeOutHeightStride_);
    (void)acldvppSetPicDescSize(g_resizeOutputDesc_, g_resizeOutBufferSize_);
    return SUCCESS;
}

Result DvppProcess::ProcessResize()
{
    // resize pic size
    aclError ret = acldvppSetResizeConfigInterpolation(g_resizeConfig_, 0);
    ret = acldvppVpcResizeAsync(g_dvppChannelDesc_, g_resizeInputDesc_,
        g_resizeOutputDesc_, g_resizeConfig_, g_stream_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acldvppVpcResizeAsync failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    ret = aclrtSynchronizeStream(g_stream_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclrtSynchronizeStream failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    return SUCCESS;
}

void DvppProcess::DestroyResizeResource()
{
    if (g_decodeOutDevBuffer_ != nullptr) {
        (void)acldvppFree(g_decodeOutDevBuffer_);
        g_decodeOutDevBuffer_ = nullptr;
    }

    if (g_resizeInputDesc_ != nullptr) {
        acldvppDestroyPicDesc(g_resizeInputDesc_);
        g_resizeInputDesc_ = nullptr;
    }

    if (g_resizeOutputDesc_ != nullptr) {
        acldvppDestroyPicDesc(g_resizeOutputDesc_);
        g_resizeOutputDesc_ = nullptr;
    }
}

void DvppProcess::DestroyResource()
{
    // g_resizeConfig_ is created in initResource
    if (g_resizeConfig_ != nullptr) {
        (void)acldvppDestroyResizeConfig(g_resizeConfig_);
        g_resizeConfig_ = nullptr;
    }

    if (g_dvppChannelDesc_ != nullptr) {
        aclError ret = acldvppDestroyChannel(g_dvppChannelDesc_);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("acldvppDestroyChannel failed, errorCode = %d", static_cast<int32_t>(ret));
        }

        (void)acldvppDestroyChannelDesc(g_dvppChannelDesc_);
        g_dvppChannelDesc_ = nullptr;
    }
}

Result DvppProcess::Process()
{
    // pic decode
    Result ret = InitDecodeOutputDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("InitDecodeOutputDesc failed");
        DestroyDecodeResource();
        return FAILED;
    }

    ret = ProcessDecode();
    if (ret != SUCCESS) {
        ERROR_LOG("ProcessDecode failed");
        DestroyDecodeResource();
        return FAILED;
    }

    DestroyDecodeResource();

    // pic resize
    ret = InitResizeInputDesc();
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