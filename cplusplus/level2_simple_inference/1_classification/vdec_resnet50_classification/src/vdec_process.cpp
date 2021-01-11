/**
* @file vdec_process.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include "vdec_process.h"
#include <string>

using namespace std;

VdecProcess::VdecProcess()
    : vdecChannelDesc_(nullptr), streamInputDesc_(nullptr),
      picOutputDesc_(nullptr), picOutBufferDev_(nullptr),
      inBufferDev_(nullptr), inBufferSize_(0), inputWidth_(0),
      inputHeight_(0), enType_(H265_MAIN_LEVEL), format_(PIXEL_FORMAT_YUV_400)
{
}

VdecProcess::~VdecProcess()
{
}

void callback(acldvppStreamDesc *input, acldvppPicDesc *output, void *userdata)
{
    uint64_t frameIndex = 0;
    if (userdata != nullptr) {
        // get frame index in callback process
        frameIndex = *((uint64_t *)userdata);
        INFO_LOG("start processing callback, frame index is %lu", frameIndex);
        free(userdata);
        userdata = nullptr;
    }
    // free input vdecInBufferDev and destroy stream desc
    if (input != nullptr) {
        void *vdecInBufferDev = acldvppGetStreamDescData(input);
        if (vdecInBufferDev != nullptr) {
            aclError ret = acldvppFree(vdecInBufferDev);
            if (ret != ACL_ERROR_NONE) {
                ERROR_LOG("fail to free input stream desc data, errorCode = %d", static_cast<int32_t>(ret));
            }
        }
        aclError ret = acldvppDestroyStreamDesc(input);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("fail to destroy input stream desc, errorCode = %d", static_cast<int32_t>(ret));
        }
    }

    // if output is nullptr, acldvppGetPicDescData return nullptr
    if (output != nullptr) {
        void *vdecOutBufferDev = acldvppGetPicDescData(output);
        // check whether decode success
        int retCode = acldvppGetPicDescRetCode(output);
        // decode fail, release resource and retuen
        if (retCode != 0) {
            ERROR_LOG("vdec decode frame failed, retCode = %d.", retCode);
            if (vdecOutBufferDev != nullptr) {
                aclError ret = acldvppFree(vdecOutBufferDev);
                if (ret != ACL_ERROR_NONE) {
                    ERROR_LOG("fail to free output pic desc data, errorCode = %d", static_cast<int32_t>(ret));
                }
            }
            aclError ret = acldvppDestroyPicDesc(output);
            if (ret != ACL_ERROR_NONE) {
                ERROR_LOG("fail to destroy output pic desc, errorCode = %d", static_cast<int32_t>(ret));
            }
            return;
        }

        // decode success, process output pic
        if (vdecOutBufferDev != nullptr) {
            uint32_t size = acldvppGetPicDescSize(output);
            std::string fileNameSave = "outdir/image" + std::to_string(frameIndex);
            if (!Utils::WriteDeviceMemoryToFile(fileNameSave.c_str(), vdecOutBufferDev, size)) {
                ERROR_LOG("write file failed.");
            }

            aclError ret = acldvppFree(vdecOutBufferDev);
            if (ret != ACL_ERROR_NONE) {
                ERROR_LOG("fail to free output pic desc data, errorCode = %d", static_cast<int32_t>(ret));
            }
        }
        aclError ret = acldvppDestroyPicDesc(output);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("fail to destroy output pic desc, errorCode = %d", static_cast<int32_t>(ret));
        }
    }

    INFO_LOG("success to process vdec callback %lu.", frameIndex);
}

Result VdecProcess::InitResource(uint64_t threadId, acldvppStreamFormat enType, acldvppPixelFormat format)
{
    threadId_ = threadId;
    enType_ = enType;
    format_ = format;
    // create vdec channelDesc
    vdecChannelDesc_ = aclvdecCreateChannelDesc();
    if (vdecChannelDesc_ == nullptr) {
        ERROR_LOG("fail to create vdec channel desc");
        return FAILED;
    }

    // channelId: 0-15
    aclError ret = aclvdecSetChannelDescChannelId(vdecChannelDesc_, 10);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set vdec ChannelId, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    ret = aclvdecSetChannelDescThreadId(vdecChannelDesc_, threadId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to create threadId, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    // callback func
    ret = aclvdecSetChannelDescCallback(vdecChannelDesc_, callback);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set vdec Callback, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    ret = aclvdecSetChannelDescEnType(vdecChannelDesc_, enType_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set vdec EnType, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    ret = aclvdecSetChannelDescOutPicFormat(vdecChannelDesc_, format_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set vdec OutPicFormat, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    // create vdec channel
    ret = aclvdecCreateChannel(vdecChannelDesc_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to create vdec channel, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    INFO_LOG("vdec init resource success");
    return SUCCESS;
}

void VdecProcess::SetInput(void *inBufferDev, uint32_t inBufferSize,
                          int inputWidth, int inputHeight)
{
    inBufferDev_ = inBufferDev;
    inBufferSize_ = inBufferSize;
    inputWidth_ = inputWidth;
    inputHeight_ = inputHeight;
}

Result VdecProcess::CreateStreamDesc()
{
    // create input stream desc
    streamInputDesc_ = acldvppCreateStreamDesc();
    if (streamInputDesc_ == nullptr) {
        ERROR_LOG("fail to create input stream desc");
        return FAILED;
    }

    aclError ret = acldvppSetStreamDescData(streamInputDesc_, inBufferDev_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set data for stream desc, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    // set size for dvpp stream desc
    ret = acldvppSetStreamDescSize(streamInputDesc_, inBufferSize_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set size for stream desc, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    return SUCCESS;
}

void VdecProcess::DestroyStreamDesc()
{
    if (inBufferDev_ != nullptr) {
        (void)acldvppFree(inBufferDev_);
        inBufferDev_ = nullptr;
    }
    if (streamInputDesc_ != nullptr) {
        (void)acldvppDestroyStreamDesc(streamInputDesc_);
        streamInputDesc_ = nullptr;
    }
}

Result VdecProcess::CreatePicDesc(size_t size)
{
    // Malloc output device memory
    aclError ret = acldvppMalloc(&picOutBufferDev_, size);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("aclrtMalloc failed, ret=%d", ret);
        return FAILED;
    }
    picOutputDesc_ = acldvppCreatePicDesc();
    if (picOutputDesc_ == nullptr) {
        ERROR_LOG("fail to create output pic desc");
        return FAILED;
    }
    ret = acldvppSetPicDescData(picOutputDesc_, picOutBufferDev_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set PicDescData, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    ret = acldvppSetPicDescSize(picOutputDesc_, size);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set PicDescSize, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    ret = acldvppSetPicDescFormat(picOutputDesc_, format_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set PicDescHeight, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    return SUCCESS;
}

void VdecProcess::DestroyPicDesc()
{
    if (picOutBufferDev_ != nullptr) {
        (void)acldvppFree(picOutBufferDev_);
        picOutBufferDev_ = nullptr;
    }
    if (picOutputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(picOutputDesc_);
        picOutputDesc_ = nullptr;
    }
}

Result VdecProcess::Process()
{
    // create stream desc
    Result err = CreateStreamDesc();
    if (err != SUCCESS) {
        DestroyStreamDesc();
        return FAILED;
    }
    // create pic desc
    size_t DataSize = (inputWidth_ * inputHeight_ * 3) / 2; // yuv format size
    err = CreatePicDesc(DataSize);
    if (err != SUCCESS) {
        DestroyStreamDesc();
        DestroyPicDesc();
        return FAILED;
    }

    // set frame index, callback function can use it
    static uint64_t index = 0;
    uint64_t *frameIndex = (uint64_t *)malloc(sizeof(uint64_t));
    if (frameIndex != nullptr) {
        *frameIndex = index++;
    }

    // send vdec frame
    aclError ret = aclvdecSendFrame(vdecChannelDesc_, streamInputDesc_,
                                    picOutputDesc_, nullptr, static_cast<void *>(frameIndex));
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to send frame, ret=%u", ret);
        DestroyStreamDesc();
        DestroyPicDesc();
        if (frameIndex != nullptr) {
            free(frameIndex);
            frameIndex = nullptr;
        }
        return FAILED;
    }
    return SUCCESS;
}

Result VdecProcess::SendVdecEos()
{
    // create stream desc
    acldvppStreamDesc *streamInputDesc = acldvppCreateStreamDesc();
    if (streamInputDesc == nullptr) {
        ERROR_LOG("fail to create input stream desc");
        return FAILED;
    }
    aclError ret = acldvppSetStreamDescEos(streamInputDesc, 1);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set eos for stream desc, errorCode = %d", static_cast<int32_t>(ret));
        (void)acldvppDestroyStreamDesc(streamInputDesc);
        return FAILED;
    }

    // send vdec eos frame. when all vdec callback are completed, aclvdecSendFrame can be returned.
    ret = aclvdecSendFrame(vdecChannelDesc_, streamInputDesc, nullptr, nullptr, nullptr);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to send eos frame, ret=%u", ret);
        (void)acldvppDestroyStreamDesc(streamInputDesc);
        return FAILED;
    }
    (void)acldvppDestroyStreamDesc(streamInputDesc);

    return SUCCESS;
}

void VdecProcess::DestroyResource()
{
    if (vdecChannelDesc_ != nullptr) {
        aclError ret = aclvdecDestroyChannel(vdecChannelDesc_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("acldvppDestroyChannel failed, , errorCode = %d", static_cast<int32_t>(ret));
        }
        (void)aclvdecDestroyChannelDesc(vdecChannelDesc_);
        vdecChannelDesc_ = nullptr;
    }
}
