/**
* @file venc_process.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include "venc_process.h"

using namespace std;
extern size_t g_vencCnt;
static const std::string g_outFile = "output/dvpp_venc_128x128.h265";
static FILE *g_outFileFp = nullptr;

VencProcess::VencProcess() : threadId_(0), vencChannelDesc_(nullptr), vencFrameConfig_(nullptr),
       inputPicputDesc_(nullptr), inBufferDev_(nullptr), inBufferSize_(0), format_(PIXEL_FORMAT_YUV_400),
       enType_(H265_MAIN_LEVEL)
{
}

VencProcess::~VencProcess()
{
    DestroyResource();
}

void callback(acldvppPicDesc *input, acldvppStreamDesc *outputStreamDesc, void *userdata)
{
    if (outputStreamDesc == nullptr) {
        ERROR_LOG("output is null");
        return;
    }
    // write to file
    void *outputDev = acldvppGetStreamDescData(outputStreamDesc);
    // check whether encode success
    uint32_t retCode = acldvppGetStreamDescRetCode(outputStreamDesc);
    if (retCode == 0) {
        //encode success, then process output pic
        uint32_t streamDescSize = acldvppGetStreamDescSize(outputStreamDesc);
        if (!Utils::WriteToFile(g_outFileFp, outputDev, streamDescSize)) {
            ERROR_LOG("write file:%s failed.", g_outFile.c_str());
        }
        INFO_LOG("success to callback, stream size:%u", streamDescSize);
    } else {
        // encode fail, retuen
        ERROR_LOG("venc encode frame failed, retCode = %u.", retCode);
        return;
    }
}

Result VencProcess::InitResource(uint64_t threadId, acldvppPixelFormat format, acldvppStreamFormat enType)
{
    threadId_ = threadId;
    format_ = format;
    enType_ = enType;
    // create vdec channelDesc
    vencChannelDesc_ = aclvencCreateChannelDesc();
    if (vencChannelDesc_ == nullptr) {
        ERROR_LOG("fail to create venc channel desc");
        return FAILED;
    }

    // set process callback thread
    auto ret = aclvencSetChannelDescThreadId(vencChannelDesc_, threadId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set threadId, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    // set callback func
    ret = aclvencSetChannelDescCallback(vencChannelDesc_, callback);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set venc Callback, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    // set output stream type
    ret = aclvencSetChannelDescEnType(vencChannelDesc_, enType_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set venc EnType, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    // set input picture type
    ret = aclvencSetChannelDescPicFormat(vencChannelDesc_, format_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set venc PicFormat, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    // set input picture width
    ret = aclvencSetChannelDescPicWidth(vencChannelDesc_, 128);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set venc PicWidth, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    // set input picture height
    ret = aclvencSetChannelDescPicHeight(vencChannelDesc_, 128);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set venc PicWidth, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    // set key frame interval
    ret = aclvencSetChannelDescKeyFrameInterval(vencChannelDesc_, 16);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set venc FrameInterval, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    // create vdec channel
    ret = aclvencCreateChannel(vencChannelDesc_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to create venc channel, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    vencFrameConfig_ = aclvencCreateFrameConfig();
    if (vencFrameConfig_ == nullptr) {
        ERROR_LOG("fail to create frame config");
        return FAILED;
    }

    INFO_LOG("venc init resource success");
    return SUCCESS;
}

void VencProcess::SetInput(void *inBufferDev, uint32_t inBufferSize)
{
    inBufferDev_ = inBufferDev;
    inBufferSize_ = inBufferSize;
}

Result VencProcess::CreatePicDesc()
{
    inputPicputDesc_ = acldvppCreatePicDesc();
    if (inputPicputDesc_ == nullptr) {
        ERROR_LOG("fail to create output pic desc");
        return FAILED;
    }
    auto ret = acldvppSetPicDescData(inputPicputDesc_, inBufferDev_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set PicDescData, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    ret = acldvppSetPicDescSize(inputPicputDesc_, inBufferSize_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set PicDescSize, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    return SUCCESS;
}

Result VencProcess::SetFrameConfig(uint8_t eos, uint8_t forceIFrame)
{
    // set eos
    aclError ret = aclvencSetFrameConfigEos(vencFrameConfig_, eos);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set eos, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    ret = aclvencSetFrameConfigForceIFrame(vencFrameConfig_, forceIFrame);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set venc ForceIFrame, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    return SUCCESS;
}

Result VencProcess::Process()
{
    g_outFileFp = fopen(g_outFile.c_str(), "wb+");
    if (g_outFileFp == nullptr) {
        ERROR_LOG("fopen out file %s failed, error=%s.\n", g_outFile.c_str(), strerror(errno));
        return FAILED;
    }
    // create picture desc
    Result err = CreatePicDesc();
    if (err != SUCCESS) {
        ERROR_LOG("fail to create picture description");
        fclose(g_outFileFp);
        g_outFileFp = nullptr;
        return FAILED;
    }

    // set frame config, no eos frame
    err = SetFrameConfig(0, 0);
    if (err != SUCCESS) {
        fclose(g_outFileFp);
        g_outFileFp = nullptr;
        ERROR_LOG("fail to set frame config");
        return FAILED;
    }

    // send fram
    aclError ret;
    while (g_vencCnt > 0) {
        ret = aclvencSendFrame(vencChannelDesc_, inputPicputDesc_, nullptr, vencFrameConfig_, nullptr);
        if (ret != ACL_ERROR_NONE) {
            fclose(g_outFileFp);
            g_outFileFp = nullptr;
            ERROR_LOG("fail to send frame, errorCode = %d", static_cast<int32_t>(ret));
            return FAILED;
        }
        g_vencCnt--;
    }

    // set frame config, eos frame
    err = SetFrameConfig(1, 0);
    if (err != SUCCESS) {
        fclose(g_outFileFp);
        g_outFileFp = nullptr;
        ERROR_LOG("fail to set eos frame config");
        return FAILED;
    }

    // send eos frame
    ret = aclvencSendFrame(vencChannelDesc_, nullptr,
        nullptr, vencFrameConfig_, nullptr);
    if (ret != ACL_ERROR_NONE) {
        fclose(g_outFileFp);
        g_outFileFp = nullptr;
        ERROR_LOG("fail to send eos frame, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }

    fclose(g_outFileFp);
    g_outFileFp = nullptr;

    INFO_LOG("venc process success");

    return SUCCESS;
}

void VencProcess::DestroyResource()
{
    if (vencChannelDesc_ != nullptr) {
        (void)aclvencDestroyChannel(vencChannelDesc_);
        (void)aclvencDestroyChannelDesc(vencChannelDesc_);
        vencChannelDesc_ = nullptr;
    }

    if (inputPicputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(inputPicputDesc_);
        inputPicputDesc_ == nullptr;
    }

    if (vencFrameConfig_ != nullptr) {
        (void)aclvencDestroyFrameConfig(vencFrameConfig_);
        vencFrameConfig_ = nullptr;
    }

    if (inBufferDev_ != nullptr) {
        (void)acldvppFree(inBufferDev_);
        inBufferDev_ = nullptr;
    }
}
