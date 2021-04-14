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
static const std::string g_outFile = "output/dvpp_venc.h264";
static FILE *g_outFileFp = nullptr;

VencProcess::VencProcess() : threadId_(0), vencChannelDesc_(nullptr), vencFrameConfig_(nullptr),
       inputPicputDesc_(nullptr), inBufferDev_(nullptr), inBufferSize_(0), format_(PIXEL_FORMAT_YUV_400),
       enType_(H264_MAIN_LEVEL)
{
}

VencProcess::~VencProcess()
{
    //DestroyResource();
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
        ERROR_LOG("venc encode frame failed, ret = %u.", retCode);
        return;
    }
}

Result VencProcess::InitResource(pthread_t threadId, acldvppPixelFormat format, acldvppStreamFormat enType,
    uint32_t width, uint32_t height)
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
        ERROR_LOG("fail to create threadId");
        return FAILED;
    }

    // set callback func
    ret = aclvencSetChannelDescCallback(vencChannelDesc_, callback);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set venc Callback");
        return FAILED;
    }

    // set output stream type
    ret = aclvencSetChannelDescEnType(vencChannelDesc_, enType_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set venc EnType");
        return FAILED;
    }

    // set input picture type
    ret = aclvencSetChannelDescPicFormat(vencChannelDesc_, format_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set venc PicFormat");
        return FAILED;
    }

    // set input picture width //不知道
    ret = aclvencSetChannelDescPicWidth(vencChannelDesc_, width);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set venc PicWidth");
        return FAILED;
    }

    // set input picture height //不知道
    ret = aclvencSetChannelDescPicHeight(vencChannelDesc_, height);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set venc PicWidth");
        return FAILED;
    }

    // set key frame interval
    ret = aclvencSetChannelDescKeyFrameInterval(vencChannelDesc_, 16);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set venc FrameInterval");
        return FAILED;
    }

    // create vdec channel
    ret = aclvencCreateChannel(vencChannelDesc_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to create venc channel");
        return FAILED;
    }

    vencFrameConfig_ = aclvencCreateFrameConfig();
    if (vencFrameConfig_ == nullptr) {
        ERROR_LOG("fail to create frame config");
        return FAILED;
    }

    g_outFileFp = fopen(g_outFile.c_str(), "wb+");
    if (g_outFileFp == nullptr) {
        ERROR_LOG("fopen out file %s failed, error=%s.\n", g_outFile.c_str(), strerror(errno));
        return FAILED;
    }

    // set frame config, no eos frame
    Result err = SetFrameConfig(0, 0);
    if (err != SUCCESS) {
        fclose(g_outFileFp);
        g_outFileFp = nullptr;
        ERROR_LOG("fail to set frame config");
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
        ERROR_LOG("fail to set PicDescData");
        return FAILED;
    }
    ret = acldvppSetPicDescSize(inputPicputDesc_, inBufferSize_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set PicDescSize");
        return FAILED;
    }
    return SUCCESS;
}

Result VencProcess::SetFrameConfig(uint8_t eos, uint8_t forceIFrame)
{
    // set eos
    aclError ret = aclvencSetFrameConfigEos(vencFrameConfig_, eos);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set eos, ret = %d", ret);
        return FAILED;
    }

    ret = aclvencSetFrameConfigForceIFrame(vencFrameConfig_, forceIFrame);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("fail to set venc ForceIFrame");
        return FAILED;
    }

    return SUCCESS;
}

Result VencProcess::Process()
{
    // create picture desc
    Result err = CreatePicDesc();
    if (err != SUCCESS) {
        ERROR_LOG("fail to create picture description");
        fclose(g_outFileFp);
        g_outFileFp = nullptr;
        return FAILED;
    }

    // send frame
    acldvppStreamDesc *outputStreamDesc = nullptr;
    aclError ret;

    ret = aclvencSendFrame(vencChannelDesc_, inputPicputDesc_,
        static_cast<void *>(outputStreamDesc), vencFrameConfig_, nullptr);
    if (ret != ACL_ERROR_NONE) {
        fclose(g_outFileFp);
        g_outFileFp = nullptr;
        ERROR_LOG("fail to send frame, ret=%u", ret);
        return FAILED;
    }
    return SUCCESS;
}

Result VencProcess::SendEosFrame()
{
    // set frame config, eos frame
    Result err = SetFrameConfig(1, 0);
    if (err != SUCCESS) {
        fclose(g_outFileFp);
        g_outFileFp = nullptr;
        ERROR_LOG("fail to set eos frame config");
        return FAILED;
    }

    // send eos frame
    aclError ret = aclvencSendFrame(vencChannelDesc_, nullptr,
    nullptr, vencFrameConfig_, nullptr);
    if (ret != ACL_ERROR_NONE) {
        fclose(g_outFileFp);
        g_outFileFp = nullptr;
        ERROR_LOG("fail to send eos frame, ret=%u", ret);
        return FAILED;
    }
    INFO_LOG("venc process success");
    return SUCCESS;
}

void VencProcess::DestroyResource()
{
    SendEosFrame();
    fclose(g_outFileFp);
    g_outFileFp = nullptr;

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
