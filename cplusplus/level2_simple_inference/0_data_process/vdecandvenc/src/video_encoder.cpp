/**
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2022. All rights reserved.
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

* File sample_process.cpp
* Description: handle acl resource
*/
#include <iostream>
#include "acl/acl.h"
#include "acllite/AclLiteModel.h"
#include "video_encoder.h"
using namespace std;
namespace {
    int g_rcMode = 2;
    int g_maxBitRate = 6000;
    int g_keyFrameInterval = 16;
}
static const std::string g_outFile = "output/dvpp_venc.h264";
static FILE *g_outFileFp = nullptr;
static bool g_runFlag = true;

void *ThreadFunc(aclrtContext sharedContext)
{
    if (sharedContext == nullptr) {
        ERROR_LOG("sharedContext can not be nullptr");
        return ((void *)(-1));
    }
    INFO_LOG("use shared context for this thread");
    aclError ret = aclrtSetCurrentContext(sharedContext);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("aclrtSetCurrentContext failed, errorCode = %d", static_cast<int32_t>(ret));
        return ((void *)(-1));
    }
    INFO_LOG("process callback thread start ");
    while (g_runFlag) {
        // Notice: timeout 1000ms
        (void)aclrtProcessReport(1000);
    }

    return nullptr;
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
        // encode success, then process output pic
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

VideoEncoder::VideoEncoder():deviceId_(0), context_(nullptr), stream_(nullptr), 
                             vencChannelDesc_(nullptr), vencFrameConfig_(nullptr), 
                             inputPicputDesc_(nullptr), outputStreamDesc_(nullptr),
                             inBufferDev_(nullptr)
{
    threadId_ = 0;
    inBufferSize_ = 0;
    format_ = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    enType_ = H264_MAIN_LEVEL;
    outFolder_ = (char*)"output/";
    isInited_ = false;
    InitAclResource();
}

VideoEncoder::~VideoEncoder()
{
    DestroyResource();
}

AclLiteError VideoEncoder::InitAclResource()
{
    /* 1. ACL初始化 */
    char aclConfigPath[32] = {'\0'};
    aclInit(aclConfigPath);

    /* 2. 运行管理资源申请,包括Device、Context、Stream */
    aclError ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Acl open device %d failed", deviceId_);
        return FAILED;
    }
    ACLLITE_LOG_INFO("Open device %d success", deviceId_);

    ret = aclrtCreateContext(&context_, deviceId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acl create context failed");
        return FAILED;
    }
    ACLLITE_LOG_INFO("create context success");

    // create stream
    ret = aclrtCreateStream(&stream_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acl create stream failed\n");
        return ACLLITE_ERROR_CREATE_STREAM;
    }
    ACLLITE_LOG_INFO("create stream success");

    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acl get run mode failed\n");
        return ACLLITE_ERROR_GET_RUM_MODE;
    }

    return ACLLITE_OK;
}

Result VideoEncoder::SetFrameConfig(uint8_t eos, uint8_t forceIFrame)
{
    // set eos
    aclError ret = aclvencSetFrameConfigEos(vencFrameConfig_, eos);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("fail to set eos, ret = %d", ret);
        return FAILED;
    }

    ret = aclvencSetFrameConfigForceIFrame(vencFrameConfig_, forceIFrame);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("fail to set venc ForceIFrame");
        return FAILED;
    }

    return SUCCESS;
}

AclLiteError VideoEncoder::InitVencResource(uint32_t width, uint32_t height)
{
    // create process callback thread
    int createThreadErr = pthread_create(&threadId_, nullptr, ThreadFunc, context_);
    if (createThreadErr != 0) {
        ERROR_LOG("create thread failed, err = %d", createThreadErr);
        return FAILED;
    }
    INFO_LOG("create process callback thread successfully, threadId = %lu", threadId_);

        // create vdec channelDesc
    vencChannelDesc_ = aclvencCreateChannelDesc();
    if (vencChannelDesc_ == nullptr) {
        ERROR_LOG("fail to create venc channel desc");
        return FAILED;
    }

    aclvencSetChannelDescThreadId(vencChannelDesc_, threadId_);
    aclvencSetChannelDescCallback(vencChannelDesc_, callback);
    aclvencSetChannelDescEnType(vencChannelDesc_, enType_);
    aclvencSetChannelDescPicFormat(vencChannelDesc_, format_);
    aclvencSetChannelDescPicWidth(vencChannelDesc_, width);
    aclvencSetChannelDescPicHeight(vencChannelDesc_, height);
    aclvencSetChannelDescKeyFrameInterval(vencChannelDesc_, g_keyFrameInterval);
    aclvencSetChannelDescRcMode(vencChannelDesc_, g_rcMode);
    aclvencSetChannelDescMaxBitRate(vencChannelDesc_, g_maxBitRate);
    // create vdec channel
    auto ret = aclvencCreateChannel(vencChannelDesc_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("fail to create venc channel");
        return FAILED;
    }
    vencFrameConfig_ = aclvencCreateFrameConfig();
    if (vencFrameConfig_ == nullptr) {
        ERROR_LOG("fail to create frame config");
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

AclLiteError VideoEncoder::Init(uint32_t width, uint32_t height)
{
    if (isInited_) {
        ACLLITE_LOG_INFO("instance is initied already!\n");
        return ACLLITE_OK;
    }

    // check output folder
    AclLiteError ret = Utils::CheckFolder(outFolder_);
    if (ret != ACLLITE_OK) {
        ERROR_LOG("mkdir out folder error.");
        return ret;
    }

    g_outFileFp = fopen(g_outFile.c_str(), "wb+");
    if (g_outFileFp == nullptr) {
        ERROR_LOG("fopen out file %s failed, error=%s.\n", g_outFile.c_str(), strerror(errno));
        return FAILED;
    }

    // ret = InitAclResource();
    // if (ret != ACLLITE_OK) {
    //     ACLLITE_LOG_ERROR("Init acl resource failed, error: %d", ret);
    //     return ret;
    // }

    ret = InitVencResource(width, height);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Init venc resource failed, error: %d", ret);
        return ret;
    }

    isInited_ = true;
    return ACLLITE_OK;
}

Result VideoEncoder::CreatePicDesc()
{
    inputPicputDesc_ = acldvppCreatePicDesc();
    if (inputPicputDesc_ == nullptr) {
        ERROR_LOG("fail to create output pic desc");
        return FAILED;
    }
    auto ret = acldvppSetPicDescData(inputPicputDesc_, inBufferDev_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("fail to set PicDescData");
        return FAILED;
    }
    ret = acldvppSetPicDescSize(inputPicputDesc_, inBufferSize_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("fail to set PicDescSize");
        return FAILED;
    }
    return SUCCESS;
}

AclLiteError VideoEncoder::VideoEncode(ImageData& image)
{
    inBufferSize_ = image.size;
    // Malloc input device memory
    auto aclRet = acldvppMalloc(&inBufferDev_, inBufferSize_);
    if (aclRet != ACL_SUCCESS) {
        ERROR_LOG("acl malloc dvpp data failed, inBufferSize_=%u, ret=%d.\n", inBufferDev_, inBufferSize_);
        return false;
    }
    if (runMode_ != ACL_DEVICE) {
        aclRet = aclrtMemcpy(inBufferDev_, inBufferSize_, image.data.get(), image.size, ACL_MEMCPY_HOST_TO_DEVICE);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("acl memcpy data to dev failed, image.size=%u, ret=%d.\n", image.size, aclRet);
            (void)acldvppFree(inBufferDev_);
            inBufferDev_ = nullptr;
            return false;
        }
    } else {
        // copy input to device memory
        aclRet = aclrtMemcpy(inBufferDev_, inBufferSize_, image.data.get(), image.size, ACL_MEMCPY_DEVICE_TO_DEVICE);
        if (aclRet != ACL_SUCCESS) {
            ERROR_LOG("acl memcpy data to dev failed, image.size=%u, ret=%d.\n", image.size, aclRet);
            (void)acldvppFree(inBufferDev_);
            inBufferDev_ = nullptr;
            return false;
        }
    }

    // create picture desc
    Result err = CreatePicDesc();
    if (err != SUCCESS) {
        ERROR_LOG("fail to create picture description");
        fclose(g_outFileFp);
        g_outFileFp = nullptr;
        return FAILED;
    }

    // send frame
    aclRet = aclvencSendFrame(vencChannelDesc_, inputPicputDesc_,
        static_cast<void *>(outputStreamDesc_), vencFrameConfig_, nullptr);
    if (aclRet != ACL_SUCCESS) {
        fclose(g_outFileFp);
        g_outFileFp = nullptr;
        ERROR_LOG("fail to send frame, aclRet=%u", aclRet);
        return FAILED;
    }

    /*************************************/
    //destory inBufferDev_
     (void)acldvppFree(inBufferDev_);
    inBufferDev_ = nullptr;
    /*************************************/
    return ACLLITE_OK;
}

AclLiteError VideoEncoder::DestroyVencResource()
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
    if (ret != ACL_SUCCESS) {
        fclose(g_outFileFp);
        g_outFileFp = nullptr;
        ERROR_LOG("fail to send eos frame, ret=%u", ret);
        return FAILED;
    }
    fclose(g_outFileFp);
    g_outFileFp = nullptr;
    INFO_LOG("send eos frame success");

    if (vencFrameConfig_ != nullptr) {
        (void)aclvencDestroyFrameConfig(vencFrameConfig_);
        vencFrameConfig_ = nullptr;
    }

    if (inputPicputDesc_ != nullptr) {
        (void)acldvppDestroyPicDesc(inputPicputDesc_);
        inputPicputDesc_ == nullptr;
    }

    if (vencChannelDesc_ != nullptr) {
        (void)aclvencDestroyChannel(vencChannelDesc_);
        (void)aclvencDestroyChannelDesc(vencChannelDesc_);
        vencChannelDesc_ = nullptr;
    }

    // destory thread
    g_runFlag = false;
    void *res = nullptr;
    int joinThreadErr = pthread_join(threadId_, &res);
    if (joinThreadErr != 0) {
        ERROR_LOG("join thread failed, threadId = %lu, err = %d", threadId_, joinThreadErr);
    } else {
        if ((uint64_t)res != 0) {
            ERROR_LOG("thread run failed. ret is %lu.", (uint64_t)res);
        }
    }
    INFO_LOG("destory venc callback thread success");
}

void VideoEncoder::DestroyResource()
{
    DestroyVencResource();
    aclError ret;
    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("destroy stream failed");
        }
        stream_ = nullptr;
    }

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("destroy context failed");
        }
        context_ = nullptr;
    }
    ACLLITE_LOG_INFO("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("reset device failed\n");
    }
    ACLLITE_LOG_INFO("end to reset device is %d\n", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("finalize acl failed\n");
    }
    ACLLITE_LOG_INFO("end to finalize acl");
}