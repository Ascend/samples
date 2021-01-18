/**
* @file vdec_process.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include "atlas_utils.h"
#include "vdec_process.h"

using namespace std;

namespace {
    const uint32_t kFrameWidthMax = 4096;
    const uint32_t kFrameHeightMax = 4096;
}

VdecProcess::VdecProcess(int channelId, uint32_t width, uint32_t height, 
                         int type, aclvdecCallback callback, uint32_t outFormat)
:channelId_(channelId)
,format_(outFormat)
,enType_(type)
,frameWidth_(width)
,frameHeight_(height)
,callback_(callback)
,isExit_(false)
,isReleased_(false){
    alignWidth_ = ALIGN_UP16(frameWidth_);
    alignHeight_ = ALIGN_UP2(frameHeight_);
    outputPicSize_ = YUV420SP_SIZE(alignWidth_, alignHeight_);

    vdecChannelDesc_ = nullptr;
    inputStreamDesc_ = nullptr;
    outputPicDesc_ = nullptr;
    outputPicBuf_ = nullptr;
   
    ATLAS_LOG_INFO("VDEC width %d, height %d", frameWidth_, frameHeight_);
}

VdecProcess::~VdecProcess(){
    DestroyResource();
}

void VdecProcess::DestroyResource(){   
    if (isReleased_) return;

    aclError ret;
    if (vdecChannelDesc_ != nullptr) {
        ret = aclvdecDestroyChannel(vdecChannelDesc_);
        if (ret != ACL_ERROR_NONE) {
            ATLAS_LOG_ERROR("Vdec destroy channel failed, errorno: %d", ret);
        }
        aclvdecDestroyChannelDesc(vdecChannelDesc_);
        vdecChannelDesc_ = nullptr;      
    }

    UnsubscribReportThread();

    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_ERROR_NONE) {
            ATLAS_LOG_ERROR("Vdec destroy stream failed");
        }
        stream_ = nullptr;
        ATLAS_LOG_INFO("Destory report thread success.");
    }

    isReleased_ = true;
    ATLAS_LOG_INFO("Destory vdec resource success.");
}

void* VdecProcess::SubscribeReportThreadFunc(void *arg) {
    ATLAS_LOG_INFO("Start vdec subscribe thread...");

    // Notice: create context for this thread
    int deviceId = 0;
    aclrtContext context = nullptr;
    aclError ret = aclrtCreateContext(&context, deviceId);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Vdec subscribe thread create"
                        " context failed, errorno:%d.", ret);
        return (void*)ATLAS_ERROR_CREATE_ACL_CONTEXT;
    }
    
    VdecProcess* thisPtr = (VdecProcess*)arg;
    while (!thisPtr->IsExit()) {
        // Notice: timeout 1000ms
        aclrtProcessReport(1000);
    }

    ret = aclrtDestroyContext(context);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Vdec subscribe thread destroy"
                        " context failed, errorno:%d.", ret);
    }

    ATLAS_LOG_INFO("Vdec subscribe thread exit!");

    return (void*)ATLAS_OK;
}

void VdecProcess::UnsubscribReportThread() {
    if ((subscribeThreadId_ == 0) || (stream_ == nullptr)) return;

    (void)aclrtUnSubscribeReport(static_cast<uint64_t>(subscribeThreadId_),
                                                       stream_);
    // destory thread
    isExit_ = true;

    void *res = nullptr;
    int joinThreadErr = pthread_join(subscribeThreadId_, &res);
    if (joinThreadErr) {
        ATLAS_LOG_ERROR("Join thread failed, threadId = %lu, err = %d", 
                        subscribeThreadId_, joinThreadErr);
    } else {
        if ((uint64_t)res != 0) {
            ATLAS_LOG_ERROR("thread run failed. ret is %lu.", (uint64_t)res);
        }
    }
    subscribeThreadId_ = 0;
    ATLAS_LOG_INFO("Destory report thread success.");
}

AtlasError VdecProcess::Init() {
    ATLAS_LOG_INFO("Vdec process init start...");
    
    aclError aclRet = aclrtCreateStream(&stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Vdec create stream failed, errorno:%d", aclRet);
        return ATLAS_ERROR_CREATE_STREAM;
    }
    ATLAS_LOG_INFO("Vdec create stream ok");
         
    int ret = pthread_create(&subscribeThreadId_, nullptr, 
                             SubscribeReportThreadFunc, (void *)this);
    if (ret) {
        ATLAS_LOG_ERROR("Start vdec subscribe thread failed, return:%d", ret);
        return ATLAS_ERROR_CREATE_THREAD;
    }
    (void)aclrtSubscribeReport(static_cast<uint64_t>(subscribeThreadId_), 
                               stream_);

    ret = CreateVdecChannelDesc();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Create vdec channel failed");
        return ret;
    }

    return ATLAS_OK;
}

AtlasError VdecProcess::CreateVdecChannelDesc() {
    vdecChannelDesc_ = aclvdecCreateChannelDesc();
    if (vdecChannelDesc_ == nullptr) {
        ATLAS_LOG_ERROR("Create vdec channel desc failed");
        return ATLAS_ERROR_CREATE_DVPP_CHANNEL_DESC;
    }

   // channelId: 0-15
    aclError ret = aclvdecSetChannelDescChannelId(vdecChannelDesc_,
                                                  channelId_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Set vdec channel id to %d failed, errorno:%d", 
                      channelId_, ret);
        return ATLAS_ERROR_SET_VDEC_CHANNEL_ID;
    }

    ret = aclvdecSetChannelDescThreadId(vdecChannelDesc_, subscribeThreadId_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Set vdec channel thread id failed, errorno:%d", ret);
        return ATLAS_ERROR_SET_VDEC_CHANNEL_THREAD_ID;
    }

    // callback func
    ret = aclvdecSetChannelDescCallback(vdecChannelDesc_, callback_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Set vdec channel callback failed, errorno:%d", ret);
        return ATLAS_ERROR_SET_VDEC_CALLBACK;
    }

    ret = aclvdecSetChannelDescEnType(vdecChannelDesc_, 
                                      static_cast<acldvppStreamFormat>(enType_));
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Set vdec channel entype failed, errorno:%d", ret);
        return ATLAS_ERROR_SET_VDEC_ENTYPE;
    }

    ret = aclvdecSetChannelDescOutPicFormat(vdecChannelDesc_, 
                                            static_cast<acldvppPixelFormat>(format_));
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Set vdec channel pic format failed, errorno:%d", ret);
        return ATLAS_ERROR_SET_VDEC_PIC_FORMAT;
    }

    // create vdec channel
    ATLAS_LOG_INFO("Start create vdec channel by desc...");
    ret = aclvdecCreateChannel(vdecChannelDesc_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("fail to create vdec channel");
        return ATLAS_ERROR_CREATE_VDEC_CHANNEL;
    }
    ATLAS_LOG_INFO("Create vdec channel ok");

    return ATLAS_OK;
}

AtlasError VdecProcess::CreateInputStreamDesc(shared_ptr<FrameData> frameData)
{  
   inputStreamDesc_ = acldvppCreateStreamDesc();
    if (inputStreamDesc_ == nullptr) {
        ATLAS_LOG_ERROR("Create input stream desc failed");
        return ATLAS_ERROR_CREATE_STREAM_DESC;
    }

    aclError ret;
    //如果是最后一帧,则给dvpp vdec送一个结束帧
    if (frameData->isFinished) {
        ret = acldvppSetStreamDescEos(inputStreamDesc_, 1);
        if (ret != ACL_ERROR_NONE) {
            ATLAS_LOG_ERROR("Set EOS to input stream desc failed, errorno:%d", ret);
            return ATLAS_ERROR_SET_STREAM_DESC_EOS;
        }
        return ATLAS_OK;
    }

    ret = acldvppSetStreamDescData(inputStreamDesc_, frameData->data);                                   
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Set input stream data failed, errorno:%d", ret);
        return ATLAS_ERROR_SET_STREAM_DESC_DATA;
    }
    
    // set size for dvpp stream desc
    ret = acldvppSetStreamDescSize(inputStreamDesc_, frameData->size);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Set input stream size failed, errorno:%d", ret);
        return ATLAS_ERROR_SET_STREAM_DESC_SIZE;
    }


    return ATLAS_OK;
}

AtlasError VdecProcess::CreateOutputPicDesc(size_t size)
{
    // Malloc output device memory
    aclError ret = acldvppMalloc(&outputPicBuf_, size);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Malloc vdec output buffer failed when create "
                      "vdec output desc, errorno:%d", ret);
        return ATLAS_ERROR_MALLOC_DVPP;
    }

    outputPicDesc_ = acldvppCreatePicDesc();
    if (outputPicDesc_ == nullptr) {
        ATLAS_LOG_ERROR("Create vdec output pic desc failed");
        return ATLAS_ERROR_CREATE_PIC_DESC;
    }

    ret = acldvppSetPicDescData(outputPicDesc_, outputPicBuf_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Set vdec output pic desc data failed, errorno:%d", ret);
        return ATLAS_ERROR_SET_PIC_DESC_DATA;
    }

    ret = acldvppSetPicDescSize(outputPicDesc_, size);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Set vdec output pic size failed, errorno:%d", ret);
        return ATLAS_ERROR_SET_PIC_DESC_SIZE;
    }

    ret = acldvppSetPicDescFormat(outputPicDesc_, 
                                  static_cast<acldvppPixelFormat>(format_));
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Set vdec output pic format failed, errorno:%d", ret);
        return ATLAS_ERROR_SET_PIC_DESC_FORMAT;
    }

    return ATLAS_OK;
}

AtlasError VdecProcess::Process(shared_ptr<FrameData> frameData, void* userData)
{
    //创建输入desc
    AtlasError atlRet = CreateInputStreamDesc(frameData);
    if (atlRet != ATLAS_OK) {
        ATLAS_LOG_ERROR("Create stream desc failed");
        return atlRet;
    }
    //创建输出desc
    atlRet = CreateOutputPicDesc(outputPicSize_);
    if (atlRet != ATLAS_OK) {
        ATLAS_LOG_ERROR("Create pic desc failed");
        return atlRet;
    }
    //将数据送到dvpp vdec解码,解码后dvpp vdec会调用注册的回调函数
    //(VideoDecode::DvppVdecCallback)处理
    aclError ret = aclvdecSendFrame(vdecChannelDesc_, inputStreamDesc_,
                                    outputPicDesc_, nullptr, userData);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Send frame to vdec failed, errorno:%d", ret);
        return ATLAS_ERROR_VDEC_SEND_FRAME;
    }

    return ATLAS_OK;
}

AtlasError VdecProcess::SetFormat(uint32_t format) {
    if ((format != PIXEL_FORMAT_YUV_SEMIPLANAR_420) ||
        (format != PIXEL_FORMAT_YVU_SEMIPLANAR_420)) {
        ATLAS_LOG_ERROR("Set video decode output image format to %d failed, "
            "only support %d(YUV420SP NV12) and %d(YUV420SP NV21)", 
            format,
            (int)PIXEL_FORMAT_YUV_SEMIPLANAR_420,
            (int)PIXEL_FORMAT_YVU_SEMIPLANAR_420);
        return ATLAS_ERROR_VDEC_FORMAT_INVALID;
    }

    format_ = format;
    ATLAS_LOG_INFO("Set video decode output image format to %d ok", format);

    return ATLAS_OK;
}

AtlasError VdecProcess::VideoParamCheck() {
    if (((frameWidth_ == 0) || (frameWidth_ > kFrameWidthMax)) ||
        ((frameHeight_ == 0) || (frameHeight_ > kFrameHeightMax)) ||
        ((format_ != PIXEL_FORMAT_YUV_SEMIPLANAR_420) &&
         (format_ != PIXEL_FORMAT_YVU_SEMIPLANAR_420)) ||
        (enType_ > (uint32_t)H264_HIGH_LEVEL)) {
        return ATLAS_ERROR_VDEC_INVALID_PARAM;
    }

    return ATLAS_OK;
}


