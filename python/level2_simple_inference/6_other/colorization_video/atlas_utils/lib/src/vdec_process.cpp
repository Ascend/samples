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
                         int type, aclvdecCallback callback)
: status_(VDEC_UNINITED)
,channelId_(channelId)
,format_(PIXEL_FORMAT_YUV_SEMIPLANAR_420)
,enType_(type)
,frameWidth_(width)
,frameHeight_(height)
,callback_(callback)
,isReleased_(false){
    alignWidth_ = ALIGN_UP16(frameWidth_);
    alignHeight_ = ALIGN_UP2(frameHeight_);
    outputPicSize_ = YUV420SP_SIZE(alignWidth_, alignHeight_);
    outputBufSize_ = outputPicSize_ * 2;

    vdecChannelDesc_ = nullptr;
    inputStreamDesc_ = nullptr;
    outputPicDesc_ = nullptr;
    outputPicBuf_ = nullptr;
   
    ASC_LOG_INFO("VDEC width %u, height %u", frameWidth_, frameHeight_);
}

VdecProcess::~VdecProcess(){
    destroy_resource();
}

void VdecProcess::destroy_resource(){
    if (isReleased_) {
        return;
    }

    aclError ret;
    if (vdecChannelDesc_ != nullptr) {
        ret = aclvdecDestroyChannel(vdecChannelDesc_);
        if (ret != ACL_ERROR_NONE) {
            ASC_LOG_ERROR("Vdec destroy channel failed, errorno: %d", ret);
        }
        aclvdecDestroyChannelDesc(vdecChannelDesc_);
        vdecChannelDesc_ = nullptr;
    }

    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_ERROR_NONE) {
            ASC_LOG_ERROR("Vdec destroy stream failed");
        }
        stream_ = nullptr;
    }

    isReleased_ = true;
}

void* VdecProcess::subscibe_report_thread_func(void *arg)
{
    ASC_LOG_INFO("Start vdec subscribe thread...");

    // Notice: create context for this thread
    int deviceId = 0;
    VdecProcess* vdec = (VdecProcess*)arg;

    aclrtContext context = nullptr;
    aclError ret = aclrtCreateContext(&context, deviceId);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Vdec subscribe thread create context failed, errorno:%d.", ret);
        return ((void*)FAILED);
    }

    while (!vdec->is_decode_finish()) {
        // Notice: timeout 1000ms
        aclrtProcessReport(1000);
    }

    ret = aclrtDestroyContext(context);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Vdec subscribe thread destroy context failed, errorno:%d.", ret);
    }
    vdec->set_status(VDEC_REPORT_THREAD_EXIT);
    ASC_LOG_INFO("Vdec subscribe thread exit!");
    return (void*)SUCCESS;
}

Result VdecProcess::Init() {
    ASC_LOG_INFO("Vdec process init start...");

    if (SUCCESS != init_resource()) {
        ASC_LOG_ERROR("Create vdec channel failed");
        return FAILED;
    }
         
    int ret = pthread_create(&subscribeThreadId_, nullptr, 
                             subscibe_report_thread_func, (void *)this);
    if (ret) {
        ASC_LOG_ERROR("Start vdec subscribe thread failed, return:%d", ret);
        return FAILED;
    }
    (void)aclrtSubscribeReport(static_cast<uint64_t>(subscribeThreadId_), stream_);

    if (SUCCESS != creat_vdec_channel_desc()) {
        ASC_LOG_ERROR("Create vdec channel failed");
        return FAILED;
    }

    status_ = VDEC_INITED;
    return SUCCESS;
}

Result VdecProcess::init_resource()
{
    aclError aclRet = aclrtCreateStream(&stream_);
    if (aclRet != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Vdec create stream failed, errorno:%d", aclRet);
        return FAILED;
    }
    ASC_LOG_INFO("Vdec create stream ok");

    aclRet = aclrtGetRunMode(&runMode_);
    if (aclRet != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("acl get run mode failed");
        return FAILED;
    }

    ASC_LOG_INFO("Vdec init resource ok");
    return SUCCESS;
}

Result VdecProcess::creat_vdec_channel_desc() {
    vdecChannelDesc_ = aclvdecCreateChannelDesc();
    if (vdecChannelDesc_ == nullptr) {
        ASC_LOG_ERROR("Create vdec channel desc failed");
        return FAILED;
    }

   // channelId: 0-15
    aclError ret = aclvdecSetChannelDescChannelId(vdecChannelDesc_, channelId_);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Set vdec channel id to %d failed, errorno:%d", 
                      channelId_, ret);
        return FAILED;
    }

    ret = aclvdecSetChannelDescThreadId(vdecChannelDesc_, subscribeThreadId_);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Set vdec channel thread id failed, errorno:%d", ret);
        return FAILED;
    }

    // callback func
    ret = aclvdecSetChannelDescCallback(vdecChannelDesc_, callback_);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Set vdec channel callback failed, errorno:%d", ret);
        return FAILED;
    }

    ret = aclvdecSetChannelDescEnType(vdecChannelDesc_, 
                                      static_cast<acldvppStreamFormat>(enType_));
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Set vdec channel entype failed, errorno:%d", ret);
        return FAILED;
    }

    ret = aclvdecSetChannelDescOutPicFormat(vdecChannelDesc_, 
                                            static_cast<acldvppPixelFormat>(format_));
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Set vdec channel pic format failed, errorno:%d", ret);
        return FAILED;
    }

    // create vdec channel
    ASC_LOG_INFO("Start create vdec channel by desc...");
    ret = aclvdecCreateChannel(vdecChannelDesc_);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("fail to create vdec channel");
        return FAILED;
    }
    ASC_LOG_INFO("Create vdec channel ok");

    return SUCCESS;
}

Result VdecProcess::create_input_stream_desc(shared_ptr<FrameData> frameData)
{
    aclError ret;

    inputStreamDesc_ = acldvppCreateStreamDesc();
    if (inputStreamDesc_ == nullptr) {
        ASC_LOG_ERROR("Create input stream desc failed");
        return FAILED;
    }

    //如果是最后一帧,则给dvpp vdec送一个结束帧
    if (frameData->isFinished) {
        ret = acldvppSetStreamDescEos(inputStreamDesc_, 1);
        if (ret != ACL_ERROR_NONE) {
            ASC_LOG_ERROR("Set EOS to input stream desc failed, errorno:%d", ret);
            return FAILED;
        }
        return SUCCESS;
    }

    ret = acldvppSetStreamDescData(inputStreamDesc_, frameData->data);                                   
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Set input stream data failed, errorno:%d", ret);
        return FAILED;
    }
    
    // set size for dvpp stream desc
    ret = acldvppSetStreamDescSize(inputStreamDesc_, frameData->size);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Set input stream size failed, errorno:%d", ret);
        return FAILED;
    }


    return SUCCESS;
}

Result VdecProcess::create_output_pic_desc(size_t size)
{
    // Malloc output device memory
    aclError ret = acldvppMalloc(&outputPicBuf_, size);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Malloc vdec output buffer failed when create "
                      "vdec output desc, errorno:%d", ret);
        return FAILED;
    }

    outputPicDesc_ = acldvppCreatePicDesc();
    if (outputPicDesc_ == nullptr) {
        ASC_LOG_ERROR("Create vdec output pic desc failed");
        return FAILED;
    }

    ret = acldvppSetPicDescData(outputPicDesc_, outputPicBuf_);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Set vdec output pic desc data failed, errorno:%d", ret);
        return FAILED;
    }

    ret = acldvppSetPicDescSize(outputPicDesc_, size);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Set vdec output pic size failed, errorno:%d", ret);
        return FAILED;
    }

    ret = acldvppSetPicDescFormat(outputPicDesc_, 
                                  static_cast<acldvppPixelFormat>(format_));
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Set vdec output pic format failed, errorno:%d", ret);
        return FAILED;
    }

    return SUCCESS;
}

Result VdecProcess::process(shared_ptr<FrameData> frameData, void* userData)
{
    if (is_exit_ready()) {
        ASC_LOG_ERROR("Decode finished, not decode any more");
        return FAILED;
    }
    //创建输入desc
    if (SUCCESS != create_input_stream_desc(frameData)) {
        ASC_LOG_ERROR("Create stream desc failed");
        return FAILED;
    }
    //创建输出desc
    if (SUCCESS != create_output_pic_desc(outputPicSize_)) {
        ASC_LOG_ERROR("Create pic desc failed");
        return FAILED;
    }
    //将数据送到dvpp vdec解码,解码后dvpp vdec会调用注册的回调函数
    //(VideoDecode::DvppVdecCallback)处理
    aclError ret = aclvdecSendFrame(vdecChannelDesc_, inputStreamDesc_,
                                    outputPicDesc_, nullptr, userData);
    if (ret != ACL_ERROR_NONE) {
        ASC_LOG_ERROR("Send frame to vdec failed, errorno:%d", ret);
        return FAILED;
    }

    if (frameData->isFinished) {
        ASC_LOG_INFO("dvpp vdec send eos frame success");
        int wait = 3;
        while ((status_ != VDEC_REPORT_THREAD_EXIT) && (wait > 0)) {
            usleep(3000);
            wait--;
        }
        status_ = VDEC_EXIT_READY;
        return SUCCESS;
    }

    return SUCCESS;
}

Result VdecProcess::set_width(uint32_t width) {
    if (frameWidth_ != 0) {
        ASC_LOG_ERROR("Frame width parameter is exist, forbidon modify");
        return FAILED;
    }

    if (width > kFrameWidthMax) {
        ASC_LOG_ERROR("Set video frame width to %u failed, not support width lager than %u",
            width, kFrameWidthMax);
        return FAILED;
    }

    frameWidth_ = width;
    alignWidth_ = ALIGN_UP16(frameWidth_);
    alignHeight_ =  ALIGN_UP16(frameHeight_);
    outputPicSize_ = YUV420SP_SIZE(alignWidth_, alignHeight_);
    outputBufSize_ = outputPicSize_ * 2;

    ASC_LOG_INFO("Set video frame width to %u ok", width);

    return SUCCESS;
}

Result VdecProcess::set_hight(uint32_t height) {
    if (frameWidth_ != 0) {
        ASC_LOG_ERROR("Frame width parameter is exist, forbidon modify");
        return FAILED;
    }

    if (height > kFrameHeightMax) {
        ASC_LOG_ERROR("Set video frame height to %u failed, not support height lager than %u",
            height, kFrameHeightMax);
        return FAILED;
    }

    frameHeight_ = height;
    alignWidth_ = ALIGN_UP16(frameWidth_);
    alignHeight_ =  ALIGN_UP16(frameHeight_);
    outputPicSize_ = YUV420SP_SIZE(alignWidth_, alignHeight_);
    outputBufSize_ = outputPicSize_ * 2;
    ASC_LOG_INFO("Set video frame height to %u ok", height);

    return SUCCESS;
}

Result VdecProcess::set_en_type(uint32_t enType) {
    if (enType > (uint32_t)H264_HIGH_LEVEL) {
        ASC_LOG_ERROR("Set video type to %u failed, the type range is [%d, %d]",
            enType, (int32_t)H265_MAIN_LEVEL, (int32_t)H264_HIGH_LEVEL);
        return FAILED;
    }

    enType_ = enType;
    ASC_LOG_INFO("Set video type to %u ok", enType);

    return SUCCESS;
}

Result VdecProcess::set_format(uint32_t format) {
    if ((format != PIXEL_FORMAT_YUV_SEMIPLANAR_420) ||
        (format != PIXEL_FORMAT_YVU_SEMIPLANAR_420)) {
        ASC_LOG_ERROR("Set video decode output image format to %u failed, "
            "only support %d(YUV420SP NV12) and %d(YUV420SP NV21)", 
            format,
            (int)PIXEL_FORMAT_YUV_SEMIPLANAR_420,
            (int)PIXEL_FORMAT_YVU_SEMIPLANAR_420);
        return FAILED;
    }

    format_ = format;
    ASC_LOG_INFO("Set video decode output image format to %u ok", format);

    return SUCCESS;
}

Result VdecProcess::video_param_check() {
    if ((frameWidth_ > kFrameWidthMax) ||
        (frameHeight_ > kFrameHeightMax) ||
        ((format_ != PIXEL_FORMAT_YUV_SEMIPLANAR_420) &&
         (format_ != PIXEL_FORMAT_YVU_SEMIPLANAR_420)) ||
        (enType_ > (uint32_t)H264_HIGH_LEVEL)) {
        return FAILED;
    }

    return SUCCESS;
}


