/**
 * ============================================================================
 *
 * Copyright (C) 2018, Hisilicon Technologies Co., Ltd. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1 Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *   2 Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *   3 Neither the names of the copyright holders nor the names of the
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * ============================================================================
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>
#include <unistd.h>
#include <sys/prctl.h>
#include <sys/time.h>
#include <fstream>
#include <memory>
#include <thread>
#include <cstring>
#include <iostream>

#include "atlas_utils.h"
#include "channel_id_generator.h"
#include "video_decode.h"

using namespace std;

namespace {
    const int64_t kUsec = 1000000;
    const uint32_t kDecodeFrameQueueSize = 512;
    const int kDecodeQueueOpWait = 1000000; //每次等待10毫秒
    const int kFrameEnQueueRetryTimes = 1000;//为了防止丢帧,ffmpeg解码得到的h26x入队最多等待 100秒
    const int kQueueOpRetryTimes = 1000;
    const int kOutputJamWait = 1000;
    const int kInvalidTpye = -1;
    const int kWaitStatusInterval = 1000;

    const int kDefaultFps = 1;
}

VideoDecode::VideoDecode(const std::string& videoName, int channelId):                          
context_(nullptr), streamType_(STREAM_VIDEO), channelId_(channelId),
frameId_(0), finFrameCnt_(0), status_(DECODE_UNINIT), streamName_(videoName),
streamFormat_(H264_MAIN_LEVEL), lastDecodeTime_(0), fpsInterval_(0), 
ffmpegDecoder_(nullptr), dvppVdec_(nullptr),
frameImageQueue_(kDecodeFrameQueueSize), isStop_(false), isReleased_(false) {
    if (IsRtspAddr(videoName)) {
        streamType_ = STREAM_RTSP;
    }
}

VideoDecode::~VideoDecode() {
    if ((status_ >= DECODE_READY) && (status_ < DECODE_FINISHED)) {
        isStop_ = true;
        ffmpegDecoder_->StopDecode();
        while (status_ != DECODE_FINISHED) {
            usleep(kWaitStatusInterval);
        } 
    }
}

void VideoDecode::DestroyResource() {
    if (isReleased_) return; 

    ATLAS_LOG_INFO("Start release video resource");   

    delete ffmpegDecoder_;
    ffmpegDecoder_ = nullptr;
    ATLAS_LOG_INFO("Release ffmpeg decoder ok");
    
    delete dvppVdec_;
    dvppVdec_ = nullptr;
    ATLAS_LOG_INFO("Release dvpp vdec ok");
    
    do {
        shared_ptr<ImageData> frame = FrameImageOutQueue(true);
        if (frame == nullptr)  {
            break;
        }

        if (frame->data != nullptr) {
            acldvppFree(frame->data); 
            frame->data = nullptr;
        }       
    }while(1);
    ATLAS_LOG_INFO("Release image in decoded queue ok");

    isReleased_ = true;
    status_ = DECODE_FINISHED;
    ATLAS_LOG_INFO("Release video decoder resource success");
}

AtlasError VideoDecode::Open() {
    if (status_ == DECODE_UNINIT) {
        decodeThread_ = thread(DecodeThreadFunction, (void*)this);
        decodeThread_.detach();
        while (status_ == DECODE_UNINIT) {
            usleep(kWaitStatusInterval);
        } 

        return (status_ == DECODE_READY)? ATLAS_OK : ATLAS_ERROR;
    } 
    
    return ((status_ >= DECODE_READY) && (status_ <= DECODE_DVPP_FINISHED)) ?
            ATLAS_OK : ATLAS_ERROR;    
}

void VideoDecode::DecodeThreadFunction(void* decoderSelf) {
    VideoDecode* thisPtr =  (VideoDecode*)decoderSelf;

    AtlasError ret = thisPtr->Init();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Video decoder init failed, error %d", ret); 
        return;
    }

    while(thisPtr->GetStatus() == DECODE_READY) {
        usleep(kWaitStatusInterval);
    }

    //调用ffmpeg解码器开始解码视频,这是一个阻塞接口,一直到视频解析完
    thisPtr->FFmpegDecode();
    thisPtr->SetStatus(DECODE_FFMPEG_FINISHED);       
    //ffmpeg解码完视频后vdec解码器发一个结束帧
    shared_ptr<FrameData> videoFrame = make_shared<FrameData>();    
    videoFrame->isFinished = true;
    videoFrame->data = nullptr;
    videoFrame->size = 0;   
    thisPtr->dvppVdec_->Process(videoFrame, decoderSelf); 
    
    while((thisPtr->GetStatus() != DECODE_FINISHED) && !thisPtr->IsStop()) {
        usleep(kWaitStatusInterval);
    }
    thisPtr->DestroyResource();    
}

AtlasError VideoDecode::Init() {
    //防止多次初始化
    if (status_ == DECODE_ERROR) 
        return ATLAS_ERROR_OPEN_VIDEO_UNREADY;

    if (status_ != DECODE_UNINIT)
        return ATLAS_OK;

    AtlasError ret = InitResource();
    if (ret != ATLAS_OK) {
        this->SetStatus(DECODE_ERROR);
        ATLAS_LOG_ERROR("Open %s failed for init resource error: %d", 
                        streamName_.c_str(), ret);
        return ret;
    }

    ret = InitFFmpegDecoder();
    if (ret != ATLAS_OK) {
        this->SetStatus(DECODE_ERROR);
        ATLAS_LOG_ERROR("Open %s failed for init ffmpeg error: %d", 
                        streamName_.c_str(), ret);
        return ret;
    }
    
    ret = InitVdecDecoder();
    if (ret != ATLAS_OK) {
        this->SetStatus(DECODE_ERROR);
        ATLAS_LOG_ERROR("Open %s failed for init vdec error: %d", 
                        streamName_.c_str(), ret);
        return ret;
    }
 
    this->SetStatus(DECODE_READY);
    ATLAS_LOG_INFO("Video %s decode init ok", streamName_.c_str());
    
    return ATLAS_OK;
} 

AtlasError VideoDecode::InitResource() {
    aclError aclRet = aclrtCreateContext(&context_, 0);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Create acl context failed, error:%d", aclRet);
        return ATLAS_ERROR_CREATE_ACL_CONTEXT;
    }
    
    aclRet = aclrtGetRunMode(&runMode_);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("acl get run mode failed");
        return ATLAS_ERROR_GET_RUM_MODE;
    } 

    return ATLAS_OK;  
}

AtlasError VideoDecode::InitFFmpegDecoder() {
    //创建视频解码为H26X的解码器实例
    ffmpegDecoder_ = new FFmpegDecoder(streamName_);
    if (kInvalidTpye == GetVdecType()) {
        this->SetStatus(DECODE_ERROR);        
        delete ffmpegDecoder_;
        ATLAS_LOG_ERROR("Video %s type is invalid", streamName_.c_str());
        return ATLAS_ERROR_FFMPEG_DECODER_INIT;
    } 

    //h26x裸流文件没有帧率参数
    int fps =  ffmpegDecoder_->GetFps();
    if (fps == 0) {
        fps = kDefaultFps;
        ATLAS_LOG_INFO("Video %s fps is 0, change to %d", 
                       streamName_.c_str(), fps);
    }
    fpsInterval_ = kUsec / fps;

    return ATLAS_OK;
}

int VideoDecode::GetVdecType() {
    //VDEC支持　H265 main level，264 baseline level，main level，high level
    //等４种格式的视频解码.根据ffmpeg的解码确定视频文件属于哪一种.同样的,如果时h26x裸流,
    //ffmpeg是无法解析出格式信息的
    int type = ffmpegDecoder_->GetVideoType();
    int profile = ffmpegDecoder_->GetProfile();
    if (type == AV_CODEC_ID_HEVC) {        
        streamFormat_ = H265_MAIN_LEVEL;         
    } else if (type == AV_CODEC_ID_H264) {
        switch(profile) {
            case FF_PROFILE_H264_BASELINE:
                streamFormat_ = H264_BASELINE_LEVEL;
                break;
            case FF_PROFILE_H264_MAIN:
                streamFormat_ = H264_MAIN_LEVEL;
                break;
            case FF_PROFILE_H264_HIGH:
            case FF_PROFILE_H264_HIGH_10:
            case FF_PROFILE_H264_HIGH_10_INTRA:
            case FF_PROFILE_H264_MULTIVIEW_HIGH:
            case FF_PROFILE_H264_HIGH_422:
            case FF_PROFILE_H264_HIGH_422_INTRA:
            case FF_PROFILE_H264_STEREO_HIGH:
            case FF_PROFILE_H264_HIGH_444:
            case FF_PROFILE_H264_HIGH_444_PREDICTIVE:
            case FF_PROFILE_H264_HIGH_444_INTRA:
                streamFormat_ = H264_HIGH_LEVEL;
                break;
            default:
                ATLAS_LOG_INFO("Not support h264 profile %d, use as mp", profile);
                streamFormat_ = H264_MAIN_LEVEL; 
                break;
        }
    } else {
        streamFormat_ = kInvalidTpye;
        ATLAS_LOG_ERROR("Not support stream, type %d,  profile %d", type, profile);
    }

    return streamFormat_;
}

AtlasError VideoDecode::InitVdecDecoder() {
    //实例化vdec解码器,将输入的h26x帧解码为图片
    dvppVdec_ = new VdecProcess(channelId_, ffmpegDecoder_->GetFrameWidth(), 
                                ffmpegDecoder_->GetFrameHeight(), 
                                streamFormat_, VideoDecode::DvppVdecCallback); 
    AtlasError ret = dvppVdec_->Init();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Dvpp vdec init failed");
    }

    return ret; 
}

//dvpp vdec　callback函数
void VideoDecode::DvppVdecCallback(acldvppStreamDesc *input, 
                                   acldvppPicDesc *output, void *userData)
{
    VideoDecode* decoder = (VideoDecode*)userData;
    //获取dvpp vdec解码后的yuv图片数据
    shared_ptr<ImageData> image = make_shared<ImageData>();
    image->format = acldvppGetPicDescFormat(output);
    image->width = acldvppGetPicDescWidth(output);
    image->height = acldvppGetPicDescHeight(output);
    image->alignWidth = acldvppGetPicDescWidthStride(output);
    image->alignHeight = acldvppGetPicDescHeightStride(output);
    image->size = acldvppGetPicDescSize(output);
    image->data = (uint8_t *)acldvppGetPicDescData(output);    

    //将解码后的图片放入队列等待读取
    decoder->ProcessDecodedImage(image);

    //释放dvpp vdec的输入输出desc.为了减少内存申请和拷贝,输入和输出数据内存都是
    //和上下游操作复用的,并且解码时异步的,内存无法复用, 所以需要对每一帧重新创建输入输出desc
    aclError ret = acldvppDestroyPicDesc(output);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("fail to destroy pic desc, error %d", ret);
    }

    if (input != nullptr) {
        void* inputBuf = acldvppGetStreamDescData(input);
        if (inputBuf != nullptr) {
            acldvppFree(inputBuf);
        }
        
        aclError ret = acldvppDestroyStreamDesc(input);
        if (ret != ACL_ERROR_NONE) {
            ATLAS_LOG_ERROR("fail to destroy input stream desc");
        }
    }       
}

void VideoDecode::ProcessDecodedImage(shared_ptr<ImageData> frameData) {
    if (YUV420SP_SIZE(frameData->width, frameData->height) != frameData->size) {
        ATLAS_LOG_ERROR("Invalid decoded frame parameter, "
                        "width %d, height %d, size %d",
                        frameData->width, frameData->height, frameData->size);
        return;
    }

    FrameImageEnQueue(frameData);

    finFrameCnt_++;
    if ((status_ == DECODE_FFMPEG_FINISHED) && (finFrameCnt_ >= frameId_)) {
        ATLAS_LOG_INFO("Last frame decoded by dvpp, change status to %d",
                       DECODE_DVPP_FINISHED);
        this->SetStatus(DECODE_DVPP_FINISHED);
    } 
}

AtlasError VideoDecode::FrameImageEnQueue(shared_ptr<ImageData> frameData) {
    for (int count = 0; count < kFrameEnQueueRetryTimes; count++) {
        if (frameImageQueue_.Push(frameData)) 
            return ATLAS_OK;
        usleep(kDecodeQueueOpWait); 
    }
    ATLAS_LOG_ERROR("Video %s lost decoded image for queue full", 
	                streamName_.c_str());

    return ATLAS_ERROR_VDEC_QUEUE_FULL;
}

//ffmpeg解码回调
AtlasError VideoDecode::FrameDecodeCallback(void* decoder, void* frameData, 
                                            int frameSize) {
    if ((frameData == NULL) || (frameSize == 0)) {
        ATLAS_LOG_ERROR("Frame data is null");
        return ATLAS_ERROR_H26X_FRAME;
    }

    //将ffmpeg解码得到的h26x数据拷贝到dvpp内存
    VideoDecode* videoDecoder = (VideoDecode*)decoder;
    void* buffer = CopyDataToDevice(frameData, frameSize,
                                    videoDecoder->runMode_, MEMORY_DVPP);  
    if (buffer == nullptr) {
        ATLAS_LOG_ERROR("Copy frame h26x data to dvpp failed");
        return ATLAS_ERROR_COPY_DATA;
    }

    shared_ptr<FrameData> videoFrame = make_shared<FrameData>();
    videoDecoder->frameId_++;
    videoFrame->frameId = videoDecoder->frameId_;
    videoFrame->data = buffer;
    videoFrame->size = frameSize;
    //使用dvpp vdec解码h26x帧数据 
    AtlasError ret = videoDecoder->dvppVdec_->Process(videoFrame, decoder);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Dvpp vdec process %dth frame failed, error:%u", 
                        videoDecoder->frameId_, ret);
        return ret;
    }

    return ATLAS_OK;
}

void VideoDecode::SleepToNextFrameTime() {
    while ((frameImageQueue_.Size() >  2) && 
           !isStop_) {
        usleep(kOutputJamWait);
    }

    if (streamType_ == STREAM_RTSP) {
        usleep(0);
        return;
    }

    //获取当前时间
    timeval tv;
    gettimeofday(&tv, 0);
    int64_t now = (int64_t)tv.tv_sec * 1000000 + (int64_t)tv.tv_usec;

    if (lastDecodeTime_ == 0) {
        lastDecodeTime_ = now;
        return;
    }
    //计算到解码一帧后还剩余的时间
    int64_t lastInterval = (now - lastDecodeTime_);
    int64_t sleepTime = (lastInterval < fpsInterval_)?(fpsInterval_-lastInterval):0;
    //耗完一帧解码时间
    usleep(sleepTime);
    //记录下一帧解码的开始时间
    gettimeofday(&tv, 0);
    lastDecodeTime_ = (int64_t)tv.tv_sec * 1000000 + (int64_t)tv.tv_usec;

    return;
}

//当前解码器是否准备好
bool VideoDecode::IsOpened() { 
    ATLAS_LOG_INFO("Video %s decode status %d", streamName_.c_str(), status_);
    return (status_ == DECODE_READY) || (status_ == DECODE_START);
}

/*读取一帧解码后的yuv图像*/
AtlasError VideoDecode::Read(ImageData& image) {
    //如果当前解码器异常或者解码结束,则直接返回nullptr
    if (status_ == DECODE_ERROR) {
        ATLAS_LOG_ERROR("Read failed for decode %s failed", 
                        streamName_.c_str()); 
        return ATLAS_ERROR_VIDEO_DECODER_STATUS;
    }

    if (status_ == DECODE_FINISHED) {
        ATLAS_LOG_INFO("No frame to read for decode %s finished", 
                        streamName_.c_str()); 
        return ATLAS_ERROR_DECODE_FINISH;
    }

    //如果当前只是准备好,但是还未开始解码.Read的调用触发解码开始
    if (status_ == DECODE_READY) {
        StartDecode();
        usleep(kDecodeQueueOpWait);
    }
    //从解码后图片存放队列读取一帧图片
    bool noWait = (status_ == DECODE_DVPP_FINISHED);
    shared_ptr<ImageData> frame = FrameImageOutQueue(noWait);
    if (noWait && (frame == nullptr)) {
        SetStatus(DECODE_FINISHED);
        ATLAS_LOG_INFO("No frame to read anymore");
        return ATLAS_ERROR_DECODE_FINISH;
    }

    if (frame == nullptr) {
        ATLAS_LOG_ERROR("No frame image to read abnormally");
        Close();
        return ATLAS_ERROR_READ_EMPTY;
    }  

    image.format = frame->format;
    image.width = frame->width;
    image.height = frame->height;
    image.size = frame->size;
    image.data = frame->data;

    return ATLAS_OK;
}

shared_ptr<ImageData> VideoDecode::FrameImageOutQueue(bool noWait) {
    shared_ptr<ImageData> image = frameImageQueue_.Pop();
    if (noWait || (image != nullptr)) return image;

    for (int count = 0; count < kQueueOpRetryTimes - 1; count++) {
        usleep(kDecodeQueueOpWait);

        image = frameImageQueue_.Pop();
        if (image != nullptr)
            return image;
    }

    return nullptr;
}

AtlasError VideoDecode::Set(StreamProperty key, int value) {
    AtlasError ret = ATLAS_OK;
    switch(key) {
        case OUTPUT_IMAGE_FORMAT:
            ret = dvppVdec_->SetFormat(value);
            break;
        case RTSP_TRANSPORT:
            ret = SetRtspTransType(value);
            break;
        default:
            ret = ATLAS_ERROR_UNSURPPORT_PROPERTY;
            ATLAS_LOG_ERROR("Unsurpport property %d to set for video %s",
                            (int)key, streamName_.c_str());
            break;
    }

    return ret;
}

AtlasError VideoDecode::SetRtspTransType(uint32_t transCode) {
    AtlasError ret = ATLAS_OK;

    if (transCode == RTSP_TRANS_UDP)
        ffmpegDecoder_->SetTransport(RTSP_TRANSPORT_UDP);
    else if (transCode == RTSP_TRANS_TCP)    
        ffmpegDecoder_->SetTransport(RTSP_TRANSPORT_TCP);
    else {
        ret = ATLAS_ERROR_INVALID_PROPERTY_VALUE;
        ATLAS_LOG_ERROR("Unsurport rtsp transport property value %u", 
                        transCode);
    }

    return ret;
}

uint32_t VideoDecode::Get(StreamProperty key) {
    uint32_t value = 0;

    switch(key){
        case FRAME_WIDTH:
            value = ffmpegDecoder_->GetFrameWidth();
            break;
        case FRAME_HEIGHT:
            value = ffmpegDecoder_->GetFrameHeight();
            break;
        case VIDEO_FPS:
            value = ffmpegDecoder_->GetFps();
            break;
        default:
            ATLAS_LOG_ERROR("Unsurpport property %d to get for video", key);
            break;
    }

    return value;
}

AtlasError VideoDecode::SetAclContext() {
    if (context_ == nullptr) {
        ATLAS_LOG_ERROR("Video decoder context is null");
        return ATLAS_ERROR_SET_ACL_CONTEXT;
    }
    
    aclError ret = aclrtSetCurrentContext(context_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Video decoder set context failed, error: %d", ret);
        return ATLAS_ERROR_SET_ACL_CONTEXT;
    }
    
    return ATLAS_OK;   
}

AtlasError VideoDecode::Close() {
    isStop_ = true;
    ffmpegDecoder_->StopDecode();
    return ATLAS_OK;
}