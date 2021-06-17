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
#include "video_decode.h"

using namespace std;

namespace {
    const int64_t kUsec = 1000000;
    const uint32_t kDecodeFrameQueueSize = 256;
    const int kDecodeQueueOpWait = 10000; //每次等待10毫秒
    const int kFrameEnQueueRetryTimes = 1000;//为了防止丢帧,ffmpeg解码得到的h26x入队最多等待 100秒
    const int kQueueOpRetryTimes = 1000;
    const int kOutputJamWait = 10000;
    const int kInvalidTpye = -1;
    const int kWaitDecodeFinishInterval = 1000;

    const int kDefaultFps = 1;
    const int kReadSlow = 2;

    ChannelIdGenerator channelIdGenerator;
}

VideoDecode::VideoDecode(const std::string& videoName, aclrtContext context) : 
context_(context), streamType_(STREAM_VIDEO), channelId_(INVALID_CHANNEL_ID),
frameId_(0), finFrameCnt_(0), status_(DECODE_UNINIT), streamName_(videoName),
streamFormat_(H264_MAIN_LEVEL), lastDecodeTime_(0), fpsInterval_(0), 
ffmpegDecoder_(nullptr), dvppVdec_(nullptr),
frameImageQueue_(kDecodeFrameQueueSize), isStop_(false), isReleased_(false),
isJam_(false) {
    if (IsRtspAddr(videoName)) {
        streamType_ = STREAM_RTSP;
    }
}

VideoDecode::~VideoDecode() {
    DestroyResource();
}

void VideoDecode::DestroyResource() {
    if (isReleased_) return;    
    //1. stop ffmpeg
    isStop_ = true;
    ffmpegDecoder_->StopDecode();
    while ((status_ >= DECODE_START) && (status_ < DECODE_FFMPEG_FINISHED)) {
        usleep(kWaitDecodeFinishInterval);
    }
    //2. delete ffmpeg decoder
    delete ffmpegDecoder_;
    ffmpegDecoder_ = nullptr;    
    //3. release dvpp vdec
    delete dvppVdec_;
    dvppVdec_ = nullptr;
    //4. release image memory in decode output queue
    do {
        shared_ptr<ImageData> frame = FrameImageOutQueue(true);
        if (frame == nullptr) {
            break;
        }

        if (frame->data != nullptr) {
            acldvppFree(frame->data.get());
            frame->data = nullptr;
        }       
    }while(1);
    //5. release channel id
    channelIdGenerator.ReleaseChannelId(channelId_);

    isReleased_ = true;
}

AtlasError VideoDecode::InitResource() {
    aclError aclRet;
    //use current thread context default
    if (context_ == nullptr) {
        aclRet = aclrtGetCurrentContext(&context_);
        if ((aclRet != ACL_ERROR_NONE) || (context_ == nullptr)) {
            ATLAS_LOG_ERROR("Get current acl context error:%d", aclRet);
            return ATLAS_ERROR_GET_ACL_CONTEXT;
        }       
    }
    //Get current run mode
    aclRet = aclrtGetRunMode(&runMode_);
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("acl get run mode failed");
        return ATLAS_ERROR_GET_RUM_MODE;
    } 

    return ATLAS_OK;  
}

AtlasError VideoDecode::InitVdecDecoder() {
    //Generate a unique channel id for video decoder
    channelId_ = channelIdGenerator.GenerateChannelId();
    if (channelId_ == INVALID_CHANNEL_ID) {
        ATLAS_LOG_ERROR("Decoder number excessive %d", VIDEO_CHANNEL_MAX);
        return ATLAS_ERROR_TOO_MANY_VIDEO_DECODERS;
    }

    //Create dvpp vdec to decode h26x data
    dvppVdec_ = new VdecProcess(channelId_, ffmpegDecoder_->GetFrameWidth(), 
                                ffmpegDecoder_->GetFrameHeight(), 
                                streamFormat_, VideoDecode::DvppVdecCallback); 
    AtlasError ret = dvppVdec_->Init();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Dvpp vdec init failed");
    }

    return ret; 
}

AtlasError VideoDecode::InitFFmpegDecoder() {
    //Create ffmpeg decoder to parse video stream to h26x frame data
    ffmpegDecoder_ = new FFmpegDecoder(streamName_);
    if (kInvalidTpye == GetVdecType()) {
        this->SetStatus(DECODE_ERROR);        
        delete ffmpegDecoder_;
        ATLAS_LOG_ERROR("Video %s type is invalid", streamName_.c_str());
        return ATLAS_ERROR_FFMPEG_DECODER_INIT;
    } 

    //Get video fps, if no fps, use 1 as default
    int fps =  ffmpegDecoder_->GetFps();
    if (fps == 0) {
        fps = kDefaultFps;
        ATLAS_LOG_INFO("Video %s fps is 0, change to %d", 
                       streamName_.c_str(), fps);
    }
    //Cal the frame interval time(us)
    fpsInterval_ = kUsec / fps;

    return ATLAS_OK;
}

AtlasError VideoDecode::Open() {
    //Open video stream, if open failed before, return error directly
    if (status_ == DECODE_ERROR) 
        return ATLAS_ERROR_OPEN_VIDEO_UNREADY;
    //If open ok already
    if (status_ != DECODE_UNINIT)
        return ATLAS_OK;
    //Init acl resource
    AtlasError ret = InitResource();
    if (ret != ATLAS_OK) {
        this->SetStatus(DECODE_ERROR);
        ATLAS_LOG_ERROR("Open %s failed for init resource error: %d", 
                        streamName_.c_str(), ret);
        return ret;
    }
    //Init ffmpeg decoder
    ret = InitFFmpegDecoder();
    if (ret != ATLAS_OK) {
        this->SetStatus(DECODE_ERROR);
        ATLAS_LOG_ERROR("Open %s failed for init ffmpeg error: %d", 
                        streamName_.c_str(), ret);
        return ret;
    }
    //Init dvpp vdec decoder
    ret = InitVdecDecoder();
    if (ret != ATLAS_OK) {
        this->SetStatus(DECODE_ERROR);
        ATLAS_LOG_ERROR("Open %s failed for init vdec error: %d", 
                        streamName_.c_str(), ret);
        return ret;
    }
    //Set init ok
    this->SetStatus(DECODE_READY);
    ATLAS_LOG_INFO("Video %s decode init ok", streamName_.c_str());
    
    return ATLAS_OK;
}

int VideoDecode::GetVdecType() {
    //VDEC only support　H265 main level，264 baseline level，main level，high level
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

//dvpp vdec　callback
void VideoDecode::DvppVdecCallback(acldvppStreamDesc *input, 
                                   acldvppPicDesc *output, void *userData)
{
    VideoDecode* decoder = (VideoDecode*)userData;
    //Get decoded image parameters
    shared_ptr<ImageData> image = make_shared<ImageData>();
    image->format = acldvppGetPicDescFormat(output);
    image->width = acldvppGetPicDescWidth(output);
    image->height = acldvppGetPicDescHeight(output);
    image->alignWidth = acldvppGetPicDescWidthStride(output);
    image->alignHeight = acldvppGetPicDescHeightStride(output);
    image->size = acldvppGetPicDescSize(output);

    void* vdecOutBufferDev = acldvppGetPicDescData(output);
    image->data = SHARED_PRT_DVPP_BUF(vdecOutBufferDev);

    //Put the decoded image to queue for read
    decoder->ProcessDecodedImage(image);
    //Release resouce
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
    finFrameCnt_++;
    if (YUV420SP_SIZE(frameData->width, frameData->height) != frameData->size) {
        ATLAS_LOG_ERROR("Invalid decoded frame parameter, "
                        "width %d, height %d, size %d, buffer %p",
                        frameData->width, frameData->height, 
                        frameData->size, frameData->data.get());
        return;
    }

    FrameImageEnQueue(frameData);

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

//start decoder
void VideoDecode::StartFrameDecoder() {
    if (status_ == DECODE_READY) {

        decodeThread_ = thread(FrameDecodeThreadFunction, (void*)this);
        decodeThread_.detach();

        status_ = DECODE_START;
    }
}

//ffmpeg decoder entry
void VideoDecode::FrameDecodeThreadFunction(void* decoderSelf) {
    VideoDecode* thisPtr =  (VideoDecode*)decoderSelf;

    aclError aclRet = thisPtr->SetAclContext();
    if (aclRet != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("Set frame decoder context failed, errorno:%d",
                        aclRet);
        return;
    }
    //start decode until complete
    thisPtr->FFmpegDecode();
    if (thisPtr->IsStop())  {
        thisPtr->SetStatus(DECODE_FINISHED);
        return;
    }
    thisPtr->SetStatus(DECODE_FFMPEG_FINISHED);  
    //when ffmpeg decode finish, send eos to vdec
    shared_ptr<FrameData> videoFrame = make_shared<FrameData>();    
    videoFrame->isFinished = true;
    videoFrame->data = nullptr;
    videoFrame->size = 0;   
    thisPtr->dvppVdec_->Process(videoFrame, decoderSelf); 
    
    while((thisPtr->GetStatus() != DECODE_DVPP_FINISHED) && !thisPtr->IsStop()) {
        usleep(kWaitDecodeFinishInterval);
    } 
}

//callback of ffmpeg decode frame 
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
        ATLAS_LOG_ERROR("Dvpp vdec process %dth frame failed, error:%d", 
                        videoDecoder->frameId_, ret);
        return ret;
    }

    //根据视频帧率等待下一帧
    videoDecoder->SleeptoNextFrameTime();
    return ATLAS_OK;
}

void VideoDecode::SleeptoNextFrameTime() {
    while(frameImageQueue_.Size() >  kReadSlow) {
        if (isStop_) {
            return;
        }
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
        StartFrameDecoder();
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
        return ATLAS_ERROR_READ_EMPTY;
    }

    image.format = frame->format;
    image.width = frame->width;
    image.height = frame->height;
    image.alignWidth = frame->alignWidth;
    image.alignHeight = frame->alignHeight;
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
        ATLAS_LOG_ERROR("Unsurport rtsp transport property value %d", 
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
    DestroyResource();
    return ATLAS_OK;
}

