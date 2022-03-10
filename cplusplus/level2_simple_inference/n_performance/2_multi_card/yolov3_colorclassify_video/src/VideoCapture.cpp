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
#include "AclLiteUtils.h"
#include "VideoCapture.h"

using namespace std;

namespace {
    const int64_t kUsec = 1000000;
    const uint32_t kDecodeFrameQueueSize = 256;
    const int kDecodeQueueOpWait = 10000; //  decode wait 10ms/frame
    const int kFrameEnQueueRetryTimes = 1000; //  max wait time for the frame to enter in queue
    const int kQueueOpRetryTimes = 1000;
    const int kOutputJamWait = 10000;
    const int kInvalidTpye = -1;
    const int kWaitDecodeFinishInterval = 1000;
    const int kDefaultFps = 1;
    const int kReadSlow = 2;

    ChannelIdGenerator channelIdGenerator;

    const int kNoFlag = 0; // no flag
    const int kInvalidVideoIndex = -1; // invalid video index
    const string kRtspTransport = "rtspTransport"; // rtsp transport
    const string kUdp = "udp"; // video format udp
    const string kTcp = "tcp";
    const string kBufferSize = "buffer_size"; // buffer size string
    const string kMaxBufferSize = "10485760"; // maximum buffer size:10MB
    const string kMaxDelayStr = "max_delay"; // maximum delay string
    const string kMaxDelayValue = "100000000"; // maximum delay time:100s
    const string kTimeoutStr = "stimeout"; // timeout string
    const string kTimeoutValue = "5000000"; // timeout:5s
    const string kPktSize = "pkt_size"; // ffmpeg pakect size string
    const string kPktSizeValue = "10485760"; // ffmpeg packet size value:10MB
    const string kReorderQueueSize = "reorder_queue_size"; // reorder queue size
    const string kReorderQueueSizeValue = "0"; // reorder queue size value
    const int kErrorBufferSize = 1024; // buffer size for error info
    const uint32_t kDefaultStreamFps = 5;
    const uint32_t kOneSecUs = 1000 * 1000;
}

FFmpegDecoder::FFmpegDecoder(const std::string& streamName)
:streamName_(streamName){
    rtspTransport_.assign(kTcp.c_str());
    isFinished_ = false;
    isStop_ = false;
    GetVideoInfo();
}

void FFmpegDecoder::SetTransport(const std::string& transportType) {
    rtspTransport_.assign(transportType.c_str());
};

int FFmpegDecoder::GetVideoIndex(AVFormatContext* avFormatContext) {
    if (avFormatContext == nullptr) { // verify input pointer
        return kInvalidVideoIndex;
    }

    // get video index in streams
    for (uint32_t i = 0; i < avFormatContext->nb_streams; i++) {
        if (avFormatContext->streams[i]->codecpar->codec_type
            == AVMEDIA_TYPE_VIDEO) { // check is media type is video
            return i;
        }
    }

    return kInvalidVideoIndex;
}

void FFmpegDecoder::InitVideoStreamFilter(const AVBitStreamFilter*& videoFilter) {
    if (videoType_ == AV_CODEC_ID_H264) { // check video type is h264
        videoFilter = av_bsf_get_by_name("h264_mp4toannexb");
    }
    else { // the video type is h265
        videoFilter = av_bsf_get_by_name("hevc_mp4toannexb");
    }
}

void FFmpegDecoder::SetDictForRtsp(AVDictionary*& avdic) {
    ACLLITE_LOG_INFO("Set parameters for %s", streamName_.c_str());

    av_dict_set(&avdic, kRtspTransport.c_str(), rtspTransport_.c_str(), kNoFlag);
    av_dict_set(&avdic, kBufferSize.c_str(), kMaxBufferSize.c_str(), kNoFlag);
    av_dict_set(&avdic, kMaxDelayStr.c_str(), kMaxDelayValue.c_str(), kNoFlag);
    av_dict_set(&avdic, kTimeoutStr.c_str(), kTimeoutValue.c_str(), kNoFlag);
    av_dict_set(&avdic, kReorderQueueSize.c_str(),
                kReorderQueueSizeValue.c_str(), kNoFlag);
    av_dict_set(&avdic, kPktSize.c_str(), kPktSizeValue.c_str(), kNoFlag);
    ACLLITE_LOG_INFO("Set parameters for %s end", streamName_.c_str());
}

bool FFmpegDecoder::OpenVideo(AVFormatContext*& avFormatContext) {
    bool ret = true;
    AVDictionary* avdic = nullptr;

    av_log_set_level(AV_LOG_DEBUG);

    ACLLITE_LOG_INFO("Open video %s ...", streamName_.c_str());
    SetDictForRtsp(avdic);
    int openRet = avformat_open_input(&avFormatContext,
                                       streamName_.c_str(), nullptr,
                                       &avdic);
    if (openRet < 0) { // check open video result
        char buf_error[kErrorBufferSize];
        av_strerror(openRet, buf_error, kErrorBufferSize);

        ACLLITE_LOG_ERROR("Could not open video:%s, return :%d, error info:%s",
                      streamName_.c_str(), openRet, buf_error);
        ret = false;
    }

    if (avdic != nullptr) { // free AVDictionary
        av_dict_free(&avdic);
    }

    return ret;
}

bool FFmpegDecoder::InitVideoParams(int videoIndex, 
                                        AVFormatContext* avFormatContext,
                                        AVBSFContext*& bsfCtx) {
    const AVBitStreamFilter* videoFilter;
    InitVideoStreamFilter(videoFilter);
    if (videoFilter == nullptr) { // check video fileter is nullptr
        ACLLITE_LOG_ERROR("Unkonw bitstream filter, videoFilter is nullptr!");
        return false;
    }

    // checke alloc bsf context result
    if (av_bsf_alloc(videoFilter, &bsfCtx) < 0) {
        ACLLITE_LOG_ERROR("Fail to call av_bsf_alloc!");
        return false;
    }

    // check copy parameters result
    if (avcodec_parameters_copy(bsfCtx->par_in,
        avFormatContext->streams[videoIndex]->codecpar) < 0) {
        ACLLITE_LOG_ERROR("Fail to call avcodec_parameters_copy!");
        return false;
    }

    bsfCtx->time_base_in = avFormatContext->streams[videoIndex]->time_base;

    // check initialize bsf contextreult
    if (av_bsf_init(bsfCtx) < 0) {
        ACLLITE_LOG_ERROR("Fail to call av_bsf_init!");
        return false;
    }

    return true;
}

void FFmpegDecoder::Decode(FrameProcessCallBack callback, 
                               void *callbackParam) {
    ACLLITE_LOG_INFO("Start ffmpeg decode video %s ...", streamName_.c_str());
    avformat_network_init(); // init network

    AVFormatContext* avFormatContext = avformat_alloc_context();

    // check open video result
    if (!OpenVideo(avFormatContext)) {
        return;
    }

    int videoIndex = GetVideoIndex(avFormatContext);
    if (videoIndex == kInvalidVideoIndex) { // check video index is valid
        ACLLITE_LOG_ERROR("Rtsp %s index is -1", streamName_.c_str());
        return;
    }

    AVBSFContext* bsfCtx = nullptr;
    // check initialize video parameters result
    if (!InitVideoParams(videoIndex, avFormatContext, bsfCtx)) {
        return;
    }

    ACLLITE_LOG_INFO("Start decode frame of video %s ...", streamName_.c_str());

    AVPacket avPacket;
    int processOk = true;
    // loop to get every frame from video stream
    while ((av_read_frame(avFormatContext, &avPacket) == 0) && processOk && !isStop_) {
        if (avPacket.stream_index == videoIndex) { // check current stream is video
          // send video packet to ffmpeg
            if (av_bsf_send_packet(bsfCtx, &avPacket)) {
                ACLLITE_LOG_ERROR("Fail to call av_bsf_send_packet, channel id:%s",
                    streamName_.c_str());
            }

            // receive single frame from ffmpeg
            while ((av_bsf_receive_packet(bsfCtx, &avPacket) == 0) && !isStop_) {
                int ret = callback(callbackParam, avPacket.data, avPacket.size);
                if (ret != 0) {
                    processOk = false;
                    break;
                }
            }
        }
        av_packet_unref(&avPacket);
    }

    av_bsf_free(&bsfCtx); // free AVBSFContext pointer
    avformat_close_input(&avFormatContext); // close input video

    isFinished_ = true;
    ACLLITE_LOG_INFO("Ffmpeg decoder %s finished", streamName_.c_str());
}

void FFmpegDecoder::GetVideoInfo() {
    avformat_network_init(); // init network
    AVFormatContext* avFormatContext = avformat_alloc_context();
    bool ret = OpenVideo(avFormatContext);
    if (ret == false) {
        ACLLITE_LOG_ERROR("Open %s failed", streamName_.c_str());
        return;
    }

    if (avformat_find_stream_info(avFormatContext,NULL)<0) {
		ACLLITE_LOG_ERROR("Get stream info of %s failed", streamName_.c_str());
		return;
	}

    int videoIndex = GetVideoIndex(avFormatContext);
    if (videoIndex == kInvalidVideoIndex) { // check video index is valid
        ACLLITE_LOG_ERROR("Video index is %d, current media stream has no "
                        "video info:%s",
                        kInvalidVideoIndex, streamName_.c_str());

        avformat_close_input(&avFormatContext);
        return;
    }

    AVStream* inStream = avFormatContext->streams[videoIndex];

    frameWidth_ = inStream->codecpar->width;
    frameHeight_ = inStream->codecpar->height;
    if (inStream->avg_frame_rate.den) {
        fps_ = inStream->avg_frame_rate.num / inStream->avg_frame_rate.den;
    }
    else {
        fps_ = kDefaultStreamFps;
    }

    videoType_ = inStream->codecpar->codec_id;
    profile_ = inStream->codecpar->profile;

    avformat_close_input(&avFormatContext);

    ACLLITE_LOG_INFO("Video %s, type %d, profile %d, width:%d, height:%d, fps:%d",
                 streamName_.c_str(), videoType_, profile_, frameWidth_, frameHeight_, fps_);
    return;
}                                                                   

VideoCapture::VideoCapture(const std::string& videoName, aclrtContext context) :
  isStop_(false),
  isReleased_(false),
  isJam_(false),
  streamType_(STREAM_VIDEO),
  status_(DECODE_UNINIT),
  context_(context),
  channelId_(INVALID_CHANNEL_ID),
  streamFormat_(H264_MAIN_LEVEL),
  frameId_(0),
  finFrameCnt_(0),
  lastDecodeTime_(0),
  fpsInterval_(0),
  streamName_(videoName), 
  ffmpegDecoder_(nullptr),
  dvppVdec_(nullptr),
  frameImageQueue_(kDecodeFrameQueueSize) {
    if (IsRtspAddr(videoName)) {
        streamType_ = STREAM_RTSP;
    }
}

VideoCapture::~VideoCapture() {
    DestroyResource();
}

void VideoCapture::DestroyResource() {
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

AclLiteError VideoCapture::InitResource() {
    aclError aclRet;
    //use current thread context default
    if (context_ == nullptr) {
        aclRet = aclrtGetCurrentContext(&context_);
        if ((aclRet != ACL_SUCCESS) || (context_ == nullptr)) {
            ACLLITE_LOG_ERROR("Get current acl context error:%d", aclRet);
            return ACLLITE_ERROR_GET_ACL_CONTEXT;
        }       
    }
    //Get current run mode
    aclRet = aclrtGetRunMode(&runMode_);
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acl get run mode failed");
        return ACLLITE_ERROR_GET_RUM_MODE;
    } 

    return ACLLITE_OK;  
}

AclLiteError VideoCapture::InitVdecDecoder() {
    //Generate a unique channel id for video decoder
    channelId_ = channelIdGenerator.GenerateChannelId();
    if (channelId_ == INVALID_CHANNEL_ID) {
        ACLLITE_LOG_ERROR("Decoder number excessive %d", VIDEO_CHANNEL_MAX);
        return ACLLITE_ERROR_TOO_MANY_VIDEO_DECODERS;
    }

    //Create dvpp vdec to decode h26x data
    dvppVdec_ = new VdecHelper(channelId_, ffmpegDecoder_->GetFrameWidth(), 
                                ffmpegDecoder_->GetFrameHeight(), 
                                streamFormat_, VideoCapture::DvppVdecCallback); 
    AclLiteError ret = dvppVdec_->Init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Dvpp vdec init failed");
    }

    return ret; 
}

AclLiteError VideoCapture::InitFFmpegDecoder() {
    //Create ffmpeg decoder to parse video stream to h26x frame data
    ffmpegDecoder_ = new FFmpegDecoder(streamName_);
    if (kInvalidTpye == GetVdecType()) {
        this->SetStatus(DECODE_ERROR);        
        delete ffmpegDecoder_;
        ACLLITE_LOG_ERROR("Video %s type is invalid", streamName_.c_str());
        return ACLLITE_ERROR_FFMPEG_DECODER_INIT;
    } 

    //Get video fps, if no fps, use 1 as default
    int fps =  ffmpegDecoder_->GetFps();
    if (fps == 0) {
        fps = kDefaultFps;
        ACLLITE_LOG_INFO("Video %s fps is 0, change to %d", 
                       streamName_.c_str(), fps);
    }
    //Cal the frame interval time(us)
    fpsInterval_ = kUsec / fps;

    return ACLLITE_OK;
}

AclLiteError VideoCapture::Open() {
    //Open video stream, if open failed before, return error directly
    if (status_ == DECODE_ERROR) 
        return ACLLITE_ERROR_OPEN_VIDEO_UNREADY;
    //If open ok already
    if (status_ != DECODE_UNINIT)
        return ACLLITE_OK;
    //Init acl resource
    AclLiteError ret = InitResource();
    if (ret != ACLLITE_OK) {
        this->SetStatus(DECODE_ERROR);
        ACLLITE_LOG_ERROR("Open %s failed for init resource error: %d", 
                        streamName_.c_str(), ret);
        return ret;
    }
    //Init ffmpeg decoder
    ret = InitFFmpegDecoder();
    if (ret != ACLLITE_OK) {
        this->SetStatus(DECODE_ERROR);
        ACLLITE_LOG_ERROR("Open %s failed for init ffmpeg error: %d", 
                        streamName_.c_str(), ret);
        return ret;
    }
    //Init dvpp vdec decoder
    ret = InitVdecDecoder();
    if (ret != ACLLITE_OK) {
        this->SetStatus(DECODE_ERROR);
        ACLLITE_LOG_ERROR("Open %s failed for init vdec error: %d", 
                        streamName_.c_str(), ret);
        return ret;
    }
    //Set init ok
    this->SetStatus(DECODE_READY);
    ACLLITE_LOG_INFO("Video %s decode init ok", streamName_.c_str());
    
    return ACLLITE_OK;
}

int VideoCapture::GetVdecType() {
    //VDEC only support H265 main level，264 baseline level，main level，high level
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
                ACLLITE_LOG_INFO("Not support h264 profile %d, use as mp", profile);
                streamFormat_ = H264_MAIN_LEVEL; 
                break;
        }
    } else {
        streamFormat_ = kInvalidTpye;
        ACLLITE_LOG_ERROR("Not support stream, type %d,  profile %d", type, profile);
    }

    return streamFormat_;
}

//dvpp vdec callback
void VideoCapture::DvppVdecCallback(acldvppStreamDesc *input, 
                                   acldvppPicDesc *output, void *userData)
{
    VideoCapture* decoder = (VideoCapture*)userData;
    //Get decoded image parameters
    shared_ptr<ImageData> image = make_shared<ImageData>();
    image->format = acldvppGetPicDescFormat(output);
    image->width = acldvppGetPicDescWidth(output);
    image->height = acldvppGetPicDescHeight(output);
    image->alignWidth = acldvppGetPicDescWidthStride(output);
    image->alignHeight = acldvppGetPicDescHeightStride(output);
    image->size = acldvppGetPicDescSize(output);

    void* vdecOutBufferDev = acldvppGetPicDescData(output);
    image->data = SHARED_PTR_DVPP_BUF(vdecOutBufferDev);

    //Put the decoded image to queue for read
    decoder->ProcessDecodedImage(image);
    //Release resouce
    aclError ret = acldvppDestroyPicDesc(output);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("fail to destroy pic desc, error %d", ret);
    }

    if (input != nullptr) {
        void* inputBuf = acldvppGetStreamDescData(input);
        if (inputBuf != nullptr) {
            acldvppFree(inputBuf);
        }
        
        aclError ret = acldvppDestroyStreamDesc(input);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("fail to destroy input stream desc");
        }
    }
}

void VideoCapture::ProcessDecodedImage(shared_ptr<ImageData> frameData) {
    finFrameCnt_++;
    if (YUV420SP_SIZE(frameData->width, frameData->height) != frameData->size) {
        ACLLITE_LOG_ERROR("Invalid decoded frame parameter, "
                        "width %d, height %d, size %d, buffer %p",
                        frameData->width, frameData->height, 
                        frameData->size, frameData->data.get());
        return;
    }

    FrameImageEnQueue(frameData);

    if ((status_ == DECODE_FFMPEG_FINISHED) && (finFrameCnt_ >= frameId_)) {
        ACLLITE_LOG_INFO("Last frame decoded by dvpp, change status to %d",
                       DECODE_DVPP_FINISHED);
        this->SetStatus(DECODE_DVPP_FINISHED);
    }    
}

AclLiteError VideoCapture::FrameImageEnQueue(shared_ptr<ImageData> frameData) {
    for (int count = 0; count < kFrameEnQueueRetryTimes; count++) {
        if (frameImageQueue_.Push(frameData)) 
            return ACLLITE_OK;
        usleep(kDecodeQueueOpWait); 
    }
    ACLLITE_LOG_ERROR("Video %s lost decoded image for queue full", 
	                streamName_.c_str());

    return ACLLITE_ERROR_VDEC_QUEUE_FULL;
}

//start decoder
void VideoCapture::StartFrameDecoder() {
    if (status_ == DECODE_READY) {

        decodeThread_ = thread(FrameDecodeThreadFunction, (void*)this);
        decodeThread_.detach();

        status_ = DECODE_START;
    }
}

//ffmpeg decoder entry
void VideoCapture::FrameDecodeThreadFunction(void* decoderSelf) {
    VideoCapture* thisPtr =  (VideoCapture*)decoderSelf;

    aclError aclRet = thisPtr->SetAclContext();
    if (aclRet != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Set frame decoder context failed, errorno:%d",
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
AclLiteError VideoCapture::FrameDecodeCallback(void* decoder, void* frameData, 
                                            int frameSize) {
    if ((frameData == NULL) || (frameSize == 0)) {
        ACLLITE_LOG_ERROR("Frame data is null");
        return ACLLITE_ERROR_H26X_FRAME;
    }

    //copy data to dvpp memory
    VideoCapture* videoDecoder = (VideoCapture*)decoder;

    void* buffer = CopyDataToDevice(frameData, frameSize,
                                    videoDecoder->runMode_, MEMORY_DVPP);  
    if (buffer == nullptr) {
        ACLLITE_LOG_ERROR("Copy frame h26x data to dvpp failed");
        return ACLLITE_ERROR_COPY_DATA;
    }

    shared_ptr<FrameData> videoFrame = make_shared<FrameData>();
    videoDecoder->frameId_++;
    videoFrame->frameId = videoDecoder->frameId_;
    videoFrame->data = buffer;
    videoFrame->size = frameSize;
    //decode data by dvpp vdec 
    AclLiteError ret = videoDecoder->dvppVdec_->Process(videoFrame, decoder);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Dvpp vdec process %dth frame failed, error:%d", 
                        videoDecoder->frameId_, ret);
        return ret;
    }

    //wait next frame by fps
    videoDecoder->SleeptoNextFrameTime();
    return ACLLITE_OK;
}

void VideoCapture::SleeptoNextFrameTime() {
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

    //get current time
    timeval tv;
    gettimeofday(&tv, 0);
    int64_t now = (int64_t)tv.tv_sec * 1000000 + (int64_t)tv.tv_usec;

    if (lastDecodeTime_ == 0) {
        lastDecodeTime_ = now;
        return;
    }
    //calculate interval
    int64_t lastInterval = (now - lastDecodeTime_);
    int64_t sleepTime = (lastInterval < fpsInterval_)?(fpsInterval_-lastInterval):0;
    //consume rest time
    usleep(sleepTime);
    //record start time of next frame
    gettimeofday(&tv, 0);
    lastDecodeTime_ = (int64_t)tv.tv_sec * 1000000 + (int64_t)tv.tv_usec;

    return;
}

//check decoder status
bool VideoCapture::IsOpened() { 
    ACLLITE_LOG_INFO("Video %s decode status %d", streamName_.c_str(), status_);
    return (status_ == DECODE_READY) || (status_ == DECODE_START);
}

//read decoded frame
AclLiteError VideoCapture::Read(ImageData& image) {
    //return nullptr,if decode fail/finish
    if (status_ == DECODE_ERROR) {
        ACLLITE_LOG_ERROR("Read failed for decode %s failed", 
                        streamName_.c_str()); 
        return ACLLITE_ERROR_VIDEO_DECODER_STATUS;
    }

    if (status_ == DECODE_FINISHED) {
        ACLLITE_LOG_INFO("No frame to read for decode %s finished", 
                        streamName_.c_str()); 
        return ACLLITE_ERROR_DECODE_FINISH;
    }
    //start decode if status is ok
    if (status_ == DECODE_READY) {
        StartFrameDecoder();
        usleep(kDecodeQueueOpWait);
    }
    //read frame from decode queue
    bool noWait = (status_ == DECODE_DVPP_FINISHED);
    shared_ptr<ImageData> frame = FrameImageOutQueue(noWait);
    if (noWait && (frame == nullptr)) {
        SetStatus(DECODE_FINISHED);
        ACLLITE_LOG_INFO("No frame to read anymore");
        return ACLLITE_ERROR_DECODE_FINISH;
    }

    if (frame == nullptr) {
        ACLLITE_LOG_ERROR("No frame image to read abnormally");
        return ACLLITE_ERROR_READ_EMPTY;
    }

    image.format = frame->format;
    image.width = frame->width;
    image.height = frame->height;
    image.alignWidth = frame->alignWidth;
    image.alignHeight = frame->alignHeight;
    image.size = frame->size;
    image.data = frame->data;

    return ACLLITE_OK;
}

shared_ptr<ImageData> VideoCapture::FrameImageOutQueue(bool noWait) {
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

AclLiteError VideoCapture::Set(StreamProperty key, int value) {
    AclLiteError ret = ACLLITE_OK;
    switch(key) {
        case OUTPUT_IMAGE_FORMAT:
            ret = dvppVdec_->SetFormat(value);
            break;
        case RTSP_TRANSPORT:
            ret = SetRtspTransType(value);
            break;
        default:
            ret = ACLLITE_ERROR_UNSURPPORT_PROPERTY;
            ACLLITE_LOG_ERROR("Unsurpport property %d to set for video %s",
                            (int)key, streamName_.c_str());
            break;
    }

    return ret;
}

AclLiteError VideoCapture::SetRtspTransType(uint32_t transCode) {
    AclLiteError ret = ACLLITE_OK;

    if (transCode == RTSP_TRANS_UDP)
        ffmpegDecoder_->SetTransport(RTSP_TRANSPORT_UDP);
    else if (transCode == RTSP_TRANS_TCP)    
        ffmpegDecoder_->SetTransport(RTSP_TRANSPORT_TCP);
    else {
        ret = ACLLITE_ERROR_INVALID_PROPERTY_VALUE;
        ACLLITE_LOG_ERROR("Unsurport rtsp transport property value %d", 
                        transCode);
    }

    return ret;
}

uint32_t VideoCapture::Get(StreamProperty key) {
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
            ACLLITE_LOG_ERROR("Unsurpport property %d to get for video", key);
            break;
    }

    return value;
}

AclLiteError VideoCapture::SetAclContext() {
    if (context_ == nullptr) {
        ACLLITE_LOG_ERROR("Video decoder context is null");
        return ACLLITE_ERROR_SET_ACL_CONTEXT;
    }
    
    aclError ret = aclrtSetCurrentContext(context_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Video decoder set context failed, error: %d", ret);
        return ACLLITE_ERROR_SET_ACL_CONTEXT;
    }
    
    return ACLLITE_OK;   
}

AclLiteError VideoCapture::Close() {
    DestroyResource();
    return ACLLITE_OK;
}