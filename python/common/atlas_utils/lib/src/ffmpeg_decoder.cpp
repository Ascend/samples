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
#include <fstream>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <thread>

#include "atlas_utils.h"
#include "ffmpeg_decoder.h"

using namespace std;

namespace {
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

    const uint32_t kDefaultFps = 5;

    const uint32_t kOneSecUs = 1000 * 1000;
}

FFmpegDecoder::FFmpegDecoder(const std::string& streamName)
:streamName_(streamName){
    rtspTransport_.assign(kUdp.c_str());
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
    ATLAS_LOG_INFO("Set parameters for %s", streamName_.c_str());

    av_dict_set(&avdic, kRtspTransport.c_str(), rtspTransport_.c_str(), kNoFlag);
    av_dict_set(&avdic, kBufferSize.c_str(), kMaxBufferSize.c_str(), kNoFlag);
    av_dict_set(&avdic, kMaxDelayStr.c_str(), kMaxDelayValue.c_str(), kNoFlag);
    av_dict_set(&avdic, kTimeoutStr.c_str(), kTimeoutValue.c_str(), kNoFlag);
    av_dict_set(&avdic, kReorderQueueSize.c_str(),
                kReorderQueueSizeValue.c_str(), kNoFlag);
    av_dict_set(&avdic, kPktSize.c_str(), kPktSizeValue.c_str(), kNoFlag);
    ATLAS_LOG_INFO("Set parameters for %s end", streamName_.c_str());
}

bool FFmpegDecoder::OpenVideo(AVFormatContext*& avFormatContext) {
    bool ret = true;
    AVDictionary* avdic = nullptr;

    ATLAS_LOG_INFO("Open video %s ...", streamName_.c_str());

    SetDictForRtsp(avdic);

    int openRet = avformat_open_input(&avFormatContext,
                                       streamName_.c_str(), nullptr,
                                       &avdic);
    if (openRet < 0) { // check open video result
        char buf_error[kErrorBufferSize];
        av_strerror(openRet, buf_error, kErrorBufferSize);

        ATLAS_LOG_ERROR("Could not open video:%s, return :%d, error info:%s",
                      streamName_.c_str(), openRet, buf_error);
        ret = false;
    }

    if (avdic != nullptr) { // free AVDictionary
        av_dict_free(&avdic);
    }
    ATLAS_LOG_INFO("Open video %s ok", streamName_.c_str());

    return ret;
}

bool FFmpegDecoder::InitVideoParams(int videoIndex, 
                                        AVFormatContext* avFormatContext,
                                        AVBSFContext*& bsfCtx) {
    const AVBitStreamFilter* videoFilter;
    InitVideoStreamFilter(videoFilter);
    if (videoFilter == nullptr) { // check video fileter is nullptr
        ATLAS_LOG_ERROR("Unkonw bitstream filter, videoFilter is nullptr!");
        return false;
    }

    // checke alloc bsf context result
    if (av_bsf_alloc(videoFilter, &bsfCtx) < 0) {
        ATLAS_LOG_ERROR("Fail to call av_bsf_alloc!");
        return false;
    }

    // check copy parameters result
    if (avcodec_parameters_copy(bsfCtx->par_in,
        avFormatContext->streams[videoIndex]->codecpar) < 0) {
        ATLAS_LOG_ERROR("Fail to call avcodec_parameters_copy!");
        return false;
    }

    bsfCtx->time_base_in = avFormatContext->streams[videoIndex]->time_base;

    // check initialize bsf contextreult
    if (av_bsf_init(bsfCtx) < 0) {
        ATLAS_LOG_ERROR("Fail to call av_bsf_init!");
        return false;
    }

    return true;
}


void FFmpegDecoder::Decode(FrameProcessCallBack callback, 
                               void *callbackParam) {
    ATLAS_LOG_INFO("Start ffmpeg decode video %s ...", streamName_.c_str());
    avformat_network_init(); // init network

    AVFormatContext* avFormatContext = avformat_alloc_context();

    // check open video result
    if (!OpenVideo(avFormatContext)) {
        return;
    }

    int videoIndex = GetVideoIndex(avFormatContext);
    if (videoIndex == kInvalidVideoIndex) { // check video index is valid
        ATLAS_LOG_ERROR("Rtsp %s index is -1", streamName_.c_str());
        return;
    }

    AVBSFContext* bsfCtx = nullptr;
    // check initialize video parameters result
    if (!InitVideoParams(videoIndex, avFormatContext, bsfCtx)) {
        return;
    }

    ATLAS_LOG_INFO("Start decode frame of video %s ...", streamName_.c_str());


    AVPacket avPacket;
    int processOk = true;
    // loop to get every frame from video stream
    while ((av_read_frame(avFormatContext, &avPacket) == 0) && processOk && !isStop_) {
        if (avPacket.stream_index == videoIndex) { // check current stream is video
          // send video packet to ffmpeg
            if (av_bsf_send_packet(bsfCtx, &avPacket)) {
                ATLAS_LOG_ERROR("Fail to call av_bsf_send_packet, channel id:%s",
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
    ATLAS_LOG_INFO("Ffmpeg decoder %s finished", streamName_.c_str());
}

void FFmpegDecoder::GetVideoInfo() {
    avformat_network_init(); // init network
    AVFormatContext* avFormatContext = avformat_alloc_context();
    bool ret = OpenVideo(avFormatContext);
    if (ret == false) {
        ATLAS_LOG_ERROR("Open %s failed", streamName_.c_str());
        return;
    }

    if (avformat_find_stream_info(avFormatContext,NULL)<0) {
		ATLAS_LOG_ERROR("Get stream info of %s failed", streamName_.c_str());
		return;
	}

    int videoIndex = GetVideoIndex(avFormatContext);
    if (videoIndex == kInvalidVideoIndex) { // check video index is valid
        ATLAS_LOG_ERROR("Video index is %d, current media stream has no "
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
        fps_ = kDefaultFps;
    }

    videoType_ = inStream->codecpar->codec_id;
    profile_ = inStream->codecpar->profile;

    avformat_close_input(&avFormatContext);

    ATLAS_LOG_INFO("Video %s, type %d, profile %d, width:%d, height:%d, fps:%d",
                 streamName_.c_str(), videoType_, profile_, frameWidth_, frameHeight_, fps_);
    return;
}










