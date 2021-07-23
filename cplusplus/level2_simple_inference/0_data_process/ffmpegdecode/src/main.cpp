/**
* Copyright 2020 Huawei Technologies Co., Ltd
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

* File main.cpp
* Description: dvpp sample main func
*/
#include <cstdint>
#include <iostream>
#include <stdlib.h>
#include <dirent.h>
#include <unistd.h>
#include <memory>
#include <vector>
#include <thread>
#include "main.h"
#include <string>
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
}

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
    const int64_t kUsec = 1000000;
    const int kOutputJamWait = 10000;
    const int kInvalidTpye = -1;
}

bool isFinished_;
int frameWidth_;
int frameHeight_;
int videoType_;
int profile_;
int fps_;
int streamFormat_ = H264_MAIN_LEVEL;
std::string streamName_;  
std::string rtspTransport_;
aclrtRunMode runMode_;
bool isStop_;

int GetVideoIndex(AVFormatContext* avFormatContext) {
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

void SetDictForRtsp(AVDictionary*& avdic) {
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

bool OpenVideo(AVFormatContext*& avFormatContext) {
    bool ret = true;
    AVDictionary* avdic = nullptr;
    av_log_set_level(AV_LOG_DEBUG);
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
    return ret;
}

void GetVideoInfo() {
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

void FFmpegDecoder(){
    rtspTransport_.assign(kTcp.c_str());
    isFinished_ = false;
    isStop_ = false;
    GetVideoInfo();
}

int GetVdecType() {
    //VDEC only support　H265 main level，264 baseline level，main level，high level
    if (videoType_ == AV_CODEC_ID_HEVC) {        
        streamFormat_ = H265_MAIN_LEVEL;         
    } else if (videoType_ == AV_CODEC_ID_H264) {
        switch(profile_) {
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
                ATLAS_LOG_INFO("Not support h264 profile %d, use as mp", profile_);
                streamFormat_ = H264_MAIN_LEVEL; 
                break;
        }
    } else {
        streamFormat_ = kInvalidTpye;
        ATLAS_LOG_ERROR("Not support stream, type %d,  profile %d", videoType_, profile_);
    }

    return streamFormat_;
}

AtlasError FrameDecodeCallback(void* frameData, int frameSize) {
    if ((frameData == NULL) || (frameSize == 0)) {
        ATLAS_LOG_ERROR("Frame data is null");
        return ATLAS_ERROR_H26X_FRAME;
    }

    //将ffmpeg解码得到的h26x数据拷贝到dvpp内存
    void* buffer = CopyDataToDevice(frameData, frameSize,
                                    runMode_, MEMORY_DVPP);  
    if (buffer == nullptr) {
        ATLAS_LOG_ERROR("Copy frame h26x data to dvpp failed");
        return ATLAS_ERROR_COPY_DATA;
    }

    shared_ptr<FrameData> videoFrame = make_shared<FrameData>();
    videoFrame->data = buffer;
    videoFrame->size = frameSize;

    usleep(kOutputJamWait);
    return ATLAS_OK;
}

void InitVideoStreamFilter(const AVBitStreamFilter*& videoFilter) {
    if (videoType_ == AV_CODEC_ID_H264) { // check video type is h264
        videoFilter = av_bsf_get_by_name("h264_mp4toannexb");
    }
    else { // the video type is h265
        videoFilter = av_bsf_get_by_name("hevc_mp4toannexb");
    }
}

bool InitVideoParams(int videoIndex, 
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

void Decode() {
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
                int ret = FrameDecodeCallback(avPacket.data, avPacket.size);
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

int main(int argc, char *argv[]) {
    //Check the input when the application executes, which takes the path to the input video file
    if((argc < 2) || (argv[1] == nullptr)){
        ATLAS_LOG_ERROR("Please input: ./main <image_dir>");
        return FAILED;
    }

    streamName_ = string(argv[1]);
    AclDevice aclDev;
    AtlasError ret = aclDev.Init();
    if (ret) {
        ATLAS_LOG_ERROR("Init resource failed, error %d", ret);
        return ATLAS_ERROR;
    } 
    runMode_ = aclDev.GetRunMode();  

    //intialize ffmpeg decoder
    FFmpegDecoder();
    //verify video type
    if (kInvalidTpye == GetVdecType()) {      
        ATLAS_LOG_ERROR("Video %s type is invalid", streamName_.c_str());
    } 

    //Get video fps, if no fps, use 1 as default
    if (fps_ == 0) {
        fps_ = kDefaultFps;
        ATLAS_LOG_INFO("Video %s fps is 0, change to %d", 
                       streamName_.c_str(), fps_);
    }
    //Call the frame interval time(us)
    int fpsInterval_ = kUsec / fps_;
    Decode();

    return SUCCESS;
}
