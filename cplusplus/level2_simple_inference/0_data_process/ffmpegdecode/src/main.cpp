/**
* Copyright (c) Huawei Technologies Co., Ltd. 2022-2022. All rights reserved.
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
    const int K_NO_FLAG = 0; // no flag
    const int K_INVALID_VIDEO_INDEX = -1; // invalid video index
    const int K_ARGV_NUM(2);
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
    const int K_ERROR_BUFFER_SIZE = 1024; // buffer size for error info
    const uint32_t K_DEFAULT_FPS = 5;
    const int64_t K_USEC = 1000000;
    const int K_OUTPUT_JAM_WAIT = 10000;
    const int K_INVALID_TYPE = -1;
}

bool g_isFinished;
int g_frameWidth;
int g_frameHeight;
int g_videoType;
int g_profile;
int g_fps;
int g_streamFormat = H264_MAIN_LEVEL;
std::string streamName_;
std::string rtspTransport_;
aclrtRunMode runMode_;
bool g_isStop;

int GetVideoIndex(AVFormatContext* avFormatContext)
{
    if (avFormatContext == nullptr) { // verify input pointer
        return K_INVALID_VIDEO_INDEX;
    }

    // get video index in streams
    for (uint32_t i = 0; i < avFormatContext->nb_streams; i++) {
        if (avFormatContext->streams[i]->codecpar->codec_type
            == AVMEDIA_TYPE_VIDEO) { // check is media type is video
            return i;
        }
    }

    return K_INVALID_VIDEO_INDEX;
}

void SetDictForRtsp(AVDictionary*& avdic)
{
    ACLLITE_LOG_INFO("Set parameters for %s", streamName_.c_str());
    av_dict_set(&avdic, kRtspTransport.c_str(), rtspTransport_.c_str(), K_NO_FLAG);
    av_dict_set(&avdic, kBufferSize.c_str(), kMaxBufferSize.c_str(), K_NO_FLAG);
    av_dict_set(&avdic, kMaxDelayStr.c_str(), kMaxDelayValue.c_str(), K_NO_FLAG);
    av_dict_set(&avdic, kTimeoutStr.c_str(), kTimeoutValue.c_str(), K_NO_FLAG);
    av_dict_set(&avdic, kReorderQueueSize.c_str(),
                kReorderQueueSizeValue.c_str(), K_NO_FLAG);
    av_dict_set(&avdic, kPktSize.c_str(), kPktSizeValue.c_str(), K_NO_FLAG);
    ACLLITE_LOG_INFO("Set parameters for %s end", streamName_.c_str());
}

bool OpenVideo(AVFormatContext*& avFormatContext)
{
    bool ret = true;
    AVDictionary* avdic = nullptr;
    av_log_set_level(AV_LOG_DEBUG);
    ACLLITE_LOG_INFO("Open video %s ...", streamName_.c_str());
    SetDictForRtsp(avdic);
    int openRet = avformat_open_input(&avFormatContext,
                                      streamName_.c_str(), nullptr,
                                      &avdic);
    if (openRet < 0) { // check open video result
        char bufError[K_ERROR_BUFFER_SIZE];
        av_strerror(openRet, bufError, K_ERROR_BUFFER_SIZE);
        ACLLITE_LOG_ERROR("Could not open video:%s, return :%d, error info:%s",
                          streamName_.c_str(), openRet, bufError);
        ret = false;
    }
    if (avdic != nullptr) { // free AVDictionary
        av_dict_free(&avdic);
    }
    return ret;
}

void GetVideoInfo()
{
    avformat_network_init(); // init network
    AVFormatContext* avFormatContext = avformat_alloc_context();
    bool ret = OpenVideo(avFormatContext);
    if (ret == false) {
        ACLLITE_LOG_ERROR("Open %s failed", streamName_.c_str());
        return;
    }

    if (avformat_find_stream_info(avFormatContext, NULL) < 0) {
        ACLLITE_LOG_ERROR("Get stream info of %s failed", streamName_.c_str());
        return;
    }

    int videoIndex = GetVideoIndex(avFormatContext);
    if (videoIndex == K_INVALID_VIDEO_INDEX) { // check video index is valid
        ACLLITE_LOG_ERROR("Video index is %d, current media stream has no "
                        "video info:%s",
                          K_INVALID_VIDEO_INDEX, streamName_.c_str());

        avformat_close_input(&avFormatContext);
        return;
    }

    AVStream* inStream = avFormatContext->streams[videoIndex];
    g_frameWidth = inStream->codecpar->width;
    g_frameHeight = inStream->codecpar->height;
    if (inStream->avg_frame_rate.den) {
        g_fps = inStream->avg_frame_rate.num / inStream->avg_frame_rate.den;
    } else {
        g_fps = K_DEFAULT_FPS;
    }

    g_videoType = inStream->codecpar->codec_id;
    g_profile = inStream->codecpar->profile;
    avformat_close_input(&avFormatContext);
    ACLLITE_LOG_INFO("Video %s, type %d, profile %d, width:%d, height:%d, fps:%d",
                     streamName_.c_str(), g_videoType, g_profile, g_frameWidth, g_frameHeight, g_fps);
    return;
}

void FFmpegDecoder()
{
    rtspTransport_.assign(kTcp.c_str());
    g_isFinished = false;
    g_isStop = false;
    GetVideoInfo();
}

int GetVdecType()
{
    // VDEC only support　H265 main level，264 baseline level，main level，high level
    if (g_videoType == AV_CODEC_ID_HEVC) {
        g_streamFormat = H265_MAIN_LEVEL;
    } else if (g_videoType == AV_CODEC_ID_H264) {
        switch (g_profile) {
            case FF_PROFILE_H264_BASELINE:
                g_streamFormat = H264_BASELINE_LEVEL;
                break;
            case FF_PROFILE_H264_MAIN:
                g_streamFormat = H264_MAIN_LEVEL;
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
                g_streamFormat = H264_HIGH_LEVEL;
                break;
            default:
                ACLLITE_LOG_INFO("Not support h264 profile %d, use as mp", g_profile);
                g_streamFormat = H264_MAIN_LEVEL;
                break;
        }
    } else {
        g_streamFormat = K_INVALID_TYPE;
        ACLLITE_LOG_ERROR("Not support stream, type %d,  profile %d", g_videoType, g_profile);
    }

    return g_streamFormat;
}

AclLiteError FrameDecodeCallback(void* frameData, int frameSize)
{
    if ((frameData == NULL) || (frameSize == 0)) {
        ACLLITE_LOG_ERROR("Frame data is null");
        return ACLLITE_ERROR_H26X_FRAME;
    }

    // 将ffmpeg解码得到的h26x数据拷贝到dvpp内存
    void* buffer = CopyDataToDevice(frameData, frameSize,
                                    runMode_, MEMORY_DVPP);
    if (buffer == nullptr) {
        ACLLITE_LOG_ERROR("Copy frame h26x data to dvpp failed");
        return ACLLITE_ERROR_COPY_DATA;
    }

    shared_ptr<FrameData> videoFrame = make_shared<FrameData>();
    videoFrame->data = buffer;
    videoFrame->size = frameSize;

    usleep(K_OUTPUT_JAM_WAIT);
    return ACLLITE_OK;
}

void InitVideoStreamFilter(const AVBitStreamFilter*& videoFilter)
{
    if (g_videoType == AV_CODEC_ID_H264) { // check video type is h264
        videoFilter = av_bsf_get_by_name("h264_mp4toannexb");
    } else { // the video type is h265
        videoFilter = av_bsf_get_by_name("hevc_mp4toannexb");
    }
}

bool InitVideoParams(int videoIndex,
                     AVFormatContext* avFormatContext,
                     AVBSFContext*& bsfCtx)
{
    const AVBitStreamFilter* videoFilter = nullptr;
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

void Decode()
{
    ACLLITE_LOG_INFO("Start ffmpeg decode video %s ...", streamName_.c_str());
    avformat_network_init(); // init network
    AVFormatContext* avFormatContext = avformat_alloc_context();

    // check open video result
    if (!OpenVideo(avFormatContext)) {
        return;
    }

    int videoIndex = GetVideoIndex(avFormatContext);
    if (videoIndex == K_INVALID_VIDEO_INDEX) { // check video index is valid
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
    while ((av_read_frame(avFormatContext, &avPacket) == 0) && processOk && !g_isStop) {
        if (avPacket.stream_index == videoIndex) { // check current stream is video
          // send video packet to ffmpeg
            if (av_bsf_send_packet(bsfCtx, &avPacket)) {
                ACLLITE_LOG_ERROR("Fail to call av_bsf_send_packet, channel id:%s",
                    streamName_.c_str());
            }

            // receive single frame from ffmpeg
            while ((av_bsf_receive_packet(bsfCtx, &avPacket) == 0) && !g_isStop) {
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

    g_isFinished = true;
    ACLLITE_LOG_INFO("Ffmpeg decoder %s finished", streamName_.c_str());
}

int main(int argc, char *argv[])
{
    // Check the input when the application executes, which takes the path to the input video file
    if ((argc < K_ARGV_NUM) || (argv[1] == nullptr)) {
        ACLLITE_LOG_ERROR("Please input: ./main <file_dir>");
        return FAILED;
    }

    streamName_ = string(argv[1]);
    AclLiteResource aclDev;
    AclLiteError ret = aclDev.Init();
    if (ret) {
        ACLLITE_LOG_ERROR("Init resource failed, error %d", ret);
        return ACLLITE_ERROR;
    }
    runMode_ = aclDev.GetRunMode();

    // intialize ffmpeg decoder
    FFmpegDecoder();
    // verify video type
    if (K_INVALID_TYPE == GetVdecType()) {
        ACLLITE_LOG_ERROR("Video %s type is invalid", streamName_.c_str());
    }

    // Get video fps, if no fps, use 1 as default
    if (g_fps == 0) {
        g_fps = K_DEFAULT_FPS;
        ACLLITE_LOG_INFO("Video %s fps is 0, change to %d",
                         streamName_.c_str(), g_fps);
    }

    Decode();

    return SUCCESS;
}
