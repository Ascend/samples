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

#ifndef VIDEO_DECODE_H_
#define VIDEO_DECODE_H_
#ifndef VIDEO_FRAME_DECODE_H_
#define VIDEO_fRAME_DECODE_H_

#include <dirent.h>
#include <stdint.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <thread>
#include "ThreadSafeQueue.h"
#include "VdecHelper.h"
#include "AclLiteVideoProc.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
}

#define INVALID_CHANNEL_ID -1
#define INVALID_STREAM_FORMAT -1
#define VIDEO_CHANNEL_MAX  23
#define RTSP_TRANSPORT_UDP "udp"
#define RTSP_TRANSPORT_TCP "tcp"

typedef int (*FrameProcessCallBack)(void* callback_param, void *frame_data,
                                    int frame_size);

enum StreamType {
    STREAM_VIDEO = 0,
    STREAM_RTSP,
};

enum DecodeStatus {
    DECODE_ERROR  = -1,
    DECODE_UNINIT = 0,
    DECODE_READY  = 1,
    DECODE_START  = 2,
    DECODE_FFMPEG_FINISHED = 3,
    DECODE_DVPP_FINISHED = 4,
    DECODE_FINISHED = 5
};

class ChannelIdGenerator {
public:
    ChannelIdGenerator() {
        for (int i = 0; i < VIDEO_CHANNEL_MAX; i++) {
            channelId_[i] = INVALID_CHANNEL_ID;
        }
    }
    ~ChannelIdGenerator(){};
 
    int GenerateChannelId(void) {
        std::lock_guard<std::mutex> lock(mutex_lock_);
        for (int i = 0; i < VIDEO_CHANNEL_MAX; i++) {
            if (channelId_[i] == INVALID_CHANNEL_ID) {
                channelId_[i] = i;
                return i;
            }
        }

        return INVALID_CHANNEL_ID;
    }

    void ReleaseChannelId(int channelId) {
        std::lock_guard<std::mutex> lock(mutex_lock_);
        if ((channelId >= 0) && (channelId < VIDEO_CHANNEL_MAX)) {
            channelId_[channelId] = INVALID_CHANNEL_ID;
        }
    }

private:
    int channelId_[VIDEO_CHANNEL_MAX];  
    mutable std::mutex mutex_lock_;
};

class FFmpegDecoder{
public:
    FFmpegDecoder(const std::string& name);
    ~FFmpegDecoder(){}  
    void Decode(FrameProcessCallBack callback_func, void *callback_param);                
    int GetFrameWidth() { return frameWidth_; }
    int GetFrameHeight() { return frameHeight_; }
    int GetVideoType() { return videoType_; }
    int GetFps() { return fps_; }
    int IsFinished() { return isFinished_; };
    int GetProfile() { return profile_; }
    void SetTransport(const std::string& transportType);
    void StopDecode(){ isStop_ = true; }

private: 
    int GetVideoIndex(AVFormatContext* av_format_context);
    void GetVideoInfo();
    void InitVideoStreamFilter(const AVBitStreamFilter* &video_filter);
    bool OpenVideo(AVFormatContext*& av_format_context);
    void SetDictForRtsp(AVDictionary* &avdic);
    bool InitVideoParams(int videoIndex, 
                         AVFormatContext* av_format_context,
                         AVBSFContext* &bsf_ctx);

private:
    bool isFinished_;
    bool isStop_;
    int frameWidth_;
    int frameHeight_;
    int videoType_;
    int profile_;
    int fps_;
    std::string streamName_;  
    std::string rtspTransport_;
};

class VideoCapture : public AclLiteVideoCapBase {
 public:
    /**
     * @brief VideoCapture constructor
     */
    VideoCapture(const std::string& videoName, aclrtContext context = nullptr);

    /**
     * @brief VideoCapture destructor
     */
    ~VideoCapture();

    static void FrameDecodeThreadFunction(void* decoderSelf); 
    static AclLiteError FrameDecodeCallback(void* context, void* frameData, 
                                          int frameSize);
    static void DvppVdecCallback(acldvppStreamDesc *input, 
                                 acldvppPicDesc *output, void *userdata);

    AclLiteError DecodeH26xFrame();
    void ProcessDecodedImage(std::shared_ptr<ImageData> frameData);
    AclLiteError Read(ImageData& image);

    void FFmpegDecode() { ffmpegDecoder_->Decode(&VideoCapture::FrameDecodeCallback, (void*) this); }

    bool IsOpened();
    AclLiteError Open();

    void SetStatus(DecodeStatus status) { status_ = status; }
    DecodeStatus GetStatus() { return status_; }
    
    AclLiteError Set(StreamProperty key, int value);
    uint32_t Get(StreamProperty key);

    void SleeptoNextFrameTime();
    AclLiteError SetAclContext();
    AclLiteError Close();
    
    void DestroyResource();
    bool IsStop() { return isStop_; }
    bool IsJam() { return isJam_; }

private:  
    AclLiteError InitResource();
    AclLiteError InitVdecDecoder();
    AclLiteError InitFFmpegDecoder();
    void StartFrameDecoder();  
    int GetVdecType();
    AclLiteError FrameImageEnQueue(std::shared_ptr<ImageData> frameData);
    std::shared_ptr<ImageData> FrameImageOutQueue(bool noWait = false);
    AclLiteError SetRtspTransType(uint32_t transCode);
	
private:
    bool isStop_;
    bool isReleased_;
    bool isJam_;
    StreamType streamType_;
    DecodeStatus status_;
    aclrtContext context_;
    aclrtRunMode runMode_;
    int channelId_;
    int streamFormat_;
    uint32_t frameId_;
    uint32_t finFrameCnt_;
    int64_t lastDecodeTime_;
    int64_t fpsInterval_;
    std::string streamName_;
    std::thread decodeThread_;
    FFmpegDecoder* ffmpegDecoder_;
    VdecHelper* dvppVdec_;
    ThreadSafeQueue<std::shared_ptr<ImageData>> frameImageQueue_;
};

#endif /* VIDEO_FRAME_DECODE_H_ */
#endif /* VIDEO_DECODE_H_ */
