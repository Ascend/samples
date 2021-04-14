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

#include <dirent.h>
#include <stdint.h>
#include <unistd.h>

#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <thread>

#include "thread_safe_queue.h"
#include "video_frame_decoder.h"
#include "vdec_process.h"

#define INVALID_STREAM_FORMAT -1

#define RTSP_TRANSPORT_UDP "udp"
#define RTSP_TRANSPORT_TCP "tcp"

enum StreamProperty {
    VIDEO_TRANSPORT = 1,
    FRAME_WIDTH = 2,
    FRAME_HEIGHT = 3,
    IMAGE_ENTYPE = 4,
    IMAGE_FORMAT = 5,
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

class VideoDecode {
 public:

    /**
     * @brief VideoDecode constructor
     */
    VideoDecode(const std::string& videoName, 
                uint32_t channelId, aclrtContext context);

    /**
     * @brief VideoDecode destructor
     */
    ~VideoDecode();

    static void FrameDecodeThreadFunction(void* decoderSelf); 
    static void FrameDecodeCallback(void* context, void* frameData, 
                                    int frameSize);
    static void DvppVdecThreadFunc(void* context);
    static void DvppVdecCallback(acldvppStreamDesc *input, 
                                 acldvppPicDesc *output, void *userdata);

    Result DecodeH26xFrame();
    void ProcessDecodedImage(shared_ptr<ImageData> frameData);
    shared_ptr<ImageData> Read();

    void FFmpegDecode(){frameDecoder_->Decode(&VideoDecode::FrameDecodeCallback, (void*) this);};
    bool IsOpened();
    int IsFinished() { return status_ == DECODE_FINISHED; };
    void set_status(DecodeStatus status) { status_ = status; }
    int GetChannelId() { return channelId_; }
    int GetVideoType() { return frameDecoder_->GetVideoType(); }
    int GetFrameWidth() { return frameDecoder_->GetFrameWidth(); }
    int GetFrameHeight() { return frameDecoder_->GetFrameHeight(); }
    int GetFps() { return frameDecoder_->GetFps(); }
    Result Set(StreamProperty key, std::string& value);
    Result Set(StreamProperty key, int value);
    int InitDvppVdec() { return dvppVdec_->init(); }
    bool IsDecodeFinished() { return (status_ >= DECODE_DVPP_FINISHED); }
    void SleeptoNextFrameTime();
    aclError SetAclContext();
    bool IsVdecExitReady() {return dvppVdec_->is_exit_ready();}

private:
    int init();
    void StartFrameDecoder();
    bool IsVdecProcessReady();    
    bool StartupVdecProcess();
    int GetVdecType();
    Result FrameH26xEnQueue(shared_ptr<FrameData> frameData);
    Result FrameImageEnQueue(shared_ptr<ImageData> frameData);
    shared_ptr<ImageData> FrameImageOutQueue();
    shared_ptr<FrameData> FrameH26xOutQueue();
	
private:
    aclrtContext context_;
    aclrtRunMode runModel_;

    int channelId_;  
    uint32_t frameId_;
    uint32_t finFrameCnt_;
    DecodeStatus status_;
    std::string streamName_;
    int streamFormat_;
    int64_t lastDecodeTime_;
    int64_t fpsInterval_;
    thread decodeThread_;
    thread dvppVdecThread_;  
    VideoFrameDecoder* frameDecoder_;
    VdecProcess* dvppVdec_;
    ThreadSafeQueue<shared_ptr<FrameData>> frameH26xQueue_;
    ThreadSafeQueue<shared_ptr<ImageData>> frameImageQueue_;
};

#endif /* VIDEO_DECODE_H_ */
