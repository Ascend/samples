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
#include "ffmpeg_decoder.h"
#include "vdec_process.h"

#define INVALID_STREAM_FORMAT -1

#define RTSP_TRANSPORT_UDP "udp"
#define RTSP_TRANSPORT_TCP "tcp"

#define RTSP_TRANS_UDP ((uint32_t)0)
#define RTSP_TRANS_TCP ((uint32_t)1)

#define VIDEO_CHANNEL_MAX  16
#define INVALID_CHANNEL_ID -1

enum StreamProperty {   
    FRAME_WIDTH = 1,
    FRAME_HEIGHT = 2,
    VIDEO_FPS = 3,
    OUTPUT_IMAGE_FORMAT = 4,
    RTSP_TRANSPORT = 5
};

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

class VideoDecode {
 public:

    /**
     * @brief VideoDecode constructor
     */
    VideoDecode(const std::string& videoName, int channelId);         
                       

    /**
     * @brief VideoDecode destructor
     */
    ~VideoDecode();

    static void DecodeThreadFunction(void* decoderSelf); 
    static AtlasError FrameDecodeCallback(void* context, void* frameData, 
                                          int frameSize);
    static void DvppVdecCallback(acldvppStreamDesc *input, 
                                 acldvppPicDesc *output, void *userdata);

    AtlasError DecodeH26xFrame();
    void ProcessDecodedImage(std::shared_ptr<ImageData> frameData);
    AtlasError Read(ImageData& image);

    void FFmpegDecode(){ffmpegDecoder_->Decode(&VideoDecode::FrameDecodeCallback, (void*) this);}

    bool IsOpened();
    AtlasError Open();

    AtlasError Init();

    void SetStatus(DecodeStatus status) { status_ = status; }
    DecodeStatus GetStatus() { return status_; }
    
    AtlasError Set(StreamProperty key, int value);
    uint32_t Get(StreamProperty key);

    void SleepToNextFrameTime();
    AtlasError SetAclContext();
    AtlasError Close();
    
    void DestroyResource();
    bool IsStop() { return isStop_; }

private:  
    AtlasError InitResource();
    AtlasError InitVdecDecoder();
    AtlasError InitFFmpegDecoder();
    void StartDecode() { status_ = DECODE_START; }  
    int GetVdecType();
    AtlasError FrameImageEnQueue(std::shared_ptr<ImageData> frameData);
    std::shared_ptr<ImageData> FrameImageOutQueue(bool noWait = false);
    AtlasError SetRtspTransType(uint32_t transCode);
	
private:
    aclrtContext context_;
    aclrtRunMode runMode_;
    StreamType streamType_;

    int channelId_;  
    uint32_t frameId_;
    uint32_t finFrameCnt_;
    DecodeStatus status_;
    std::string streamName_;
    int streamFormat_;
    int64_t lastDecodeTime_;
    int64_t fpsInterval_;
    std::thread decodeThread_;
    FFmpegDecoder* ffmpegDecoder_;
    VdecProcess* dvppVdec_;

    ThreadSafeQueue<std::shared_ptr<ImageData>> frameImageQueue_;
    
    bool isStop_;
    bool isReleased_;
};

#endif /* VIDEO_DECODE_H_ */
