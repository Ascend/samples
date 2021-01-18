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

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
}

typedef int (*FrameProcessCallBack)(void* callback_param, void *frame_data,
                                    int frame_size);

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
    int frameWidth_;
    int frameHeight_;
    int videoType_;
    int profile_;
    int fps_;
    std::string streamName_;  
    std::string rtspTransport_;

    bool isStop_;
};

#endif /* VIDEO_DECODE_H_ */
