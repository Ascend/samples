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

using namespace std;



typedef void (*FrameProcessCallBack)(void* callback_param, void *frame_data,
	                                 int frame_size);

class VideoFrameDecoder{
public:
    /**
     * @brief VideoDecode constructor
     */
    VideoFrameDecoder(const std::string& name, 
		              const std::string& transport)
    :rtsp_transport_(transport)
	,stream_name_(name){

    }

    /**
     * @brief VideoDecode destructor
     */
    ~VideoFrameDecoder() {};
    
    bool VerifyVideoType(int& format);
    int Decode(FrameProcessCallBack callback_func, 
                void *callback_param);
private:  
    std::string stream_name_;  
    std::string rtsp_transport_;
    /**
     * @brief get video index form video format context
     * @param [in] av_format_context: video format context
     * @return video index
     */
    int GetVideoIndex(AVFormatContext* av_format_context);

    bool CheckVideoType(int video_index,
                        AVFormatContext* av_format_context,
                        VideoType& video_type);    
    /**
     * @brief initialize video stream filter
     * @param [in] video_type: video type
     * @param [in] video_filter: video filter
     */
    void InitVideoStreamFilter(VideoType video_type,
                               const AVBitStreamFilter* &video_filter);

    bool OpenVideo(AVFormatContext*& av_format_context);

    /**
     * @brief set dictionary for rtsp
     * @param [in] channel_value: the input channel value
     * @param [in] avdic: video dictionary
     */
    void SetDictForRtsp(AVDictionary* &avdic);



    /**
     * @brief open video format from input channel
     * @param [in] videoindex: the video index
     * @param [out] video_type: the input channel value
     * @param [in] av_format_context: the video format context
     * @param [out] bsf_ctx: video bsf context
     * @return true: success to initialize; false: fail to initialize
     */
    bool InitVideoParams(int videoindex, 
                        VideoType &video_type,
                        AVFormatContext* av_format_context,
                        AVBSFContext* &bsf_ctx);

};

#endif /* VIDEO_DECODE_H_ */
