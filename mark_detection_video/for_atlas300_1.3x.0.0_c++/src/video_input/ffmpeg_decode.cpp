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

#include "data_type.h"
#include "tool_api.h"
#include "ffmpeg_decode.h"

using namespace std;

namespace {
	const int kNoFlag = 0; // no flag

	const int kInvalidVideoIndex = -1; // invalid video index

	const string kRtspTransport = "rtsp_transport"; // rtsp transport

	const string kUdp = "udp"; // video format udp

	const string kBufferSize = "buffer_size"; // buffer size string

	const string kMaxBufferSize = "104857600"; // maximum buffer size:100MB

	const string kMaxDelayStr = "max_delay"; // maximum delay string

	const string kMaxDelayValue = "100000000"; // maximum delay time:100s

	const string kTimeoutStr = "stimeout"; // timeout string

	const string kTimeoutValue = "5000000"; // timeout:5s

	const string kPktSize = "pkt_size"; // ffmpeg pakect size string

	const string kPktSizeValue = "10485760"; // ffmpeg packet size value:10MB

	const string kReorderQueueSize = "reorder_queue_size"; // reorder queue size

	const string kReorderQueueSizeValue = "0"; // reorder queue size value

	const int kErrorBufferSize = 1024; // buffer size for error info
}

int VideoFrameDecoder::GetVideoIndex(AVFormatContext* av_format_context) {
	if (av_format_context == nullptr) { // verify input pointer
		return kInvalidVideoIndex;
	}

	// get video index in streams
	for (uint32_t i = 0; i < av_format_context->nb_streams; i++) {
		if (av_format_context->streams[i]->codecpar->codec_type
			== AVMEDIA_TYPE_VIDEO) { // check is media type is video
			return i;
		}
	}

	return kInvalidVideoIndex;
}

bool VideoFrameDecoder::CheckVideoType(int video_index,
								AVFormatContext* av_format_context,
								VideoType& video_type) {
	AVStream* in_stream = av_format_context->streams[video_index];
	// INFO_LOG("Display video stream resolution, width:%d, height:%d",
	// 	         in_stream->codecpar->width, in_stream->codecpar->height);

	if (in_stream->codecpar->codec_id == AV_CODEC_ID_H264) { // video type: h264
		video_type = kH264;
		// INFO_LOG("Video type:H264");
		return true;
	}
	else if (in_stream->codecpar->codec_id == AV_CODEC_ID_HEVC) { // h265
		video_type = kH265;
		INFO_LOG("Video type:H265");
		return true;
	}
	else { // the video type is invalid
		video_type = kInvalidTpye;
		ERROR_LOG("The video type %d is not h264 or h265", in_stream->codecpar->codec_id);
		return false;
	}
}

void VideoFrameDecoder::InitVideoStreamFilter(
	VideoType video_type, const AVBitStreamFilter*& video_filter) {
	if (video_type == kH264) { // check video type is h264
		video_filter = av_bsf_get_by_name("h264_mp4toannexb");
	}
	else { // the video type is h265
		video_filter = av_bsf_get_by_name("hevc_mp4toannexb");
	}
}

void VideoFrameDecoder::SetDictForRtsp(AVDictionary*& avdic) {
	// INFO_LOG("Set parameters for %s", stream_name_.c_str());
	avformat_network_init();

	av_dict_set(&avdic, kRtspTransport.c_str(), rtsp_transport_.c_str(), kNoFlag);
	av_dict_set(&avdic, kBufferSize.c_str(), kMaxBufferSize.c_str(), kNoFlag);
	av_dict_set(&avdic, kMaxDelayStr.c_str(), kMaxDelayValue.c_str(), kNoFlag);
	av_dict_set(&avdic, kTimeoutStr.c_str(), kTimeoutValue.c_str(), kNoFlag);
	av_dict_set(&avdic, kReorderQueueSize.c_str(),
		        kReorderQueueSizeValue.c_str(), kNoFlag);
	av_dict_set(&avdic, kPktSize.c_str(), kPktSizeValue.c_str(), kNoFlag);

}

bool VideoFrameDecoder::OpenVideo(AVFormatContext*& av_format_context) {
	bool ret = true;
	AVDictionary* avdic = nullptr;

	SetDictForRtsp(avdic);

	int open_ret = avformat_open_input(&av_format_context,
                                       stream_name_.c_str(), nullptr,
                                       &avdic);
  	if (open_ret < 0) { // check open video result
		char buf_error[kErrorBufferSize];
		av_strerror(open_ret, buf_error, kErrorBufferSize);

		ERROR_LOG("Could not open video:%s, return :%d, error info:%s",
					  stream_name_.c_str(), open_ret, buf_error);
    	ret = false;
	}

	if (avdic != nullptr) { // free AVDictionary
		av_dict_free(&avdic);
	}

	return ret;
}

bool VideoFrameDecoder::InitVideoParams(int videoindex, 
                                        VideoType& video_type,
										AVFormatContext* av_format_context,
										AVBSFContext*& bsf_ctx) {
	// check video type, only support h264 and h265
	if (!CheckVideoType(videoindex, av_format_context, video_type)) {
		avformat_close_input(&av_format_context);

		return false;
	}

	const AVBitStreamFilter* video_filter;
	InitVideoStreamFilter(video_type, video_filter);
	if (video_filter == nullptr) { // check video fileter is nullptr
		ERROR_LOG("Unkonw bitstream filter, video_filter is nullptr!");
		return false;
	}

	// checke alloc bsf context result
	if (av_bsf_alloc(video_filter, &bsf_ctx) < 0) {
		ERROR_LOG("Fail to call av_bsf_alloc!");
		return false;
	}

	// check copy parameters result
	if (avcodec_parameters_copy(bsf_ctx->par_in,
		av_format_context->streams[videoindex]->codecpar) < 0) {
		ERROR_LOG("Fail to call avcodec_parameters_copy!");
		return false;
	}

	bsf_ctx->time_base_in = av_format_context->streams[videoindex]->time_base;

	// check initialize bsf contextreult
	if (av_bsf_init(bsf_ctx) < 0) {
		ERROR_LOG("Fail to call av_bsf_init!");
		return false;
	}

	return true;
}

int VideoFrameDecoder::Decode(FrameProcessCallBack call_back, 
							   void *callback_param) {
	AVFormatContext* av_format_context = avformat_alloc_context();

	// INFO_LOG("Start decode video");

	// check open video result
	if (!OpenVideo(av_format_context)) {
		return 0;
	}

	int videoindex = GetVideoIndex(av_format_context);
	if (videoindex == kInvalidVideoIndex) { // check video index is valid
		ERROR_LOG("Rtsp %s index is -1", stream_name_.c_str());
		return 0;
	}

	AVBSFContext* bsf_ctx;
	VideoType video_type = kInvalidTpye;

	// check initialize video parameters result
	if (!InitVideoParams(videoindex, video_type, av_format_context, bsf_ctx)) {
		return 0;
	}

	AVPacket av_packet;
	int i = 0;
	// loop to get every frame from video stream
	while (av_read_frame(av_format_context, &av_packet) == 0) {
		if (av_packet.stream_index == videoindex) { // check current stream is video
		  // send video packet to ffmpeg
			if (av_bsf_send_packet(bsf_ctx, &av_packet) != 0) {
				ERROR_LOG("Fail to call av_bsf_send_packet, channel id:%s",
					stream_name_.c_str());
			}

			// receive single frame from ffmpeg
			while (av_bsf_receive_packet(bsf_ctx, &av_packet) == 0) {
				call_back(callback_param, av_packet.data,
					      av_packet.size);
				av_packet_unref(&av_packet);
			}
		}
	}

	av_bsf_free(&bsf_ctx); // free AVBSFContext pointer
	avformat_close_input(&av_format_context); // close input video

	// INFO_LOG("Ffmpeg decoder %s finished", stream_name_.c_str());
	return 1;
}

bool VideoFrameDecoder::VerifyVideoType(int& format) {
	
	// INFO_LOG("Start to verify unpack video file:%s", stream_name_.c_str());

	AVFormatContext* av_format_context = avformat_alloc_context();
	bool ret = OpenVideo(av_format_context);
	if (ret == false) {
		ERROR_LOG("Open %s failed", stream_name_.c_str());
		return ret;
	}

	int video_index = GetVideoIndex(av_format_context);
	if (video_index == kInvalidVideoIndex) { // check video index is valid
		ERROR_LOG("Video index is -1, current media stream has no video info:%s",
			stream_name_.c_str());

		avformat_close_input(&av_format_context);
		return false;
	}

	VideoType video_type = kInvalidTpye;
	bool is_valid = CheckVideoType(video_index, av_format_context, video_type);
	avformat_close_input(&av_format_context);

	format = video_type;

	return is_valid;
}











