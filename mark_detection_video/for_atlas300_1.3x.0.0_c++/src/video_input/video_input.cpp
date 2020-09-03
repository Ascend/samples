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

#include "video_input.h"

#include <cstdlib>
#include <dirent.h>
#include <fstream>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <regex>
#include <vector>

#include "hiaiengine/log.h"
#include "tool_api.h"
#include "ffmpeg_decode.h"
#include "hiaiengine/ai_memory.h"


using hiai::Engine;
using namespace std;

namespace {
	// output port (engine port begin with 0)
	const uint32_t kSendDataPort = 0;

	// sleep interval when queue full (unit:microseconds)
	const __useconds_t kSleepInterval = 200000;

	// get stat success
	const int kStatSuccess = 0;
	// image file path split character
	const string kImagePathSeparator = ",";
	// path separator
	const string kPathSeparator = "/";

	// regex for verify .mp4 file name
	const string kRegexMp4File = ".+\\.(mp4)$";
}

HIAI_StatusT VideoInput::Init(
	const hiai::AIConfig& config,
	const vector<hiai::AIModelDescription>& model_desc) {
	//read the config of dataset
	for (int index = 0; index < config.items_size(); ++index) {
		const ::hiai::AIConfigItem& item = config.items(index);
		std::string name = item.name();
		if (name == "path") {
			src_path_ = item.value();
		}
	}
	return HIAI_OK;
}

// 1. Enter the path of the picture 2. vector
void VideoInput::GetAllFiles(const string& path, vector<string>& file_vec) {
	// split file path
	// path_vector saves the path of multiple input images
	vector<string> path_vector;
	SplitPath(path, path_vector);

	for (string every_path : path_vector) {
		// check path exist or not
		cout << "every_path.c_str() =" << every_path.c_str() << endl;
		if (!IsPathExist(path)) {
			ERROR_LOG("Failed to deal path=%s. Reason: not exist or can not access.",
				every_path.c_str());
			continue;
		}
		// get files in path and sub-path
		GetPathFiles(every_path, file_vec);
	}
}

bool VideoInput::IsDirectory(const string& path) {
	// get path stat
	struct stat buf;
	//Get the file attributes of the file and save it in the structure struct stat
	if (stat(path.c_str(), &buf) != kStatSuccess) {
		return false;
	}

	// check
	// Determine whether it is a folder
	if (S_ISDIR(buf.st_mode)) {
		return true;
	}
	else {
		return false;
	}
}

bool VideoInput::IsPathExist(const string& path) {
	ifstream file(path);
	if (!file) {
		return false;
	}
	return true;
}

// 1. Data input path
// 2. Save data with output parameters
void VideoInput::SplitPath(const string& path, vector<string>& path_vec) {

	char* char_path = const_cast<char*>(path.c_str());
	cout << "char_path = " << char_path << endl;
	const char* char_split = kImagePathSeparator.c_str();
	char* tmp_path = strtok(char_path, char_split);
	while (tmp_path) {
		path_vec.emplace_back(tmp_path);
		tmp_path = strtok(nullptr, char_split);
	}
}


bool VideoInput::IsValidMp4File(const string &input_str) {
  regex regex_mp4_file_name(kRegexMp4File.c_str());
  // verify input string is valid mp4 file name
  if (regex_match(input_str, regex_mp4_file_name)) {
    return true;
  }
  return false;
}

void VideoInput::GetPathFiles(const string& path, vector<string>& file_vec) {
	struct dirent* dirent_ptr = nullptr;
	DIR* dir = nullptr;

	if (IsDirectory(path)) {
		dir = opendir(path.c_str());

		while ((dirent_ptr = readdir(dir)) != nullptr) {
			// skip . and ..
			// Get the file name in the directory
			// Skip when the first character is.
			if (dirent_ptr->d_name[0] == '.') {
				continue;
			}
			// file path
			string full_path = path + kPathSeparator + dirent_ptr->d_name;
			if (!IsValidMp4File(full_path)){
				continue;
			}
			// directory need recursion
			// Folder recursive entry
			if (IsDirectory(full_path)) {
				GetPathFiles(full_path, file_vec);
			}
			else {
				// put file
				file_vec.emplace_back(full_path);
			}
		}
	}
	else {
		// If it is a file directly append
		file_vec.emplace_back(path);
	}
}

bool VideoInput::SendSentinel() {
	HIAI_StatusT hiai_ret = HIAI_OK;
	shared_ptr<EvbImageInfo> frame_handle = nullptr;
	MAKE_SHARED_NO_THROW(frame_handle, EvbImageInfo);
	if (frame_handle == nullptr) {
		ERROR_LOG("Failed to send finish data. Reason: new EvbImageInfo failed.");
		ERROR_LOG("Please stop this process manually.");
		return HIAI_ERROR;
	}
	frame_handle->is_finished = true;

	hiai_ret = SendToEngine(frame_handle);
	// send failed
	if (hiai_ret != HIAI_OK) {
		HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT,
						"call SendData failed, err_code=%d", hiai_ret);
		return false;
	}
  	return true;
}

int VideoInput::DecodeVideo(const string& stream_name,
	                          int channel_id, const string& transport) {
	VideoInfo video_info;
	VideoFrameDecoder* frame_decoder
	    = new VideoFrameDecoder(stream_name, transport);

	if (!frame_decoder->VerifyVideoType(video_info.format)) {
		return HIAI_ERROR;
	}
	video_info.channel_id = channel_id;
	video_info.context = (void*)this;

	if(!frame_decoder->Decode(VideoInput::FrameDecodeCallback, (void*)(&video_info))){
		return 0;
	}

	//notice video decode finish

	return 1;
}

void VideoInput::FrameDecodeCallback(void* context, void* frame_data,
	                                   int frame_size) {
	shared_ptr<EvbImageInfo> video_frame = nullptr;
	MAKE_SHARED_NO_THROW(video_frame, EvbImageInfo);
	if (video_frame == nullptr) {
		ERROR_LOG("Failed to deal video for new EvbImageInfo failed.");
		return;
	}

	uint8_t* image_buf_ptr = nullptr;
	HIAI_StatusT ret = hiai::HIAIMemory::HIAI_DMalloc(frame_size, (void*&)image_buf_ptr);
	if (HIAI_OK != ret || nullptr == image_buf_ptr) {
		HIAI_ENGINE_LOG(HIAI_ENGINE_RUN_ARGS_NOT_RIGHT, "[DataInput] Failed to call HIAI_DMalloc.");
		return;
	}

	int memcpy_result = memcpy_s(image_buf_ptr, frame_size, frame_data, frame_size);
	if (memcpy_result != EOK) {
		ERROR_LOG("Copy frame data failed, return %d", memcpy_result);
		hiai::HIAIMemory::HIAI_DFree(image_buf_ptr);
		return;
	}

	VideoInfo* video_info = (VideoInfo*)context;

	video_frame->format = video_info->format;
	video_frame->channel_id = video_info->channel_id;

	std::shared_ptr<uint8_t> buf_ptr = shared_ptr<uint8_t>(image_buf_ptr,
		[](uint8_t* p) {hiai::HIAIMemory::HIAI_DFree(p); });
	video_frame->data = buf_ptr.get();
	video_frame->size = frame_size;	
	video_frame->is_finished = false;

	VideoInput* this_engine = (VideoInput*)(video_info->context);
	this_engine->SendToEngine(video_frame);
}

bool VideoInput::SendToEngine(const shared_ptr<EvbImageInfo>& image_handle) {
	// can not discard when queue full
	HIAI_StatusT hiai_ret;
	do {
		hiai_ret = SendData(kSendDataPort, "EvbImageInfo",
			static_pointer_cast<void>(image_handle));
		// when queue full, sleep
		if (hiai_ret == HIAI_QUEUE_FULL) {
			ERROR_LOG("queue full, sleep 200ms");
			usleep(kSleepInterval);
		}
	} while (hiai_ret == HIAI_QUEUE_FULL);

	// send failed
	if (hiai_ret != HIAI_OK) {
		ERROR_LOG("call SendData failed, err_code=%d", hiai_ret);
		return false;
	}
	return true;
}

HIAI_IMPL_ENGINE_PROCESS("video_input",
	VideoInput, INPUT_SIZE) {
	//file_vec saves all files in the input path and files in subdirectories
	vector<string> file_vec;
	GetAllFiles(src_path_, file_vec);

	// If the directory is empty, an error will be reported
	if (file_vec.empty()) {
		ERROR_LOG("Failed to deal all empty path=%s.", src_path_.c_str());
		return HIAI_ERROR;
	}
	int tmp = 0;
	// Send every video to dvpp vdec engine
	int channel_id = 0;
	for (string path : file_vec) {
		cout << "tmp = " << tmp <<endl;
		// Isn't it a blocking function
		if (!DecodeVideo(path, channel_id, "udp")) {
			ERROR_LOG("Decode %s failed, stop decode.", path.c_str());
			return HIAI_ERROR;
		}
		usleep(kSleepInterval);
		tmp++;
	}
	SendSentinel();
	return HIAI_OK;
}



