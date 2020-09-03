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

#ifndef COMMON_DATA_TYPE_H_
#define COMMON_DATA_TYPE_H_

#include <string>
#include <memory>

#include <unistd.h>
#include <sys/stat.h>
#include "hiaiengine/data_type.h"
#include "hiaiengine/data_type_reg.h"
#include "hiaiengine/status.h"
#include "hiaiengine/ai_tensor.h"

#define DVPP_STRIDE_WIDTH  128
#define DVPP_STRIDE_HEIGHT 16

 // video type
enum VideoType {
	kH264,
	kH265,
	kInvalidTpye
};

/**
 * @brief: image information
 */
struct ImageInfo {
  std::string path = ""; // the path of image
  int32_t width = 0; // original width
  int32_t height = 0; // original height
  int32_t size = 0; // data size
  std::shared_ptr<u_int8_t> data;
};

/**
 * @brief: serialize for ImageInfo
 */
template<class Archive>
void serialize(Archive& ar, ImageInfo& data);




/**
 * @brief: inference output data
 */
struct Output {
  int32_t size = 0;
  std::shared_ptr<u_int8_t> data;
};

/**
 * @brief: serialize for Output
 */
template<class Archive>
void serialize(Archive& ar, Output& data);



/**
 * @brief: Inference engine error message
 */
struct ErrorInferenceMsg {
  bool error = false;
  std::string err_msg = "";
};

/**
 * @brief: serialize for EngineTrans
 */
template<class Archive>
void serialize(Archive& ar, ErrorInferenceMsg& data);



/**
 * @brief: Engine Transform information
 */

// The structure sent by the data acquisition engine
struct EvbImageInfo {
	int format;
	int channel_id;
  // original width
  int32_t width = 0; 
  // original height
  int32_t height = 0; 

  // data size
  uint32_t size = 0; 
  u_int8_t* data;

  bool is_finished = false;
  // the path of image
  std::string path = ""; 
};

/**
 * @brief: serialize for EvbImageInfo
 */

template <class Archive>
void serialize(Archive & ar, EvbImageInfo& data);



//The new version of serialize function
void GetEvbImageInfoSearPtr(void *input_ptr, std::string& ctrl_str, uint8_t*& data_ptr, uint32_t& data_len);


//The new version of deserialize function
std::shared_ptr<void> GetEvbImageInfoDearPtr(const char* ctrl_ptr, const uint32_t& ctr_len, const uint8_t* data_ptr, const uint32_t& data_len);


// Structure received by the inference engine
struct EngineTrans {
	int format;
	int channel_id;
	int inference_width;
  int inference_height;
  ImageInfo image_info;
  ErrorInferenceMsg err_msg;
  std::vector<Output> inference_res;
  bool is_finished = false;
};

/**
 * @brief: serialize for EngineTrans
 */
template<class Archive>
void serialize(Archive& ar, EngineTrans& data);




struct BoundingBox {
  float lt_x;
  float lt_y;
  float rb_x;
  float rb_y;
  uint32_t attribute;
  float score;
};

/**
 * error code
 */
enum ErrorCode {
  kOk,
  kHiaiInitFailed,
  kCreateGraphFailed,
  kGetGraphInstanceFailed,
  kSetCallbackFunctionError,
  kFilePathError,
  kModelInitFailed,
  kImageFileParseError,
  kImageConversionError,
  kInferenceError,
  kResultError,
  kOtherError
};

#endif /* COMMON_DATA_TYPE_H_ */
