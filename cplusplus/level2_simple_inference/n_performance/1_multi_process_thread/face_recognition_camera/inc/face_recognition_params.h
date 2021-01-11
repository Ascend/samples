/**
 * ============================================================================
 *
 * Copyright (C) 2018-2020, Hisilicon Technologies Co., Ltd. All Rights Reserved.
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

#ifndef FACE_RECOGNITION_PARAMS_H_
#define FACE_RECOGNITION_PARAMS_H_

#include "utils.h"
#include <iostream>
#include <mutex>
#include <unistd.h>

#include "acl/acl.h"
#include "model_process.h"
#include "dvpp_process.h"

#include "face_feature_train_mean.h"
#include "face_feature_train_std.h"
#include "opencv2/opencv.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc/types_c.h"
#include<opencv2/core/core.hpp>


using namespace std;
#define CHECK_MEM_OPERATOR_RESULTS(ret) \
if (ret != SUCCESS) { \
  ERROR_LOG("memory operation failed, error=%d", ret); \
  return FAILED; \
}

/*#define CHECK_MEM_OPERATOR_RESULTS(ret) \
if (ret != SUCCESS) { \
  return FAILED; \
}*/

// NV12 image's transformation param
// The memory size of the NV12 image is 1.5 times that of width*height.
const int32_t kNv12SizeMolecule = 3;
const int32_t kNv12SizeDenominator = 2;

// model path parameter key in graph.config
const string kModelPathParamKey = "model_path";

// batch size parameter key in graph.config
const string kBatchSizeParamKey = "batch_size";

/**
 * @brief: face recognition APP error code definition
 */
enum class AppErrorCode {
  // Success, no error
  kNone = 0,

  // register engine failed
  kRegister,

  // detection engine failed
  kDetection,

  // feature mask engine failed
  kFeatureMask,

  // recognition engine failed 
  kRecognition
};

/**
 * @brief: frame information
 */
struct FrameInfo {
  uint32_t frame_id = 0;  // frame id
  uint32_t channel_id = 0;  // channel id for current frame
  uint32_t timestamp = 0;  // timestamp for current frame
  uint32_t image_source = 0;  // 0:Camera 1:Register
  std::string face_id = "";  // registered face id
  // original image format and rank using for org_img addition
  acldvppPixelFormat  org_img_format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
  bool img_aligned = false; // original image already aligned or not
  unsigned char *original_jpeg_pic_buffer; // ouput buffer
  unsigned int original_jpeg_pic_size; // size of output buffer
};

/**
 * @brief: Error information
 */
struct ErrorInfo {
  AppErrorCode err_code = AppErrorCode::kNone;
  std::string err_msg = "";
};


/**
 * @brief: face rectangle
 */
struct FaceRectangle {
  cv::Point lt;  // left top
  cv::Point rb;  // right bottom
};


/**
 * @brief: face feature
 */
struct FaceFeature {
  bool flag;
  cv::Point left_eye;  // left eye
  cv::Point right_eye;  // right eye
  cv::Point nose;  // nose
  cv::Point left_mouth;  // left mouth
  cv::Point right_mouth;  // right mouth
};

/**
 * @brief: face image
 */
struct FaceImage {
  ImageData image;  // cropped image from original image
  FaceRectangle rectangle;  // face rectangle
  FaceFeature feature_mask;  // face feature mask
  std::vector<float> feature_vector;  // face feature vector
};

/**
 * @brief: information for face recognition
 */
struct FaceRecognitionInfo {
  FrameInfo frame;  // frame information
  ErrorInfo err_info;  // error information
  ImageData org_img;  // original image
  std::vector<FaceImage> face_imgs;  // cropped image
};

#endif /* FACE_RECOGNITION_PARAMS_H_ */
