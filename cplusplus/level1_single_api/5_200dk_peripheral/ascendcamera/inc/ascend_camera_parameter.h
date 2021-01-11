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

#ifndef ASCENDDK_ASCENDCAMERA_ASCEND_CAMERA_PARAMETER_H_
#define ASCENDDK_ASCENDCAMERA_ASCEND_CAMERA_PARAMETER_H_

#include <stdio.h>

#include <map>
#include <string>
#include <vector>

#include "parameter_utils.h"

namespace ascend {

namespace ascendcamera {

// media type, IMAGE: image, VIDEO: video
enum MediaType {
  kImage,
  kVideo
};

/*
 * AscendCameraParameter: used for initialize and verify the input parameters
 * */
class AscendCameraParameter {
 public:
  /**
   * @brief constructor
   */
  AscendCameraParameter();

  /**
   * @brief destructor
   */
  ~AscendCameraParameter();

  /**
   * @brief initialize ascendcamera parameters
   * @param [in] argc: input parameters number
   * @param [in] argv: input parameters
   * @return true: initialize success; false: initialize failed
   */
  bool Init(int argc, char* const argv[]);

  /**
   * @brief: verify ascendcamera input parameters
   * @return true: verify pass; false: verify not pass
   */
  bool Verify();

  /**
   * @brief get camera channel
   * @return camera channel number
   */
  const int GetCameraChannel() const;

  /**
   * @brief check contains parameter help or not
   * @return true: contains parameter help;
   *         false: not contains parameter --help
   */
  const bool ContainsHelp() const;

  /**
   * @brief get output to file parameter value
   * @return output to file parameter value
   */
  const std::string &GetOutputFile() const;

  /**
   * @brief get output to presenter parameter value
   * @return output to presenter parameter value
   */
  const std::string &GetOutputPresenter() const;

  /**
   * @brief get image width)
   * @return image width
   */
  const int GetImageWidth() const;

  /**
   * @brief get image height
   * @return image height
   */
  const int GetImageHeight() const;

  /**
   * @brief get media type
   * @return IMAGE: image; VIDEO: vedio
   */
  const MediaType GetMediaType() const;

  /**
   * @brief get timeout
   * @return time value
   */
  const int GetTimeout() const;

  /**
   * @brief get fps
   * @return fps value
   */
  const int GetFps() const;

 private:
  // fps default value
  const int kDefaultFps = 10;

  // timeout default value
  const int kDeaultTimeout = 0;

  // camera channel default value
  const int kDefaultCameraChannel = 0;

  // image width default value
  const int kDefaultImageWidth = 1280;

  // image height default value
  const int kDefaultImageHeight = 720;

  // default invalid value
  const int kInvalidValue = -1;

  // default buffer size
  const int kDefaultBufferSize = 1000;

  // regex for verify ip:port/channelname
  const std::string kRegexPresenter =
    "^(1\\d{2}|2[0-4]\\d|25[0-5]|[1-9]\\d|[0-9])\\."
    "(1\\d{2}|2[0-4]\\d|25[0-5]|[1-9]\\d|\\d)\\."
    "(1\\d{2}|2[0-4]\\d|25[0-5]|[1-9]\\d|\\d)\\."
    "(1\\d{2}|2[0-4]\\d|25[0-5]|[1-9]\\d|\\d)"
    ":([1-9]|[1-9]\\d|[1-9]\\d{2}|[1-9]\\d{3}|[1-5]\\d{4}|"
    "6[0-4]\\d{3}|65[0-4]\\d{2}|655[0-2]\\d|6553[0-5])/"
    "([a-zA-Z0-9/]{1,25})$";

  // numerical value maximum length
  const unsigned int kNumericValueLength = 9;

  // fps maximum value
  const int kMaxFps = 20;

  // fps minimum value 1
  const int kMinFps = 1;

  // camera channel default value
  const int kMaxCameraChannel = 1;

  // parameter minimum length
  const unsigned int kMinParamLength = 2;

  // supported resolutions number
  const int kResolutionNumber = 5;

  // do not output getopt error
  const int kNotOutputGetoptError = 0;

  // check has parameter help(--help) or not
  bool contains_help_ = false;

  // camera channel
  int camera_channel_ = kDefaultCameraChannel;

  // image width
  int image_width_ = kInvalidValue;

  // image height
  int image_height_ = kInvalidValue;

  // presenter value ip:port/path
  std::string output_presenter_ = "";

  // output to file value
  std::string output_file_ = "";

  // check is image or not
  bool is_image_ = false;

  // check is video or not
  bool is_video_ = false;

  // overwrite the exist file or not
  bool overwrite_ = false;

  // timeout
  int timeout_ = kInvalidValue;

  // frames per second
  int fps_ = kInvalidValue;

  // record valid parameters
  std::map<std::string, std::string> valid_params_;

  // record invalid parameters
  std::vector<std::string> invalid_params_;

  // record duplicate parameters
  std::vector<std::string> duplicate_params_;

  /**
   * @brief pre-process ascendcamera parameters, verify that input parameters
   *        are supported
   * @param [in] argc: input parameters number
   * @param [in] argv: input parameters
   * @return true: verify pass; false: verify not pass
   */
  bool Preprocess(int argc, char* const argv[]);

  /**
   * @brief check the input parameter has value or not
   * @param [in] param_name: input parameter
   * @return true: has value; false: has no value
   */
  bool CheckParamHasValue(const std::string &param_name);

  /**
   * @brief check input parameter is valid short parameter or not
   * @param [in] param_name: input parameter
   * @return true: is valid short parameter;
   *         false: is invalid short parameter
   */
  bool IsValidShortParam(const std::string &param_name);

  /**
   * @brief check input parameter is valid long parameter or not
   * @param [in] param_name: input parameter
   * @return true: is valid long parameter; false: is invalid long parameter
   */
  bool IsValidLongParam(const std::string &param_name);

  /**
   * @brief display help info
   */
  void DisplayHelpInfo() const;

  /**
   * @brief verify image width and height
   * @return true: verify pass; false: verify not pass
   */
  bool VerifyWidthHeight();

  /**
   * @brief obtain valid parameters
   * @param [in] param_name: input parameter
   * @param [in] param_value: input parameter value
   * @param [out] have_duplicate_param: check have duplicate parameter or not
   * @return true: obtain valid parameter success;
   *         false: obtain valid parameter failed
   */
  bool ObtainValidParams(const std::string &param_name,
                         const std::string &param_value,
                         bool &have_duplicate_param);

  /**
   * @brief obtain integer parameters
   * @param [in] param_name: input parameter
   * @param [in] param_value: input parameter value
   * @param [out] is_initialize_fail: is initialize failed
   * @param [in] default_value: default parameter value
   * @return parameter value
   */
  const int ObtainIntParams(const std::string &param_name,
                            const std::string &param_value,
                            bool &is_initialize_fail, const int default_value);

  /**
   * @brief obtain string parameters
   * @param [in] param_name: input parameter
   * @param [in] param_value: input parameter value
   * @param [out] is_initialize_fail: is initialize failed
   * @param [in] default_value: default parameter value
   * @return parameter value
   */
  const std::string ObtainStrParams(const std::string &param_name,
                                    const std::string &param_value,
                                    bool &is_initialize_fail,
                                    const std::string default_value);

  /**
   * @brief parse input parameters
   * @param [in] argc: input parameter number
   * @param [in] argv: input parameters
   * @return true: parse parameter success; false: parse parameter failed
   */
  bool ParseInputParams(int argc, char* const argv[]);

  /**
   * @brief display invalid parameters
   */
  void DisplayInvalidParams() const;

  /**
   * @brief display duplicated parameters
   */
  void DisplayDuplicateParams() const;

  /**
   * @brief display valid parameters
   */
  void DisplayValidParams() const;

  /**
   * @brief set fps default value
   */
  void SetFpsTimeoutDefaultValue();

  /**
   * @brief check parameters which should be ignored
   */
  void CheckIgnoreParams() const;

  /**
   * @brief verify camera channel
   * @return true: verify pass; false: verify not pass
   */
  bool VerifyCameraChannel() const;

  /**
   * @brief verify media type
   * @return true: verify pass; false: verify not pass
   */
  bool VerifyMediaType() const;

  /**
   * @brief verify fps value
   * @return true: verify pass; false: verify not pass
   */
  bool VerifyFpsValue();

  /**
   * @brief verify output value
   * @return true: verify pass; false: verify not pass
   */
  bool VerifyOutputType() const;

  /**
   * @brief handle the situation where the file already exists
   * @return true: handle success; false: handle failed
   */
  bool HandleExistFile() const;

  /**
   * @brief verify output to presenter value
   * @return true: verify pass; false: verify not pass
   */
  bool VerifyPresenterValue() const;

  /**
   * @brief verify output to a file value
   * @param [in] verify_pass: verify pass
   * @return true: verify pass; false: verify not pass
   */
  bool VerifyOutputFileValue(const bool verify_pass);

  /**
   * @brief verify images width and height value
   * @return true: verify pass; false: verify not pass
   */
  bool VerifyWidthHeightValue() const;

  /**
   * @brief verify numerical input parameter value
   * @param [in] param_name: input parameter
   * @param [in] param_value: input parameter value
   * @param [out] is_initialize_fail: is initialize fail
   * @return true: verify pass; false: verify not pass
   */
  bool VerifyNumericalParam(const std::string &param_name,
                            const std::string &param_value,
                            bool &is_initialize_fail) const;

  /**
   * @brief check input parameters are supported
   * @param [in] argc: parameters number
   * @param [in] argv: input parameters
   * @param [out] all_input_params: record all input parameters
   * @param [out] unrecognized_params: unrecognized parameters
   */
  void CheckParamsSupport(int argc, char* const argv[],
                          std::string &all_input_params,
                          std::string &unrecognized_params);
};
}
}

#endif /* ASCENDDK_ASCENDCAMERA_ASCEND_CAMERA_PARAMETER_H_ */
