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

#include "ascend_camera_parameter.h"

#include <regex>
#include <iostream>

#include <getopt.h>
#include <stdlib.h>
#include <unistd.h>

#include "ascend_camera_common.h"

using namespace std;

namespace {
// The image width list supported by the camera
const int kImageWidths[5] = { 1920, 1280, 704, 704, 352 };

// The image height list supported by the camera
const int kImageHeights[5] = { 1080, 720, 576, 288, 288 };

// parameter has no value
const int kParamHasNoValue = 0;

// parameter has value
const int kParamHasValue = 1;

// slash string length
const int kSlashStrLength = 1;



// long options for getopt_long function
const struct option kLongOptions[] = {
    { "width", kParamHasValue, nullptr, 'w' },
    { "height", kParamHasValue, nullptr, 'h' },
    { "timeout", kParamHasValue, nullptr, 't' },
    { "fps", kParamHasValue, nullptr, 'F' },
    { "overwrite", kParamHasNoValue, nullptr, 'O' },
    { nullptr, kParamHasNoValue, nullptr, kParamHasNoValue } };

// short options for getopt_long function
const char* kShortOptions = "c:w:h:ivo:s:t:HF";

// valid short parameters
const string kValidShortParams[] = { "-i", "-v", "-s", "-o", "-w", "-h", "-t",
    "-c" };

// valid long parameters
const string kValidLongParams[] = { "--timeout", "--fps", "--width", "--height",
    "--help", "--overwrite" };

// the parameters who  have value
const string kHasValueParams[] = { "-s", "-o", "-w", "-h", "-t", "-c",
    "--timeout", "--fps", "--width", "--height" };

// used for concatenate logging information
stringstream log_info_stream("");
}

namespace ascend {

namespace ascendcamera {

AscendCameraParameter::AscendCameraParameter() {
  valid_params_.clear();
  invalid_params_.clear();
  duplicate_params_.clear();
}

AscendCameraParameter::~AscendCameraParameter() {
}

void AscendCameraParameter::CheckParamsSupport(int argc, char* const argv[],
                                               string &all_input_params,
                                               string &unrecognized_params) {
  // traverse all entry parameters, ignore first parameter(command itself)
  for (int i = kIndexSecond; i < argc; i++) {
    all_input_params += " ";
    all_input_params += argv[i];

    string param_name = argv[i];
    // obtain previous parameter, so index minus 1
    string pre_param = argv[i - 1];

    // verify that input parameter are supported or not
    // check parameter length > 2 and start with "-"
    if (param_name.length() >= kMinParamLength
        && ParameterUtils::IsStartWithDash(param_name)) {
      // check parameter second index is not '-'
      if (param_name.at(kIndexSecond) != '-') {
        if (IsValidShortParam(param_name)) {
          // verify short parameter
          continue;
        }
      } else {
        // check parameter second index is "-"
        if (IsValidLongParam(param_name)) {
          // verify long parameter
          continue;
        }
      }
    }

    // check current argc[i] is a value of parameter
    if (!ParameterUtils::IsStartWithDash(param_name)
        || param_name.compare(string("-")) == kCompareEqual) {
      if (CheckParamHasValue(pre_param)) {
        continue;
      }
    }

    unrecognized_params += param_name;
    unrecognized_params += "; ";
  }
}

bool AscendCameraParameter::Preprocess(int argc, char* const argv[]) {
  string all_input_params = "the input command:";
  all_input_params += argv[kIndexFirst];

  string unrecognized_params = "";

  CheckParamsSupport(argc, argv, all_input_params, unrecognized_params);

  // record all input parameters to dlog
  ASC_LOG_INFO("%s", all_input_params.c_str());

  // if parameters contain "--help", then display help info
  if (contains_help_) {
    DisplayHelpInfo();
    return false;
  }

  // check unrecognized parameters is empty
  if (unrecognized_params.empty()) {
    return true;
  } else {  // print unrecognized parameters
    string cerr_info = "The following input parameter(s) "
        "can not be recognized:";
    cerr_info += unrecognized_params;

    cerr << "[ERROR] " << cerr_info << endl;
    ASC_LOG_ERROR("%s", cerr_info.c_str());

    return false;
  }
}

bool AscendCameraParameter::CheckParamHasValue(const string &param_name) {
  // traverse all valid short parameters
  for (const string &hasValueParam : kHasValueParams) {
    // check is valid short parameter
    if (hasValueParam.compare(param_name) == kCompareEqual) {
      return true;
    }
  }

  return false;
}

bool AscendCameraParameter::IsValidShortParam(const string &param_name) {
  // traverse all valid short parameters
  for (const string &validParam : kValidShortParams) {
    // check is valid short parameter
    if (validParam.compare(param_name) == kCompareEqual) {
      return true;
    }
  }

  return false;
}

bool AscendCameraParameter::IsValidLongParam(const string &param_name) {
  // check parameter is "--help"
  if (param_name.compare("--help") == kCompareEqual) {
    contains_help_ = true;
    return true;
  }

  // traverse all valid long parameters
  for (const string &validParam : kValidLongParams) {
    // check is valid long parameter
    if (validParam.compare(param_name) == kCompareEqual) {
      return true;
    }
  }

  return false;
}

bool AscendCameraParameter::ParseInputParams(int argc, char* const argv[]) {
  // do not output getopt error to stderr
  opterr = kNotOutputGetoptError;

  // initialize optind value
  optind = kIndexSecond;

  // single parameter, initialized to 0
  int opt = 0;

  // last optind
  int opt_index_last = kIndexFirst;

  // initialize fail flag
  bool is_initialize_fail = false;

  // parse input parameters
  while ((opt = getopt_long_only(argc, argv, kShortOptions, kLongOptions,
                                 nullptr)) != kInvalidValue) {
    printf("opt=%c\n",opt);
//    if(opt =! 'N')
    {
      switch (opt) {
        case 'c':  // handle parameter -c, camera channel
        camera_channel_ = ObtainIntParams(string("-c"), string(optarg),
        is_initialize_fail,
        kDefaultCameraChannel);
        break;

        case 'w':  // handle parameter -w(--width), iamge width
        image_width_ = ObtainIntParams(string("-w(--width)"), string(optarg),
        is_initialize_fail, kDefaultImageWidth);
        break;

        case 'h':  // handle parameter -h(--height), image height
        image_height_ = ObtainIntParams(string("-h(--height)"), string(optarg),
        is_initialize_fail,
        kDefaultImageHeight);
        break;

        case 's':  // handle parameter -s, output to presenter
        output_presenter_.assign(
        ObtainStrParams(string("-s"), string(optarg), is_initialize_fail,
        ""));
        break;

        case 'o':  // handle parameter -o, output to file
        output_file_.assign(
        ObtainStrParams(string("-o"), string(optarg), is_initialize_fail,
        ""));
        break;

        case 'i':  // handle parameter -i, image
        is_image_ = true;
        ObtainValidParams(string("-i"), string(""), is_initialize_fail);
        break;

        case 'v':  // handle parameter -v, video
        is_video_ = true;
        ObtainValidParams(string("-v"), string(""), is_initialize_fail);
        break;

        case 't':  // handle parameter -t(--timeout)
        timeout_ = ObtainIntParams(string("-t(--timeout)"), string(optarg),
        is_initialize_fail, kDeaultTimeout);
        break;

        case 'F':  // handle parameter fps
        fps_ = ObtainIntParams(string("--fps"), string(optarg),
        is_initialize_fail, kDefaultFps);
        break;

        case 'O':  // handle parameter overwrite
        overwrite_ = true;
        ObtainValidParams(string("--overwrite"), string(""),
        is_initialize_fail);
        break;



        default:  // handle parameter can not be recognized or missing value
        if (optind > opt_index_last) {
          is_initialize_fail = true;
          invalid_params_.push_back(argv[optind - kIndexSecond]);
        }
//        printf("H-opt=%c\n",opt);
        break;
      }
    }
//    else
//    {
//      printf("H-opt=%c\n",opt);
//    }


    opt_index_last = optind;
  }

  return is_initialize_fail;
}

void AscendCameraParameter::DisplayInvalidParams() const {
  if (!contains_help_) {  // check parameters contains "--help"
    string invalid_params_info =
        "The following input parameter is missing parameter value:";

    // traverse all invalid parameters
    for (vector<string>::const_iterator iter = invalid_params_.begin();
        iter != invalid_params_.end(); iter++) {
      // ignore command(./ascendcamera) itself
      if (((*iter).compare("./ascendcamera")) != kCompareEqual) {
        invalid_params_info += *iter;
        invalid_params_info += "; ";
      }
    }

    cerr << "[ERROR] " << invalid_params_info << endl;
    ASC_LOG_ERROR("%s", invalid_params_info.c_str());
  }
}

void AscendCameraParameter::DisplayDuplicateParams() const {
  if (!contains_help_) {  // check parameters contains "--help"
    string duplicate_params_info =
        "The following input parameter(s) is duplicated:";

    // traverse all duplicated parameters
    for (vector<string>::const_iterator iter = duplicate_params_.begin();
        iter != duplicate_params_.end(); iter++) {
      duplicate_params_info += *iter;
      duplicate_params_info += "; ";
    }

    cerr << "[ERROR] " << duplicate_params_info << endl;
    ASC_LOG_ERROR("%s", duplicate_params_info.c_str());
  }
}

void AscendCameraParameter::DisplayValidParams() const {
  string valid_params_info = "ascendcamera recognized parameters:";

  // traverse all valid parameters
  for (map<string, string>::const_iterator iter = valid_params_.begin();
      iter != valid_params_.end(); iter++) {
    valid_params_info += iter->first;

    // ignore empty parameter value
    if (iter->second.compare(string("")) != kCompareEqual) {
      valid_params_info += " = ";
      valid_params_info += iter->second;
    }

    valid_params_info += "; ";
  }

  ASC_LOG_INFO("%s", valid_params_info.c_str());
}

bool AscendCameraParameter::Init(int argc, char* const argv[]) {
  // check all input parameters are valid
  if (argv == nullptr || !Preprocess(argc, argv)) {
    return false;
  }

  // parse input parameters flag
  bool is_initialize_fail = ParseInputParams(argc, argv);

  // if have no input parameters, then display help info
  if ((valid_params_.empty() && invalid_params_.empty())) {
    contains_help_ = true;
    cout<<11111;
    DisplayHelpInfo();

    return false;
  }

  // display ascendcamera unrecognized input parameter
  if (!invalid_params_.empty()) {
    DisplayInvalidParams();
    cout<<222;
  }

  // display ascendcamera duplicate input parameter
  if (!duplicate_params_.empty()) {
    DisplayDuplicateParams();
    cout<<33333;
  }

  // check input parameters initialize failed
  if (is_initialize_fail) {
    return false;
  }

  // display input parameters
  if (!valid_params_.empty()) {
    DisplayValidParams();
  }

  cerr << "[INFO] Success to initialize ascendcamera parameters." << endl;
  ASC_LOG_INFO("Success to initialize ascendcamera parameters.");

  return true;
}

bool AscendCameraParameter::VerifyNumericalParam(
    const string& param_name, const string& param_value,
    bool& is_initialize_fail) const {
  // check current parameter value is integer type
  if (param_name.compare(string("-c")) == kCompareEqual
      || param_name.compare(string("-w(--width)")) == kCompareEqual
      || param_name.compare(string("-h(--height)")) == kCompareEqual
      || param_name.compare(string("-t(--timeout)")) == kCompareEqual
      || param_name.compare(string("--fps")) == kCompareEqual) {

    if (param_value.length() > kNumericValueLength) {
      is_initialize_fail = true;

      log_info_stream.str("");
      log_info_stream << "The ascendcamera parameter " << param_name
                      << " has value:" << optarg
                      << ", the input value is too long"
                      << ", please input a shorter value(length <= "
                      << kNumericValueLength << ").";
      string cerr_info = log_info_stream.str();

      cerr << "[ERROR] " << cerr_info << endl;
      ASC_LOG_ERROR("%s", cerr_info.c_str());

      return false;
    }

    // regular expression check numbers
    regex regex_number("[0-9]+");

    // check parameter value is match number
    if (regex_match(param_value, regex_number)) {
      return true;
    }

    is_initialize_fail = true;

    log_info_stream.str("");
    log_info_stream << "The ascendcamera parameter " << param_name
                    << " has non-integer value:" << optarg
                    << ", please input a integer value.";
    string cerr_info = log_info_stream.str();

    cerr << "[ERROR] " << cerr_info << endl;
    ASC_LOG_ERROR("%s", cerr_info.c_str());

    return false;
  }

  return true;
}

const int AscendCameraParameter::ObtainIntParams(const string &param_name,
                                                 const string &param_value,
                                                 bool &is_initialize_fail,
                                                 const int default_value) {
  //check obtain valid parameter
  if (ObtainValidParams(param_name, param_value, is_initialize_fail)) {
    return atoi(param_value.c_str());
  }

  return default_value;
}

const string AscendCameraParameter::ObtainStrParams(
    const string &param_name, const string &param_value,
    bool &is_initialize_fail, const string default_value) {
  //check obtain valid parameter
  if (ObtainValidParams(param_name, param_value, is_initialize_fail)) {
    return param_value;
  }

  return default_value;
}

bool AscendCameraParameter::ObtainValidParams(const string &param_name,
                                              const string &param_value,
                                              bool &is_initialize_fail) {

  // check parameter value length is too big
  if (param_value.length() > kMaxParamLength) {
    is_initialize_fail = true;

    log_info_stream.str("");
    log_info_stream << "The ascendcamera parameter " << param_name
                    << " has value:" << param_value
                    << ", the value is too long, "
                    << "please input a shorter value(length <= "
                    << kMaxParamLength << ").";
    string cerr_info = log_info_stream.str();

    cerr << "[ERROR] " << cerr_info << endl;
    ASC_LOG_ERROR("%s", cerr_info.c_str());

    return false;
  }

  // current parameter is not duplicated when count parameter equals 0
  if (valid_params_.count(param_name) == 0) {
    valid_params_.insert(pair<string, string>(param_name, param_value));
  } else {  // current parameter is duplicated
    // ignore parameters "-v" "-i" "--help"
    if (param_name.compare(string("-v")) == kCompareEqual
        || param_name.compare(string("-i")) == kCompareEqual
        || param_name.compare(string("--overwrite")) == kCompareEqual
        || param_name.compare(string("--help")) == kCompareEqual) {
      return false;
    }

    is_initialize_fail = true;

    // current parameter is not record when count parameter equals 0
    if (count(duplicate_params_.begin(), duplicate_params_.end(), param_name)
        == 0) {
      duplicate_params_.push_back(param_name);
    }
  }

  // handle parameter value start with "-"
  if (ParameterUtils::IsStartWithDash(param_value)) {
    // check parameter is "-o" and value is "-"
    if (param_name.compare("-o") == kCompareEqual
        && param_value.compare("-") == kCompareEqual) {
      return true;
    }

    string cerr_info = "The ascendcamera parameter ";
    cerr_info += param_name;
    cerr_info += " has unrecognized value:";
    cerr_info += param_value;

    // the -o parameter ignore this description
    if (param_name.compare("-o") != kCompareEqual) {
      cerr_info += ", the value should not start with '-'";
    }

    cerr_info += ".";

    cerr << "[ERROR] " << cerr_info << endl;
    ASC_LOG_ERROR("%s", cerr_info.c_str());

    is_initialize_fail = true;
    return false;
  }

  // verify nemerical parameter value
  return VerifyNumericalParam(param_name, param_value, is_initialize_fail);
}

void AscendCameraParameter::SetFpsTimeoutDefaultValue() {
  // set fps default value
  if (is_video_) {
    if (fps_ == kInvalidValue) {
      fps_ = kDefaultFps;
    }

    // set timeout default value
    if (timeout_ == kInvalidValue) {
      timeout_ = kDeaultTimeout;
    }
  }
}

void AscendCameraParameter::CheckIgnoreParams() const {
  if (is_image_ && !is_video_) {  // check media type is image
    string ignore_str = "The ascendcamera has parameter -i, ignore:";
    bool is_need_output = false;

    if (timeout_ != kInvalidValue) {  // check timeout is -1
      ignore_str += "-t(--time); ";
      is_need_output = true;
    }

    if (fps_ != kInvalidValue) {  // check fps is -1
      ignore_str += "--fps;";
      is_need_output = true;
    }

    if (is_need_output) {  // check is need output
      cerr << "[WARNING] " << ignore_str << endl;
      ASC_LOG_WARN("%s", ignore_str.c_str());
    }
  }

  // check ignore "overwrite" or not
  if (overwrite_
      && (output_file_.empty() || output_file_.compare("-") == kCompareEqual)) {
    cerr << "[WARNING] The ascendcamera dose not have -o parameter value"
         " or -o parameter value is '-', so ignore:--overwrite."
         << endl;
    ASC_LOG_WARN(
        "[WARNING] The ascendcamera dose not have -o parameter"
        " value or -o parameter value is '-', so ignore:--overwrite.");
  }
}

bool AscendCameraParameter::VerifyCameraChannel() const {
  // check camera channel in valid rang
  if (camera_channel_ >= kDefaultCameraChannel
      && camera_channel_ <= kMaxCameraChannel) {
    return true;
  }

  log_info_stream.str("");
  log_info_stream << "The ascendcamera parameter -c has invalid value:"
                  << camera_channel_ << ", value range:0 1.";
  string cerr_info = log_info_stream.str();

  cerr << "[ERROR] " << cerr_info << endl;
  ASC_LOG_ERROR("%s", cerr_info.c_str());

  return false;
}

bool AscendCameraParameter::VerifyMediaType() const {
  // media type is neither image nor video
  if (!(is_image_ || is_video_)) {
    cerr << "[ERROR] The ascendcamera parameter -i and -v "
         << "should input one of them." << endl;
    ASC_LOG_ERROR(
        "The ascendcamera parameter -i and -v should input one of them.");

    return false;
  }

  // media type is image and video
  if (is_image_ && is_video_) {
    cerr << "[ERROR] The ascendcamera parameter -i and -v "
         << "should only input one of them." << endl;
    ASC_LOG_ERROR(
        "The ascendcamera parameter -i and -v should only "
        "input one of them.");

    return false;
  }

  if (is_video_ && output_presenter_.empty()) {
    cerr << "[ERROR] The ascendcamera has -v parameter, "
         << "should input -s parameter." << endl;
    ASC_LOG_ERROR(
        "The ascendcamera has -v parameter, should input -s parameter.");

    return false;
  }

  return true;
}

bool AscendCameraParameter::VerifyFpsValue() {
  SetFpsTimeoutDefaultValue();
  // check fps in valid rang
  if (is_video_ && (fps_ < kMinFps || fps_ > kMaxFps)) {
    log_info_stream.str("");
    log_info_stream << "The ascendcamera parameter --fps has invalid value:"
                    << fps_ << ", value range:1~20.";
    string cerr_info = log_info_stream.str();

    cerr << "[ERROR] " << cerr_info << endl;
    ASC_LOG_ERROR("%s", cerr_info.c_str());

    return false;
  }

  return true;
}

bool AscendCameraParameter::Verify() {
  bool verify_pass = true;

  // verify camera channel
  if (!VerifyCameraChannel()) {
    verify_pass = false;
  }

  // check image width and height
  if (!VerifyWidthHeight()) {
    verify_pass = false;
  }

  // verify output type
  if (!VerifyOutputType()) {
    verify_pass = false;
  }

  // verify output to presenter value
  if (!VerifyPresenterValue()) {
    verify_pass = false;
  }

  // verify media type
  if (!VerifyMediaType()) {
    return false;
  }

  // verify fps
  if (!VerifyFpsValue()) {
    verify_pass = false;
  }

  // check has need ignored parameters
  CheckIgnoreParams();

  // verify output to a file value
  if (!VerifyOutputFileValue(verify_pass)) {
    return false;
  }

  if (!verify_pass) {
    return false;
  }

  cerr << "[INFO] Success to verify ascendcamera parameters." << endl;
  ASC_LOG_INFO("Success to verify ascendcamera parameters.");

  return true;
}

bool AscendCameraParameter::VerifyOutputType() const {
  // neither output to a file nor presenter
  if (output_file_.empty() && output_presenter_.empty()) {
    cerr << "[ERROR] The ascendcamera parameter -o and -s should input "
         "one of them." << endl;
    ASC_LOG_ERROR(
        "The ascendcamera parameter -o and -s should input one of them.");

    return false;
  }

  // both output to a file and presenter
  if (!output_file_.empty() && !output_presenter_.empty()) {
    cerr << "[ERROR] The ascendcamera parameter -o and -s should only"
         " input one of them." << endl;
    ASC_LOG_ERROR(
        "The ascendcamera parameter -o and -s should only input "
        "one of them.");

    return false;
  }

  return true;
}

bool AscendCameraParameter::HandleExistFile() const {
  // check file is exist or not
  if (access(output_file_.c_str(), F_OK) == kHasNoAccessPermission) {
    return true;
  }

  // check the file(outputFile) is occupied by another program
  if (ParameterUtils::CheckFileOccupied(output_file_)) {
    return false;
  }

  // overwrite exist file without user interaction
  if (overwrite_) {
    return ParameterUtils::OverwriteExistFile(output_file_);
  }

  string cerr_info = "The file: ";
  cerr_info += output_file_;
  cerr_info += " is already exist, please save to another file or"
      " use parameter --overwrite to overwrite it.";
  cerr << "[ERROR] " << cerr_info << endl;
  ASC_LOG_ERROR("%s", cerr_info.c_str());

  return false;
}

bool AscendCameraParameter::VerifyPresenterValue() const {
  // check outout presenter value is empty
  if (!output_presenter_.empty()) {
    regex regex_presenter(kRegexPresenter.c_str());

    // check outout presenter value is match regex expression
    if (!regex_match(output_presenter_, regex_presenter)) {
      string cerr_info = "The ascendcamera parameter -s value:";
      cerr_info += output_presenter_;
      cerr_info += " is invalid, should be ip:port/channelname, "
          "ip address is ipv4 format, port value range:1~65535, "
          "channelname support number letter and '/', "
          "length value range:1~25.";

      cerr << "[ERROR] " << cerr_info << endl;
      ASC_LOG_ERROR("%s", cerr_info.c_str());

      return false;
    }
  }

  return true;
}

bool AscendCameraParameter::VerifyOutputFileValue(const bool verify_pass) {
  // verify -o parameter value
  if (output_file_.empty() || output_file_.compare("-") == kCompareEqual) {
    return true;
  }

  // get index of last '/'
  int index_last_slash = output_file_.find_last_of('/');

  // file directory
  string file_dir = "";

  // file name
  string file_name = "";

  // file name contains '/' and not start with '/'
  if (index_last_slash > kIndexFirst) {
    file_dir = output_file_.substr(kIndexFirst, index_last_slash);
    file_name = output_file_.substr(index_last_slash + kSlashStrLength);

    if (!ParameterUtils::ObtainFileAbsoluteDir(file_dir)) {
      return false;
    }
  } else if (index_last_slash == kIndexFirst) {  // file name start with '/'
    file_dir = "/";
    file_name = output_file_.substr(index_last_slash + kSlashStrLength);
    // outputFile does not contains '/'
  } else {  // file name not contains '/'
    file_dir = "./";
    // current directory is always exist
    ParameterUtils::ObtainFileAbsoluteDir(file_dir);

    file_name = output_file_;
  }

  // verfrify file name
  if (!ParameterUtils::VerifyFileName(is_image_, file_name, output_file_)) {
    return false;
  }

  // verify path
  if (!ParameterUtils::VerifyFileDir(file_dir)) {
    return false;
  }

  // if file path is not '/', then append '/'
  if (file_dir.compare("/") != kCompareEqual) {
    file_dir.append("/");
  }

  output_file_.assign(file_dir.append(file_name));

  // handle the situation where the file already exists
  if (!verify_pass) {
    return false;
  }

  return HandleExistFile();
}

bool AscendCameraParameter::VerifyWidthHeightValue() const {
  // verify image width and height values
  // The resolution list supported by the camera([width, height]):
  // [1920, 1080], [1280, 720], [704, 576], [704, 288], [352, 288]
  for (int i = 0; i < kResolutionNumber; i++) {
    //check image width and height is in value range
    if (image_width_ == kImageWidths[i] && image_height_ == kImageHeights[i]) {
      return true;
    }
  }

  return false;
}

bool AscendCameraParameter::VerifyWidthHeight() {
  // both image width and height use default value
  if (image_width_ == kInvalidValue && image_height_ == kInvalidValue) {
    image_width_ = kDefaultImageWidth;
    image_height_ = kDefaultImageHeight;

    return true;
  }

  // image width and height only input one of them
  if (image_width_ == kInvalidValue || image_height_ == kInvalidValue) {
    cerr << "[ERROR] Image width(-w, --width) and height(-h, --height) "
         << "should set in pairs." << endl;

    ASC_LOG_ERROR(
        "Image width(-w, --width) and height(-h, --height)"
        " should set in pairs.");

    return false;
  }

  // verify width and height value
  if (VerifyWidthHeightValue()) {
    return true;
  }

  cerr << "[ERROR] The ascendcamera parameter -w(--width) and -h(--height)"
       " values are invalid, [width, height] value range:[1920, 1080], "
       "[1280, 720], [704, 576], [704, 288], [352, 288]."
       << endl;

  ASC_LOG_ERROR(
      "The ascendcamera parameter -w(--width) and -h(--height) values are"
      " invalid, [width, height] value range:[1920, 1080], [1280, 720], "
      "[704, 576], [704, 288], [352, 288].");

  return false;
}

void AscendCameraParameter::DisplayHelpInfo() const {
  string helpInfo = "usage: ascendcamera [options]\n\noptions:\n"
      "  --help\t:display this information\n"
      "  -c\t\t:camera channel, value range:0 1, default value:0\n"
      "  -i\t\t:obtain jpg format image\n"
      "  -v\t\t:obtain video, parameters -i and -v must "
      "be set only one of them\n"
      "  -w, --width\t:image width\n"
      "  -h, --height\t:image height\n"
      "    \t\t image width and height should set in pairs, "
      "[width, height] value range:\n"
      "    \t\t [1920, 1080], [1280, 720], [704, 576], [704, 288], "
      "[352, 288], default:[1280, 720]\n"
      "  -s\t\t:output image or video to presenter, parameter format: "
      "ip(ipv4):port/channelname\n"
      "    \t\t ip address is ipv4 format, port value range:1~65535, "
      "channelname support number letter and '/', length value range:1~25\n"
      "  -o\t\t:output image or video to stdout or a file\n"
      "    \t\t -o -, output image or video to stdout\n"
      "    \t\t -o file, output image or video to a file(.jpg)\n"
      "    \t\t parameters -o and -s must be set only one of them\n"
      "  --overwrite\t:overwrite the file when output image or video to"
      " an exist file \n"
      "  -t, --timeout\t:run time(unit: second), default value:"
      "0(run until the process exits), not support decimals\n"
      "  --fps\t\t:video frame rate, value range:1~20, default value:10"
      "\n\nexamples:"
      "\n\t(1)ascendcamera -i -c 0 -w 1920 -h 1080 -o image.jpg"
      "\n\tGet image from camera channel 0, image width equal to 1920 and"
      " height equal to 1080, save image to image.jpg\n"
      "\n\t(2)ascendcamera -i -c 1 -w 352 -h 288 -s 10.10.10.1:7006/channel1"
      "\n\tGet image from camera channel 1, image width equal to 352 and "
      "height equal to 288, output image to "
      "presenter:10.10.10.1:7006/channel1\n"
      "\n\t(3)ascendcamera -v -c 1 --fps 20 -t 0 -w 704 -h 576 -s 10.10.10.1"
      ":7006/channel2\n\tGet video from camera channel 1 until exits,"
      " image width equal to 704 and height equal to 576, fps equal to 20,"
      " output image to presenter:10.10.10.1:7006/channel2\n";

  cerr << helpInfo << endl;
  ASC_LOG_INFO("Display help information on cerr.");
}

const MediaType AscendCameraParameter::GetMediaType() const {
  if (is_image_) {  // check media tpye is image
    return kImage;
  } else {  // media tpye is video
    return kVideo;
  }
}

const bool AscendCameraParameter::ContainsHelp() const {
  return contains_help_;
}

const string &AscendCameraParameter::GetOutputFile() const {
  return output_file_;
}

const string &AscendCameraParameter::GetOutputPresenter() const {
  return output_presenter_;
}

const int AscendCameraParameter::GetTimeout() const {
  if (timeout_ == kInvalidValue) {
    return kDeaultTimeout;
  }

  return timeout_;
}

const int AscendCameraParameter::GetImageWidth() const {
  if (image_width_ == kInvalidValue) {
    return kDefaultImageWidth;
  }

  return image_width_;
}

const int AscendCameraParameter::GetImageHeight() const {
  if (image_height_ == kInvalidValue) {
    return kDefaultImageHeight;
  }

  return image_height_;
}

const int AscendCameraParameter::GetFps() const {
  if (fps_ == kInvalidValue) {
    return kDefaultFps;
  }

  return fps_;
}

const int AscendCameraParameter::GetCameraChannel() const {
  return camera_channel_;
}

}
}
