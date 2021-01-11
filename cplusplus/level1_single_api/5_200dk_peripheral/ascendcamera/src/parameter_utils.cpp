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

#include "parameter_utils.h"

#include <dirent.h>
#include <unistd.h>
//#include <securec.h>
#include <cstring>
#include <sys/statfs.h>

#include <regex>
#include <iostream>

#include "ascend_camera_common.h"

using namespace std;

namespace {
// file name maximum length
const unsigned int kMaxFileNameLength = 255;

// regex for verify .jpg file name
const string kRegexJpgFile = "-|(^.+\\.(jpg)$)";

// success to remove file
const int kRemoveSuccess = 0;

// byte conversion unit, used for convert byte to MB or GB
const int kByteConvertUnit = 1024;

// minimum disk space value
const int kMinDiskSpace = 100;

// the precision of the decimal
const int kDecimalPrecision = 2;

// read link failed
const int kReadLinkFailed = -1;

// success to set memory
const int kMemorySetSuccess = 0;

// used for concatenate logging information
stringstream log_info_stream("");
}

namespace ascend {

namespace ascendcamera {

const bool ParameterUtils::IsStartWithDash(const string &param_name) {
  if (param_name.empty()) {  // check parameter value is empty
    return false;
  }

  // check parameter value is start with "-"
  if (param_name.at(kIndexFirst) == '-') {
    return true;
  }

  return false;
}

const bool ParameterUtils::CheckFileOccupied(const string &output_file) {
  DIR *dir_proc = opendir("/proc");

  if (dir_proc == nullptr) {  // check open path:/proc
    string cerr_info = "Fail to open directory /proc, can not check file:";
    cerr_info += output_file;
    cerr_info += " is occupied by another program or not.";

    cerr << "[ERROR] " << cerr_info << endl;
    ASC_LOG_ERROR("%s", cerr_info.c_str());

    return true;
  }

  DIR* dir_pid = nullptr;

  struct dirent *dirent_proc = nullptr;
  struct dirent *dirent_pid = nullptr;

  string fd_path = "";
  string sub_fd_path = "";

  // array elements initialized to 0
  char symbolic_link_array[kMaxParamLength] = { 0 };
  char* symbolic_link = symbolic_link_array;

  // recycle when read /proc dirctory is not nullptr
  while ((dirent_proc = readdir(dir_proc)) != nullptr) {
    // skip the directory . and ..
    if (strcmp(".", dirent_proc->d_name) == kCompareEqual
        || strcmp("..", dirent_proc->d_name) == kCompareEqual)
      continue;

    fd_path.clear();
    fd_path += "/proc/";
    fd_path += dirent_proc->d_name;
    fd_path += "/fd/";

    dir_pid = opendir(fd_path.c_str());

    // skip when pid directory is nullptr
    if (dir_pid == nullptr) {
      continue;
    }

    // recycle when read pid dirctory is not nullptr
    while ((dirent_pid = readdir(dir_pid)) != nullptr) {
      // skip the directory . and ..
      if (strcmp(".", dirent_pid->d_name) == kCompareEqual
          || strcmp("..", dirent_pid->d_name) == kCompareEqual) {
        continue;
      }

      sub_fd_path.clear();
      sub_fd_path += fd_path;
      sub_fd_path += dirent_pid->d_name;

      // array elements set to 0
      memset(symbolic_link, 0, kMaxParamLength);


      int read_ret = readlink(sub_fd_path.c_str(), symbolic_link,
                              kMaxParamLength);
      // check read link is failed
      if (read_ret == kReadLinkFailed || read_ret >= kMaxParamLength) {
        continue;
      }

      // add a terminator to the end of the array
      symbolic_link[read_ret] = '\0';

      // check output filename have same name link in proc
      if (strcmp(output_file.c_str(), symbolic_link) == kCompareEqual) {
        string cerr_info = "Fail to open file:";
        cerr_info += output_file;
        cerr_info += ", the file is occupied by another program.";

        cerr << "[ERROR] " << cerr_info << endl;
        ASC_LOG_ERROR("%s", cerr_info.c_str());

        closedir(dir_proc);
        closedir(dir_pid);
        return true;
      }
    }

    closedir(dir_pid);
  }

  ASC_LOG_INFO("the output file is not occupied");

  closedir(dir_proc);
  return false;
}

const bool ParameterUtils::ObtainFileAbsoluteDir(string& file_path) {
  // array elements initialized to 0
  char real_path_array[PATH_MAX] = { 0 };
  char* real_path = realpath(file_path.c_str(), real_path_array);

  // check real path is valid value
  if (real_path != nullptr) {
    file_path.assign(real_path);

    return true;
  } else {  // real path is nullptr
    string cerr_info = "Can not find directory:";
    cerr_info += file_path;
    cerr_info += ", please use an exist directory.";

    cerr << "[ERROR] " << cerr_info << endl;
    ASC_LOG_ERROR("%s", cerr_info.c_str());

    return false;
  }
}

const bool ParameterUtils::VerifyFileName(const bool is_image,
                                          const string &file_name,
                                          const string &output_file) {
  // check file name length is bigger than maximum length
  if (file_name.length() > kMaxFileNameLength) {
    log_info_stream.str("");
    log_info_stream << "The length of file name:" << file_name
                    << " is greater than " << kMaxFileNameLength
                    << ", please input a shorter one.";
    string cerr_info = log_info_stream.str();

    cerr << "[ERROR] " << cerr_info << endl;
    ASC_LOG_ERROR("%s", cerr_info.c_str());

    return false;
  }

  // verify file name when media type is image
  if (is_image) {
    regex regex_jpg(kRegexJpgFile.c_str());

    // check image file name is match regex expression
    if (!regex_match(file_name, regex_jpg)) {
      string cerr_info = "The ascendcamera parameter -o value:";
      cerr_info += output_file;
      cerr_info += " should be '-' or a valid file name end with '.jpg'.";

      cerr << "[ERROR] " << cerr_info << endl;
      ASC_LOG_ERROR("%s", cerr_info.c_str());

      return false;
    }
  } else {  // verify file name when media type is video
    string cerr_info = "The ascendcamera has parameter -v,";
    cerr_info += " does not support -o parameter in the same time.";

    cerr << "[ERROR] " << cerr_info << endl;
    ASC_LOG_ERROR("%s", cerr_info.c_str());

    return false;
  }

  return true;
}

const bool ParameterUtils::OverwriteExistFile(const string &output_file) {
  string cerr_info = "The file:";
  cerr_info += output_file;

  // remove the file(outputFile)
  if (remove(output_file.c_str()) == kRemoveSuccess) {
    cerr_info += " is already exist and will be overwritten.";
    cerr << "[INFO] " << cerr_info << endl;
    ASC_LOG_INFO("%s", cerr_info.c_str());

    return true;
  } else {  // fail to remove the file(outputFile)
    cerr_info += " is already exist, and fail to overwrite it.";
    cerr << "[ERROR] " << cerr_info << endl;
    ASC_LOG_ERROR("%s", cerr_info.c_str());

    return false;
  }
}

const bool ParameterUtils::VerifyFileDir(const string& file_path) {
  // check write permission
  if (access(file_path.c_str(), W_OK) == kHasNoAccessPermission) {
    string cerr_info = "The directory:";
    cerr_info += file_path;
    cerr_info += " does not have write permission.";

    cerr << "[ERROR] " << cerr_info << endl;
    ASC_LOG_ERROR("%s", cerr_info.c_str());

    return false;
  }

  // struct elements initialized to 0
  struct statfs disk_info = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  statfs(file_path.c_str(), &disk_info);

  // obtain free space, convert bytes to MB and GB
  int free_space_mb = ((disk_info.f_bavail * disk_info.f_bsize)
      / kByteConvertUnit) / kByteConvertUnit;
  double free_space_gb = (double) free_space_mb / kByteConvertUnit;

  // check the directory has enough disk space
  if (free_space_mb < kMinDiskSpace) {
    log_info_stream.str("");
    log_info_stream << "The directory:" << file_path << " has " << free_space_mb
                    << "MB free space, less than " << kMinDiskSpace << "MB.";
    string cerr_info = log_info_stream.str();

    cerr << "[ERROR] " << cerr_info << endl;
    ASC_LOG_ERROR("%s", cerr_info.c_str());

    return false;
  } else {  // the directory has not enough disk space
    // convert bytes to MB and GB
    log_info_stream.str("");
    log_info_stream.precision(kDecimalPrecision);
    log_info_stream << fixed << "The directory:" << file_path << " has "
                    << free_space_gb << "GB(" << free_space_mb
                    << "MB) free space.";
    string cerr_info = log_info_stream.str();

    cerr << "[INFO] " << cerr_info << endl;
    ASC_LOG_INFO("%s", cerr_info.c_str());
  }

  return true;
}

}
}
