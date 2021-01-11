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

#include "output_info_process.h"

#include <cstdio>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stddef.h>
#include <unistd.h>
#include <sys/statfs.h>

#include <iostream>

#include "ascend_camera_common.h"

using namespace std;

namespace ascend {
namespace ascendcamera {
OutputInfoProcess::OutputInfoProcess(const OutputInfoPara &para) {

  if (para.mode >= kOutputMaximum) {
    return;
  }
  output_para_ = para;
  file_desc_ = nullptr;
  presenter_channel_ = nullptr;
}

OutputInfoProcess::~OutputInfoProcess() {
}

int OutputInfoProcess::OpenOutputChannel() {
  int ret = kOutputOk;

  // open local file
  if (output_para_.mode == kOutputToLocal) {
    ret = OpenLocalFile();
  } else if (output_para_.mode == kOutputToPresenter) {
    // open channel of presenter
    ret = OpenPresenterChannel();
  }

  return ret;
}

int OutputInfoProcess::SendToChannel(unsigned char *buf, int size) {
  int ret = kOutputOk;
  OutputMethod outputMethod[] = { { kOutputToLocal,
      &OutputInfoProcess::OutputToLocal }, { kOutputToStdout,
      &OutputInfoProcess::OutputToStdout }, { kOutputToPresenter,
      &OutputInfoProcess::OutputToPresenter }, };

  ret = (this->*(outputMethod[this->output_para_.mode].func))(buf, size);
  return ret;
}

int OutputInfoProcess::CloseChannel() {
  int ret = kOutputOk;

  // close file handle
  if (output_para_.mode == kOutputToLocal) {
    ret = fclose(file_desc_);
  } else if (output_para_.mode == kOutputToPresenter) {

    // delete presenter channel
    delete presenter_channel_;
    presenter_channel_ = nullptr;
  }

  return ret;
}

int OutputInfoProcess::OpenLocalFile() {
  int ret = kOutputOk;

  // open local file.
  FILE *file_id_tmp = fopen(output_para_.path.data(), "wb+");
  // failed to open file
  if (file_id_tmp == nullptr) {
    ASC_LOG_ERROR("Failed to create file.");
    ret = kOutputOpenLocalFileFail;
  } else {  // success to open file.
    file_desc_ = file_id_tmp;
    ret = kOutputOk;
  }

  return ret;
}

int OutputInfoProcess::FreeDiskIsEnough() const {
  // check file exist
  if (access(output_para_.path.data(), F_OK) == kSystemCallReturnError) {
    return kOutputLocalFileNoExist;
  }

  struct statfs disk_info = { 0 };
  // get disk info of file path
  if (kSystemCallReturnError == statfs(output_para_.path.data(), &disk_info)) {
    return kOutputLocalFreeDiskCheckFail;
  }

  // get size of free disk
  unsigned long long free_size = disk_info.f_bavail * disk_info.f_bsize;
  // check free disk is enough
  if (free_size <= kMinFreeDiskSpace) {
    //ASC_LOG_ERROR
      /* printf(
       "Failed to write file,Disk space is less than 100M."
        "free size of disk is %d", free_size);*/
      cout << "Failed to write file,Disk space is less than 100M." << "free size of disk is " << free_size << endl;
    return kOutputLocalFreeDiskNotEnough;
  }

  return kOutputOk;
}

int OutputInfoProcess::OutputToLocal(unsigned char *buf, int size) {
  int ret = kOutputOk;

  // check the free disk is enough
  ret = FreeDiskIsEnough();
  if (kOutputOk != ret) {
    return ret;
  }

  // write data to file
  int file_write_size = fwrite(buf, sizeof(unsigned char), size, file_desc_);
  if (file_write_size < size) {
    ASC_LOG_ERROR(
        "Failed to write file.we need to write %d byte,"
        "but only write %d byte", file_write_size, size);
    fclose(file_desc_);
    return kOutputWriteFileFail;
  }

  return kOutputOk;
}

int OutputInfoProcess::OutputToStdout(unsigned char *buf, int size) {
  int ret = kOutputOk;

  // write to stdout
  ret = fwrite(buf, 1, size, stdout);
  if (ret != size) {
    ASC_LOG_ERROR(
        "Failed to write to stdout.we need to write %d byte,"
        "but only write %d byte", size, ret);
    return kOutputWriteStdoutFail;
  }

  return kOutputOk;
}

int OutputInfoProcess::OpenPresenterChannel() {
  int ret = static_cast<int>(ascend::presenter::PresenterErrorCode::kNone);

  printf("OpenChannel start");
  // open channel of presenter
  ret = static_cast<int>(ascend::presenter::OpenChannel(
      presenter_channel_, output_para_.presenter_para));
  if (ret != static_cast<int>(ascend::presenter::PresenterErrorCode::kNone)) {
    //ASC_LOG_ERROR
      printf(
        "Failed to open present channel,ret:%d. paramter is ip:%s"
        "port:%d channel name:%s ",
        ret, output_para_.presenter_para.host_ip.c_str(),
        output_para_.presenter_para.port,
        output_para_.presenter_para.channel_name.c_str());
  }
  return ret;
}

int OutputInfoProcess::OutputToPresenter(unsigned char *buf, int size) {
  int ret = static_cast<int>(ascend::presenter::PresenterErrorCode::kNone);

  // fill the data of package,which send to presenter.
  ascend::presenter::ImageFrame image_para;
  image_para.format = ascend::presenter::ImageFormat::kJpeg;
  image_para.width = output_para_.width;
  image_para.height = output_para_.height;
  image_para.data = buf;
  image_para.size = size;

  // send data to presenter.
  for (int count = 0; count < kMaxOutputRetryNum; count++) {
    ret = static_cast<int>(ascend::presenter::PresentImage(presenter_channel_,
                                                         image_para));
    
    if (ret == static_cast<int>(ascend::presenter::PresenterErrorCode::kNone)
        || ret == static_cast<int>(
          ascend::presenter::PresenterErrorCode::kInvalidParam)) {
      break;
    }
  
    ASC_LOG_WARN(
        "Fail to send data to presenter,ret = %d, "
        "width:%d height:%d size:%d retry times:%d", ret,
        image_para.width, image_para.height, image_para.size, count);
    
    sleep(1); // sleep 1 second and then retry  
  }
        
  if (ret != static_cast<int>(ascend::presenter::PresenterErrorCode::kNone)) {
    //ASC_LOG_ERROR
      printf(
        "Fail to send data to presenter,ret = %d "
        "width:%d height:%d size:%d", ret,  image_para.width,
        image_para.height, image_para.size);
    
    delete presenter_channel_; // delete presenter channel
    presenter_channel_ = nullptr;
  }

  return ret;
}

void OutputInfoProcess::PrintErrorInfo(int code) const {
  static ErrorDescription ouput_description[] = {
      { kOutputWriteStdoutFail, "Failed to write stdout." },
      { kOutputWriteFileFail, "Failed to write file." },
      { kOutputLocalFreeDiskCheckFail,
          "Failed to check free size of local disk." },
      { kOutputLocalFreeDiskNotEnough,
          "Failed to write file,Disk space is less than 100M." },
      { kOutputLocalFileNoExist, "The saved file is not exist." },
      { static_cast<int>(ascend::presenter::PresenterErrorCode::kInvalidParam),
          "Invalid parameter." },
      {static_cast<int>(ascend::presenter::PresenterErrorCode::kConnection),
          "Client failed to connect server." },
      { static_cast<int>(ascend::presenter::PresenterErrorCode::kSsl),
          "Failed to verifiy Certificate." },
      { static_cast<int>(ascend::presenter::PresenterErrorCode::kCodec),
          "Failed to Connection." },  //TODO
      { static_cast<int>(ascend::presenter::PresenterErrorCode::kNoSuchChannel),
          "The channel of present is not exist." },
      { static_cast<int>(
          ascend::presenter::PresenterErrorCode::kChannelAlreadyOpened),
          "The channel is already opened." },
      { static_cast<int>(
          ascend::presenter::PresenterErrorCode::kOther),
          "Other error." },
      { static_cast<int>(
          ascend::presenter::PresenterErrorCode::kServerReturnedUnknownError),
          "Server error." } };

  // find same errorcode and get error description
  int num = sizeof(ouput_description) / sizeof(ErrorDescription);
  for (int i = 0; i < num; i++) {
    if (code == ouput_description[i].code) {
      cerr << "[ERROR] " << ouput_description[i].code_info.c_str() << endl;
      return;
    }
  }

  cerr << "[ERROR] Other error." << endl;
}

}
}
