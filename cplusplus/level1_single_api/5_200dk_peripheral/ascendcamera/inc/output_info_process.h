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

#ifndef ASCENDDK_ASCENDCAMERA_OUTPUT_INFO_PROCESS_H_
#define ASCENDDK_ASCENDCAMERA_OUTPUT_INFO_PROCESS_H_

#include <stdio.h>
#include <string>

#include "ascenddk/presenter/agent/presenter_channel.h"

namespace ascend {
namespace ascendcamera {

const unsigned long long kMinFreeDiskSpace = 100 * 1024 * 1024;
const int kSystemCallReturnError = -1;
const int kMaxOutputRetryNum = 30;

// output mode : 1. save file to local; 2. send data to stdout;
//               3. send data to presenter
enum OutputMode {
  kOutputToLocal,
  kOutputToStdout,
  kOutputToPresenter,
  kOutputMaximum,
};

enum OutputErrorCode {
  kOutputOk = 0,
  kOutputWriteStdoutFail = -1,
  kOutputWriteFileFail = -2,
  kOutputOpenLocalFileFail = -3,
  kOutputLocalFreeDiskNotEnough = -4,
  kOutputLocalFreeDiskCheckFail = -5,
  kOutputLocalFileNoExist = -6,
};

struct OutputInfoPara {
  std::string path;  // file path in local
  int mode;  // output mode
  int width;  // resolution width
  int height;  // resolution height
  ascend::presenter::OpenChannelParam presenter_para;  // presenter data
};

/*
 *
 */

class OutputInfoProcess {
 public:
  /**
   * @brief class constructor
   * @param [in] OutputInfoPara para: output instance parameter.
   */
  OutputInfoProcess(const OutputInfoPara &para);

  // class destructor
  virtual ~OutputInfoProcess();

  /**
   * @brief open channel of output
   * @return  enum OutputErrorCode and enum PresenterErrorCode .
   */
  int OpenOutputChannel();

  /**
   * @brief data send to channel
   * @param [in] unsigned char *buf: data buffer
   * @param [in] int size  : size of data buffer
   * @return  enum OutputErrorCode
   */
  int SendToChannel(unsigned char *buf, int size);

  /**
   * @brief close channel
   * @return  0
   */
  int CloseChannel();

  /**
   * @brief get a error message according to error code.
   * @param [in] int code: error code.
   */
  void PrintErrorInfo(int code) const;

  // function pointer
  typedef int (OutputInfoProcess::*OutputFunc)(unsigned char *buf, int size);

  // struct used for stoarge ouput attributes
  typedef struct OutputProc {
    int mode;
    OutputFunc func;
  } OutputMethod;

 private:
  /**
   * @brief open local file
   * @return  enum OutputErrorCode.
   */
  int OpenLocalFile();

  /**
   * @brief open chennel of presenter
   * @return  enum PresenterErrorCode
   */
  int OpenPresenterChannel();

  /**
   * @brief data save to local file
   * @param [in] unsigned char *buf: data buffer
   * @param [in] int size  : size of data buffer
   * @return  enum OutputErrorCode
   */
  int OutputToLocal(unsigned char *buf, int size);

  /**
   * @brief data save to presenter
   * @param [in] unsigned char *buf: data buffer
   * @param [in] int size  : size of data buffer
   * @return  enum PresenterErrorCode
   */
  int OutputToPresenter(unsigned char *buf, int size);

  /**
   * @brief data send to stdout
   * @param [in] unsigned char *buf: data buffer
   * @param [in] int size  : size of data buffer
   * @return  enum OutputErrorCode
   */
  int OutputToStdout(unsigned char *buf, int size);

  /**
   * @brief check whether the free disk is enough
   * @return  enum OutputErrorCode
   */
  int FreeDiskIsEnough() const;

  // the attributes of output instance
  OutputInfoPara output_para_;

  // local file id
  FILE *file_desc_ = nullptr;

  // presenter channel
  ascend::presenter::Channel *presenter_channel_ = nullptr;
};
}
}
#endif /* ASCENDDK_ASCENDCAMERA_OUTPUT_INF_OPROCESS_H_ */
