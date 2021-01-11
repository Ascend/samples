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

#ifndef ASCENDDK_ASCENDCAMERA_MAIN_PROCESS_H_
#define ASCENDDK_ASCENDCAMERA_MAIN_PROCESS_H_

#include <pthread.h>
#include <atomic>

//#include "securec.h"
#include <cstring>
#include "dvpp_process.h"
#include "camera.h"
#include "output_info_process.h"
#include "ascend_camera_parameter.h"
#include "utils.h"

namespace ascend {
namespace ascendcamera {

enum MainProcessErrorCode {
  kMainProcessOk,
  kMainProcessMallocFail,
  kMainProcessInvalidParameter,
  kMainProcessMultiframeInvalidParameter,
  kMainProcessMultiframeDvppProcError,
  kMainProcessMultiframeOutputError,
  kMainProcessMultiframeMallocFail,
  kMainProcessMultiframeCpyFail,
};

enum LoopFlag {
  kNoNeedLoop,
  kNeedLoop,
};

#define CHECK_MEMCPY_S_RESULT(ret) \
if(ret != EOK){ \
  ASC_LOG_ERROR("Failed to copy memory."); \
  return kMainProcessMultiframeCpyFail; \
}

// maximum size of yuv file
const int kYuvImageMaxSize = (1920 * 1080 * 3 / 2);

// time of waitting push element
const int kWaitTime = (100 * 1000);

// 1 second = 1000 millsecond
const int kSecToMillisec = 1000;

// 1 millisecond = 1000000 nanosecond
const int kMillSecToNanoSec = 1000000;

const int kDebugRecordFrameThresHold = 100;

const int kParaCheckError = -1;

// the quality of yuv convert to jpg
const int kDvppToJpgQualityParameter = 100;

const char KPortChannelSeparator = '/';
const char kPortIpSeparator = ':';

const int kFirstIndex = 0;

struct ControlObject {
  // used for user's input parameter
  ascend::ascendcamera::AscendCameraParameter *ascend_camera_paramter;

  // used for camera
  Camera *camera;

  // dvpp process
  DvppProcess *dvpp_process;

  // output process
  OutputInfoProcess *output_process;

  // whether need loop
  LoopFlag loop_flag;

  ImageData Image;
};

typedef struct MainProcessDebugInfo {
  // frame number
  long total_frame;
  // the maximum length of queue
  int queue_max_length;
} DebugInfo;

class MainProcess {
 public:
  // class constructor
  MainProcess();

  // class destructor
  virtual ~MainProcess();

  /**
   * @brief initialization.
   * @param [in] int argc: .
   * @param [in] char *argv[].
   * @return 0:success -1:fail
   */
  int Init(int argc, char *argv[]);

  /**
   * @brief initialization.
   * @param [in] int argc: .
   * @param [in] char *argv[].
   * @return 0:success -1:fail
   */
  int Run();
  Result InitResource();
  void DestroyResource(void);

 private:

  
  int32_t deviceId_;
  aclrtContext context_;
  aclrtStream stream_;
  aclrtRunMode runMode_;
  
  // the attributes for mainproecss instance
  struct ControlObject control_object_;

  DebugInfo debug_info_;

  /**
   * @brief create a instance for camera.
   * @param [in] int width: resolution
   * @param [in] int height: resolution
   * @return MAINPROCESS_OK:success
   */
  void CameraInstanceInit(int width, int height);

  /**
   * @brief create a instance for dvpp.
   * @param [in] int width: resolution
   * @param [in] int height: resolution
   * @return MAINPROCESS_OK:success
   */
  void DvppInstanceInit(int width, int height);

  /**
   * @brief create a instance for output process.
   * @param [in] int width: resolution
   * @param [in] int height: resolution
   * @return
   */
  int OutputInstanceInit(int width, int height);

  /**
   * @brief Dvpp process and output the data
   * @param [in] CameraOutputPara *output_para: data buffer from camera
   * @param [in] DvppProcess *dvpp_process: point to dvpp instance
   * @param [in] OutputInfoProcess *output_info_process: output instance
   * @return 0:success -1:fail
   */
  int DvppAndOutputProc(CameraOutputPara *output_para,
                       DvppProcess *dvpp_process, ImageData &image,
                       OutputInfoProcess *output_info_process);

  /**
   * @brief deal with a frame data from camera.
   * @return
   */
  int DoOnce();

  /**
   * @brief process before exiting
   * @param [in] int ret: result in upper level.
   */
  void ExitProcess(int ret);
};
}
}
#endif /* ASCENDDK_ASCENDCAMERA_MAIN_PROCESS_H_ */

