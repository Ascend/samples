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

#ifndef ASCENDDK_ASCENDCAMERA_CAMERA_H_
#define ASCENDDK_ASCENDCAMERA_CAMERA_H_

#include <vector>
#include <memory>
#include "utils.h"
extern "C" {
#include "driver/peripheral_api.h"
}

namespace ascend {
namespace ascendcamera {

const int kCameraReturnInvalid = 0;
const int kCameraReturnValid = 1;

enum AscendCameraErrorID {
  kCameraInitOk = 0,
  kCameraRunOk = 0,
  kCameraInitError = -1,
  kCameraStatusError = -2,
  kCameraOpenError = -3,
  kCameraSetFpsError = -4,
  kCameraSetFormatError = -5,
  kCameraSetResolutionError = -6,
  kCameraSetWorkModeError = -7,
  kCameraGetInfoError = -8,
  kCameraMediaStatusError = -9,
};

struct CameraPara {
  // camera fps
  int fps;

  // camera channel ID
  unsigned int channel_id;

  // format of yuv
  CameraImageFormat image_format;

  // width height
  CameraResolution resolution;

  // camera capture time
  int timeout;

  // jpg or h264
  int capture_obj_flag;
};

struct CameraOutputPara {
  // frame id
  int frame_id;

  // frame timeStamp
  time_t timestamp;

  // output buffer
  std::shared_ptr<uint8_t> data;

  // output buffer size
  unsigned int size;

  // resolution(width height)
  CameraResolution resolution;

  // camera id
  unsigned int channel_id;
};

class Camera {
 public:

  /**
   * @brief class constructor
   * @param [in] camera_para camera_para: instance paramter
   */
  Camera(const CameraPara& camera_para);

  // class destructor
  virtual ~Camera();

  // initialization camera
  int InitCamera();

  /**
   * @brief get one frame data from camera
   * @param [out] output_para *pOutputPara: used for storage frame data.
   * @return kCameraRunOk: success to get frame data;
   *         kCameraGetInfoError: failed to get frame data.
   */
  int CaptureCameraInfo(CameraOutputPara *output_para);

  /**
   * @brief get a error message according to error code.
   * @param [in] int code: error code.
   */
  void PrintErrorInfo(int code) const;

  /**
   * @brief get camera id.
   * @return camera id
   */
  int GetChannelId() const;

  /**
   * @brief get timeout(user set).
   * @return timeout
   */
  int GetUserTimeout() const;

 private:
  // frame id.
  int frame_id_;

  // size of one frame data from camera.
  int image_size_;

  // the attributes date of camera
  CameraPara camera_instance_para_;
};





}
}

class Camera_265 {
    public:
    Camera_265(uint32_t id =0, uint32_t fps = 5, uint32_t width = 1280, uint32_t height = 720);
    ~Camera_265();
    Result Open(int channelID);
    Result Close(int channelID);
    Result Read(int channelID,ImageData& frame);
    bool IsOpened(int channelID);
    private:
    uint32_t id_;
    uint32_t fps_;
    uint32_t width_;
    uint32_t height_;
    uint32_t size_;

    Result SetProperty(int channelID);
    bool isAlign_;
    bool isOpened_[2];

    void* outBuf_ = nullptr ;
};

#endif /* ASCENDDK_ASCENDCAMERA_CAMERA_H_ */
