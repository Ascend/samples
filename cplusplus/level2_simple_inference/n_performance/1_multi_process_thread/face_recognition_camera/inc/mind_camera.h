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

#ifndef MIND_CAMERA_H
#define MIND_CAMERA_H

#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>
#include <stdio.h>

#include "face_recognition_params.h"

#define CAMERAL_1 (0)
#define CAMERAL_2 (1)


#define CAMERADATASETS_INIT (0)
#define CAMERADATASETS_RUN  (1)
#define CAMERADATASETS_STOP (2)
#define CAMERADATASETS_EXIT (3)

#define PARSEPARAM_FAIL (-1)
#define MAX_VALUESTRING_LENGTH 25

/**
 * CameraDatasets used to capture image from camera
 */
class MindCamera {
public:
  struct CameraDatasetsConfig {
    int fps;
    int channel_id;
    int image_format;
    int resolution_width;
    int resolution_height;
    std::string ToString() const;
  };

  enum CameraOperationCode {
    kCameraOk = 0,
    kCameraNotClosed = -1,
    kCameraOpenFailed = -2,
    kCameraSetPropertyFailed = -3,
  };

  /**
   * @brief   constructor
   */
      MindCamera();

  /**
   * @brief   destructor
   */
      ~MindCamera();

  /**
   * @brief  init config of CameraDatasets by AI Config
   * @param [in]  initialized aiConfig
   * @param [in]  modelDesc
   * @return  success --> HIAI_OK ; fail --> HIAI_ERROR
   */
      bool Init();

  /**
   * @brief  Splite String source by obj and store in tmp
   * @param [in] string source
   * @param [in] string obj     value used to cut resolution ratio
   * @param [in] vector tmp     used to conserve the value of width and heigth
   * @param [out] vector tmp    value of width and heigth
   */
      static void SplitString(const std::string& source,
                          std::vector<std::string>& tmp,
                          const std::string& obj);

  /**
   * @brief   translate value to string
   * @param [in] value    channel id of camera
   * @return   string translate by value
   */
      static std::string IntToString(int value);

  /**
   * @brief  Process
   */
      bool Process();

private:
    Result GetOriginPic(
    const shared_ptr<FaceRecognitionInfo> &image_handle);
  /**
   * @brief  create Image object
   * @return : shared_ptr of data frame
   */
    std::shared_ptr<FaceRecognitionInfo> CreateBatchImageParaObj();

  /**
   * @brief : init map params
   */
    void InitConfigParams();

  /**
   * @brief   preprocess for cap camera
   * @return  camera code
   */
    MindCamera::CameraOperationCode PreCapProcess();

  /**
   * @brief  cap camera
   * @return  success-->true ; fail-->false
   */
    bool DoCapProcess();

  /**
   * @brief  parse param
   * @return value of config
   */
    int CommonParseParam(const std::string& val) const;

  /**
   * @brief  get exit flag
   * @return the value of exit
   */
    int GetExitFlag();

  /**
   * @brief  set exit flag
   * @param [in]  which value want to set
   */
    void SetExitFlag(int flag = CAMERADATASETS_STOP);

  /**
   * @brief  get width and height from string val
   * @param [in]  val     resolution ratio of picture
   * @param [in]  width   used ot conserve width of picture
   * @param [in]  height  used ot conserve height of picture
   * @param [out] width   value of width
   * @param [out] height  value of height
   */
    void ParseImageSize(const std::string& val, int& width, int& height) const;

private:
    typedef std::unique_lock<std::mutex> TLock;
    std::shared_ptr<CameraDatasetsConfig> config_;
    std::map<std::string, std::string> params_;
    // thread variable to protect exitFlag
    std::mutex mutex_;
    // ret of cameradataset
    int exit_flag_;
    uint32_t frame_id_;

};

#endif /* MindCamera_H */
