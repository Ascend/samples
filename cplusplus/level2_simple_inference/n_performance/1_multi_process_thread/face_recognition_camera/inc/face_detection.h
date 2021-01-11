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

#ifndef FACE_DETECTION_ENGINE_H_
#define FACE_DETECTION_ENGINE_H_

#include "face_recognition_params.h"
#include "opencv2/opencv.hpp"

/**
 * @brief: inference engine class
 */
class FaceDetection {
public:
  /**
   * @brief: construction function
   */
    FaceDetection();

  /**
   * @brief: destruction function
   */
    ~FaceDetection() = default;

  /**
   * @brief: face detection inference engine initialize
   * @param [in]: engine's parameters which configured in graph.config
   * @param [in]: model description
   * @return: HIAI_StatusT
   */
    Result Init();

    /**
   * @brief: face detection
   * @param [out]: original information from front-engine
   * @return: HIAI_StatusT
   */
    Result Process(std::shared_ptr<FaceRecognitionInfo> &image_handle);

private:
  // cache AI model parameters
  //std::shared_ptr<hiai::AIModelManager> ai_model_manager_;
  //  DvppProcess dvpp_;
  // confidence : used to check inference result
    float confidence_;

  /**
   * @brief: check confidence is valid or not
   * param [in]: confidence
   * @return: false:invalid, true: valid
   */
    Result IsValidConfidence(float confidence);

  /**
   * @brief: check inference results is valid or not
   * param [in]: attribute
   * param [in]: score
   * param [in]: face rectangle
   * @return: false:invalid, true: valid
   */
    Result IsValidResults(float attr, float score, const FaceRectangle &rectangle);

  /**
   * @brief: correction ratio
   * param [in]: coordinate ratio
   * @return: ratio in [0, 1]
   *          when ratio less than zero, then return zero
   *          when ratio more than one, then return one
   */
    float CorrectionRatio(float ratio);

  /**
   * @brief: pre-process
   * param [in]: image_handle: original image
   * param [out]: resized_image: ez_dvpp output image
   * @return: true: success; false: failed
   */
    Result PreProcess(const std::shared_ptr<FaceRecognitionInfo> &image_handle,
                  ImageData &resized_image);

  /**
   * @brief: inference
   * param [in]: resized_image: ez_dvpp output image
   * param [out]: output_data_vec: inference output
   * @return: true: success; false: failed
   */
    Result Inference(
        const ImageData &resized_image,
        aclmdlDataset*& detection_inference);

  /**
   * @brief: post process
   * param [out]: image_handle: engine transform image
   * param [in]: output_data_vec: inference output
   * @return: true: success; false: failed
   */
    Result PostProcess(
        std::shared_ptr<FaceRecognitionInfo> &image_handle,
        aclmdlDataset* detection_inference);



  /**
   * @brief: handle the error scene
   * param [in]: err_code: the error code
   * param [in]: err_msg: the error message
   * param [out]: image_handle: engine transform image
   */
    void HandleErrors(AppErrorCode err_code, const std::string &err_msg,
                    std::shared_ptr<FaceRecognitionInfo> &image_handle);

  /**
   * @brief: send result
   * param [out]: image_handle: engine transform image
   */
   void SendResult(std::shared_ptr<FaceRecognitionInfo> &image_handle);

};

#endif /* FACE_DETECTION_ENGINE_H_ */
