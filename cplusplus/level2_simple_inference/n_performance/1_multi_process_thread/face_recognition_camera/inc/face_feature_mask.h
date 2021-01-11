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
#ifndef FACE_FEATURE_MASK_ENGINE_H_
#define FACE_FEATURE_MASK_ENGINE_H_

#include "face_recognition_params.h"
#include <iostream>
#include <string>
#include <dirent.h>
#include <memory>
#include <unistd.h>
#include <vector>
#include <stdint.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/types_c.h"


class FaceFeatureMaskProcess {
 public:
    /*
   * @brief: Init the FaceFeatureMaskProcess model
   * @param [in]:
   * @return: Whether init success
   */
    Result Init();

    /*
 * @brief: Handle failed when some step has the error
 * param [in]: face_detail_info->err_info Error detail
 * @return: flag for whether it is success
 */
    Result  Process(shared_ptr<FaceRecognitionInfo> &face_detail_info);

 private:
  // Private implementation a member variable, which is used to cache the input queue

    int32_t batch_size_;

  // Mean value after trained
    cv::Mat train_mean_;

  // Std value after trained
    cv::Mat train_std_;

  /*
   * Define the face feature position
   */
  enum FaceFeaturePos {
    kLeftEyeX,
    kLeftEyeY,
    kRightEyeX,
    kRightEyeY,
    kNoseX,
    kNoseY,
    kLeftMouthX,
    kLeftMouthY,
    kRightMouthX,
    kRightMouthY
  };




  /*
   * @brief: Init the normlized mean and std value, the data source is from
   *   trainMean.png and trainSTD.png
   * @return: Whether init success
   */
    Result InitNormlizedData();

  /*
   * @brief: Crop the face from original image base on the face coordinate
   *   Invoke the ez_dvpp interface to do the crop action
   * @param [in]: face_recognition_info->frame Frame info
   * @param [in]: org_img The original image information
   * @param [in]: face_imgs->rectangle Face points based on the original image,
   *   face_imgs->images Face image data after cropped, NV12
   * @return: Whether init success
   */
    Result Crop(const std::shared_ptr<FaceRecognitionInfo> &face_recognition_info, const ImageData &org_img,
            std::vector<FaceImage> &face_imgs);

  /*
   * @brief: Resize the face from cropped image base on the face size 40*40
   *   Invoke the ez_dvpp interface to do the resize action
   * @param [in]: face_imgs->image Image data after cropped
   * @param [in]: resized_image Face info after resize
   * @return: Whether init success
   */
    Result Resize( std::vector<FaceImage> &face_imgs,
              std::vector<ImageData> &resized_image);

  /*
   * @brief: Transform the image from resized YUV image to BGR image
   *   Invoke the opencv's interface to transf
   * @param [in]: resized_image The resized YUV image
   * @param [in]: bgr_image BGE images after transf
   * @return: Whether init success
   */
    Result ImageYUV2BGR(std::vector<ImageData> &resized_image,
                    std::vector<cv::Mat> &bgr_image);

  /*
   * @brief: Transform the image from (0,255) to little number
   *   Invoke the opencv's interface to do the normalization,
   *   sub mean and divide std.
   * @param [in]: normalized_image The BGR image data,between(0,255)
   * @param [in]: normalized_image The data after normalization
   * @return: Whether init success
   */
    Result NormalizeData (std::vector<cv::Mat> &normalized_image);

  /*
   * @brief: Inference the data by the FWK's Process interface
   * @param [in]: normalized_image The data after normalization
   * @param [in]: face_imgs->feature_mask The inference result
   * @return: Whether init success
   */
    Result Inference(std::vector<cv::Mat> &normalized_image,
                 std::vector<FaceImage> &face_imgs);

  /*
   * @brief: Copy the data from Mat to Memory Buffer
   * @param [in]: normalized_image The original data after normalized
   * @param [in]: number How many Mat need to be fulfilled
   */
    void EnrichDataByLastMat(std::vector<cv::Mat> &normalized_image,
                           int number);

  /*
   * @brief: Enrich the face's position by inference result
   * @param [in]: face_position Face position array
   * @param [in]: face_feature Enrich data
   */
    void EnrichFacePosition(int *face_position,
                          FaceFeature* face_feature);

  /*
   * @brief: Copy the data from Mat to Memory Buffer
   * @param [in]: normalized_image The data after normalized
   * @param [in]: start_index start index in the splited_image data.
   * @param [in]: tensor_buffer The buffer for the inference
   * @return: Handle result
   */
    int CopyDataToBuffer(std::vector<cv::Mat> &normalized_image,
                       int start_index, float* tensor_buffer);

  /*
   * @brief: Arrange the inference result from result_tensor to face_imgs->feature_mask
   * param [in]: result_tensor Inference result in the face_imgs
   * param [in]: start_number Begin number in the face_imgs
   * param [in]: end_number End number
   * param [in]: face_imgs->feature_mask face feature info
   */
    Result ArrangeFaceMarkInfo(
    const float* inference_result,
	const uint32_t inference_size,
    int start_number, int end_number,
    std::vector<FaceImage> &face_imgs);

  /*
   * @brief: Judge that whether the data is wrong, if the error is from last
   *   node, pass the data to next node directly
   * param [in]: face_detail_info->err_info
   * param [in]: flag for whether it is success
   * @return: Whether init success
   */
    Result IsDataHandleWrong(std::shared_ptr<FaceRecognitionInfo> &face_detail_info);

  /*
   * @brief: Handle failed when some step has the error
   * param [in]: error_log Error log info
   * param [in]: face_detail_info->err_info Error detail
   * @return: flag for whether it is success
   */
    Result SendFailed(const std::string error_log,
                          std::shared_ptr<FaceRecognitionInfo> &face_recognition_info);

    Result SendSuccess(shared_ptr<FaceRecognitionInfo> &face_recognition_info);

};

#endif
