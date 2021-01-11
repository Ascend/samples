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

#ifndef FACE_RECOGNITION_ENGINE_H_
#define FACE_RECOGNITION_ENGINE_H_

#include <vector>

#include "face_recognition_params.h"

// aligned face data
struct AlignedFace {
// face index (using for set result)
    int32_t face_index;
// aligned face
    cv::Mat aligned_face;
// flip face according to aligned face
    cv::Mat aligned_flip_face;
};

/**
 * @brief: inference engine class
 */
class FaceRecognition {
public:

  /**
   * @brief: destruction function
   */
    ~FaceRecognition() = default;

    /**
   * @brief: face recognition
   * @param [out]: original information from front-engine
   * @return: HIAI_StatusT
   */
    Result Process(std::shared_ptr<FaceRecognitionInfo> &image_handle);

private:
  /**
   * @brief: pre-process
   * param [in]: face_imgs: face images
   * param [out]: aligned_imgs: aligned output images (RGB)
   */
    void PreProcess(const std::vector<FaceImage> &face_imgs,
                  std::vector<AlignedFace> &aligned_imgs);
  /**
   * @brief: resize image (already padding)
   * param [in]: face_img: cropped face image
   * param [out]: resized_image: call ez_dvpp output image
   * @return: true: success; false: failed
   */
    Result ResizeImg(const FaceImage &face_img,
                 ImageData &resized_image);

  /**
   * @brief Image format conversion, call OpenCV interface to transform
   *        the image, from YUV420SP_NV12 to BGR
   * @param [in] src_image: source image
   * @param [out] dst: image after conversion, Mat type
   * @return true: yuv420spnv12 convert to BGR success
   *         false: yuv420spnv12 convert to BGR failed
   */
    Result Nv12ToBgr(const ImageData &src_image, cv::Mat &dst);

  /**
   * @brief check transformation matrix for openCV wapAffine
   * @param [in] mat: transformation matrix
   * @return true: match
   *         false: not match
   */
    Result checkTransfromMat(const cv::Mat &mat);

  /**
   * @brief: aligned and flip face
   * param [in]: face_img: cropped face image
   * param [in]: resized_image: call ez_dvpp output image
   * param [in]: index: image index
   * param [out]: aligned_imgs: result image
   * @return: true: success; false: failed
   */
    Result AlignedAndFlipFace(const FaceImage &face_img,
                          const ImageData &resized_image,
                          int32_t index,
                          std::vector<AlignedFace> &aligned_imgs);

  /**
   * @brief: prepare batch buffer
   * param [in]: batch_begin: batch begin index
   * param [in]: img_count: total face count
   * param [out]: batch_buffer: batch buffer
   * param [in]: buffer_size: batch buffer total size
   * param [in]: each_img_size: each face image size
   * param [in]: aligned_imgs: aligned face and flip images
   * @return: true: success; false: failed
   */
    Result PrepareBuffer(int32_t batch_begin,
                     std::shared_ptr<uint8_t> &batch_buffer,
                     uint32_t buffer_size, uint32_t each_img_size,
                     const std::vector<AlignedFace> &aligned_imgs);

  /**
   * @brief: prepare batch buffer
   * param [in]: batch_begin: batch begin index
   * param [in]: output_data_vec: inference output data for each batch
   * param [in]: aligned_imgs: aligned face and flip images
   * param [out]: face_imgs: face images
   * @return: true: success; false: failed
   */
    Result ArrangeResult(
      int32_t batch_begin,
      float* inference_result,
	  const uint32_t inference_size,
      const std::vector<AlignedFace> &aligned_imgs,
      std::vector<FaceImage> &face_imgs);

  /**
   * @brief: inference
   * param [in]: aligned_imgs: aligned face images
   * param [out]: face_imgs: face images
   */
    Result InferenceFeatureVector(const std::vector<AlignedFace> &aligned_imgs,
                              std::vector<FaceImage> &face_imgs);

  /**
   * @brief: send result
   * param [out]: image_handle: engine transform data
   */
    void SendResult(std::shared_ptr<FaceRecognitionInfo> &image_handle);

};

#endif /* FACE_RECOGNITION_ENGINE_H_ */
