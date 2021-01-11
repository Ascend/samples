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

#include "face_recognition.h"

#include <cstdint>
#include <unistd.h>
#include <memory>
#include <sstream>

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/video.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc/types_c.h"

#include "resource_load.h"
using cv::Mat;
using namespace std;

namespace {
// output port (engine port begin with 0)
const uint32_t kSendDataPort = 0;

// level for call Dvpp
const int32_t kDvppToJpegLevel = 100;

// model need resized image to 300 * 300
const float kResizeWidth = 96.0;
const float kResizeHeight = 112.0;

// image source from register
const uint32_t kRegisterSrc = 1;

// The memory size of the BGR image is 3 times that of width*height.
const int32_t kBgrBufferMultiple = 3;

// destination points for aligned face
const float kLeftEyeX = 30.2946;
const float kLeftEyeY = 51.6963;
const float kRightEyeX = 65.5318;
const float kRightEyeY = 51.5014;
const float kNoseX = 48.0252;
const float kNoseY = 71.7366;
const float kLeftMouthCornerX = 33.5493;
const float kLeftMouthCornerY = 92.3655;
const float kRightMouthCornerX = 62.7299;
const float kRightMouthCornerY = 92.2041;

// wapAffine estimate check cols(=2) and rows(=3)
const int32_t kEstimateRows = 2;
const int32_t kEstimateCols = 3;

// flip face
// Horizontally flip for OpenCV
const int32_t kHorizontallyFlip = 1;
// Vertically and Horizontally flip for OpenCV
const int32_t kVerticallyAndHorizontallyFlip = -1;

// inference batch
const int32_t kBatchSize = 4;
// every batch has one aligned face and one flip face, total 2
const int32_t kBatchImgCount = 2;
// inference result index
const int32_t kInferenceResIndex = 0;
// every face inference result should contains 1024 vector (float)
const int32_t kEveryFaceResVecCount = 1024;

// sleep interval when queue full (unit:microseconds)
const __useconds_t kSleepInterval = 200000;
}

Result FaceRecognition::ResizeImg(const FaceImage &faceImg,
                                ImageData &resizedImage) {
    // input size is less than zero, return failed
    if (faceImg.image.size <= 0) {
        ERROR_LOG("image size less than or equal zero, size=%d", faceImg.image.size);
        return FAILED;
    }

    Result ret = ResourceLoad::GetInstance().GetDvpp().Resize(resizedImage, const_cast<ImageData &>(faceImg.image), kResizeWidth, kResizeHeight);
    if (ret == FAILED) {
        ERROR_LOG("Resize image failed\n");
		return FAILED;
    }
    INFO_LOG("PreProcess FaceRecognition resizedImage.width %d, resizedImage.height %d", resizedImage.width, resizedImage.height);

    return SUCCESS;
}

Result FaceRecognition::Nv12ToBgr(const ImageData &src_image,
                                Mat &dst) {
    // transforming smart pointer data into Mat type data
    // for calling OpenCV interface
    Mat src(src_image.height * kNv12SizeMolecule / kNv12SizeDenominator,
          src_image.width, CV_8UC1);

    // number of characters to copy, the size of yuv image is 1.5 times
    // than width * height;
    int copy_size = src_image.width * src_image.height * kNv12SizeMolecule
        / kNv12SizeDenominator;

    // size of the destination buffer
    int destination_size = src.cols * src.rows * src.elemSize();

    // copy the input image data into the Mat matrix for image conversion.
    int ret = Utils::memcpy_s(src.data, destination_size, src_image.data.get(),
                     copy_size);
    // check memory_s result
    CHECK_MEM_OPERATOR_RESULTS(ret);

    // call OpenCV interface to convert image from yuv420sp_nv12 to bgr
    // OpenCV interface have no return value
    cvtColor(src, dst, CV_YUV2BGR_NV12);

    return SUCCESS;
}

Result FaceRecognition::checkTransfromMat(const Mat &mat) {
    // openCV warpAffine method, need transformation matrix should match
    // 1. type need CV_32F or CV_64F
    // 2. rows must be 2
    // 3. cols must be 3
    if ((mat.type() == CV_32F || mat.type() == CV_64F)
        && mat.rows == kEstimateRows && mat.cols == kEstimateCols) {
        return SUCCESS;
    }
    INFO_LOG("transformation matrix not match, real type=%d, rows=%d, cols=%d",
         mat.type(), mat.rows, mat.cols);
    return FAILED;
}

Result FaceRecognition::AlignedAndFlipFace(
    const FaceImage &face_img, const ImageData &resized_image,
    int32_t index, vector<AlignedFace> &aligned_imgs) {
    // Step1: convert NV12 to BGR
    Mat bgr_img;
    if (!Nv12ToBgr(resized_image, bgr_img)) {
        // failed, no need to do anything
        return FAILED;
    }

    // Step2: aligned face
    // arrange destination points
    vector<cv::Point2f> dst_points;
    dst_points.emplace_back(cv::Point2f(kLeftEyeX, kLeftEyeY));
    dst_points.emplace_back(cv::Point2f(kRightEyeX, kRightEyeY));
    dst_points.emplace_back(cv::Point2f(kNoseX, kNoseY));
    dst_points.emplace_back(cv::Point2f(kLeftMouthCornerX, kLeftMouthCornerY));
    dst_points.emplace_back(cv::Point2f(kRightMouthCornerX, kRightMouthCornerY));

    // arrange source points
    // should be changed to resized image point
    vector<cv::Point2f> src_points;
    float face_width = (float) face_img.image.width;
    float face_height = (float) face_img.image.height;
    // left eye
    float left_eye_x = face_img.feature_mask.left_eye.x / face_width
        * kResizeWidth;
    float left_eye_y = face_img.feature_mask.left_eye.y / face_height
        * kResizeHeight;
    src_points.emplace_back(left_eye_x, left_eye_y);
    // right eye
    float right_eye_x = face_img.feature_mask.right_eye.x / face_width
        * kResizeWidth;
    float right_eye_y = face_img.feature_mask.right_eye.y / face_height
        * kResizeHeight;
    src_points.emplace_back(right_eye_x, right_eye_y);
    // nose
    float nose_x = face_img.feature_mask.nose.x / face_width * kResizeWidth;
    float nose_y = face_img.feature_mask.nose.y / face_height * kResizeHeight;
    src_points.emplace_back(nose_x, nose_y);
    // left mouth corner
    float left_mouth_x = face_img.feature_mask.left_mouth.x / face_width
        * kResizeWidth;
    float left_mouth_y = face_img.feature_mask.left_mouth.y / face_height
        * kResizeHeight;
    src_points.emplace_back(left_mouth_x, left_mouth_y);
    // right mouth corner
    float right_mouth_x = face_img.feature_mask.right_mouth.x / face_width
        * kResizeWidth;
    float right_mouth_y = face_img.feature_mask.right_mouth.y / face_height
        * kResizeHeight;
    src_points.emplace_back(right_mouth_x, right_mouth_y);
    // call OpenCV to aligned
    // first set FAILED (partialAffine), limited to combinations of translation,
    // rotation, and uniform scaling (4 degrees of freedom)
    Mat point_estimate = estimateAffinePartial2D(src_points, dst_points);

    // transform matrix not match warpAffine, set SUCCESS (fullAffine)
    if (!checkTransfromMat(point_estimate)) {
        INFO_LOG("estimateRigidTransform using partialAffine failed,"
                    "try to using fullAffine");
        point_estimate = estimateAffine2D(src_points, dst_points);
    }

    /*for (vector<cv::Point2f>::iterator iter = src_points.begin();
    iter != src_points.end(); ++iter) {
        cv::circle(bgr_img, *iter, 2, cv::Scalar(0, 0, 255));
    }
    imwrite("Recognition_AlignedAndFlipFace.jpg",bgr_img);*/
  // check again, if not match, return FAILED
    if (!checkTransfromMat(point_estimate)) {
        INFO_LOG("estimateRigidTransform using partialAffine and fullAffine all failed."
        "skip this image.");
        INFO_LOG( "left_eye=(%f, %f)",
                    left_eye_x, left_eye_y);
        INFO_LOG("right_eye=(%f, %f)",
                    right_eye_x, right_eye_y);
        INFO_LOG("nose=(%f, %f)", nose_x,
                    nose_y);
        INFO_LOG("left_mouth=(%f, %f)",
                    left_mouth_x, left_mouth_y);
        INFO_LOG("right_mouth=(%f, %f)",
                    right_mouth_x, right_mouth_y);
        return FAILED;
    }

    cv::Size img_size(kResizeWidth, kResizeHeight);
    Mat aligned_img;
    warpAffine(bgr_img, aligned_img, point_estimate, img_size);

    // Step3: flip image (call OpenCV)
    // first Horizontally flip
    Mat h_flip;
    flip(aligned_img, h_flip, kHorizontallyFlip);
    // then Vertically and Horizontally flip
    Mat hv_flip;
    flip(h_flip, hv_flip, kVerticallyAndHorizontallyFlip);

    // Step4: change to RGB (because AIPP need this format)
    cvtColor(aligned_img, aligned_img, cv::COLOR_BGR2RGB);
    cvtColor(hv_flip, hv_flip, cv::COLOR_BGR2RGB);

    // Step5: set back to aligned images
    AlignedFace result;
    result.face_index = index;
    result.aligned_face = aligned_img;
    result.aligned_flip_face = hv_flip;
    aligned_imgs.emplace_back(result);
    return SUCCESS;
}

void FaceRecognition::PreProcess(const vector<FaceImage> &face_imgs,
                                 vector<AlignedFace> &aligned_imgs) {
    // loop each cropped face image
    for (int32_t index = 0; index < face_imgs.size(); ++index) {
        // check flag, if FAILED need not to do anything
        if (!face_imgs[index].feature_mask.flag) {
            INFO_LOG("flag is FAILED, skip it");
            continue;
        }

        // resize image, if failed, skip it
        ImageData resized_image;
        if (!ResizeImg(face_imgs[index], resized_image)) {
            continue;
        }

        // aligned and flip face, if failed, skip it
        if (!AlignedAndFlipFace(face_imgs[index], resized_image, index,
                            aligned_imgs)) {
            continue;
        }

        // success
        INFO_LOG("aligned face success, index=%d", index);
    }
}

Result FaceRecognition::PrepareBuffer(int32_t batch_begin,
                                    shared_ptr<uint8_t> &batch_buffer,
                                    uint32_t buffer_size,
                                    uint32_t each_img_size,
                                    const vector<AlignedFace> &aligned_imgs) {
    // loop for each image in one batch
    uint32_t last_size = 0;
    for (int i = 0; i < kBatchSize; ++i) {
        // real image
        if (batch_begin + i < aligned_imgs.size()) {
            AlignedFace face_img = aligned_imgs[batch_begin + i];
            // copy aligned face image
            int ret = Utils::memcpy_s(batch_buffer.get() + last_size,
                             buffer_size - last_size,
                             face_img.aligned_face.ptr<uint8_t>(),
                             each_img_size);
            // check Utils::memcpy_s result
            CHECK_MEM_OPERATOR_RESULTS(ret);
            last_size += each_img_size;

            // copy aligned flip face image
            ret = Utils::memcpy_s(batch_buffer.get() + last_size, buffer_size - last_size,
                     face_img.aligned_flip_face.ptr<uint8_t>(), each_img_size);
            //imwrite("aligned_face.jpg",face_img.aligned_face);
            // check Utils::memcpy_s result
            CHECK_MEM_OPERATOR_RESULTS(ret);
            last_size += each_img_size;
        } else {  // image size less than batch size
            // need set all data to 0
            memset(batch_buffer.get() + last_size, static_cast<char>(0),
            buffer_size - last_size);

            last_size += each_img_size * kBatchImgCount;
        }
    }

    return SUCCESS;
}

Result FaceRecognition::ArrangeResult(
    int32_t batch_begin,
    float* inference_result,
	const uint32_t inference_size,
    const vector<AlignedFace> &aligned_imgs, vector<FaceImage> &face_imgs) {

    // size check, get real face batch count
    int32_t batch_count = kBatchSize;
    if (aligned_imgs.size() - batch_begin < kBatchSize) {
        batch_count = aligned_imgs.size() - batch_begin;
    }

    // (2) every image need 1024 float vector
    int32_t check_size = batch_count * kEveryFaceResVecCount;
    if (check_size > inference_size) {
        INFO_LOG("inference result size not correct. batch_begin=%d, total_face=%d, inference_size=%d, need_size=%d, cont %d",
            batch_begin, aligned_imgs.size(), inference_size, check_size);
        return FAILED;
    }

    // Step3: copy results
    float result[inference_size];

    INFO_LOG("set inference sizeof(result) =%d", sizeof(result));
    int ret = Utils::memcpy_s(result, sizeof(result), inference_result, sizeof(result));

    // check Utils::memcpy_s result
    CHECK_MEM_OPERATOR_RESULTS(ret);

    // Step4: set every face inference result
    for (int i = 0; i < batch_count; ++i) {
        // set to face_imgs according to index
        int32_t org_crop_index = aligned_imgs[batch_begin + i].face_index;
        for (int j = 0; j < kEveryFaceResVecCount; ++j) {
        int32_t result_index = i * kEveryFaceResVecCount + j;
        face_imgs[org_crop_index].feature_vector.emplace_back(
             result[result_index]);
        }
        INFO_LOG("set inference result successfully. face index=%d",
                    i + batch_begin);
    }

    return SUCCESS;
}

Result FaceRecognition::InferenceFeatureVector(
    const vector<AlignedFace> &aligned_imgs, vector<FaceImage> &face_imgs) {
    // initialize buffer
    uint32_t each_img_size = aligned_imgs[0].aligned_face.total()
      * aligned_imgs[0].aligned_face.channels();
    uint32_t buffer_size = each_img_size * kBatchImgCount * kBatchSize;
    shared_ptr<uint8_t> batch_buffer = shared_ptr<uint8_t>(
        new uint8_t[buffer_size], default_delete<uint8_t[]>());

    // loop for each batch
    for (int32_t index = 0; index < aligned_imgs.size(); index += kBatchSize) {
        // 1. prepare input buffer for each batch
        // if prepare failed, also need to deal next batch
        if (!PrepareBuffer(index, batch_buffer, buffer_size, each_img_size,
                       aligned_imgs)) {
            INFO_LOG("prepare buffer failed, batch_begin=%d, batch_size=%d",
                      index, kBatchSize);
        continue;
    }

    // 2. prepare input data
	Result ret = ResourceLoad::GetInstance().GetModel(3).CreateInput(batch_buffer.get(), buffer_size);

    // 4. Process inference

	ret = ResourceLoad::GetInstance().GetModel(3).Execute();
	if (ret != SUCCESS) {
        ResourceLoad::GetInstance().GetModel(3).DestroyInput();
	    ERROR_LOG("Execute model inference failed\n");
	    return FAILED;
	}
    ResourceLoad::GetInstance().GetModel(3).DestroyInput();

    aclmdlDataset* recognition_inference = ResourceLoad::GetInstance().GetModel(3).GetModelOutputData();
	uint32_t inference_size = 0;
    float* inference_result = (float *)ResourceLoad::GetInstance().GetInferenceOutputItem(inference_size, recognition_inference, 0);
	  
	INFO_LOG("arrange GetInferenceOutputItem, inference=%d",
        sizeof(inference_result));

    // 5. arrange result for each batch
    // if failed, also need to deal next batch
        if (!ArrangeResult(index, inference_result, inference_size, aligned_imgs, face_imgs)) {
            INFO_LOG("arrange inference result failed, batch_begin=%d, batch_size=%d",
                      index, kBatchSize);
            continue;
        }
    }
    return SUCCESS;
}

unsigned char * CopyImageData(ImageData& inputData)
{
    void *originalJpegPicBuffer;
    INFO_LOG("CopyImageData inputData.size %d.", inputData.size);
    aclError ret = aclrtMalloc(&originalJpegPicBuffer, inputData.size, ACL_MEM_MALLOC_HUGE_FIRST);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("malloc originalJpegPicBuffer fail .");
        return nullptr;
    }
    // copy input to device memory
    aclrtMemcpy(originalJpegPicBuffer, inputData.size, inputData.data.get(), inputData.size, ACL_MEMCPY_DEVICE_TO_DEVICE);
    //free(dataHost);
    return (unsigned char *)originalJpegPicBuffer;
}

void FaceRecognition::SendResult(
    shared_ptr<FaceRecognitionInfo> &image_handle) {

    ResourceLoad::GetInstance().SendNextModelProcess("FaceRecognition",image_handle);
}

static struct timespec time1 = {0, 0};
static struct timespec time2 = {0, 0};

Result FaceRecognition::Process(
    shared_ptr<FaceRecognitionInfo> &image_handle) {
    string err_msg = "";
    if (image_handle->err_info.err_code != AppErrorCode::kNone) {
        INFO_LOG("front engine dealing failed, err_code=%d, err_msg=%s",
                    image_handle->err_info.err_code,
                    image_handle->err_info.err_msg.c_str());
        SendResult(image_handle);
        return FAILED;
    }
    clock_gettime(CLOCK_REALTIME, &time1);
    // pre-process
    vector<AlignedFace> aligned_imgs;
    PreProcess(image_handle->face_imgs, aligned_imgs);

    // need to inference or not
    if (aligned_imgs.empty()) {

        SendResult(image_handle);
        return SUCCESS;
    }

    // inference and set results
    InferenceFeatureVector(aligned_imgs, image_handle->face_imgs);
    clock_gettime(CLOCK_REALTIME, &time2);
    cout << "FaceRecognition::Process costtime is: " << (time2.tv_sec - time1.tv_sec)*1000 + (time2.tv_nsec - time1.tv_nsec)/1000000 << "ms" << endl;

    // send result
    SendResult(image_handle);
    return SUCCESS;
}
