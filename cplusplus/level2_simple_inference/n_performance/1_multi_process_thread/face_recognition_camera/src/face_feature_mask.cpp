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
#include "face_feature_mask.h"
#include "face_feature_train_mean.h"
#include "face_feature_train_std.h"

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc/types_c.h"

#include <memory>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <cstring>
#include "resource_load.h"

using namespace std;
using namespace cv;

namespace {
// The image's width need to be resized
const int32_t kResizedImgWidth = 40;

// The image's height need to be resized
const int32_t kResizedImgHeight = 40;

// The rgb image's channel number
const int32_t kRgbChannel = 3;

// For each input, the result should be one tensor
const int32_t kEachResultTensorNum = 10;

// The center's size for the inference result
const float kNormalizedCenterData = 0.5;

const int32_t kSendDataIntervalMiss = 20;

const int32_t kBatch = 4;
}


Result FaceFeatureMaskProcess::Init() {
    batch_size_ = kBatch;

    if (!InitNormlizedData()) {
        return FAILED;
    }

    return SUCCESS;
}


Result FaceFeatureMaskProcess::InitNormlizedData() {
    // Load the mean data
    Mat train_mean_value(kResizedImgWidth, kResizedImgHeight, CV_32FC3, (void *)kTrainMean);
    train_mean_ = train_mean_value;
    if (train_mean_.empty()) {
        ERROR_LOG("Load mean failed!");
        return FAILED;
    }

    // Load the STD data
    Mat train_std_value(kResizedImgWidth, kResizedImgHeight, CV_32FC3, (void *)kTrainStd);
    train_std_ = train_std_value;
    if (train_std_.empty()) {
        ERROR_LOG("Load std failed!");
        return FAILED;
    }

    return SUCCESS;
}

Result Nv12ToBgr1(const ImageData &src_image,
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

Result FaceFeatureMaskProcess::Crop(const shared_ptr<FaceRecognitionInfo> &face_recognition_info, const ImageData &org_img,
                                  vector<FaceImage> &face_imgs) {

    for (vector<FaceImage>::iterator face_img_iter = face_imgs.begin();
        face_img_iter != face_imgs.end(); ++face_img_iter) {

        // Change the left top coordinate to even numver
        u_int32_t lt_horz = ((face_img_iter->rectangle.lt.x) >> 1) << 1;
        u_int32_t lt_vert = ((face_img_iter->rectangle.lt.y) >> 1) << 1;

        // Change the left top coordinate to odd numver
        u_int32_t rb_horz = (((face_img_iter->rectangle.rb.x) >> 1) << 1) - 1;
        u_int32_t rb_vert = (((face_img_iter->rectangle.rb.y) >> 1) << 1) - 1;
        INFO_LOG("The crop is from left-top(%d,%d) to right-bottom(%d,%d)",
                    lt_horz, lt_vert, rb_horz, rb_vert);

        //cout << "iter->ltX" << lt_horz << "iter->ltY" << lt_vert << "iter->rbX" << rb_horz << "iter->rbY" << rb_vert << endl;
        Result ret = ResourceLoad::GetInstance().GetDvpp().cropimage(face_img_iter->image, const_cast<ImageData &>(org_img),
                                       lt_horz, lt_vert, rb_horz, rb_vert);
        if (ret == FAILED) {
            ERROR_LOG("crop image failed\n");
        }

        cout << "face_img_iter->image.size" << face_img_iter->image.width << face_img_iter->image.height << endl;

    }
    return SUCCESS;
}

Result FaceFeatureMaskProcess::Resize( vector<FaceImage> &face_imgs,
                                    vector<ImageData> &resized_imgs) {
    // Begin to resize all the resize image
    for (vector<FaceImage>::iterator face_img_iter = face_imgs.begin();
        face_img_iter != face_imgs.end(); ++face_img_iter) {

        int32_t img_size = face_img_iter->image.size;
        if (img_size <= 0) {
            ERROR_LOG("image size less than or equal zero, size=%d", img_size);
            return FAILED;
        }
        ImageData resized_image;

        Result ret = ResourceLoad::GetInstance().GetDvpp().Resize(resized_image, const_cast<ImageData &>(face_img_iter->image), kResizedImgWidth*2, kResizedImgHeight*2);
        if (ret == FAILED) {
            ERROR_LOG("crop image failed\n");
        }
        INFO_LOG("FaceFeatureMaskProcess::Resize, push_resized_image = %x", resized_image.data.get());

        resized_imgs.push_back(resized_image);
    }
    return SUCCESS;
}

//Mat img_temp;
Result FaceFeatureMaskProcess::ImageYUV2BGR (
vector<ImageData> &resized_image,
vector<Mat> &bgr_image) {
    for (vector<ImageData>::iterator resized_img_iter = resized_image.begin();
    resized_img_iter != resized_image.end(); ++resized_img_iter) {

        int img_height = resized_img_iter->height;
        int img_width = resized_img_iter->width;
        INFO_LOG("FaceFeatureMaskProcess::ImageYUV2BGR, push_resized_image = %x", resized_img_iter->data.get());
        INFO_LOG("FaceFeatureMaskProcess::ImageYUV2BGR img_height = %d img_width %d", img_height, img_width);
        Mat src(img_height * kNv12SizeMolecule / kNv12SizeDenominator,
        img_width, CV_8UC1);

        int copy_size = img_width * img_height * kNv12SizeMolecule
                         / kNv12SizeDenominator;
        int destination_size = src.cols * src.rows * src.elemSize();
        int ret = Utils::memcpy_s(src.data, destination_size, resized_img_iter->data.get(),
                       copy_size);
        CHECK_MEM_OPERATOR_RESULTS(ret);

        Mat dst_temp;
        cvtColor(src, dst_temp, CV_YUV2BGR_NV12);
        Size dsize = Size(kResizedImgHeight, kResizedImgWidth);
        Mat dst = Mat(dsize, CV_32S);
        resize(dst_temp, dst, dsize);
        Mat dst32;
        dst.convertTo(dst32, CV_32FC3);
        bgr_image.push_back(dst32);
    }
    return SUCCESS;
}

Result FaceFeatureMaskProcess::NormalizeData (
    vector<Mat> &normalized_image) {
    // The flag to record
    Result failuar_flag = SUCCESS;
    for (vector<Mat>::iterator iter = normalized_image.begin();
        iter != normalized_image.end(); ++iter) {
        // Sub the mean and divide the std
        *iter = *iter  - train_mean_;
        *iter = *iter  / train_std_;
        // Record that whether all the data is empty
        if (failuar_flag && !(*iter).empty()) {
            failuar_flag = FAILED;
        }
    }
    // If all the data is empty, return FAILED
    if (failuar_flag) {

        return FAILED;
    }
    return SUCCESS;
}

Result FaceFeatureMaskProcess::Inference(vector<Mat> &normalized_image,
                                       vector<FaceImage> &face_imgs) {
    // Define the ai model's data

    int normalized_image_size = normalized_image.size();
    int normalized_image_mod = normalized_image_size % batch_size_;

    // calcuate the iter number
    // calcuate the value by batch
    int iter_num = normalized_image_mod == 0 ?
                (normalized_image_size / batch_size_) : (normalized_image_size / batch_size_ + 1);

    // Invoke interface to do the inference
    for (int i = 0; i < iter_num; i++) {
        INFO_LOG("Batch data's number is %d!", i);
        int start_index = batch_size_ * i;
        int end_index = start_index + batch_size_;

        // Last group data, need to fulfill the extra data
        // fulfill with last Mat in the vector
        if (i == iter_num - 1 && normalized_image_mod != 0) {
            int fulfill_number = batch_size_ - normalized_image_mod;
            EnrichDataByLastMat(normalized_image, fulfill_number);
            end_index = i * batch_size_ + normalized_image_mod;
        }

        float *tensor_buffer = new(std::nothrow) float[batch_size_ * kResizedImgWidth * kResizedImgHeight * kRgbChannel];
        if (tensor_buffer == nullptr) {
            ERROR_LOG("New the tensor buffer error.");
            return FAILED;
        }
        int last_size = CopyDataToBuffer(normalized_image, start_index, tensor_buffer);

        if (last_size == -1) {
            return FAILED;
        }

        cout << "CreateInput size" << last_size * sizeof(float);
        Result ret = ResourceLoad::GetInstance().GetModel(2).CreateInput(tensor_buffer, last_size * sizeof(float));
        if (ret != SUCCESS) {
            ERROR_LOG("Create mode input dataset failed\n");
            return FAILED;
        }
        ret = ResourceLoad::GetInstance().GetModel(2).Execute();
        if (ret != SUCCESS) {
            ResourceLoad::GetInstance().GetModel(2).DestroyInput();
            ERROR_LOG("Execute model inference failed\n");
            return FAILED;
        }
        ResourceLoad::GetInstance().GetModel(2).DestroyInput();

        aclmdlDataset* feature_mask_inference = ResourceLoad::GetInstance().GetModel(2).GetModelOutputData();
        uint32_t result_tensor_size = 0;
        float* inference_result = (float *)ResourceLoad::GetInstance().GetInferenceOutputItem(result_tensor_size, feature_mask_inference, 0);

        if (ret != SUCCESS) {
            ERROR_LOG("Fail to process the data in FWK");
            delete [] tensor_buffer;
            return FAILED;
        }

        // Get the inference result.
        if (!ArrangeFaceMarkInfo(inference_result, result_tensor_size, start_index, end_index, face_imgs)) {
            delete [] tensor_buffer;
            return FAILED;
        }

        delete [] tensor_buffer;
    }
    return SUCCESS;
}

Result FaceFeatureMaskProcess::ArrangeFaceMarkInfo(
    const float* inference_result,
    const uint32_t result_tensor_size,
    int start_number, int end_number,
    vector<FaceImage> &face_imgs) {

    // Store the output to result
    //float inference_result[result_tensor_size];
    int *face_result = new(nothrow) int[kEachResultTensorNum];
    if (face_result == nullptr) {
        ERROR_LOG("New the tensor buffer error.");
        return FAILED;
    }
    FaceImage *face_image;
    for (int i = start_number; i < end_number; ++i ) {
        face_image = &(face_imgs[i]);

        // Get image's width and height
        int width = face_image->image.width;
        int height = face_image->image.height;
	    INFO_LOG("ArrangeFaceMarkInfo width %d height %d !",width, height);

        for (int j = 0; j < kEachResultTensorNum; j++) {
            int index = (i - start_number) * kEachResultTensorNum + j;

            // Convert the data from inference data to real data
            if (index % 2 == 0) {
                face_result[j] = (int)((inference_result[index] + kNormalizedCenterData) * width);
            } else {
                face_result[j] = (int)((inference_result[index] + kNormalizedCenterData) * height);
        }
    }
    EnrichFacePosition(face_result, &(face_image->feature_mask));
    INFO_LOG(
        "left_eye=(%d, %d),right_eye=(%d, %d),nose=(%d, %d),left_mouth=(%d, %d),right_mouth=(%d, %d)",
        face_image->feature_mask.left_eye.x, face_image->feature_mask.left_eye.y,
        face_image->feature_mask.right_eye.x, face_image->feature_mask.right_eye.y,
        face_image->feature_mask.nose.x, face_image->feature_mask.nose.y,
        face_image->feature_mask.left_mouth.x, face_image->feature_mask.left_mouth.y,
        face_image->feature_mask.right_mouth.x, face_image->feature_mask.right_mouth.y);

    }
    delete[] face_result;
    return SUCCESS;
}

void FaceFeatureMaskProcess::EnrichFacePosition(int *face_position,
    FaceFeature *face_feature) {
    face_feature->flag = SUCCESS;
    face_feature->left_eye.x = face_position[FaceFeaturePos::kLeftEyeX];
    face_feature->left_eye.y = face_position[FaceFeaturePos::kLeftEyeY];
    face_feature->right_eye.x = face_position[FaceFeaturePos::kRightEyeX];
    face_feature->right_eye.y = face_position[FaceFeaturePos::kRightEyeY];
    face_feature->nose.x = face_position[FaceFeaturePos::kNoseX];
    face_feature->nose.y = face_position[FaceFeaturePos::kNoseY];
    face_feature->left_mouth.x = face_position[FaceFeaturePos::kLeftMouthX];
    face_feature->left_mouth.y = face_position[FaceFeaturePos::kLeftMouthY];
    face_feature->right_mouth.x = face_position[FaceFeaturePos::kRightMouthX];
    face_feature->right_mouth.y = face_position[FaceFeaturePos::kRightMouthY];
}

void FaceFeatureMaskProcess::EnrichDataByLastMat(vector<Mat> &normalized_image,
    int number) {
    Mat last_mat = normalized_image[normalized_image.size() - 1];
    for (int i = 0; i < number; i++) {
        normalized_image.push_back(last_mat);
    }
}

int FaceFeatureMaskProcess::CopyDataToBuffer(vector<Mat> &normalized_image,
    int start_index, float *tensor_buffer) {

    int last_size = 0;
    for (int i = start_index; i < start_index + batch_size_; i++) {

        // Split the normalized data by opencv
        vector<Mat> temp_splited_image;
        split(normalized_image[i], temp_splited_image);

        // Copy data to buffer
        for (vector<Mat>::iterator
            splited_data_iter = temp_splited_image.begin();
            splited_data_iter != temp_splited_image.end(); ++splited_data_iter) {

            // Change data type to float
            (*splited_data_iter).convertTo(*splited_data_iter, CV_32FC3);
            int splited_data_length = (*splited_data_iter).rows * (*splited_data_iter).cols;
            int each_size = splited_data_length * sizeof(float);
            int ret = Utils::memcpy_s(tensor_buffer + last_size, each_size, (*splited_data_iter).ptr<float>(0), each_size);
            if (ret != SUCCESS) {
                ERROR_LOG("memory operation failed in the feature mask's CopyDataToBuffer, error=%d", ret);
                return 0;
            }

            // Calcuate the memory's end position
            last_size += splited_data_length;
        }
    }
    return last_size;
}

Result FaceFeatureMaskProcess::IsDataHandleWrong(shared_ptr<FaceRecognitionInfo> &face_detail_info) {
    ErrorInfo err_info = face_detail_info->err_info;
    if (err_info.err_code != AppErrorCode::kNone) {
        return FAILED;
    }
    return SUCCESS;
}

Result FaceFeatureMaskProcess::SendFailed(const string error_log,
    shared_ptr<FaceRecognitionInfo> &face_recognition_info) {


    ErrorInfo err_info = face_recognition_info->err_info;
    err_info.err_code = AppErrorCode::kFeatureMask;
    err_info.err_msg = error_log;

    ResourceLoad::GetInstance().SendNextModelProcess("FaceFeatureMaskProcess",face_recognition_info);
    return SUCCESS;
}

Result FaceFeatureMaskProcess::SendSuccess(
    shared_ptr<FaceRecognitionInfo> &face_recognition_info) {


    ResourceLoad::GetInstance().SendNextModelProcess("FaceFeatureMaskProcess",face_recognition_info);

    return SUCCESS;
}

static struct timespec time1 = {0, 0};
static struct timespec time2 = {0, 0};

Result FaceFeatureMaskProcess::Process(shared_ptr<FaceRecognitionInfo> &face_recognition_info)  {

    if (!IsDataHandleWrong(face_recognition_info)) {
        ERROR_LOG("The message status is not normal");
        ResourceLoad::GetInstance().SendNextModelProcess("FaceFeatureMaskProcess",face_recognition_info);
        return FAILED;
    }
    clock_gettime(CLOCK_REALTIME, &time1);


    if (!Crop(face_recognition_info, face_recognition_info->org_img, face_recognition_info->face_imgs)) {
        return SendFailed("Crop all the data failed, all the data failed",
                      face_recognition_info);
    }
    vector<ImageData> resized_imgs;
    if (!Resize(face_recognition_info->face_imgs, resized_imgs)) {
        return SendFailed("Resize all the data failed, all the data failed",
                      face_recognition_info);
    }
    vector<Mat> bgr_imgs;
    if (!ImageYUV2BGR(resized_imgs, bgr_imgs)) {
        return SendFailed("Convert all the data failed, all the data failed",
                      face_recognition_info);
    }
    if (!NormalizeData(bgr_imgs)) {
        return SendFailed("Normalize all the data failed, all the data failed",
                      face_recognition_info);
    }
    // Inference the data
    Result inference_flag = Inference(bgr_imgs, face_recognition_info->face_imgs);
    if (!inference_flag) {
        return SendFailed("Inference the data failed",
                      face_recognition_info);
    }

    clock_gettime(CLOCK_REALTIME, &time2);
    cout << "FaceFeatureMaskProcess::Process costtime is: " << (time2.tv_sec - time1.tv_sec)*1000 + (time2.tv_nsec - time1.tv_nsec)/1000000 << "ms" << endl;

    return SendSuccess(face_recognition_info);
}
