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

#include <vector>
#include <sstream>
#include <unistd.h>
#include "face_detection.h"
#include "dvpp_process.h"
#include <fstream>
#include "resource_load.h"

using namespace std;


namespace {

// output port (engine port begin with 0)
const uint32_t kSendDataPort = 0;

// model need resized image to 300 * 300
const float kResizeWidth = 300.0;
const float kResizeHeight = 300.0;

// confidence parameter key in graph.confi
const string kConfidenceParamKey = "confidence";

// confidence range (0.0, 1.0]
const float kConfidenceMin = 0.0;
const float kConfidenceMax = 1.0;

// results
// inference output result index
const int32_t kResultIndex = 0;
// each result size (7 float)
const int32_t kEachResultSize = 7;
// attribute index
const int32_t kAttributeIndex = 1;
// score index
const int32_t kScoreIndex = 2;
// left top X-axis coordinate point
const int32_t kLeftTopXaxisIndex = 3;
// left top Y-axis coordinate point
const int32_t kLeftTopYaxisIndex = 4;
// right bottom X-axis coordinate point
const int32_t kRightBottomXaxisIndex = 5;
// right bottom Y-axis coordinate point
const int32_t kRightBottomYaxisIndex = 6;

// face attribute
const float kAttributeFaceLabelValue = 1.0;
const float kAttributeFaceDeviation = 0.00001;

// ratio
const float kMinRatio = 0.0;
const float kMaxRatio = 1.0;

// image source from register
const uint32_t kRegisterSrc = 1;

const uint32_t kBBoxDataBufId = 1;
const uint32_t kBoxNumDataBufId = 0;

enum BBoxIndex {EMPTY = 0, LABEL,SCORE,TOPLEFTX,TOPLEFTY, BOTTOMRIGHTX, BOTTOMRIGHTY};

// sleep interval when queue full (unit:microseconds)
const __useconds_t kSleepInterval = 200000;

}

FaceDetection::FaceDetection(){
    confidence_ = -1.0;  // initialized as invalid value
}

Result FaceDetection::Init() {
    confidence_ = 0.9;
    // validate confidence
    if (!IsValidConfidence(confidence_)) {
      ERROR_LOG("confidence invalid, please check your configuration.");
      return FAILED;
    }

    return SUCCESS;
}

Result FaceDetection::IsValidConfidence(float confidence) {
    if((confidence > kConfidenceMin) && (confidence <= kConfidenceMax) == false) {
        return FAILED;
    }
    return SUCCESS;
}

Result FaceDetection::IsValidResults(float attr, float score,
                                   const FaceRectangle &rectangle) {
    // attribute is not face (background)
    if (abs(attr - kAttributeFaceLabelValue) > kAttributeFaceDeviation) {
        return FAILED;
    }

    // confidence check
    if ((score < confidence_) || !IsValidConfidence(score)) {
        return FAILED;
    }

    // position check : lt == rb invalid
    if ((rectangle.lt.x == rectangle.rb.x)
      && (rectangle.lt.y == rectangle.rb.y)) {
        return FAILED;
    }
    return SUCCESS;
}

float FaceDetection::CorrectionRatio(float ratio) {
    float tmp = (ratio < kMinRatio) ? kMinRatio : ratio;
    return (tmp > kMaxRatio) ? kMaxRatio : tmp;
}

Result FaceDetection::PreProcess(
    const shared_ptr<FaceRecognitionInfo> &image_handle,
    ImageData &resized_image) {
    // input size is less than zero, return failed
    int32_t img_size = image_handle->org_img.size;
    if (img_size <= 0) {
        ERROR_LOG("original image size less than or equal zero, size=%d",
                    img_size);
        return FAILED;
    }

    // call ez_dvpp to resize image
    Result ret = ResourceLoad::GetInstance().GetDvpp().Resize(resized_image, image_handle->org_img, kResizeWidth, kResizeHeight);
    if (ret == FAILED) {
        ERROR_LOG("Resize image failed\n");
        return FAILED;
    }


    return SUCCESS;
}

Result FaceDetection::Inference(
    const ImageData &resized_image,
    aclmdlDataset*& detection_inference) {

    Result ret = ResourceLoad::GetInstance().GetModel(1).CreateInput(resized_image.data.get(),
                                    resized_image.size);
    if (ret != SUCCESS) {
        ERROR_LOG("Create mode input dataset failed\n");
        return FAILED;
    }

    ret = ResourceLoad::GetInstance().GetModel(1).Execute();
    if (ret != SUCCESS) {
        ResourceLoad::GetInstance().GetModel(1).DestroyInput();
        ERROR_LOG("Execute model inference failed\n");
        return FAILED;
    }
    ResourceLoad::GetInstance().GetModel(1).DestroyInput();

    detection_inference = ResourceLoad::GetInstance().GetModel(1).GetModelOutputData();

    return SUCCESS;
}

Result FaceDetection::PostProcess(
    shared_ptr<FaceRecognitionInfo> &image_handle,
    aclmdlDataset* detection_inference) {
    // inference result vector only need get first result
    // because batch is fixed as 1
    uint32_t dataSize = 0;
    float* detectData = (float *)ResourceLoad::GetInstance().GetInferenceOutputItem(dataSize, detection_inference, kBBoxDataBufId);


    if (detectData == nullptr)
        return FAILED;

    uint32_t width = image_handle->org_img.width;
    uint32_t height = image_handle->org_img.height;
    float *ptr = detectData;
    for (int32_t i = 0; i < dataSize - kEachResultSize; i += kEachResultSize) {
        ptr = detectData + i;

    //for (uint32_t i = 0; i < totalBox; i++) {

        //Point point_lt, point_rb;
        uint32_t score_int = uint32_t(ptr[SCORE] * 100);
        if (score_int < 70.0)
            break;
		
		// attribute
        float attr = ptr[LABEL];
        // confidence
        float score = ptr[SCORE];

	    // position
        FaceRectangle rectangle;
        rectangle.lt.x = CorrectionRatio(ptr[TOPLEFTX]) * width;
        rectangle.lt.y = CorrectionRatio(ptr[TOPLEFTY]) * height;
        rectangle.rb.x = CorrectionRatio(ptr[BOTTOMRIGHTX]) * width;
        rectangle.rb.y = CorrectionRatio(ptr[BOTTOMRIGHTY]) * height;	
		cout << "ltX " << rectangle.lt.x << "ltY " << rectangle.lt.y << "rbX " << rectangle.rb.x << "rbY " << rectangle.rb.y << endl;
        // check results is invalid, skip it
        if (!IsValidResults(attr, score, rectangle)) {
            continue;
        }

        // push back to image_handle
        FaceImage faceImage;
        faceImage.rectangle = rectangle;
        image_handle->face_imgs.emplace_back(faceImage);
    }
    return SUCCESS;
}

void FaceDetection::HandleErrors(
    AppErrorCode err_code, const string &err_msg,
    shared_ptr<FaceRecognitionInfo> &image_handle) {

    // set error information
    image_handle->err_info.err_code = err_code;
    image_handle->err_info.err_msg = err_msg;

    // send data
    SendResult(image_handle);
}

void FaceDetection::SendResult(
    shared_ptr<FaceRecognitionInfo> &image_handle) {
    ResourceLoad::GetInstance().SendNextModelProcess("FaceDetection", image_handle);
}

static struct timespec time1 = {0, 0};
static struct timespec time2 = {0, 0};

static struct timespec time3 = {0, 0};
static struct timespec time4 = {0, 0};

Result FaceDetection::Process(
    shared_ptr<FaceRecognitionInfo> &image_handle) {
    string err_msg = "";
    if (image_handle->err_info.err_code != AppErrorCode::kNone) {
        ERROR_LOG("front engine dealing failed, err_code=%d, err_msg=%s",
                    int(image_handle->err_info.err_code),
                    image_handle->err_info.err_msg.c_str());

        SendResult(image_handle);
        return FAILED;
    }
    clock_gettime(CLOCK_REALTIME, &time1);
    // resize image
    ImageData resized_image;
    if (PreProcess(image_handle, resized_image) == FAILED) {
        err_msg = "face_detection call ez_dvpp to resize image failed.";
        HandleErrors(AppErrorCode::kDetection, err_msg, image_handle);
        return FAILED;
    }
    clock_gettime(CLOCK_REALTIME, &time2);


    // inference
    aclmdlDataset* detection_inference;
    if (Inference(resized_image, detection_inference) == FAILED) {
        err_msg = "face_detection inference failed.";
        HandleErrors(AppErrorCode::kDetection, err_msg, image_handle);
        return FAILED;
    }
    clock_gettime(CLOCK_REALTIME, &time3);


    // post process
    if (PostProcess(image_handle, detection_inference) == FAILED) {
        err_msg = "face_detection deal result failed.";
        HandleErrors(AppErrorCode::kDetection, err_msg, image_handle);
        return FAILED;
    }
    clock_gettime(CLOCK_REALTIME, &time4);
    // send result
    SendResult(image_handle);
    return SUCCESS;
}
