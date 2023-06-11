/**
* Copyright 2020 Huawei Technologies Co., Ltd
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at

* http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

* File sample_process.cpp
* Description: handle acl resource
*/
#include <iostream>
#include "acl/acl.h"
#include "acllite/AclLiteModel.h"
#include "colorize_process.h"

using namespace std;
using namespace ascend::presenter;

namespace {
    const uint32_t kTopNConfidenceLevels = 5;
    const uint32_t kScorePercent = 100;
}

ColorizeProcess::ColorizeProcess(const char* modelPath,
uint32_t modelWidth, uint32_t modelHeight)
:deviceId_(0), context_(nullptr), stream_(nullptr), inputBuf_(nullptr),
modelWidth_(modelWidth), modelHeight_(modelHeight), channel_(nullptr), isInited_(false){
    modelPath_ = modelPath;
}

ColorizeProcess::~ColorizeProcess() {
    DestroyResource();
}

AclLiteError ColorizeProcess::InitResource() {
    // ACL init
    const char *aclConfigPath = "../src/acl.json";
    AclLiteError ret = aclInit(aclConfigPath);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Acl init failed");
        return ACLLITE_ERROR;
    }
    ACLLITE_LOG_INFO("Acl init success");

    // open device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Acl open device %d failed", deviceId_);
        return ACLLITE_ERROR;
    }
    ACLLITE_LOG_INFO("Open device %d success", deviceId_);

    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acl get run mode failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError ColorizeProcess::Init() {
    if (isInited_) {
        ACLLITE_LOG_INFO("Classify instance is initied already!");
        return ACLLITE_OK;
    }

    AclLiteError ret = InitResource();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Init acl resource failed");
        return ACLLITE_ERROR;
    }

    ret = model_.Init(modelPath_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Init model failed");
        return ACLLITE_ERROR;
    }

    inputDataSize_ = model_.GetModelInputSize(0);

    ret = CreateInput();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create model input failed");
        return ACLLITE_ERROR;
    }
    ret = OpenPresenterChannel();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Open presenter channel failed");
        return ACLLITE_ERROR;
    }

    isInited_ = true;
    return ACLLITE_OK;
}

AclLiteError ColorizeProcess::CreateInput() {
    
    aclrtMalloc(&inputBuf_, (size_t)(inputDataSize_), ACL_MEM_MALLOC_HUGE_FIRST);
    if (inputBuf_ == nullptr) {
        ACLLITE_LOG_ERROR("Acl malloc image buffer failed.");
        return ACLLITE_ERROR;
    }

    AclLiteError ret = model_.CreateInput(inputBuf_, inputDataSize_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create model input failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError ColorizeProcess::OpenPresenterChannel() {
    PresenterErrorCode openChannelret = OpenChannelByConfig(channel_, "./colorization.conf");
    if (openChannelret != PresenterErrorCode::kNone) {
        ACLLITE_LOG_ERROR("Open channel failed, error %d\n", (int)openChannelret);
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError ColorizeProcess::Preprocess(cv::Mat& frame) {
    //resize
    cv::Mat reiszeMat;
    cv::resize(frame, reiszeMat, cv::Size(modelWidth_, modelHeight_));
    if (reiszeMat.empty()) {
        ACLLITE_LOG_ERROR("Resize image failed");
        return ACLLITE_ERROR;
    }

    // deal image
    reiszeMat.convertTo(reiszeMat, CV_32FC3);
    reiszeMat = 1.0 * reiszeMat / 255;
    cv::cvtColor(reiszeMat, reiszeMat, CV_BGR2Lab);

    // pull out L channel and subtract 50 for mean-centering
    std::vector<cv::Mat> channels;
    cv::split(reiszeMat, channels);
    cv::Mat reiszeMatL = channels[0] - 50;

    if (runMode_ == ACL_HOST) {
        aclError ret = aclrtMemcpy(inputBuf_, inputDataSize_,
        reiszeMatL.ptr<uint8_t>(), inputDataSize_,
        ACL_MEMCPY_HOST_TO_DEVICE);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("Copy resized image data to device failed.");
            return ACLLITE_ERROR;
        }
    }
    else {
        memcpy(inputBuf_, reiszeMatL.ptr<uint8_t>(), inputDataSize_);
    }

    return ACLLITE_OK;
}

AclLiteError ColorizeProcess::Inference(std::vector<InferenceOutput>& inferOutputs) {
    AclLiteError ret = model_.Execute(inferOutputs);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Execute model inference failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError ColorizeProcess::Postprocess(cv::Mat& frame, vector<InferenceOutput>& modelOutput){

    uint32_t dataSize = 0;
    void* data = modelOutput[0].data.get();
    if (data == nullptr) 
    {
        return ACLLITE_ERROR;
    }

    dataSize = modelOutput[0].size;

    uint32_t size = static_cast<uint32_t>(dataSize) / sizeof(float);
    // get a channel and b channel result data
    cv::Mat mat_a(56, 56, CV_32FC1, const_cast<float*>((float*)data));
    cv::Mat mat_b(56, 56, CV_32FC1, const_cast<float*>((float*)data + size / 2));

    frame.convertTo(frame, CV_32FC3);
    frame = 1.0 * frame / 255;
    cv::cvtColor(frame, frame, CV_BGR2Lab);
    std::vector<cv::Mat> channels;
    cv::split(frame, channels);

    // resize to match size of original image L
    int r = frame.rows;
    int c = frame.cols;
    cv::Mat mat_a_up(r, c, CV_32FC1);
    cv::Mat mat_b_up(r, c, CV_32FC1);
    cv::resize(mat_a, mat_a_up, cv::Size(c, r));
    cv::resize(mat_b, mat_b_up, cv::Size(c, r));

    // result Lab image
    cv::Mat newChannels[3] = { channels[0], mat_a_up, mat_b_up };
    cv::Mat resultImage;
    cv::merge(newChannels, 3, resultImage);

    //convert back to rgb
    cv::cvtColor(resultImage, resultImage, CV_Lab2BGR);
    resultImage = resultImage * 255;
    SendImage(resultImage);

    return ACLLITE_OK;
}

void ColorizeProcess::EncodeImage(vector<uint8_t>& encodeImg, cv::Mat& origImg) {
    vector<int> param = vector<int>(2);
    param[0] = cv::IMWRITE_JPEG_QUALITY;
    param[1] = 95;//default(95) 0-100

    cv::imencode(".jpg", origImg, encodeImg, param);
}

AclLiteError ColorizeProcess::SendImage(cv::Mat& image) {
    vector<uint8_t> encodeImg;
    EncodeImage(encodeImg, image);

    ImageFrame imageParam;
    imageParam.format = ImageFormat::kJpeg;
    imageParam.width = image.cols;
    imageParam.height = image.rows;
    imageParam.size = encodeImg.size();
    imageParam.data = reinterpret_cast<uint8_t*>(encodeImg.data());

    std::vector<DetectionResult> detectionResults;
    imageParam.detection_results = detectionResults;
    PresenterErrorCode ret = PresentImage(channel_, imageParam);
    // send to presenter failedPresentImage
    if (ret != PresenterErrorCode::kNone) {
        ACLLITE_LOG_ERROR("Send JPEG image to presenter failed, error %d\n", (int)ret);
        return ACLLITE_ERROR;
    }
    return ACLLITE_OK;
}

void ColorizeProcess::DestroyResource()
{
    model_.DestroyInput();
    model_.DestroyResource();

    aclrtFree(inputBuf_);
    inputBuf_ = nullptr;
    delete channel_;

    AclLiteError ret;
    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("destroy stream failed");
        }
        stream_ = nullptr;
    }
    ACLLITE_LOG_INFO("end to destroy stream");

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("destroy context failed");
        }
        context_ = nullptr;
    }
    ACLLITE_LOG_INFO("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("reset device failed");
    }
    ACLLITE_LOG_INFO("end to reset device is %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("finalize acl failed");
    }
    ACLLITE_LOG_INFO("end to finalize acl");
}
