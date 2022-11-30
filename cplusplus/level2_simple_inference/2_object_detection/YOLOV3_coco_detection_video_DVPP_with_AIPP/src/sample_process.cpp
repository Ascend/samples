/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
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
*/

#include <iostream>
#include <dirent.h>
#include <string>
#include <stdio.h>
#include <sys/stat.h>
#include "acl/acl.h"
#include "sample_process.h"
using namespace std;

namespace {
    const static std::vector<std::string> yolov3Label = { "person", "bicycle", "car", "motorbike",
        "aeroplane", "bus", "train", "truck", "boat",
        "traffic light", "fire hydrant", "stop sign", "parking meter",
        "bench", "bird", "cat", "dog", "horse",
        "sheep", "cow", "elephant", "bear", "zebra",
        "giraffe", "backpack", "umbrella", "handbag", "tie",
        "suitcase", "frisbee", "skis", "snowboard", "sports ball",
        "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
        "tennis racket", "bottle", "wine glass", "cup",
        "fork", "knife", "spoon", "bowl", "banana",
        "apple", "sandwich", "orange", "broccoli", "carrot",
        "hot dog", "pizza", "donut", "cake", "chair",
        "sofa", "potted plant", "bed", "dining table", "toilet",
        "TV monitor", "laptop", "mouse", "remote", "keyboard",
        "cell phone", "microwave", "oven", "toaster", "sink",
        "refrigerator", "book", "clock", "vase", "scissors",
        "teddy bear", "hair drier", "toothbrush" };

    const uint32_t g_bBoxDataBufId = 0;
    const uint32_t g_boxNumDataBufId = 1;

    enum BBoxIndex { TOPLEFTX = 0, TOPLEFTY, BOTTOMRIGHTX, BOTTOMRIGHTY, SCORE, LABEL };
    // bounding box line solid
    const uint32_t g_lineSolid = 2;
    // opencv draw label params.
    const double g_fountScale = 0.5;
    const cv::Scalar g_fontColor(0, 0, 255);
    const uint32_t g_labelOffset = 11;
    // opencv color list for boundingbox
    const vector<cv::Scalar> g_colors {
        cv::Scalar(237, 149, 100), cv::Scalar(0, 215, 255), cv::Scalar(50, 205, 50),
        cv::Scalar(139, 85, 26) };
    const char* g_omModelPath = "../model/yolov3.om";
    const uint32_t g_modelInputWidth = 416;
    const uint32_t g_modelInputHeight = 416;
}

SampleProcess::SampleProcess(string streamName)
    : g_cap_(nullptr),
      g_model_(g_omModelPath),
      g_streamName_(streamName)
{
    g_imageInfoSize_ = 0;
    g_imageInfoBuf_ = nullptr;
}

SampleProcess::~SampleProcess()
{
    DestroyResource();
}

AclLiteError SampleProcess::InitResource()
{
    AclLiteError ret = g_aclDev_.Init();
    if (ret) {
        ACLLITE_LOG_ERROR("Init resource failed, error %d", ret);
        return ACLLITE_ERROR;
    }

    if (ACLLITE_OK != OpenVideoCapture()) {
        return ACLLITE_ERROR;
    }

    ret = g_dvpp_.Init();
    if (ret) {
        ACLLITE_LOG_ERROR("Dvpp init failed, error %d", ret);
        return ACLLITE_ERROR;
    }

    ret = g_model_.Init();
    if (ret) {
        ACLLITE_LOG_ERROR("Model init failed, error %d", ret);
        return ACLLITE_ERROR;
    }

    g_runMode_ = g_aclDev_.GetRunMode();
    const float imageInfo[4] = {(float)g_modelInputWidth, (float)g_modelInputHeight,
                                (float)g_modelInputWidth, (float)g_modelInputHeight};
    g_imageInfoSize_ = sizeof(imageInfo);
    g_imageInfoBuf_ = CopyDataToDevice((void *)imageInfo, g_imageInfoSize_,
                                       g_runMode_, MEMORY_DEVICE);
    if (g_imageInfoBuf_ == nullptr) {
        ACLLITE_LOG_ERROR("Copy image info to device failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError SampleProcess::OpenVideoCapture()
{
    if (IsRtspAddr(g_streamName_)) {
        g_cap_ = new AclLiteVideoProc(g_streamName_);
    } else if (IsVideoFile(g_streamName_)) {
        if (!IsPathExist(g_streamName_)) {
            ACLLITE_LOG_ERROR("The %s is inaccessible", g_streamName_.c_str());
            return ACLLITE_ERROR;
        }
        g_cap_ = new AclLiteVideoProc(g_streamName_);
    } else {
        ACLLITE_LOG_ERROR("Invalid param. The arg should be accessible rtsp,"
                        " video file or camera id");
        return ACLLITE_ERROR;
    }

    if (!g_cap_->IsOpened()) {
        delete g_cap_;
        ACLLITE_LOG_ERROR("Failed to open video");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError SampleProcess::Postprocess(const vector<InferenceOutput>& modelOutput, cv::Mat& srcImg,
                                        int modelWidth, int modelHeight)
{
    uint32_t dataSize = 0;
    float* detectData = (float *)modelOutput[g_bBoxDataBufId].data.get();
    uint32_t* boxNum = (uint32_t *)modelOutput[g_boxNumDataBufId].data.get();

    uint32_t totalBox = boxNum[0];
    float widthScale = (float)(srcImg.cols) / g_modelInputWidth;
    float heightScale = (float)(srcImg.rows) / g_modelInputHeight;

    vector<BBox> detectResults;
    for (uint32_t i = 0; i < totalBox; i++) {
        BBox boundBox;
        uint32_t score = uint32_t(detectData[totalBox * SCORE + i] * 100);
        boundBox.rect.ltX = detectData[totalBox * TOPLEFTX + i] * widthScale;
        boundBox.rect.ltY = detectData[totalBox * TOPLEFTY + i] * heightScale;
        boundBox.rect.rbX = detectData[totalBox * BOTTOMRIGHTX + i] * widthScale;
        boundBox.rect.rbY = detectData[totalBox * BOTTOMRIGHTY + i] * heightScale;
        uint32_t objIndex = (uint32_t)detectData[totalBox * LABEL + i];
        boundBox.text = yolov3Label[objIndex] + std::to_string(score) + "\%";
        printf("%d %d %d %d %s\n", boundBox.rect.ltX, boundBox.rect.ltY,
               boundBox.rect.rbX, boundBox.rect.rbY, boundBox.text.c_str());
        detectResults.emplace_back(boundBox);
    }

    DrawBoundBoxToImage(detectResults, srcImg);
    return ACLLITE_OK;
}

void SampleProcess::DrawBoundBoxToImage(vector<BBox>& detectionResults, cv::Mat& origImage)
{
    for (int i = 0; i < detectionResults.size(); ++i) {
        cv::Point p1, p2;
        p1.x = detectionResults[i].rect.ltX;
        p1.y = detectionResults[i].rect.ltY;
        p2.x = detectionResults[i].rect.rbX;
        p2.y = detectionResults[i].rect.rbY;
        cv::rectangle(origImage, p1, p2, g_colors[i % g_colors.size()], g_lineSolid);
        cv::putText(origImage, detectionResults[i].text, cv::Point(p1.x, p1.y + g_labelOffset),
                    cv::FONT_HERSHEY_COMPLEX, g_fountScale, g_fontColor);
    }
    g_outputVideo_ << origImage;
}

AclLiteError SampleProcess::Process()
{
    string outputVideoPath_ = "./output/test1.mp4";
    uint32_t videoWidth_ = g_cap_->Get(FRAME_WIDTH);
    uint32_t videoHeight_ = g_cap_->Get(FRAME_HEIGHT);
    float fps = 25.0;
    cout << "videoWidth_ and videoHeight_ is" << " " << videoWidth_ << " " << videoHeight_ << endl;
    g_outputVideo_.open(outputVideoPath_, cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                        fps, cv::Size(videoWidth_,videoHeight_));

    ImageData testPic;
    bool readflag = true;
    while (readflag) {
        AclLiteError ret = g_cap_->Read(testPic);
        if (ret != ACLLITE_OK) {
            break;
        }
        ImageData resizedImage;
        ret = g_dvpp_.Resize(resizedImage, testPic, g_modelInputWidth, g_modelInputHeight);
        if (ret == ACLLITE_ERROR) {
            ACLLITE_LOG_ERROR("Resize image failed");
            return ACLLITE_ERROR;
        }

        // 2.model process
        ret = g_model_.CreateInput(resizedImage.data.get(), resizedImage.size,
                                   g_imageInfoBuf_, g_imageInfoSize_);
        if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed, error:%d", ret);
        return ACLLITE_ERROR;
        }
        std::vector<InferenceOutput> inferenceOutput;
        ret = g_model_.Execute(inferenceOutput);
        if (ret != ACLLITE_OK) {
            g_model_.DestroyInput();
            ACLLITE_LOG_ERROR("Execute model inference failed, error: %d", ret);
            return ACLLITE_ERROR;
        }
        g_model_.DestroyInput();

        ImageData yuvImage;
        ret = CopyImageToLocal(yuvImage, testPic, g_runMode_);
        if (ret == ACLLITE_ERROR) {
            ACLLITE_LOG_ERROR("Copy image to host failed");
            return ACLLITE_ERROR;
        }
        cv::Mat yuvimg(yuvImage.height * 3 / 2, yuvImage.width, CV_8UC1, yuvImage.data.get());
        cv::Mat origImage;
        cv::cvtColor(yuvimg, origImage, CV_YUV2BGR_NV12);
        ret = Postprocess(inferenceOutput, origImage, g_modelInputWidth, g_modelInputHeight);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Postprocess failed");
            return ACLLITE_ERROR;
        }
    }
    g_outputVideo_.release();
    return ACLLITE_OK;
}

void SampleProcess::DestroyResource()
{
    if (g_cap_ != nullptr) {
        g_cap_->Close();
        delete g_cap_;
    }
    g_dvpp_.DestroyResource();
    g_model_.DestroyResource();
}
