/**
* @file sample_process.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include <iostream>
#include "acl/acl.h"
#include <dirent.h>
#include <string>
#include <stdio.h>
#include <sys/stat.h>
#include "sample_process.h"
using namespace std;

namespace {
    const static std::vector<std::string> yolov3Label = { "person", "bicycle", "car", "motorbike",
    "aeroplane","bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter",
    "bench", "bird", "cat", "dog", "horse",
    "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag","tie",
    "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana",
    "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair",
    "sofa", "potted plant", "bed", "dining table", "toilet",
    "TV monitor", "laptop", "mouse", "remote", "keyboard",
    "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase","scissors",
    "teddy bear", "hair drier", "toothbrush" };

    const uint32_t kBBoxDataBufId = 0;
    const uint32_t kBoxNumDataBufId = 1;

    enum BBoxIndex { TOPLEFTX = 0, TOPLEFTY, BOTTOMRIGHTX, BOTTOMRIGHTY, SCORE, LABEL };
    // bounding box line solid
    const uint32_t kLineSolid = 2;
    // opencv draw label params.
    const double kFountScale = 0.5;
    const cv::Scalar kFontColor(0, 0, 255);
    const uint32_t kLabelOffset = 11;
    // opencv color list for boundingbox
    const vector<cv::Scalar> kColors{
        cv::Scalar(237, 149, 100), cv::Scalar(0, 215, 255), cv::Scalar(50, 205, 50),
        cv::Scalar(139, 85, 26) };
    const char* omModelPath = "../model/yolov3.om";
    const uint32_t modelInputWidth = 416;
    const uint32_t modelInputHeight = 416;
}

SampleProcess::SampleProcess(string streamName):
cap_(nullptr),
model_(omModelPath),
streamName_(streamName) {
    imageInfoSize_ = 0;
    imageInfoBuf_ = nullptr;
}

SampleProcess::~SampleProcess() {
    DestroyResource();
}

AclLiteError SampleProcess::InitResource() {
    AclLiteError ret = aclDev_.Init();
    if (ret) {
        ACLLITE_LOG_ERROR("Init resource failed, error %d", ret);
        return ACLLITE_ERROR;
    }

    if (ACLLITE_OK != OpenVideoCapture()) {
        return ACLLITE_ERROR;
    }

    ret = dvpp_.Init();
    if (ret) {
        ACLLITE_LOG_ERROR("Dvpp init failed, error %d", ret);
        return ACLLITE_ERROR;
    }

    ret = model_.Init();
    if (ret) {
        ACLLITE_LOG_ERROR("Model init failed, error %d", ret);
        return ACLLITE_ERROR;
    }

    runMode_ = aclDev_.GetRunMode();
    const float imageInfo[4] = {(float)modelInputWidth, (float)modelInputHeight,
    (float)modelInputWidth, (float)modelInputHeight};
    imageInfoSize_ = sizeof(imageInfo);
    imageInfoBuf_ = CopyDataToDevice((void *)imageInfo, imageInfoSize_,
                                        runMode_, MEMORY_DEVICE);
    if (imageInfoBuf_ == nullptr) {
        ACLLITE_LOG_ERROR("Copy image info to device failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError SampleProcess::OpenVideoCapture() {
    if (IsRtspAddr(streamName_)) {
        cap_ = new AclLiteVideoProc(streamName_);
    } else if (IsVideoFile(streamName_)) {
        if (!IsPathExist(streamName_)) {
            ACLLITE_LOG_ERROR("The %s is inaccessible", streamName_.c_str());
            return ACLLITE_ERROR;
        }
        cap_ = new AclLiteVideoProc(streamName_);
    } else {
        ACLLITE_LOG_ERROR("Invalid param. The arg should be accessible rtsp,"
                        " video file or camera id");
        return ACLLITE_ERROR;
    }

    if(!cap_->IsOpened()) {
        delete cap_;
        ACLLITE_LOG_ERROR("Failed to open video");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError SampleProcess::Postprocess(const vector<InferenceOutput>& modelOutput, cv::Mat& srcImg, int modelWidth, int modelHeight) {
    uint32_t dataSize = 0;
    float* detectData = (float *)modelOutput[kBBoxDataBufId].data.get();
    uint32_t* boxNum = (uint32_t *)modelOutput[kBoxNumDataBufId].data.get();

    uint32_t totalBox = boxNum[0];
    float widthScale = (float)(srcImg.cols) / modelInputWidth;
    float heightScale = (float)(srcImg.rows) / modelInputHeight;

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

void SampleProcess::DrawBoundBoxToImage(vector<BBox>& detectionResults, cv::Mat& origImage) {

    for (int i = 0; i < detectionResults.size(); ++i) {
        cv::Point p1, p2;
        p1.x = detectionResults[i].rect.ltX;
        p1.y = detectionResults[i].rect.ltY;
        p2.x = detectionResults[i].rect.rbX;
        p2.y = detectionResults[i].rect.rbY;
        cv::rectangle(origImage, p1, p2, kColors[i % kColors.size()], kLineSolid);
        cv::putText(origImage, detectionResults[i].text, cv::Point(p1.x, p1.y + kLabelOffset),
        cv::FONT_HERSHEY_COMPLEX, kFountScale, kFontColor);
    }
    outputVideo_ << origImage;
}

AclLiteError SampleProcess::Process() {
    string outputVideoPath_ = "./test1.mp4";    
    uint32_t videoWidth_ = cap_->Get(FRAME_WIDTH);
    uint32_t videoHeight_ = cap_->Get(FRAME_HEIGHT);
    cout << "videoWidth_ and videoHeight_ is" << " " << videoWidth_ << " " << videoHeight_ << endl;
    outputVideo_.open(outputVideoPath_, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 25.0, cv::Size(videoWidth_,videoHeight_));

    ImageData testPic;
    bool readflag = true;
    while(readflag){
        AclLiteError ret = cap_->Read(testPic);
        if (ret != ACLLITE_OK) {
            break;
        }
        ImageData resizedImage;
        ret = dvpp_.Resize(resizedImage, testPic, modelInputWidth, modelInputHeight);
        if (ret == ACLLITE_ERROR) {
            ACLLITE_LOG_ERROR("Resize image failed");
            return ACLLITE_ERROR;
        }

        // 2.model process
        ret = model_.CreateInput(resizedImage.data.get(), resizedImage.size,
                                 imageInfoBuf_, imageInfoSize_);
        if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed, error:%d", ret);
        return ACLLITE_ERROR;
        }
        std::vector<InferenceOutput> inferenceOutput;
        ret = model_.Execute(inferenceOutput);
        if (ret != ACLLITE_OK) {
            model_.DestroyInput();
            ACLLITE_LOG_ERROR("Execute model inference failed, error: %d", ret);
            return ACLLITE_ERROR;
        }
        model_.DestroyInput();

        ImageData yuvImage;
        ret = CopyImageToLocal(yuvImage, testPic, runMode_);
        if (ret == ACLLITE_ERROR) {
            ACLLITE_LOG_ERROR("Copy image to host failed");
            return ACLLITE_ERROR;
        }
        cv::Mat yuvimg(yuvImage.height * 3 / 2, yuvImage.width, CV_8UC1, yuvImage.data.get());
        cv::Mat origImage;
        cv::cvtColor(yuvimg, origImage, CV_YUV2BGR_NV12);
        ret = Postprocess(inferenceOutput, origImage, modelInputWidth, modelInputHeight);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Postprocess failed");
            return ACLLITE_ERROR;
        }
    }
    outputVideo_.release();
    return ACLLITE_OK;
}

void SampleProcess::DestroyResource()
{
    if (cap_ != nullptr) {
        cap_->Close();
        delete cap_;
    }
    dvpp_.DestroyResource();
    model_.DestroyResource();
}
