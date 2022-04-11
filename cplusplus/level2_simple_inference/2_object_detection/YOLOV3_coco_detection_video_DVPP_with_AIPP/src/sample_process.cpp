/**
* @file sample_process.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include "sample_process.h"
#include <iostream>
#include "dvpp_process.h"
#include "model_process.h"
#include "decode.h"
#include "acl/acl.h"
#include "utils.h"
#include <dirent.h>
#include <string>
#include <stdio.h>
#include <sys/stat.h>
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

    // output image prefix
    const string kOutputFilePrefix = "out_";
    // opencv draw label params.
    const double kFountScale = 0.5;
    const cv::Scalar kFontColor(0, 0, 255);
    const uint32_t kLabelOffset = 11;
    const string kFileSperator = "/";

    // opencv color list for boundingbox
    const vector<cv::Scalar> kColors{
        cv::Scalar(237, 149, 100), cv::Scalar(0, 215, 255), cv::Scalar(50, 205, 50),
        cv::Scalar(139, 85, 26) };
}

SampleProcess::SampleProcess(string streamName):
deviceId_(0), 
context_(nullptr), 
stream_(nullptr), 
streamName_(streamName) {
}

SampleProcess::~SampleProcess() {
    DestroyResource();
}

Result SampleProcess::InitResource() {
    // ACL init
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl init failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("acl init success");

    // set device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl set device %d failed, errorCode = %d", deviceId_, static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("set device %d success", deviceId_);

    // create context (set current)
    ret = aclrtCreateContext(&context_, deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl create context failed, deviceId = %d, errorCode = %d",
            deviceId_, static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("create context success");

    // create stream
    ret = aclrtCreateStream(&stream_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl create stream failed, deviceId = %d, errorCode = %d",
            deviceId_, static_cast<int32_t>(ret));
        return FAILED;
    }
    INFO_LOG("create stream success");

    // get run mode
    // runMode is ACL_HOST which represents app is running in host
    // runMode is ACL_DEVICE which represents app is running in device
    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl get run mode failed, errorCode = %d", static_cast<int32_t>(ret));
        return FAILED;
    }
    return SUCCESS;
}


Result SampleProcess::Postprocess(const aclmdlDataset* modelOutput, PicDesc &picDesc, int modelWidth, int modelHeight) {
    uint32_t dataSize = 0;
    float* detectData = (float *)GetInferenceOutputItem(dataSize, modelOutput,
    kBBoxDataBufId);

    uint32_t* boxNum = (uint32_t *)GetInferenceOutputItem(dataSize, modelOutput,
    kBoxNumDataBufId);
    if (boxNum == nullptr) return FAILED;

    uint32_t totalBox = boxNum[0];
    vector<BBox> detectResults;
    float widthScale = (float)(picDesc.width) / modelWidth;
    float heightScale = (float)(picDesc.height) / modelHeight;
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

    DrawBoundBoxToImage(detectResults, picDesc);
    if (runMode_ == ACL_HOST) {
        delete[]((uint8_t *)detectData);
        delete[]((uint8_t*)boxNum);
    }

    return SUCCESS;
}

void* SampleProcess::GetInferenceOutputItem(uint32_t& itemDataSize,
const aclmdlDataset* inferenceOutput, uint32_t idx) {
    aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(inferenceOutput, idx);
    if (dataBuffer == nullptr) {
        ERROR_LOG("Get the %dth dataset buffer from model "
        "inference output failed", idx);
        return nullptr;
    }

    void* dataBufferDev = aclGetDataBufferAddr(dataBuffer);
    if (dataBufferDev == nullptr) {
        ERROR_LOG("Get the %dth dataset buffer address "
        "from model inference output failed", idx);
        return nullptr;
    }

    size_t bufferSize = aclGetDataBufferSizeV2(dataBuffer);
    if (bufferSize == 0) {
        ERROR_LOG("The %dth dataset buffer size of "
        "model inference output is 0", idx);
        return nullptr;
    }

    void* data = nullptr;
    if (runMode_ == ACL_HOST) {
        data = Utils::CopyDataDeviceToLocal(dataBufferDev, bufferSize);
        if (data == nullptr) {
            ERROR_LOG("Copy inference output to host failed");
            return nullptr;
        }
    } else {
        data = dataBufferDev;
    }

    itemDataSize = bufferSize;
    return data;
}

void SampleProcess::DrawBoundBoxToImage(vector<BBox>& detectionResults, PicDesc &picDesc) {

    for (int i = 0; i < detectionResults.size(); ++i) {
        cv::Point p1, p2;
        p1.x = detectionResults[i].rect.ltX;
        p1.y = detectionResults[i].rect.ltY;
        p2.x = detectionResults[i].rect.rbX;
        p2.y = detectionResults[i].rect.rbY;
        cv::rectangle(picDesc.origImage, p1, p2, kColors[i % kColors.size()], kLineSolid);
        cv::putText(picDesc.origImage, detectionResults[i].text, cv::Point(p1.x, p1.y + kLabelOffset),
        cv::FONT_HERSHEY_COMPLEX, kFountScale, kFontColor);
    }
    outputVideo_ << picDesc.origImage;
}

Result SampleProcess::Process()
{
    // dvpp init
    DvppProcess dvppProcess(stream_);
    Result ret = dvppProcess.InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("init dvpp resource failed");
        return FAILED;
    }

    // model init
    ModelProcess modelProcess;
    const char* omModelPath = "../model/yolov3.om";
    ret = modelProcess.LoadModel(omModelPath);
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadModel failed");
        return FAILED;
    }
    ret = modelProcess.CreateDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateDesc failed");
        return FAILED;
    }
    ret = modelProcess.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed");
        return FAILED;
    }
    int modelInputWidth;
    int modelInputHeight;
    ret = modelProcess.GetModelInputWH(modelInputWidth, modelInputHeight);
    if (ret != SUCCESS) {
        ERROR_LOG("execute GetModelInputWH failed");
        return FAILED;
    }

    const float imageInfo[4] = {(float)modelInputWidth, (float)modelInputHeight,
    (float)modelInputWidth, (float)modelInputHeight};
    size_t imageInfoSize_ = sizeof(imageInfo);
    void *imageInfoBuf_;
    if (runMode_ == ACL_HOST)
        imageInfoBuf_ = Utils::CopyDataHostToDevice((void *)imageInfo, imageInfoSize_);
    else
        imageInfoBuf_ = Utils::CopyDataDeviceToDevice((void *)imageInfo, imageInfoSize_);
    if (imageInfoBuf_ == nullptr) {
        ERROR_LOG("Copy image info to device failed");
        return FAILED;
    }

    // ffmpeg && vdec init
    DecodeProcess decodeProcess(stream_, runMode_, streamName_, context_);
    ret = decodeProcess.InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("init ffmpeg & vdec resource failed");
        return FAILED;
    }

    string outputVideoPath_ = "./test1.mp4";
    uint32_t videoWidth_ = decodeProcess.GetFrameWidth();
    uint32_t videoHeight_ = decodeProcess.GetFrameHeight();
    cout << "videoWidth_ and videoHeight_ is" << " " << videoWidth_ << " " << videoHeight_ << endl;
    outputVideo_.open(outputVideoPath_, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 25.0, cv::Size(videoWidth_,videoHeight_));

    PicDesc testPic;
    bool readflag = true;
    while(readflag){
        ret = decodeProcess.ReadFrame(testPic);
        if (ret != SUCCESS) {
            break;
        }
        ret = dvppProcess.InitDvppOutputPara(modelInputWidth, modelInputHeight);
        if (ret != SUCCESS) {
            ERROR_LOG("init dvpp output para failed");
            return FAILED;
        }
        ret = dvppProcess.Process(testPic);
        if (ret != SUCCESS) {
            ERROR_LOG("dvpp process failed");
            return FAILED;
        }
        void *dvppOutputBuffer = nullptr;
        int dvppOutputSize;
        dvppProcess.GetDvppOutput(&dvppOutputBuffer, dvppOutputSize);

        // 2.model process
        ret = modelProcess.CreateInput(dvppOutputBuffer, dvppOutputSize, 
                                       imageInfoBuf_, imageInfoSize_);
        if (ret != SUCCESS) {
            ERROR_LOG("execute CreateInput failed");
            (void)acldvppFree(dvppOutputBuffer);
            return FAILED;
        }

        ret = modelProcess.Execute();
        if (ret != SUCCESS) {
            ERROR_LOG("execute inference failed");
            (void)acldvppFree(dvppOutputBuffer);
            return FAILED;
        }
        // release model input buffer
        (void)acldvppFree(dvppOutputBuffer);
        modelProcess.DestroyInput();

        const aclmdlDataset *modelOutput = modelProcess.GetModelOutputData();

        void *hostImage = Utils::CopyDataDeviceToLocal((void *)testPic.data.get(), YUV420SP_SIZE(testPic.width, testPic.height));
        cv::Mat yuvimg(testPic.height * 3 / 2, testPic.width, CV_8UC1, hostImage);
        cv::cvtColor(yuvimg, testPic.origImage, CV_YUV2BGR_NV12);

        ret = Postprocess(modelOutput, testPic, modelInputWidth, modelInputHeight);
        if (ret != SUCCESS) {
            ERROR_LOG("Postprocess failed");
            return FAILED;
        }
    }
    aclrtFree(imageInfoBuf_);
    outputVideo_.release();
    return SUCCESS;
}

void SampleProcess::DestroyResource()
{
    aclError ret;
    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("destroy stream failed, errorCode = %d", static_cast<int32_t>(ret));
        }
        stream_ = nullptr;
    }
    INFO_LOG("end to destroy stream");

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("destroy context failed, errorCode = %d", static_cast<int32_t>(ret));
        }
        context_ = nullptr;
    }
    INFO_LOG("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("reset device %d failed, errorCode = %d", deviceId_, static_cast<int32_t>(ret));
    }
    INFO_LOG("end to reset device %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("finalize acl failed, errorCode = %d", static_cast<int32_t>(ret));
    }
    INFO_LOG("end to finalize acl");
}
