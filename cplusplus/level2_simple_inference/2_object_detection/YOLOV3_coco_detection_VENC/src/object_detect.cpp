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
#include "object_detect.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs/legacy/constants_c.h"
#include "acl/acl.h"
#include "model_process.h"
#include "utils.h"
#include <pthread.h>

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
    const uint32_t kLineSolid = 2;
    const string kOutputFilePrefix = "out_";
    const double kFountScale = 0.5;
    const cv::Scalar kFontColor(0, 0, 255);
    const uint32_t kLabelOffset = 11;
    const string kFileSperator = "/";
    const vector<cv::Scalar> kColors{
        cv::Scalar(237, 149, 100), cv::Scalar(0, 215, 255), cv::Scalar(50, 205, 50),
        cv::Scalar(139, 85, 26) };

}

ObjectDetect::ObjectDetect(const char* modelPath, uint32_t modelWidth,
uint32_t modelHeight)
:deviceId_(0), context_(nullptr), stream_(nullptr), modelWidth_(modelWidth),
modelHeight_(modelHeight), isInited_(false), imageDataBuf_(nullptr), imageInfoBuf_(nullptr){
    imageInfoSize_ = 0;
    modelPath_ = modelPath;
    imageDataSize_ = RGBU8_IMAGE_SIZE(modelWidth, modelHeight);
}

ObjectDetect::~ObjectDetect() {
    //DestroyResource();
}

Result ObjectDetect::InitResource() {
    // ACL init
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("Acl init failed");
        return FAILED;
    }
    INFO_LOG("Acl init success");

    // open device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("Acl open device %d failed", deviceId_);
        return FAILED;
    }
    INFO_LOG("Open device %d success", deviceId_);

    // create context (set current)
    ret = aclrtCreateContext(&context_, deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl create context failed");
        return FAILED;
    }
    INFO_LOG("create context success");

    // create stream
    ret = aclrtCreateStream(&stream_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl create stream failed");
        return FAILED;
    }
    INFO_LOG("create stream success");

    //Gets whether the current application is running on host or Device
    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl get run mode failed");
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::InitModel(const char* omModelPath) {
    Result ret = model_.LoadModelFromFileWithMem(omModelPath);
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadModelFromFileWithMem failed");
        return FAILED;
    }

    ret = model_.CreateDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateDesc failed");
        return FAILED;
    }

    ret = model_.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed");
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::CreateModelInputdDataset()
{
    //Request image data memory for input model
    aclError aclRet = aclrtMalloc(&imageDataBuf_, imageDataSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if (aclRet != ACL_ERROR_NONE) {
        ERROR_LOG("malloc device data buffer failed, aclRet is %d", aclRet);
        return FAILED;
    }
    //The second input to Yolov3 is the input image width and height parameter
    const float imageInfo[4] = {(float)modelWidth_, (float)modelHeight_,
    (float)modelWidth_, (float)modelHeight_};
    imageInfoSize_ = sizeof(imageInfo);
    if (runMode_ == ACL_HOST)
        imageInfoBuf_ = Utils::CopyDataHostToDevice((void *)imageInfo, imageInfoSize_);
    else
        imageInfoBuf_ = Utils::CopyDataDeviceToDevice((void *)imageInfo, imageInfoSize_);
    if (imageInfoBuf_ == nullptr) {
        ERROR_LOG("Copy image info to device failed");
        return FAILED;
    }
    //Use the applied memory to create the model and input dataset. After creation, 
    //only update the memory data for each frame of inference, instead of creating the input dataset every time
    Result ret = model_.CreateInput(imageDataBuf_, imageDataSize_,
    imageInfoBuf_, imageInfoSize_);
    if (ret != SUCCESS) {
        ERROR_LOG("Create mode input dataset failed");
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::Init(int imgWidth, int imgHeight) {

    if (isInited_) {
        INFO_LOG("Object detection instance is initied already!");
        return SUCCESS;
    }

    Result ret = InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("Init acl resource failed");
        return FAILED;
    }

    ret = InitModel(modelPath_);
    if (ret != SUCCESS) {
        ERROR_LOG("Init model failed");
        return FAILED;
    }

    ret = dvpp_.InitResource(stream_, imgWidth, imgHeight);
    if (ret != SUCCESS) {
        ERROR_LOG("Init dvpp failed");
        return FAILED;
    }

    ret = CreateModelInputdDataset();
    if (ret != SUCCESS) {
        ERROR_LOG("Create image info buf failed");
        return FAILED;
    }

    isInited_ = true;
    return SUCCESS;
}

Result ObjectDetect::Preprocess(cv::Mat& frame) {
    //Scale the frame image to the desired size of the model
    cv::Mat reiszeMat;
    cv::resize(frame, reiszeMat, cv::Size(modelWidth_, modelHeight_));
    if (reiszeMat.empty()) {
        ERROR_LOG("Resize image failed");
        return FAILED;
    }
    //Copy the data into the cache of the input dataset
    aclrtMemcpyKind policy = (runMode_ == ACL_HOST)?
    ACL_MEMCPY_HOST_TO_DEVICE:ACL_MEMCPY_DEVICE_TO_DEVICE;
    aclError ret = aclrtMemcpy(imageDataBuf_, imageDataSize_,
    reiszeMat.ptr<uint8_t>(), imageDataSize_, policy);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("Copy resized image data to device failed.");
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::Inference(aclmdlDataset*& inferenceOutput) {
    Result ret = model_.Execute();
    if (ret != SUCCESS) {
        ERROR_LOG("Execute model inference failed");
        return FAILED;
    }
    inferenceOutput = model_.GetModelOutputData();

    return SUCCESS;
}

Result ObjectDetect::Postprocess(cv::Mat& frame, aclmdlDataset* modelOutput){
    uint32_t dataSize = 0;
    float* detectData = (float*)GetInferenceOutputItem(dataSize, modelOutput,
    kBBoxDataBufId);
    if (detectData == nullptr) return FAILED;

    uint32_t* boxNum = (uint32_t*)GetInferenceOutputItem(dataSize, modelOutput,
    kBoxNumDataBufId);
    if (boxNum == nullptr) return FAILED;

    uint32_t totalBox = boxNum[0];
    vector<BBox> detectResults;
    float widthScale = (float)(frame.cols) / modelWidth_;
    float heightScale = (float)(frame.rows) / modelHeight_;
    for (uint32_t i = 0; i < totalBox; i++) {
        BBox boundBox;
        uint32_t score = uint32_t(detectData[totalBox * SCORE + i] * 100);
        if (score < 90) continue;

        boundBox.rect.ltX = detectData[totalBox * TOPLEFTX + i] * widthScale;
        boundBox.rect.ltY = detectData[totalBox * TOPLEFTY + i] * heightScale;
        boundBox.rect.rbX = detectData[totalBox * BOTTOMRIGHTX + i] * widthScale;
        boundBox.rect.rbY = detectData[totalBox * BOTTOMRIGHTY + i] * heightScale;
        uint32_t objIndex = (uint32_t)detectData[totalBox * LABEL + i];
        boundBox.text = yolov3Label[objIndex] + std::to_string(score) + "\%";

        detectResults.emplace_back(boundBox);
    }

    DrawBoundBoxToImage(detectResults, frame);
    if (runMode_ == ACL_HOST) {
        delete[]((uint8_t*)detectData);
        delete[]((uint8_t*)boxNum);
    }

    return SUCCESS;
}

void* ObjectDetect::GetInferenceOutputItem(uint32_t& itemDataSize,
                                            aclmdlDataset* inferenceOutput,uint32_t idx) {

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

    size_t bufferSize = aclGetDataBufferSize(dataBuffer);
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
    }
    else {
        data = dataBufferDev;
    }

    itemDataSize = bufferSize;
    return data;
}

Result ObjectDetect::DrawBoundBoxToImage(vector<BBox>& detectionResults,cv::Mat& origImg) {
    for (int i = 0; i < detectionResults.size(); ++i) {
        cv::Point p1, p2;
        p1.x = detectionResults[i].rect.ltX;
        p1.y = detectionResults[i].rect.ltY;
        p2.x = detectionResults[i].rect.rbX;
        p2.y = detectionResults[i].rect.rbY;
        cv::rectangle(origImg, p1, p2, kColors[i % kColors.size()], kLineSolid);
        cv::putText(origImg, detectionResults[i].text, cv::Point(p1.x, p1.y + kLabelOffset),
        cv::FONT_HERSHEY_COMPLEX, kFountScale, kFontColor);
    }
    int cols = origImg.cols;
    int rows = origImg.rows;
    int Yindex = 0;
    int UVindex = rows * cols;
    cv::Mat NV21(rows+rows/2, cols, CV_8UC1);
    int UVRow{ 0 };
    for (int i=0;i<rows;i++)
    {
        for (int j=0;j<cols;j++)
        {
            uchar* YPointer = NV21.ptr<uchar>(i);
            int B = origImg.at<cv::Vec3b>(i, j)[0];
            int G = origImg.at<cv::Vec3b>(i, j)[1];
            int R = origImg.at<cv::Vec3b>(i, j)[2];
            int Y = (77 * R + 150 * G + 29 * B) >> 8;
            YPointer[j] = Y;
            uchar* UVPointer = NV21.ptr<uchar>(rows+i/2);
            if (i%2==0&&(j)%2==0)
            {
                int U = ((-44 * R - 87 * G + 131 * B) >> 8) + 128;
                int V = ((131 * R - 110 * G - 21 * B) >> 8) + 128;
                UVPointer[j] = V;
                UVPointer[j+1] = U;
            }
        }
    }
    aclError ret = dvpp_.Venc(NV21);
    if (ret == FAILED) {
        ERROR_LOG("Venc encoding failed");
        return FAILED;
    }
    return SUCCESS;
}

void ObjectDetect::DestroyResource()
{
    aclrtFree(imageDataBuf_);
    aclrtFree(imageInfoBuf_);

    //The ACL resource held by the model instance must be released 
    //before the ACL exits or ABORT will be torn down
    dvpp_.DestroyResource();
    model_.DestroyResource();
    aclError ret;
    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy stream failed");
        }
        stream_ = nullptr;
    }
    INFO_LOG("end to destroy stream");
    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy context failed");
        }
        context_ = nullptr;
    }
    INFO_LOG("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("reset device failed");
    }
    INFO_LOG("end to reset device is %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("finalize acl failed");
    }
    INFO_LOG("end to finalize acl");

}
