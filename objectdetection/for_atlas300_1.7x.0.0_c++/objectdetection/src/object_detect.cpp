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
#include "object_detect.h"
#include <iostream>

#include "opencv2/imgcodecs/legacy/constants_c.h"
#include "opencv2/opencv.hpp"
#include "acl/acl.h"
#include "model_process.h"
#include "utils.h"

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

ObjectDetect::ObjectDetect(const char* modelPath, 
                           uint32_t modelWidth, 
                           uint32_t modelHeight)
:deviceId_(0), context_(nullptr), stream_(nullptr), modelWidth_(modelWidth),
 modelHeight_(modelHeight), isInited_(false){
    imageInfoSize_ = 0;
    imageInfoBuf_ = nullptr;
    modelPath_ = modelPath;
}

ObjectDetect::~ObjectDetect() {
    DestroyResource();
}

Result ObjectDetect::InitResource() {
    // ACL init
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl init failed");
        return FAILED;
    }
    INFO_LOG("acl init success");

    // open device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl open device %d failed", deviceId_);
        return FAILED;
    }
    INFO_LOG("open device %d success", deviceId_);

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

Result ObjectDetect::CreateImageInfoBuffer()
{
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

    return SUCCESS;
}

Result ObjectDetect::Init() {
    if (isInited_) {
        INFO_LOG("Classify instance is initied already!");
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

    ret = dvpp_.InitResource(stream_);
    if (ret != SUCCESS) {
        ERROR_LOG("Init dvpp failed");
        return FAILED;
    }

    ret = CreateImageInfoBuffer();
    if (ret != SUCCESS) {
        ERROR_LOG("Create image info buf failed");
        return FAILED;
    }

    isInited_ = true;
    return SUCCESS;
}

Result ObjectDetect::Preprocess(ImageData& resizedImage, ImageData& srcImage) {
    ImageData imageDevice;
    Utils::CopyImageDataToDevice(imageDevice, srcImage, runMode_);

    ImageData yuvImage;
    Result ret = dvpp_.CvtJpegToYuv420sp(yuvImage, imageDevice);
    if (ret == FAILED) {
        ERROR_LOG("Convert jpeg to yuv failed");
        return FAILED;
    }

    //resize
    ret = dvpp_.Resize(resizedImage, yuvImage, modelWidth_, modelHeight_);
    if (ret == FAILED) {
        ERROR_LOG("Resize image failed");
        return FAILED;
    }
    
    return SUCCESS;
}

Result ObjectDetect::Inference(aclmdlDataset*& inferenceOutput,
                               ImageData& resizedImage) {
    Result ret = model_.CreateInput(resizedImage.data.get(),
                                    resizedImage.size,
                                    imageInfoBuf_, imageInfoSize_);
    if (ret != SUCCESS) {
        ERROR_LOG("Create mode input dataset failed");
        return FAILED;
    }

    ret = model_.Execute();
    if (ret != SUCCESS) {
        ERROR_LOG("Execute model inference failed");
        return FAILED;
    }

    inferenceOutput = model_.GetModelOutputData();

    return SUCCESS;
}

Result ObjectDetect::Postprocess(ImageData& image, aclmdlDataset* modelOutput,
                                 const string& origImagePath) {
    uint32_t dataSize = 0;
    float* detectData = (float *)GetInferenceOutputItem(dataSize, modelOutput,
                                                        kBBoxDataBufId);

    uint32_t* boxNum = (uint32_t *)GetInferenceOutputItem(dataSize, modelOutput,
                                                    kBoxNumDataBufId);
    if (boxNum == nullptr) return FAILED;

    uint32_t totalBox = boxNum[0];
    vector<BBox> detectResults;
    float widthScale = (float)(image.width) / modelWidth_;
    float heightScale = (float)(image.height) / modelHeight_;
    for (uint32_t i = 0; i < totalBox; i++) {
        BBox boundBox;

        uint32_t score = uint32_t(detectData[totalBox * SCORE + i] * 100);
        //if (score < 90) continue;

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

    DrowBoundBoxToImage(detectResults, origImagePath);
    if (runMode_ == ACL_HOST) {
        delete[]((uint8_t *)detectData);
        delete[]((uint8_t*)boxNum);
    }

    return SUCCESS;
}

void* ObjectDetect::GetInferenceOutputItem(uint32_t& itemDataSize,
                                           aclmdlDataset* inferenceOutput,
                                           uint32_t idx) {
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

void ObjectDetect::DrowBoundBoxToImage(vector<BBox>& detectionResults,
                                       const string& origImagePath) {
    cv::Mat image = cv::imread(origImagePath, CV_LOAD_IMAGE_UNCHANGED);
    for (int i = 0; i < detectionResults.size(); ++i) {
        cv::Point p1, p2;
        p1.x = detectionResults[i].rect.ltX;
        p1.y = detectionResults[i].rect.ltY;
        p2.x = detectionResults[i].rect.rbX;
        p2.y = detectionResults[i].rect.rbY;
        cv::rectangle(image, p1, p2, kColors[i % kColors.size()], kLineSolid);
        cv::putText(image, detectionResults[i].text, cv::Point(p1.x, p1.y + kLabelOffset),
                    cv::FONT_HERSHEY_COMPLEX, kFountScale, kFontColor);
    }

    int pos = origImagePath.find_last_of("/");
    string filename(origImagePath.substr(pos + 1));
    stringstream sstream;
    sstream.str("");
    sstream << "./output/out_" << filename;
    cv::imwrite(sstream.str(), image);
}

void ObjectDetect::DestroyResource()
{
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
    aclrtFree(imageInfoBuf_);
}
