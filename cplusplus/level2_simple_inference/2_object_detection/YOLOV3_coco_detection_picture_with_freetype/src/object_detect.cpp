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
#include "object_detect.h"
#include "acl/acl.h"
#include "model_process.h"
#include "utils.h"
#include "drawing.h"

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

    const uint32_t kBBoxDataBufId = 0;
    const uint32_t kBoxNumDataBufId = 1;
    enum BBoxIndex { TOPLEFTX = 0, TOPLEFTY, BOTTOMRIGHTX, BOTTOMRIGHTY, SCORE, LABEL };
    const uint32_t kLabelOffset = 11;
}

ObjectDetect::ObjectDetect(const char* modelPath, uint32_t modelWidth,
                           uint32_t modelHeight)
    : g_deviceId_(0), g_context_(nullptr), g_stream_(nullptr), g_modelWidth_(modelWidth),
      g_modelHeight_(modelHeight), g_isInited_(false), g_isDeviceSet_(false) {
      g_imageInfoSize_ = 0;
      g_imageInfoBuf_ = nullptr;
      g_modelPath_ = modelPath;
}

ObjectDetect::~ObjectDetect()
{
    DestroyResource();
}

Result ObjectDetect::InitResource()
{
    // ACL init
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl init failed");
        return FAILED;
    }
    INFO_LOG("acl init success");

    // open device
    ret = aclrtSetDevice(g_deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl open device %d failed", g_deviceId_);
        return FAILED;
    }
    g_isDeviceSet_ = true;
    INFO_LOG("open device %d success", g_deviceId_);

    // create context (set current)
    ret = aclrtCreateContext(&g_context_, g_deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl create context failed");
        return FAILED;
    }
    INFO_LOG("create context success");

    // create stream
    ret = aclrtCreateStream(&g_stream_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl create stream failed");
        return FAILED;
    }
    INFO_LOG("create stream success");

    ret = aclrtGetRunMode(&g_runMode_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl get run mode failed");
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::InitModel(const char* omModelPath)
{
    Result ret = g_model_.LoadModelFromFileWithMem(omModelPath);
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadModelFromFileWithMem failed");
        return FAILED;
    }

    ret = g_model_.CreateDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateDesc failed");
        return FAILED;
    }

    ret = g_model_.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed");
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::CreateImageInfoBuffer()
{
    const float imageInfo[4] = {(float)g_modelWidth_, (float)g_modelHeight_,
                                (float)g_modelWidth_, (float)g_modelHeight_};
    g_imageInfoSize_ = sizeof(imageInfo);
    if (g_runMode_ == ACL_HOST)
        g_imageInfoBuf_ = Utils::CopyDataHostToDevice((void *)imageInfo, g_imageInfoSize_);
    else
        g_imageInfoBuf_ = Utils::CopyDataDeviceToDevice((void *)imageInfo, g_imageInfoSize_);
    if (g_imageInfoBuf_ == nullptr) {
        ERROR_LOG("Copy image info to device failed");
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::Init()
{
    if (g_isInited_) {
        INFO_LOG("Object detection instance is initied already!");
        return SUCCESS;
    }

    Result ret = InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("Init acl resource failed");
        return FAILED;
    }

    ret = InitModel(g_modelPath_);
    if (ret != SUCCESS) {
        ERROR_LOG("Init model failed");
        return FAILED;
    }

    ret = g_dvpp_.InitResource(g_stream_);
    if (ret != SUCCESS) {
        ERROR_LOG("Init dvpp failed");
        return FAILED;
    }

    ret = CreateImageInfoBuffer();
    if (ret != SUCCESS) {
        ERROR_LOG("Create image info buf failed");
        return FAILED;
    }

    g_isInited_ = true;
    return SUCCESS;
}

Result ObjectDetect::Preprocess(ImageData& resizedImage, ImageData& srcImage,
                                ImageData& oriImage)
{
    ImageData imageDevice, yuvImage;
    Utils::CopyImageDataToDevice(imageDevice, srcImage, g_runMode_);

    Result ret = g_dvpp_.CvtJpegToYuv420sp(yuvImage, imageDevice);
    if (ret == FAILED) {
        ERROR_LOG("Convert jpeg to yuv failed");
        return FAILED;
    }

    ret = g_dvpp_.CropAndPaste(resizedImage, yuvImage, g_modelWidth_, g_modelHeight_);
    if (ret == FAILED) {
        ERROR_LOG("Resize image failed");
        return FAILED;
    }
    Utils::CopyDeviceToLocal(oriImage, yuvImage, g_runMode_);

    return SUCCESS;
}

Result ObjectDetect::Inference(aclmdlDataset*& inferenceOutput,
                               ImageData& resizedImage)
{
    Result ret = g_model_.CreateInput(resizedImage.data.get(),
    resizedImage.size,
    g_imageInfoBuf_, g_imageInfoSize_);
    if (ret != SUCCESS) {
        ERROR_LOG("Create mode input dataset failed");
        return FAILED;
    }

    ret = g_model_.Execute();
    if (ret != SUCCESS) {
        ERROR_LOG("Execute model inference failed");
        return FAILED;
    }

    inferenceOutput = g_model_.GetModelOutputData();

    return SUCCESS;
}

Result ObjectDetect::Postprocess(ImageData& image, aclmdlDataset* modelOutput,
                                 const string& origImagePath)
{
    uint32_t dataSize = 0;
    float* detectData = (float *)GetInferenceOutputItem(dataSize, modelOutput,
    kBBoxDataBufId);

    uint32_t* boxNum = (uint32_t *)GetInferenceOutputItem(dataSize, modelOutput,
    kBoxNumDataBufId);
    if (boxNum == nullptr) return FAILED;

    uint32_t totalBox = boxNum[0];
    vector<BBox> detectResults;
    float widthScale = (float)(image.width) / g_modelWidth_;
    float heightScale = (float)(image.height) / g_modelHeight_;
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

    DrawBoundBoxToImage(image, detectResults, origImagePath);
    if (g_runMode_ == ACL_HOST) {
        delete[]((uint8_t *)detectData);
        delete[]((uint8_t*)boxNum);
    }

    return SUCCESS;
}

void ObjectDetect::DrawBoundBoxToImage(ImageData& image,
    vector<BBox>& detectionResults, const string& origImagePath)
{

    YUVColor box_color(0, 0, 0xff);
    for (int i = 0; i < detectionResults.size(); ++i) {
        Rect p1;
        p1.ltX = detectionResults[i].rect.ltX;
        p1.ltY = detectionResults[i].rect.ltY;
        p1.rbX = detectionResults[i].rect.rbX;
        p1.rbY = detectionResults[i].rect.rbY;
        DrawText(image, p1.ltX, p1.ltY + kLabelOffset, detectionResults[i].text, box_color);
        DrawRect(image, p1.ltX, p1.ltY, p1.rbX, p1.rbY, box_color, 2);
    }

    int pos = origImagePath.find_last_of("/");
    string filename(origImagePath.substr(pos + 1));
    string name = filename.substr(0, filename.rfind("."));
    stringstream sstream;
    sstream.str("");
    sstream << "./output/out_" << name << ".yuv";
    Utils::SaveBinFile(sstream.str(), image.data.get(), image.size);
}

void* ObjectDetect::GetInferenceOutputItem(uint32_t& itemDataSize,
                                           aclmdlDataset* inferenceOutput,
                                           uint32_t idx)
{
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
    if (g_runMode_ == ACL_HOST) {
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

void ObjectDetect::DestroyResource()
{
    aclrtFree(g_imageInfoBuf_);
    g_model_.DestroyResource();
    g_dvpp_.DestroyResource();
    aclError ret;
    if (g_stream_ != nullptr) {
        ret = aclrtDestroyStream(g_stream_);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("destroy stream failed");
        }
        g_stream_ = nullptr;
    }
    INFO_LOG("end to destroy stream");

    if (g_context_ != nullptr) {
        ret = aclrtDestroyContext(g_context_);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("destroy context failed");
        }
        g_context_ = nullptr;
    }
    INFO_LOG("end to destroy context");

    if (g_isDeviceSet_) {
        ret = aclrtResetDevice(g_deviceId_);
        if (ret != ACL_SUCCESS) {
            ERROR_LOG("reset device failed");
        }
        INFO_LOG("end to reset device is %d", g_deviceId_);
    }

    ret = aclFinalize();
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("finalize acl failed");
    }
    INFO_LOG("end to finalize acl");
}
