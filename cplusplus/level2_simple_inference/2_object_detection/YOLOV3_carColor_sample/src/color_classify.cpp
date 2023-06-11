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
#include <cmath>
#include <algorithm>
#include "color_classify.h"
#include "acl/acl.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"
#define INVALID_IMAGE_NET_CLASS_ID (-1)
using namespace std;

namespace {
    const int32_t g_batch = 10;
    const int g_invalidSize = -1;
    const uint32_t g_lineSolid = 2;
    const uint32_t g_labelOffset = 11;
    const uint32_t g_eachResultTensorNum = 9;
    const double g_fountScale = 0.5;
    const cv::Scalar g_fontColor(0, 0, 255);
    const vector<cv::Scalar> g_colors {
    cv::Scalar(237, 149, 100), cv::Scalar(0, 215, 255), cv::Scalar(50, 205, 50),
    cv::Scalar(139, 85, 26) };
    const string g_carColorClass[9] = { "black", "blue", "brown", "green", "pink", "red",
        "silver", "white", "yellow"};

    uint32_t g_modelWidth = 224;
    uint32_t g_modelHeight = 224;
    const char* g_modelPath = "../model/color_dvpp_10batch.om";
}

ColorClassify::ColorClassify()
    : g_model_(g_modelPath),
      g_batchSize_(g_batch),
      g_isInited_(false),
      g_isReleased_(false)
{
}

ColorClassify::~ColorClassify()
{
    DestroyResource();
}

AclLiteError ColorClassify::InitModelInput()
{
    aclError aclRet = aclrtGetRunMode(&g_runMode_);
    g_inputSize_ = YUV420SP_SIZE(g_modelWidth, g_modelHeight) * g_batchSize_;
    void* buf = nullptr;
    aclRet = aclrtMalloc(&buf, g_inputSize_, ACL_MEM_MALLOC_HUGE_FIRST);
    if ((buf == nullptr) || (aclRet != ACL_ERROR_NONE)) {
        ACLLITE_LOG_ERROR("Malloc inference input buffer failed, "
                          "error %d", aclRet);
        return ACLLITE_ERROR;
    }
    g_inputBuf_ = (uint8_t *)buf;

    return ACLLITE_OK;
}

AclLiteError ColorClassify::Init()
{
    if (g_isInited_) {
        ACLLITE_LOG_INFO("Object detection is initied already");
        return ACLLITE_OK;
    }

    AclLiteError aclRet = g_dvpp_.Init();
    if (aclRet) {
        ACLLITE_LOG_ERROR("Dvpp init failed, error %d", aclRet);
        return ACLLITE_ERROR;
    }

    aclRet = g_model_.Init();
    if (aclRet) {
        ACLLITE_LOG_ERROR("Model init failed, error %d", aclRet);
        return ACLLITE_ERROR;
    }

    aclRet = InitModelInput();
    if (aclRet) {
        ACLLITE_LOG_ERROR("Model Input init failed, error %d", aclRet);
        return ACLLITE_ERROR;
    }

    g_isInited_ = true;
    return ACLLITE_OK;
}

AclLiteError ColorClassify::Crop(vector<CarInfo> &carInfo, ImageData &orgImg)
{
    static int cnt = 0;
    AclLiteError ret = ACLLITE_OK;
    for (int i = 0; i < carInfo.size(); i++) {
        ret = g_dvpp_.Crop(carInfo[i].cropedImgs, orgImg,
                           carInfo[i].rectangle.lt.x, carInfo[i].rectangle.lt.y,
                           carInfo[i].rectangle.rb.x, carInfo[i].rectangle.rb.y);                                    
        if (ret) {
            ACLLITE_LOG_ERROR("Crop image failed, error: %d, image width %d, "
                              "height %d, size %d, crop area (%d, %d) (%d, %d)",
                              ret, carInfo[i].cropedImgs.width, carInfo[i].cropedImgs.height,
                              carInfo[i].cropedImgs.size, carInfo[i].rectangle.lt.x,
                              carInfo[i].rectangle.lt.y, carInfo[i].rectangle.rb.x,
                              carInfo[i].rectangle.rb.y);
            return ACLLITE_ERROR;
        }
    }
    
    return ret;
}

AclLiteError ColorClassify::Resize(vector<CarInfo> &carInfo)
{
    AclLiteError ret = ACLLITE_OK;
    for (size_t i = 0; i < carInfo.size(); i++) {
        AclLiteError ret = g_dvpp_.Resize(carInfo[i].resizedImgs, carInfo[i].cropedImgs,
                                          g_modelWidth, g_modelHeight);
        if (ret) {
            ACLLITE_LOG_ERROR("ColorClassify Resize image failed");
            return ACLLITE_ERROR;
        }
    }

    return ret;
}

int ColorClassify::CopyOneBatchImages(uint8_t* buffer, uint32_t bufferSize,
                                      vector<CarInfo> &carInfo, int batchIdx)
{
    uint32_t j = 0;
    int dataLen = 0;
    int totalSize = 0;

    for (uint32_t i = batchIdx * g_batchSize_;
         i < carInfo.size() && j < g_batchSize_ && bufferSize > totalSize;
         i++, j++) {
        dataLen = CopyImageData(buffer + totalSize,
                                bufferSize - totalSize, carInfo[i].resizedImgs);
        if (dataLen == g_invalidSize) {
            return g_invalidSize;
        }
        totalSize += dataLen;
    }
    
    if (j < g_batchSize_) {
        for (uint32_t k = 0;
             k < g_batchSize_ - j && bufferSize > totalSize;
             k++) {
            dataLen = CopyImageData(buffer + totalSize,
                                    bufferSize - totalSize,
                                    carInfo[carInfo.size() - 1].resizedImgs);
            if (dataLen == g_invalidSize) {
                return g_invalidSize;
            }
            totalSize += dataLen;
        }
    }

    return j;
}

int ColorClassify::CopyImageData(uint8_t *buffer, uint32_t bufferSize, ImageData& image)
{
    AclLiteError ret = ACLLITE_OK;
    uint32_t dataSize = YUV420SP_SIZE(g_modelWidth, g_modelHeight);
    ret = CopyDataToDeviceEx(buffer, bufferSize, image.data.get(), dataSize, g_runMode_);
    if (ret) {
        ACLLITE_LOG_ERROR("Copy data to device failed");
        return g_invalidSize;
    }

    return dataSize;
}

AclLiteError ColorClassify::PreProcess(ImageData& srcImage, vector<CarInfo> &carInfo, int& flag)
{
    // No car detected
    if (carInfo.size() == 0) {
        flag = 1;
        ACLLITE_LOG_INFO("No car detected");
        return ACLLITE_OK;
    }

    ImageData imageDevice;
    AclLiteError ret = CopyImageToDevice(imageDevice, srcImage, g_runMode_, MEMORY_DVPP);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Copy image to device failed");
        return ACLLITE_ERROR;
    }
    ret = Crop(carInfo, imageDevice);
    if (ret) {
        ACLLITE_LOG_ERROR("Crop all the data failed, all the data failed");
        return ACLLITE_ERROR;
    }
    ret = Resize(carInfo);
    if (ret) {
        ACLLITE_LOG_ERROR("Resize all the data failed, all the data failed");
        return ACLLITE_ERROR;
    }
    return ACLLITE_OK;
}

AclLiteError ColorClassify::Inference(vector<CarInfo> &carInfo,
                                      std::vector<InferenceOutput>& inferenceOutput)
{
    int carInfoSize = carInfo.size();
    int batchNum = max(carInfoSize, g_batchSize_) / g_batchSize_;

    for (int i = 0; i < batchNum; i++) {
        // Copy one batch preprocessed image data to device
        int carNum = CopyOneBatchImages(g_inputBuf_, g_inputSize_,
                                        carInfo, i);
        if (carNum < 0) {
            ACLLITE_LOG_ERROR("Copy the %dth batch images failed", i);
            break;
        }
        // Inference one batch data
        AclLiteError ret = g_model_.Execute(inferenceOutput,
                                            g_inputBuf_, g_inputSize_);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Execute model inference failed\n");
            break;
        }
    }

    return ACLLITE_OK;
}

AclLiteError ColorClassify::PostProcess(std::vector<InferenceOutput>& inferenceOutput,
                                        std::vector<CarInfo>& carInfo, const string& origImagePath)
{
    void* data = (void *)inferenceOutput[0].data.get();
    if (data == nullptr) {
        return ACLLITE_ERROR;
    }
    float* outData = NULL;
    outData = reinterpret_cast<float*>(data);

    for (int i = 0; i < carInfo.size(); i++) {
        int maxConfidentIndex = i * g_eachResultTensorNum;
        for (int j = 0; j < g_eachResultTensorNum; j++) {
            int index = i * g_eachResultTensorNum + j;
            if (outData[index] > outData[maxConfidentIndex]) {
                maxConfidentIndex = index;
            }
        }
        int colorIndex = maxConfidentIndex - i * g_eachResultTensorNum;
        carInfo[i].carColor_result = g_carColorClass[colorIndex];
    }
    DrawResult(carInfo, origImagePath);
    return ACLLITE_OK;
}

void ColorClassify::DrawResult(vector<CarInfo>& carInfo,
                               const string& origImagePath)
{
    cv::Mat origImage = cv::imread(origImagePath, CV_LOAD_IMAGE_UNCHANGED);
    for (int i = 0; i < carInfo.size(); ++i) {
        cv::rectangle(origImage, carInfo[i].rectangle.lt, carInfo[i].rectangle.rb,
                      g_colors[i % g_colors.size()], g_lineSolid);
        cv::putText(origImage, carInfo[i].text,
                    cv::Point(carInfo[i].rectangle.lt.x - g_labelOffset,
                              carInfo[i].rectangle.lt.y - g_labelOffset),
                    cv::FONT_HERSHEY_COMPLEX, g_fountScale, g_fontColor);
        cv::putText(origImage, carInfo[i].carColor_result,
                    cv::Point(carInfo[i].rectangle.lt.x,
                              carInfo[i].rectangle.lt.y + g_labelOffset),
                    cv::FONT_HERSHEY_COMPLEX, g_fountScale, g_fontColor);
        ACLLITE_LOG_INFO("%d %d %d %d  %s", carInfo[i].rectangle.lt.x, carInfo[i].rectangle.lt.y,
                         carInfo[i].rectangle.rb.x, carInfo[i].rectangle.rb.y,
                         carInfo[i].text.c_str());
    }
    int pos = origImagePath.find_last_of("/");
    string fileName(origImagePath.substr(pos + 1));
    stringstream sstream;
    sstream.str("");
    sstream << "./out_" << fileName;
    cv::imwrite(sstream.str(), origImage);
}

void ColorClassify::DestroyResource()
{
    if (!g_isReleased_) {
        g_dvpp_.DestroyResource();
        g_model_.DestroyResource();

        if (g_inputBuf_) {
            aclrtFree(g_inputBuf_);
            g_inputBuf_ = nullptr;
        }
        g_isReleased_ = true;
    }
}