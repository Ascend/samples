/*
* Copyright (C) Huawei Technologies Co., Ltd. 2020-2022. All rights reserved.
*/
/**
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
#include <vector>
#include "classify_process.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"


#include <dirent.h>
#include <string>
#include <sys/stat.h>
#include <stdio.h>
#include "acl/acl.h"
#include "image_net_classes.h"

#define RGBU8_IMAGE_SIZE(width, height) ((width) * (height) * 3)
using namespace std;

namespace {
    const uint32_t kTopNConfidenceLevels = 5;
    const uint32_t kOutputDataBufId = 0;
    const uint32_t kModelWidth = 224;
    const uint32_t kModelHeight = 224;
    const string kModelPath = "../model/googlenet.om";
    const uint32_t inputDataSize = RGBU8_IMAGE_SIZE(kModelWidth, kModelHeight);
}

ClassifyProcess::ClassifyProcess():
model_(kModelPath),
inputDataSize_(inputDataSize),
inputData_(nullptr),
isInited_(false), 
isReleased_(false){
}

ClassifyProcess::~ClassifyProcess() {
    DestroyResource();
}

AclLiteError ClassifyProcess::Init()
{
    if (isInited_) {
        ACLLITE_LOG_INFO("Classify process is initied already");
        return ACLLITE_OK;
    }

    AclLiteError atlRet = model_.Init();
    if (atlRet) {
        ACLLITE_LOG_ERROR("Model init failed, error %d", atlRet);
        return ACLLITE_ERROR;
    }

    isInited_ = true;
    return ACLLITE_OK;
}

AclLiteError ClassifyProcess::Process(std::vector<std::string>& fileVec, aclrtRunMode RunMode)
{
    RunMode_ = RunMode;
    for (string imageFile : fileVec) {
        AclLiteError ret = Preprocess(imageFile);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Read file %s failed, continue to read next",imageFile.c_str());                
            continue;
        }
        std::vector<InferenceOutput> inferOutputs;
        ret = Inference(inferOutputs);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Inference model inference output data failed");
            return ACLLITE_ERROR;
        }

        ret = Postprocess(imageFile, inferOutputs);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Process model inference output data failed");
            return ACLLITE_ERROR;
        }
    }
    return ACLLITE_OK;
}

AclLiteError ClassifyProcess::Preprocess(const string& imageFile)
{
    ACLLITE_LOG_INFO("Read image %s", imageFile.c_str());
    cv::Mat origMat = cv::imread(imageFile, CV_LOAD_IMAGE_COLOR);
    if (origMat.empty()) {
        ACLLITE_LOG_ERROR("Read image failed");
        return ACLLITE_ERROR;
    }

    ACLLITE_LOG_INFO("Resize image %s", imageFile.c_str());
    cv::Mat reiszeMat;
    cv::resize(origMat, reiszeMat, cv::Size(kModelWidth, kModelHeight));
    if (reiszeMat.empty()) {
        ACLLITE_LOG_ERROR("Resize image failed");
        return ACLLITE_ERROR;
    }
    
    inputData_ = CopyDataToDevice(reiszeMat.ptr<uint8_t>(), inputDataSize_, RunMode_, MEMORY_DEVICE);
    if (inputData_ == nullptr) {
        ACLLITE_LOG_ERROR("Copy data to device failed");
        return ACLLITE_ERROR;
    }
    return ACLLITE_OK;
}

AclLiteError ClassifyProcess::Inference(std::vector<InferenceOutput>& inferOutputs)
{
    AclLiteError ret = model_.CreateInput(inputData_,inputDataSize_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed\n");
        return ACLLITE_ERROR;
    }

    ret = model_.Execute(inferOutputs);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Execute model inference failed\n");
    }
    model_.DestroyInput();

    return ret;
}

AclLiteError ClassifyProcess::Postprocess(const string& origImageFile, std::vector<InferenceOutput>& inferOutputs)
{
    uint32_t dataSize = inferOutputs[kOutputDataBufId].size;
    void* data = (void *)inferOutputs[kOutputDataBufId].data.get();
    if (data == nullptr) {
        return ACLLITE_ERROR;
    }
    float* outData = NULL;
    outData = reinterpret_cast<float*>(data);
    map<float, unsigned int, greater<float> > resultMap;
    for (uint32_t j = 0; j < dataSize / sizeof(float); ++j) {
        resultMap[*outData] = j;
        outData++;
    }

    int maxScoreCls = INVALID_IMAGE_NET_CLASS_ID;
    float maxScore = 0;
    int cnt = 0;
    for (auto it = resultMap.begin(); it != resultMap.end(); ++it) {
        // print top 5
        if (++cnt > kTopNConfidenceLevels) {
            break;
        }
        ACLLITE_LOG_INFO("top %d: index[%d] value[%lf]", cnt, it->second, it->first);

        if (it->first > maxScore) {
            maxScore = it->first;
            maxScoreCls = it->second;
        }
    }
    LabelClassToImage(maxScoreCls, origImageFile);

    return ACLLITE_OK;
}

void ClassifyProcess::LabelClassToImage(int classIdx, const string& origImagePath)
{
    cv::Mat resultImage = cv::imread(origImagePath, CV_LOAD_IMAGE_COLOR);

    // generate colorized image
    int pos = origImagePath.find_last_of("/");
    string filename(origImagePath.substr(pos + 1));

    string folderPath = "./output";
    if (NULL == opendir(folderPath.c_str())) {
        mkdir(folderPath.c_str(), 0775);
    }

    stringstream sstream;
    sstream.str("");
    sstream << "./output/out_"  << filename;

    string outputPath = sstream.str();
    string text;

    if (classIdx < 0 || classIdx >= IMAGE_NET_CLASSES_NUM) {
        text = "none";
    } else {
        text = g_str_image_net_classes[classIdx];
    }

    int fontFace = 0;
    double fontScale = 1;
    int thickness = 2;
    cv::Point origin;
    origin.x = 10;
    origin.y = 50;
    cv::putText(resultImage, text, origin, fontFace, fontScale, cv::Scalar(0, 255, 255), thickness, 4, 0);
    cv::imwrite(outputPath, resultImage);
}

void ClassifyProcess::DestroyResource()
{
    if (!isReleased_) {
        model_.DestroyResource();
        isReleased_ = true;
    }

}