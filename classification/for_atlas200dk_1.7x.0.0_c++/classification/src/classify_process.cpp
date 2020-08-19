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
#include "classify_process.h"
#include <iostream>

#include "acl/acl.h"
#include "model_process.h"
#include "image_net_classes.h"
#include "utils.h"

using namespace std;

namespace {
    uint32_t kTopNConfidenceLevels = 5;
}

ClassifyProcess::ClassifyProcess(const char* modelPath, 
                                 uint32_t modelWidth, uint32_t modelHeight)
:deviceId_(0), context_(nullptr), stream_(nullptr), inputBuf_(nullptr), 
modelWidth_(modelWidth), modelHeight_(modelHeight), isInited_(false){
    modelPath_ = modelPath;
    inputDataSize_ = RGBU8_IMAGE_SIZE(modelWidth_, modelHeight_);
}

ClassifyProcess::~ClassifyProcess() {
    DestroyResource();
}

Result ClassifyProcess::InitResource() {
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

    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl get run mode failed");
        return FAILED;
    }

    return SUCCESS;
}

Result ClassifyProcess::InitModel(const char* omModelPath) {
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

    aclrtMalloc(&inputBuf_, (size_t)(inputDataSize_), ACL_MEM_MALLOC_HUGE_FIRST);
    if (inputBuf_ == nullptr) {
        ERROR_LOG("Acl malloc image buffer failed.");
        return FAILED;
    }

    ret = model_.CreateInput(inputBuf_, inputDataSize_);
    if (ret != SUCCESS) {
        ERROR_LOG("Create mode input dataset failed");
        return FAILED;
    }

    return SUCCESS;
}

Result ClassifyProcess::Init() {
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

    isInited_ = true;
    return SUCCESS;
}

Result ClassifyProcess::Preprocess(const string& imageFile) {
    // read image using OPENCV
    INFO_LOG("Read image %s", imageFile.c_str());
    cv::Mat origMat = cv::imread(imageFile, CV_LOAD_IMAGE_COLOR);
    if (origMat.empty()) {
        ERROR_LOG("Read image failed");
        return FAILED;
    }

    INFO_LOG("Resize image %s", imageFile.c_str());
    //resize
    cv::Mat reiszeMat;
    cv::resize(origMat, reiszeMat, cv::Size(modelWidth_, modelHeight_));
    if (reiszeMat.empty()) {
        ERROR_LOG("Resize image failed");
        return FAILED;
    }
    
    if (runMode_ == ACL_HOST) {     
        //AI1上运行时,需要将图片数据拷贝到device侧   
        aclError ret = aclrtMemcpy(inputBuf_, inputDataSize_, 
                                   reiszeMat.ptr<uint8_t>(), inputDataSize_,
                                   ACL_MEMCPY_HOST_TO_DEVICE);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("Copy resized image data to device failed.");
            return FAILED;
        }
    } else {
        //Atals200DK上运行时,数据拷贝到本地即可.
        //reiszeMat是局部变量,数据无法传出函数,需要拷贝一份
        memcpy(inputBuf_, reiszeMat.ptr<void>(), inputDataSize_);
    }

    return SUCCESS;
}

Result ClassifyProcess::Inference(aclmdlDataset*& inferenceOutput) {
    Result ret = model_.Execute();
    if (ret != SUCCESS) {
        ERROR_LOG("Execute model inference failed");
        return FAILED;
    }
    //Release model input buffer
    //model_.DestroyInput();

    inferenceOutput = model_.GetModelOutputData();

    return SUCCESS;
}

Result ClassifyProcess::Postprocess(const string& origImageFile, 
                                    aclmdlDataset* modelOutput){
    uint32_t dataSize = 0;
    void* data = GetInferenceOutputItem(dataSize, modelOutput);
    if (data == nullptr) return FAILED;

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
        INFO_LOG("top %d: index[%d] value[%lf]", cnt, it->second, it->first);

        if (it->first > maxScore) {
            maxScore = it->first;
            maxScoreCls = it->second;
        }
    }

    LabelClassToImage(maxScoreCls, origImageFile);
    if (runMode_ == ACL_HOST) {
        delete[]((uint8_t *)data);
        data = nullptr;
    }

    return SUCCESS;
}

void* ClassifyProcess::GetInferenceOutputItem(uint32_t& itemDataSize,
                                              aclmdlDataset* inferenceOutput) {
    aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(inferenceOutput, 0);
    if (dataBuffer == nullptr) {
        ERROR_LOG("Get the dataset buffer from model "
                  "inference output failed");
        return nullptr;
    }

    void* dataBufferDev = aclGetDataBufferAddr(dataBuffer);
    if (dataBufferDev == nullptr) {
        ERROR_LOG("Get the dataset buffer address "
                  "from model inference output failed");
        return nullptr;
    }

    size_t bufferSize = aclGetDataBufferSize(dataBuffer);
    if (bufferSize == 0) {
        ERROR_LOG("The dataset buffer size of "
                  "model inference output is 0");
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

void ClassifyProcess::LabelClassToImage(int classIdx, const string& origImagePath) {
    cv::Mat resultImage = cv::imread(origImagePath, CV_LOAD_IMAGE_COLOR);

    // generate colorized image
    int pos = origImagePath.find_last_of("/");
    string filename(origImagePath.substr(pos + 1));
    stringstream sstream;
    sstream.str("");
    sstream << "./output/out_"  << filename;

    string outputPath = sstream.str();
    string text;

    if (classIdx < 0 || classIdx >= IMAGE_NET_CLASSES_NUM) {
        text = "none";
    } else {
        text = kStrImageNetClasses[classIdx];
    }

    int fontFace = 0;
    double fontScale = 1;
    int thickness = 2;
    int baseline;
    cv::Point origin;
    origin.x = 10;
    origin.y = 50;
    cv::putText(resultImage, text, origin, fontFace, fontScale, cv::Scalar(0, 255, 255), thickness, 4, 0);
    cv::imwrite(outputPath, resultImage);
}

void ClassifyProcess::DestroyResource()
{   aclrtFree(inputBuf_);
    inputBuf_ = nullptr;
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
