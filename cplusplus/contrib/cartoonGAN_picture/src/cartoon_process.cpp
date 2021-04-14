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
#include "cartoon_process.h"
#include <cmath>
#include <unistd.h>
#include <iostream>
#include <typeinfo>
#include "malloc.h"
#include "acl/acl.h"
#include "model_process.h"
#include "opencv2/core/hal/interface.h"
#include "opencv2/imgproc/types_c.h"
#include "utils.h"

using namespace std;

namespace {
    uint32_t kTopNConfidenceLevels = 5;
}

CartoonProcess::CartoonProcess(const char* modelPath,
                                 uint32_t modelWidth, uint32_t modelHeight)
:deviceId_(0), context_(nullptr), stream_(nullptr), inputBuf_(nullptr),
modelWidth_(modelWidth), modelHeight_(modelHeight), isInited_(false){
    modelPath_ = modelPath;
    inputDataSize_ = RGB32_CHAN_SIZE(modelWidth_, modelHeight_);
}

CartoonProcess::~CartoonProcess() {
    DestroyResource();
}

Result CartoonProcess::InitResource() {
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
    INFO_LOG("acl get run mode success");

    return SUCCESS;
}

Result CartoonProcess::InitModel(const char* omModelPath) {
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

Result CartoonProcess::Init() {
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

Result CartoonProcess::Preprocess(const string& imageFile) {
    // read image using OPENCV
    cv::Mat mat = cv::imread(imageFile, CV_LOAD_IMAGE_COLOR);
    //resize
    cv::Mat resizeMat;
    cv::resize(mat, resizeMat, cv::Size(256, 256)); // model input 256*256
    resizeMat.convertTo(resizeMat, CV_32FC3);
    resizeMat = resizeMat / 127.5 -1;
    cv::cvtColor(resizeMat, resizeMat, CV_BGR2RGB);

    if (resizeMat.empty()) {
        return FAILED;
    }

    if (runMode_ == ACL_HOST) {
        //AI1上运行时,需要将图片数据拷贝到device侧
        aclError ret = aclrtMemcpy(inputBuf_, inputDataSize_, resizeMat.ptr<uint8_t>(), inputDataSize_,
                                   ACL_MEMCPY_HOST_TO_DEVICE);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("Copy resized image data to device failed.");
            return FAILED;
        }
    } else {
        //Atals200DK上运行时,数据拷贝到本地即可.
        //reiszeMat是局部变量,数据无法传出函数,需要拷贝一份
        memcpy(inputBuf_, resizeMat.ptr<uint8_t>(), inputDataSize_);
    }

    return SUCCESS;
}

Result CartoonProcess::Inference(aclmdlDataset*& inferenceOutput) {
    Result ret = model_.Execute();
    if (ret != SUCCESS) {
        ERROR_LOG("Execute model inference failed");
        return FAILED;
    }

    inferenceOutput = model_.GetModelOutputData();

    return SUCCESS;
}

Result CartoonProcess::Postprocess(const string& imageFile, aclmdlDataset* modelOutput)
{
    cv::Mat mat_origin = cv::imread(imageFile, CV_LOAD_IMAGE_COLOR);
    int height_origin = mat_origin.rows;
    int width_origin = mat_origin.cols;

    uint32_t dataSize = 0;
    void* data = GetInferenceOutputItem(dataSize, modelOutput);
    if (data == nullptr){
        return FAILED;
    } 
    uint32_t size = static_cast<uint32_t>(dataSize) / sizeof(float);
    cv::Mat mat_result(256, 256, CV_32FC3, const_cast<float*>((float*)data));
    cv::cvtColor(mat_result, mat_result, CV_RGB2BGR);
    mat_result = (mat_result + 1) * 127.5;
    cv::resize(mat_result, mat_result, cv::Size(width_origin, height_origin));
    SaveImage(imageFile, mat_result);

    if (runMode_ == ACL_HOST) {
        delete[]((uint8_t *)data);
        data = nullptr;
    }

    return SUCCESS;
}

void CartoonProcess::SaveImage(const string& origImageFile, cv::Mat& image) {
    int pos = origImageFile.find_last_of("/");

    string filename(origImageFile.substr(pos + 1));
    string dirname = "./output";
    if (access(dirname.c_str(), 0) == -1){
        string cmd("mkdir -p ./output");
        system(cmd.c_str());	
    }
    
    stringstream sstream;
    sstream.str("");
    sstream << "./output/out_" << filename;

    string outputPath = sstream.str();
    cv::imwrite(outputPath, image);
}

void* CartoonProcess::GetInferenceOutputItem(uint32_t& itemDataSize,
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
                  "model inference output is 0 ");
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

void CartoonProcess::DestroyResource()
{
    model_.DestroyResource();
    aclrtFree(inputBuf_);
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

