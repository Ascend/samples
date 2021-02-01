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
#include "business_imp.h"
#include <iostream>

#include "acl/acl.h"
#include "model_process.h"
#include "utils.h"

using namespace std;

namespace {
    uint32_t kTopNConfidenceLevels = 5;
}

BusinessImp::BusinessImp(const char* modelPath, 
                                 uint32_t modelWidth, uint32_t modelHeight, string workPath)
:deviceId_(0), context_(nullptr), stream_(nullptr), inputBuf_(nullptr), 
modelWidth_(modelWidth), modelHeight_(modelHeight), isInited_(false){
    modelPath_ = modelPath;
    workPath_ = workPath;
    // inputDataSize_ = RGBF32_CHAN_SIZE(modelWidth_, modelHeight_);
}

BusinessImp::~BusinessImp() {
    DestroyResource();
}

Result BusinessImp::InitResource() {
    // ACL init
    string aclConfigPath = workPath_ + "/../src/acl.json";
    aclError ret = aclInit(aclConfigPath.c_str());
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

Result BusinessImp::InitModel(const char* omModelPath) {
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

    // we know that the model have only one input.
    inputDataSize_ = model_.GetModelInputSizeByIndex(0);

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

Result BusinessImp::Init() {
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

//
Result BusinessImp::Preprocess(const string& imageFile) {
    // read image using OPENCV
    cv::Mat mat = cv::imread(imageFile, CV_LOAD_IMAGE_COLOR);
    if (mat.empty()) {
        ERROR_LOG("read image failed");
        return FAILED;
    }
    //resize
    cv::Mat resizedMat;
    cv::resize(mat, resizedMat, cv::Size(modelWidth_, modelHeight_));

    // deal image
    resizedMat.convertTo(resizedMat, CV_32FC3);
        
    INFO_LOG("resizedMat size = %d", (int)resizedMat.elemSize());

    if (runMode_ == ACL_HOST) {
        //EP mode: need to copy the image data to the device side   
        aclError ret = aclrtMemcpy(inputBuf_, inputDataSize_,
                                   resizedMat.ptr<float>(), inputDataSize_,
                                   ACL_MEMCPY_HOST_TO_DEVICE);

        INFO_LOG("inputDataSize_ = %u", inputDataSize_);
        INFO_LOG("inputBuf size = %d", (int)sizeof(inputBuf_));


        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("Copy resized image data to device failed.");
            return FAILED;
        }
    } else {
        //RC mode: no need to copy to device. But resizedMat is local Variable, need to copy the data to inputBuf_.
        memcpy(inputBuf_, resizedMat.ptr<float>(), inputDataSize_);
    }

    return SUCCESS;
}

Result BusinessImp::Inference(aclmdlDataset*& inferenceOutput) {

    time_t start_t, end_t;   
    time(&start_t);
    Result ret = model_.Execute();
    time(&end_t);
    Utils::printDiffTime(end_t, start_t);
    if (ret != SUCCESS) {
        ERROR_LOG("Execute model inference failed");
        return FAILED;
    }

    inferenceOutput = model_.GetModelOutputData();

    return SUCCESS;
}

Result BusinessImp::Postprocess(const string& imageFile, aclmdlDataset* modelOutput)
{
    uint32_t dataSize = 0;
    void* data = GetInferenceOutputItem(dataSize, modelOutput);
    if (data == nullptr) return FAILED;

    // we know that the model's output data type is float.
    uint32_t size = static_cast<uint32_t>(dataSize) / sizeof(float);
    
    cv::Mat mat(modelHeight_,modelWidth_, CV_32FC3, const_cast<float*>((float*)data));

    mat = (mat + 1) / 2 * 255;

    cv::Mat resultImage;
    mat.convertTo(resultImage, CV_8UC3);


    // resize to match size of original image
    cv::Mat original_mat = cv::imread(imageFile, CV_LOAD_IMAGE_COLOR);
    int r = original_mat.rows;
    int c = original_mat.cols;

    cv::Mat mat_result(r, c, CV_8UC3);
    cv::resize(resultImage, mat_result, cv::Size(c, r));
     
    SaveImage(imageFile, mat_result);

    if (runMode_ == ACL_HOST) {
        delete[]((uint8_t *)data);
        data = nullptr;
    }

    return SUCCESS;
}

void BusinessImp::SaveImage(const string& origImageFile, cv::Mat& image) {
    int pos = origImageFile.find_last_of("/");

    string filename(origImageFile.substr(pos + 1));
    stringstream sstream;
    sstream.str("");
    sstream << "./output/out_" << filename;

    string outputPath = sstream.str();    
    cv::imwrite(outputPath, image);
}


// get output from the result of inference, which is aclmdlDataset.
// output parameter: itemDataSize 
// input parameter: inferenceOutput, the result of inference
// return: data, the result data.
void* BusinessImp::GetInferenceOutputItem(uint32_t& itemDataSize,
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

void BusinessImp::DestroyResource()
{
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
