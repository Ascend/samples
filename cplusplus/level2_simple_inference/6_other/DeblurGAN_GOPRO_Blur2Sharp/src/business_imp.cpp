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
#include "acl/acl.h"
#include "business_imp.h"
#include "acllite/AclLiteModel.h"

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
}

BusinessImp::~BusinessImp() {
    destroy_resource();
}

AclLiteError BusinessImp::init_resource() {
    // ACL init
    string aclConfigPath = workPath_ + "/../src/acl.json";
    AclLiteError ret = aclInit(aclConfigPath.c_str());
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Acl init failed");
        return ACLLITE_ERROR;
    }
    ACLLITE_LOG_INFO("Acl init success");

    // open device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Acl open device %d failed", deviceId_);
        return ACLLITE_ERROR;
    }
    ACLLITE_LOG_INFO("Open device %d success", deviceId_);

    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("acl get run mode failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError BusinessImp::create_input() {
    aclrtMalloc(&inputBuf_, (size_t)(inputDataSize_), ACL_MEM_MALLOC_HUGE_FIRST);
    if (inputBuf_ == nullptr) {
        ACLLITE_LOG_ERROR("Acl malloc image buffer failed.");
        return ACLLITE_ERROR;
    }

    AclLiteError ret = model_.CreateInput(inputBuf_, inputDataSize_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError BusinessImp::init() {
    if (isInited_) {
        ACLLITE_LOG_INFO("Classify instance is initied already!");
        return ACLLITE_OK;
    }

    AclLiteError ret = init_resource();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Init acl resource failed");
        return ACLLITE_ERROR;
    }

    ret = model_.Init(modelPath_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Init model failed");
        return ACLLITE_ERROR;
    }

    inputDataSize_ = model_.GetModelInputSize(0);

    ret = create_input();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create model input failed");
        return ACLLITE_ERROR;
    }

    isInited_ = true;
    return ACLLITE_OK;
}

AclLiteError BusinessImp::preprocess(const string& imageFile) {
    // read image using OPENCV
    cv::Mat mat = cv::imread(imageFile, CV_LOAD_IMAGE_COLOR);
    if (mat.empty()) {
        ACLLITE_LOG_ERROR("read image failed");
        return ACLLITE_ERROR;
    }
    //resize
    cv::Mat resizedMat;
    cv::resize(mat, resizedMat, cv::Size(modelWidth_, modelHeight_));

    // deal image
    resizedMat.convertTo(resizedMat, CV_32FC3);
        
    ACLLITE_LOG_INFO("resizedMat size = %d", (int)resizedMat.elemSize());

    if (runMode_ == ACL_HOST) {
        //EP mode: need to copy the image data to the device side   
        AclLiteError ret = aclrtMemcpy(inputBuf_, inputDataSize_,
                                   resizedMat.ptr<float>(), inputDataSize_,
                                   ACL_MEMCPY_HOST_TO_DEVICE);

        ACLLITE_LOG_INFO("inputDataSize_ = %u", inputDataSize_);
        ACLLITE_LOG_INFO("inputBuf size = %d", (int)sizeof(inputBuf_));

        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Copy resized image data to device failed.");
            return ACLLITE_ERROR;
        }
    } else {
        //RC mode: no need to copy to device. But resizedMat is local Variable, need to copy the data to inputBuf_.
        memcpy(inputBuf_, resizedMat.ptr<float>(), inputDataSize_);
    }

    return ACLLITE_OK;
}

AclLiteError BusinessImp::inference(std::vector<InferenceOutput>& inferOutputs) {
    AclLiteError ret = model_.Execute(inferOutputs);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Execute model inference failed");
        return ACLLITE_ERROR;
    }
}

AclLiteError BusinessImp::postprocess(const string& imageFile, vector<InferenceOutput>& modelOutput)
{
    uint32_t dataSize = 0;
    void* data = modelOutput[0].data.get();
    if (data == nullptr){
        return ACLLITE_ERROR;
    }

    dataSize = modelOutput[0].size;
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
     
    save_image(imageFile, mat_result);

    return ACLLITE_OK;
}

void BusinessImp::save_image(const string& origImageFile, cv::Mat& image) {
    int pos = origImageFile.find_last_of("/");

    string filename(origImageFile.substr(pos + 1));
    stringstream sstream;
    sstream.str("");
    sstream << "./output/out_" << filename;

    string outputPath = sstream.str();    
    cv::imwrite(outputPath, image);
}

void BusinessImp::destroy_resource()
{
    model_.DestroyInput();
    model_.DestroyResource();
    aclrtFree(inputBuf_);
    inputBuf_ = nullptr;

    AclLiteError ret;
    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("destroy stream failed");
        }
        stream_ = nullptr;
    }
    ACLLITE_LOG_INFO("end to destroy stream");

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("destroy context failed");
        }
        context_ = nullptr;
    }
    ACLLITE_LOG_INFO("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("reset device failed");
    }
    ACLLITE_LOG_INFO("end to reset device is %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("finalize acl failed");
    }
    ACLLITE_LOG_INFO("end to finalize acl");
}
