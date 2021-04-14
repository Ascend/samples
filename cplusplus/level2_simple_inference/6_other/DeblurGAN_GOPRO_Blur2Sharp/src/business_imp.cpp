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
#include "atlasutil/atlas_model.h"

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

AtlasError BusinessImp::init_resource() {
    // ACL init
    string aclConfigPath = workPath_ + "/../src/acl.json";
    AtlasError ret = aclInit(aclConfigPath.c_str());
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Acl init failed");
        return ATLAS_ERROR;
    }
    ATLAS_LOG_INFO("Acl init success");

    // open device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Acl open device %d failed", deviceId_);
        return ATLAS_ERROR;
    }
    ATLAS_LOG_INFO("Open device %d success", deviceId_);

    ret = aclrtGetRunMode(&runMode_);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("acl get run mode failed");
        return ATLAS_ERROR;
    }

    return ATLAS_OK;
}

AtlasError BusinessImp::create_input() {
    aclrtMalloc(&inputBuf_, (size_t)(inputDataSize_), ACL_MEM_MALLOC_HUGE_FIRST);
    if (inputBuf_ == nullptr) {
        ATLAS_LOG_ERROR("Acl malloc image buffer failed.");
        return ATLAS_ERROR;
    }

    AtlasError ret = model_.CreateInput(inputBuf_, inputDataSize_);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Create mode input dataset failed");
        return ATLAS_ERROR;
    }

    return ATLAS_OK;
}

AtlasError BusinessImp::init() {
    if (isInited_) {
        ATLAS_LOG_INFO("Classify instance is initied already!");
        return ATLAS_OK;
    }

    AtlasError ret = init_resource();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Init acl resource failed");
        return ATLAS_ERROR;
    }

    ret = model_.Init(modelPath_);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Init model failed");
        return ATLAS_ERROR;
    }

    inputDataSize_ = model_.GetModelInputSize(0);

    ret = create_input();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Create model input failed");
        return ATLAS_ERROR;
    }

    isInited_ = true;
    return ATLAS_OK;
}

AtlasError BusinessImp::preprocess(const string& imageFile) {
    // read image using OPENCV
    cv::Mat mat = cv::imread(imageFile, CV_LOAD_IMAGE_COLOR);
    if (mat.empty()) {
        ATLAS_LOG_ERROR("read image failed");
        return ATLAS_ERROR;
    }
    //resize
    cv::Mat resizedMat;
    cv::resize(mat, resizedMat, cv::Size(modelWidth_, modelHeight_));

    // deal image
    resizedMat.convertTo(resizedMat, CV_32FC3);
        
    ATLAS_LOG_INFO("resizedMat size = %d", (int)resizedMat.elemSize());

    if (runMode_ == ACL_HOST) {
        //EP mode: need to copy the image data to the device side   
        AtlasError ret = aclrtMemcpy(inputBuf_, inputDataSize_,
                                   resizedMat.ptr<float>(), inputDataSize_,
                                   ACL_MEMCPY_HOST_TO_DEVICE);

        ATLAS_LOG_INFO("inputDataSize_ = %u", inputDataSize_);
        ATLAS_LOG_INFO("inputBuf size = %d", (int)sizeof(inputBuf_));

        if (ret != ATLAS_OK) {
            ATLAS_LOG_ERROR("Copy resized image data to device failed.");
            return ATLAS_ERROR;
        }
    } else {
        //RC mode: no need to copy to device. But resizedMat is local Variable, need to copy the data to inputBuf_.
        memcpy(inputBuf_, resizedMat.ptr<float>(), inputDataSize_);
    }

    return ATLAS_OK;
}

AtlasError BusinessImp::inference(std::vector<InferenceOutput>& inferOutputs) {
    AtlasError ret = model_.Execute(inferOutputs);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Execute model inference failed");
        return ATLAS_ERROR;
    }
}

AtlasError BusinessImp::postprocess(const string& imageFile, vector<InferenceOutput>& modelOutput)
{
    uint32_t dataSize = 0;
    void* data = modelOutput[0].data.get();
    if (data == nullptr){
        return ATLAS_ERROR;
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

    return ATLAS_OK;
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

    AtlasError ret;
    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ATLAS_OK) {
            ATLAS_LOG_ERROR("destroy stream failed");
        }
        stream_ = nullptr;
    }
    ATLAS_LOG_INFO("end to destroy stream");

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ATLAS_OK) {
            ATLAS_LOG_ERROR("destroy context failed");
        }
        context_ = nullptr;
    }
    ATLAS_LOG_INFO("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("reset device failed");
    }
    ATLAS_LOG_INFO("end to reset device is %d", deviceId_);

    ret = aclFinalize();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("finalize acl failed");
    }
    ATLAS_LOG_INFO("end to finalize acl");
}
