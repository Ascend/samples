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
#include <stdio.h>
#include <dirent.h>
#include <string>
#include <sys/stat.h>

using namespace std;

ObjectDetect::ObjectDetect(const char* modelPath, uint32_t modelWidth,
uint32_t modelHeight)
:deviceId_(0), context_(nullptr), stream_(nullptr), modelWidth_(modelWidth),
modelHeight_(modelHeight), isInited_(false), isDeviceSet_(false), model_(modelPath) {
    imageInfoSize_ = 0;
    imageInfoBuf_ = nullptr;
    modelPath_ = modelPath;
    imgOrignWidth_ = 0;
    imgOrignHeight_ = 0;
    
}

ObjectDetect::~ObjectDetect() {
    destroy_resource();
}

AclLiteError ObjectDetect::init_resource() {
    // ACL init
    const char *aclConfigPath = "../src/acl.json";
    AclLiteError ret = aclInit(aclConfigPath);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Acl init failed");
        return ACLLITE_ERROR;
    }
    ACLLITE_LOG_INFO("Acl init success");

    // open device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("Acl open device %d failed", deviceId_);
        return ACLLITE_ERROR;
    }
    ACLLITE_LOG_INFO("Open device %d success", deviceId_);

    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acl get run mode failed");
        return ACLLITE_ERROR;
    }

    ret = dvpp_.Init();
    if (ret) {
        ACLLITE_LOG_ERROR("Dvpp init failed, error %d", ret);
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError ObjectDetect::create_input(size_t inputDataSize) {
    
    aclrtMalloc(&imageInfoBuf_, (size_t)(inputDataSize), ACL_MEM_MALLOC_HUGE_FIRST);
    if (imageInfoBuf_ == nullptr) {
        ACLLITE_LOG_ERROR("Acl malloc image buffer failed.");
        return ACLLITE_ERROR;
    }

    AclLiteError ret = model_.CreateInput(imageInfoBuf_, inputDataSize);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError ObjectDetect::init() {
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

    imageInfoSize_ = model_.GetModelInputSize(0);

    ret = create_input(imageInfoSize_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create model input failed");
        return ACLLITE_ERROR;
    }

    isInited_ = true;
    return ACLLITE_OK;
}

AclLiteError ObjectDetect::preprocess_cv(const string& imageFile) {
    // read image using OPENCV
    cv::Mat mat = cv::imread(imageFile, CV_LOAD_IMAGE_COLOR);
    imgOrignWidth_ = mat.cols;
    imgOrignHeight_ = mat.rows;
    //resize
    cv::Mat resizeMat;
    cv::resize(mat, resizeMat, cv::Size(modelWidth_, modelHeight_));

    // deal image
    resizeMat.convertTo(resizeMat, CV_32FC3);
    resizeMat = resizeMat / 127.5 -1;
    cv::cvtColor(resizeMat, resizeMat, CV_BGR2RGB);

    if (mat.empty()) {
        return ACLLITE_ERROR;
    }

    if (runMode_ == ACL_HOST) {
        aclError ret = aclrtMemcpy(imageInfoBuf_, imageInfoSize_,
                                   resizeMat.ptr<uint8_t>(), imageInfoSize_,
                                   ACL_MEMCPY_HOST_TO_DEVICE);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("Copy resized image data to device failed.");
            return ACLLITE_ERROR;
        }
    } else {
        memcpy(imageInfoBuf_, resizeMat.ptr<uint8_t>(), imageInfoSize_);
    }

    return ACLLITE_OK;
}

AclLiteError ObjectDetect::preprocess(ImageData& resizedImage, ImageData& srcImage) {
    imgOrignWidth_ = srcImage.width;
    imgOrignHeight_ = srcImage.height;
    ImageData imageDevice;
    AclLiteError ret = CopyImageToDevice(imageDevice, srcImage, runMode_, MEMORY_DVPP);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Copy image to device failed");
        return ACLLITE_ERROR;
    }

    ImageData yuvImage;
    ret = dvpp_.JpegD(yuvImage, imageDevice);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Convert jpeg to yuv failed");
        return ACLLITE_ERROR;
    }

    ret = dvpp_.Resize(resizedImage, yuvImage, modelWidth_, modelHeight_);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Resize image failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError ObjectDetect::inference(vector<InferenceOutput>& inferenceOutput, ImageData& resizedImage) {
    AclLiteError ret = model_.CreateInput(resizedImage.data.get(), resizedImage.size);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed");
        return ACLLITE_ERROR;
    }

    ret = model_.Execute(inferenceOutput);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Execute model inference failed");
        return ACLLITE_ERROR;
    }

    return ACLLITE_OK;
}

AclLiteError ObjectDetect::postprocess(vector<InferenceOutput>& inferenceOutput, const string& imageFile, const ImageData& resizedImage) {
    void* data = inferenceOutput[0].data.get();
    if(data == nullptr){
        ACLLITE_LOG_ERROR("inferoutput is null\n");
        return ACLLITE_ERROR;
    }
    uint32_t dataSize = inferenceOutput[0].size;
    uint32_t size = static_cast<uint32_t>(dataSize) / sizeof(float);

    cv::Mat mat_result(resizedImage.height, resizedImage.width, CV_32FC3, const_cast<float*>((float*)data));

    mat_result = (mat_result + 1) * 127.5;
    cv::Mat resultImage;
    cv::resize(mat_result, resultImage, cv::Size(imgOrignWidth_, imgOrignHeight_));
    cv::cvtColor(resultImage, resultImage, CV_RGB2BGR);

    cv::Mat outputImage;
    resultImage.convertTo(outputImage, CV_8U, 255.0/255);
    save_image(imageFile, resultImage);

    return ACLLITE_OK;
}

void ObjectDetect::save_image(const string& origImageFile, cv::Mat& image) {
    int pos = origImageFile.find_last_of("/");

    string filename(origImageFile.substr(pos + 1));
    stringstream sstream;
    sstream.str("");
    sstream << "./output/out_" << filename;

    string outputPath = sstream.str();    
    cv::imwrite(outputPath, image);
}

void ObjectDetect::destroy_resource()
{
    dvpp_.DestroyResource();
    model_.DestroyInput();
    model_.DestroyResource();

    aclrtFree(imageInfoBuf_);
    imageInfoBuf_ = nullptr;
    AclLiteError ret;
    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("destroy stream failed");
        }
        stream_ = nullptr;
    }
    ACLLITE_LOG_INFO("end to destroy stream");

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("destroy context failed");
        }
        context_ = nullptr;
    }
    ACLLITE_LOG_INFO("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("reset device failed");
    }
    ACLLITE_LOG_INFO("end to reset device is %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("finalize acl failed");
    }
    ACLLITE_LOG_INFO("end to finalize acl");
}
