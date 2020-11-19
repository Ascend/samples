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
    const static std::vector<std::string> vggssdLabel = {
        "background","aeroplane","bicycle","bird","boat","bottle","bus","car",
        "cat","chair","cow","diningtable","dog","horse","motorbike","person",
        "pottedplant","sheep","sofa","train","tvmonitor"
    };

    enum BBoxIndex {LABEL=1,SCORE,TOPLEFTX,TOPLEFTY,BOTTOMRIGHTX,BOTTOMRIGHTY,BOXINFOSIZE=8};
#define DATE_TYPE_SIZE 4

}

ObjectDetect::ObjectDetect(const char* modelPath,
uint32_t modelWidth,
uint32_t modelHeight)
:deviceId_(0), context_(nullptr), stream_(nullptr), modelWidth_(modelWidth),
modelHeight_(modelHeight), isInited_(false){
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

    isInited_ = true;
    return SUCCESS;
}


Result ObjectDetect::Preprocess(ImageData& resizedImage, ImageData& srcImage) {
    ImageData yuvImage;
    Result ret = dvpp_.CvtJpegToYuv420sp(yuvImage, srcImage);
    if (ret == FAILED) {
        ERROR_LOG("Convert jpeg to yuv failed");
        return FAILED;
    }

    //CropAndPaste
    ret = dvpp_.CropAndPaste(resizedImage, yuvImage, modelWidth_, modelHeight_);
    if (ret == FAILED) {
        ERROR_LOG("Resize image failed");
        return FAILED;
    }

    return SUCCESS;
}

Result ObjectDetect::Inference(aclmdlDataset*& inferenceOutput,
ImageData& resizedImage) {
    Result ret = model_.CreateInput(resizedImage.data.get(), resizedImage.size);
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

Result ObjectDetect::Postprocess(aclmdlDataset* modelOutput, const string& path)
{
    size_t outDatasetNum = aclmdlGetDatasetNumBuffers(modelOutput);
    if (outDatasetNum != 2) {
        ERROR_LOG("outDatasetNum=%zu must be 2",outDatasetNum);
        return FAILED;
    }

    for (size_t i = 0; i < outDatasetNum; i++) {
        aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(modelOutput, i);
        if (dataBuffer == nullptr) {
            ERROR_LOG("get model output aclmdlGetDatasetBuffer failed");
            return FAILED;
        }

        void* data = aclGetDataBufferAddr(dataBuffer);
        if (data == nullptr) {
            ERROR_LOG("aclGetDataBufferAddr from dataBuffer failed.");
            return FAILED;
        }
    }


    aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(modelOutput, 0);
    if (dataBuffer == nullptr) {
        ERROR_LOG("get model output aclmdlGetDatasetBuffer failed");
        return FAILED;
    }
    void* data = aclGetDataBufferAddr(dataBuffer);
    if (data == nullptr) {
        ERROR_LOG("aclGetDataBufferAddr from dataBuffer failed.");
        return FAILED;
    }

    void * ptr = nullptr;
    aclDataType dataType = aclmdlGetOutputDataType(model_.GetmodelDesc(),0);
    if (dataType == ACL_FLOAT){
        ptr = new float(0);
    }
    else if(dataType == ACL_INT32 ) {
        ptr = new uint32_t(0);
    }
    else {
        return FAILED;
    }

    //    uint32_t BBOX_MAX;
    if (runMode_ == ACL_HOST) {
        aclError ret = aclrtMemcpy(ptr, DATE_TYPE_SIZE, data, DATE_TYPE_SIZE, ACL_MEMCPY_DEVICE_TO_HOST);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("box num aclrtMemcpy failed!");
            return FAILED;
        }
    }
    else {
        aclError ret = aclrtMemcpy(ptr, DATE_TYPE_SIZE, data, DATE_TYPE_SIZE, ACL_MEMCPY_DEVICE_TO_DEVICE);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("box num aclrtMemcpy failed!");
            return FAILED;
        }
    }
    uint32_t boxNum;
    if (dataType == ACL_FLOAT){
        boxNum = *(float*)ptr;
    }
    else if(dataType == ACL_INT32 ) {
        boxNum = *(uint32_t*)ptr;
    }
    else {
        return FAILED;
    }



    dataBuffer = aclmdlGetDatasetBuffer(modelOutput, 1);
    if (dataBuffer == nullptr) {
        ERROR_LOG("get model output aclmdlGetDatasetBuffer failed");
        return FAILED;
    }
    uint32_t dataBufferSize = aclGetDataBufferSize(dataBuffer);
    data = aclGetDataBufferAddr(dataBuffer);
    if (data == nullptr) {
        ERROR_LOG("aclGetDataBufferAddr from dataBuffer failed.");
        return FAILED;
    }

    float outInfo[dataBufferSize/sizeof(float)];

    if (runMode_ == ACL_HOST) {
        aclError ret = aclrtMemcpy(outInfo, sizeof(outInfo), data, sizeof(outInfo), ACL_MEMCPY_DEVICE_TO_HOST);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("box outInfo aclrtMemcpy failed!");
            return FAILED;
        }
    }
    else {
        aclError ret = aclrtMemcpy(outInfo, sizeof(outInfo), data, sizeof(outInfo), ACL_MEMCPY_DEVICE_TO_DEVICE);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("box outInfo aclrtMemcpy failed!");
            return FAILED;
        }
    }

    cv::Rect rect;
    int font_face = 0;
    double font_scale = 1;
    int thickness = 2;
    cv::Mat resultImage = cv::imread(path, CV_LOAD_IMAGE_COLOR);

    for(uint32_t b=0;b<boxNum;b++) {
        uint32_t score=uint32_t(outInfo[SCORE+BOXINFOSIZE*b]*100);
        if(score<85) continue;

        //TODO:
        rect.x=outInfo[TOPLEFTX+BOXINFOSIZE*b]*resultImage.cols;
        rect.y=outInfo[TOPLEFTY+BOXINFOSIZE*b]*resultImage.rows;
        rect.width=outInfo[BOTTOMRIGHTX+BOXINFOSIZE*b]*resultImage.cols-rect.x;
        rect.height=outInfo[BOTTOMRIGHTY+BOXINFOSIZE*b]*resultImage.rows-rect.y;

        cout << "+++++++++++++++++++++++" << endl;
        cout << "score = " << score << endl;
        cout << rect.x << " " << rect.y << " " << rect.width << " " << rect.height << endl;

        uint32_t objIndex = (uint32_t)outInfo[LABEL+BOXINFOSIZE*b];
        string text = vggssdLabel[objIndex]+":"+std::to_string(score)+"\%";
        cv::Point origin;
        origin.x = rect.x;
        origin.y = rect.y;
        cv::putText(resultImage, text, origin, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 4, 0);
        cv::rectangle(resultImage, rect, cv::Scalar(0, 255, 255),1, 8,0);
    }

    // generate result image
    int pos = path.find_last_of(kFileSperator);
    string file_name(path.substr(pos + 1));
    stringstream sstream;
    sstream.str("");
    sstream << "./output" << kFileSperator
    << kOutputFilePrefix << file_name;


    string outputPath = sstream.str();
    cv::imwrite(outputPath, resultImage);

    if (dataType == ACL_FLOAT){
        delete (float*)ptr;
    }
    else if(dataType == ACL_INT32 ) {
        delete (uint32_t*)ptr;
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


void ObjectDetect::DestroyResource()
{
    model_.Unload();
    model_.DestroyDesc();
    model_.DestroyInput();
    model_.DestroyOutput();
    dvpp_.DestroyResource();

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