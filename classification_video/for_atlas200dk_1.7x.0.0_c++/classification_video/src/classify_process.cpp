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
#include "model_process.h"
#include "acl/acl.h"
#include "image_net_classes.h"
#include "utils.h"

using namespace std;

namespace {
    const uint32_t kTopNConfidenceLevels = 5;
    const uint32_t kScorePercent = 100;
}

ClassifyProcess::ClassifyProcess(const char* modelPath, 
                                 uint32_t modelWidth, uint32_t modelHeight)
:deviceId_(0), inputBuf_(nullptr), modelWidth_(modelWidth),
modelHeight_(modelHeight), channel_(nullptr), isInited_(false){
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
    //获取当前应用程序运行在host还是device
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
    //申请模型输入内存空间.因为本应用推理实现使用的是单线程,所以该内存可以复用
    aclrtMalloc(&inputBuf_, (size_t)(inputDataSize_),
                ACL_MEM_MALLOC_HUGE_FIRST);
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

Result ClassifyProcess::OpenPresenterChannel() {
    OpenChannelParam param;
    param.host_ip = "192.168.1.223";  //IP address of Presenter Server
    param.port = 7008;  //port of present service
    param.channel_name = "classification-video";
    param.content_type = ContentType::kVideo;  //content type is Video
    INFO_LOG("OpenChannel start");
    PresenterErrorCode errorCode = OpenChannel(channel_, param);
    INFO_LOG("OpenChannel param");
    if (errorCode != PresenterErrorCode::kNone) {
        ERROR_LOG("OpenChannel failed %d", static_cast<int>(errorCode));
        return FAILED;
    }

    return SUCCESS;
}

Result ClassifyProcess::Init() {
    //如果已经初始化,则直接返回
    if (isInited_) {
        INFO_LOG("Classify instance is initied already!");
        return SUCCESS;
    }
    //初始化ACL资源
    Result ret = InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("Init acl resource failed");
        return FAILED;
    }
    //初始化模型管理实例
    ret = InitModel(modelPath_);
    if (ret != SUCCESS) {
        ERROR_LOG("Init model failed");
        return FAILED;
    }
    //连接presenter server
    ret = OpenPresenterChannel();
    if (ret != SUCCESS) {
        ERROR_LOG("Open presenter channel failed");
        return FAILED;
    }

    isInited_ = true;
    return SUCCESS;
}

Result ClassifyProcess::Preprocess(cv::Mat& frame) {
    //resize
    cv::Mat reiszeMat;
    cv::resize(frame, reiszeMat, cv::Size(modelWidth_, modelHeight_));
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
    //执行推理
    Result ret = model_.Execute();
    if (ret != SUCCESS) {
        ERROR_LOG("Execute model inference failed");
        return FAILED;
    }
    //获取推理输出
    inferenceOutput = model_.GetModelOutputData();

    return SUCCESS;
}

Result ClassifyProcess::Postprocess(cv::Mat& frame,
                                    aclmdlDataset* modelOutput){
    //从模型推理输出中获取推理数据
    uint32_t dataSize = 0;
    void* data = GetInferenceOutputItem(dataSize, modelOutput);
    if (data == nullptr) return FAILED;
    //解析每种类别的置信度
    float* outData = reinterpret_cast<float*>(data);
    map<float, uint32_t, greater<float> > resultMap;
    for (unsigned int j = 0; j < dataSize / sizeof(float); ++j) {
        resultMap[*outData] = j;
        outData++;
    }

    //在前五的置信度中选取最高置信度的类别
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
    //将最高置信度数据构造为presenter agent要求的结构
    std::vector<DetectionResult> detectionResults;
    ConstructClassifyResult(detectionResults, maxScoreCls, maxScore);

    if (runMode_ == ACL_HOST) {
        delete[]((uint8_t*)data);
        data = nullptr;
    }
    //将推理结果和图像发给presenter server显示
    SendImage(detectionResults, frame);

    return SUCCESS;
}

void* ClassifyProcess::GetInferenceOutputItem(uint32_t& itemDataSize,
                                              aclmdlDataset* inferenceOutput) {
    //resnet50只有一个输出,是推理输出dataset的第一个单元
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
        //如果是(AI1),需要将数据从device拷贝到本地(host)
        data = Utils::CopyDataDeviceToLocal(dataBufferDev, bufferSize);
        if (data == nullptr) {
            ERROR_LOG("Copy inference output to host failed");
            return nullptr;
        }
    } else {
        //如果是atlas200dk,可以直接读取数据
        data = dataBufferDev;
    }

    itemDataSize = bufferSize;
    return data;
}

void ClassifyProcess::EncodeImage(vector<uint8_t>& encodeImg,
                                  cv::Mat& origImg) {
    vector<int> param = vector<int>(2);
    param[0] = CV_IMWRITE_JPEG_QUALITY;
    param[1] = 95;//default(95) 0-100

    cv::imencode(".jpg", origImg, encodeImg, param);
}

Result ClassifyProcess::SendImage(vector<DetectionResult>& detectionResults,
                                  cv::Mat& origImg) {
    vector<uint8_t> encodeImg;
    EncodeImage(encodeImg, origImg);

    ImageFrame imageParam;
    imageParam.format = ImageFormat::kJpeg;
    imageParam.width = origImg.cols;
    imageParam.height = origImg.rows;
    imageParam.size = encodeImg.size();
    imageParam.data = reinterpret_cast<uint8_t*>(encodeImg.data());
    imageParam.detection_results = detectionResults;

    PresenterErrorCode errorCode = PresentImage(channel_, imageParam);
    if (errorCode != PresenterErrorCode::kNone) {
        ERROR_LOG("PresentImage failed %d", static_cast<int>(errorCode));
        return FAILED;
    }

    return SUCCESS;
}


void ClassifyProcess::ConstructClassifyResult(vector<DetectionResult>& result, 
                                              int classIdx, float score) {
    DetectionResult dr;

    dr.lt.x=0;
    dr.lt.y=0;
    dr.rb.x=0;
    dr.rb.y=0;

    if (classIdx < 0 || classIdx >= IMAGE_NET_CLASSES_NUM) {
        dr.result_text = "none";
    } else {
        dr.result_text = kStrImageNetClasses[classIdx];
        dr.result_text.append(": ");

        int32_t scorePercent = score * kScorePercent;
        dr.result_text.append(to_string(scorePercent));

        dr.result_text.append("%");
    }
    printf("detection text %s\n", dr.result_text.c_str());
    result.push_back(dr);
}

void ClassifyProcess::DestroyResource()
{   
    aclrtFree(inputBuf_);
    inputBuf_ = nullptr;
    
    delete channel_;

    aclError ret;
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
