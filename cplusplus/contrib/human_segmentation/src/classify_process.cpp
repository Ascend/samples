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
#include "image_net_classes.h"
#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgcodecs/legacy/constants_c.h"
#include "opencv2/imgproc/types_c.h"

#include "atlasutil/atlas_model.h"
#include "atlasutil/atlas_utils.h"
#include "atlasutil/acl_device.h"

using namespace std;
using namespace ascend::presenter;

using namespace std;
using namespace ascend::presenter;

namespace {
    const uint32_t kTopNConfidenceLevels = 5;
    const uint32_t kScorePercent = 100;
}

ClassifyProcess::ClassifyProcess(const std::string& modelPath, 
                                 uint32_t modelWidth, uint32_t modelHeight)
:deviceId_(0), inputBuf_(nullptr), model_(modelPath), modelWidth_(modelWidth),
modelHeight_(modelHeight), channel_(nullptr), isInited_(false), isReleased_(false){
    inputDataSize_ = RGBU8_IMAGE_SIZE(modelWidth_, modelHeight_);
}

ClassifyProcess::~ClassifyProcess() {
    DestroyResource();
}

AtlasError ClassifyProcess::InitModel() {

    AtlasError atlRet;
    atlRet = model_.Init();
    if (atlRet) {
        ATLAS_LOG_ERROR("Model init failed, error %d", atlRet);
        return ATLAS_ERROR;
    }

    aclError ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_ERROR_NONE) {
        ATLAS_LOG_ERROR("acl get run mode failed");
        return ATLAS_ERROR;
    }

    //申请模型输入内存空间.因为本应用推理实现使用的是单线程,所以该内存可以复用
    aclrtMalloc(&inputBuf_, (size_t)(inputDataSize_),
                ACL_MEM_MALLOC_HUGE_FIRST);
    if (inputBuf_ == nullptr) {
        ATLAS_LOG_ERROR("Acl malloc image buffer failed.");
        return ATLAS_ERROR;
    }

    atlRet = model_.CreateInput(inputBuf_, inputDataSize_);
    if (atlRet != ATLAS_OK) {
        ATLAS_LOG_ERROR("Create mode input dataset failed");
        return ATLAS_ERROR;
    }

    return ATLAS_OK;
}

AtlasError ClassifyProcess::OpenPresenterChannel() {
    PresenterErrorCode openChannelret = OpenChannelByConfig(channel_, "./human_segmentation.conf");
    if (openChannelret != PresenterErrorCode::kNone) {
        ATLAS_LOG_ERROR("Open channel failed, error %d\n", (int)openChannelret);
    }
    return ATLAS_OK;
}

AtlasError ClassifyProcess::Init() {
    //如果已经初始化,则直接返回
    if (isInited_) {
        ATLAS_LOG_INFO("Classify instance is initied already!");
        return ATLAS_OK;
    }

    //初始化模型管理实例
    AtlasError ret = InitModel();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Init model failed");
        return ATLAS_ERROR;
    }
    //连接presenter server
    ret = OpenPresenterChannel();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Open presenter channel failed");
        return ATLAS_ERROR;
    }

    isInited_ = true;
    return ATLAS_OK;
}

AtlasError ClassifyProcess::Preprocess(cv::Mat& frame) {
    //resize
    cv::Mat reiszeMat;
    cv::resize(frame, reiszeMat, cv::Size(modelWidth_, modelHeight_));
    if (reiszeMat.empty()) {
        ATLAS_LOG_ERROR("Resize image failed");
        return ATLAS_ERROR;
    }

    if (runMode_ == ACL_HOST) {
        //AI1上运行时,需要将图片数据拷贝到device侧   
        aclError ret = aclrtMemcpy(inputBuf_, inputDataSize_, 
                                   reiszeMat.ptr<uint8_t>(), inputDataSize_,
                                   ACL_MEMCPY_HOST_TO_DEVICE);
        if (ret != ACL_ERROR_NONE) {
            ATLAS_LOG_ERROR("Copy resized image data to device failed.");
            return ATLAS_ERROR;
        }
    } else {
        //Atals200DK上运行时,数据拷贝到本地即可.
        //reiszeMat是局部变量,数据无法传出函数,需要拷贝一份
        memcpy(inputBuf_, reiszeMat.ptr<void>(), inputDataSize_);
    }

    return ATLAS_OK;
}

AtlasError ClassifyProcess::Inference(std::vector<InferenceOutput>& inferOutputs) {
    //执行推理
    AtlasError ret = model_.Execute(inferOutputs);
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("Execute model inference failed");
        return ATLAS_ERROR;
    }

    return ATLAS_OK;
}

AtlasError ClassifyProcess::Postprocess(cv::Mat& frame,
std::vector<InferenceOutput>& modelOutput){
    float* buff = (float *)modelOutput[0].data.get();
    uint32_t databuff_size = modelOutput[0].size;
    //float* buff = (float*)aclGetDataBufferAddr(databuff);
    if (buff == nullptr) {
        ATLAS_LOG_ERROR("Get the  dataset buffer address from model inference output failed");
        return ATLAS_ERROR;
    }

    cv::Mat mask(modelHeight_,modelWidth_, CV_32FC1, buff);
    
    cv::Mat ori_frame;
    cv::resize(frame, ori_frame, cv::Size(modelWidth_, modelHeight_));
    ori_frame.convertTo(ori_frame, CV_32FC3, 1.0 / 255.0);
    vector<cv::Mat> channels;
    cv::split(ori_frame, channels);
    for (int i = 0; i < 2; i++)
    {
        channels[i] = channels[i].mul(1-mask);
    }
    cv::Mat res;
    cv::merge(channels,res);

    res.convertTo(frame, CV_8UC3, 255.0);

    //将最高置信度数据构造为presenter agent要求的结构
    std::vector<DetectionResult> detectionResults;
 
    //将推理结果和图像发给presenter server显示
    SendImage(detectionResults, frame);

    return ATLAS_OK;
}

void ClassifyProcess::EncodeImage(vector<uint8_t>& encodeImg,
                                  cv::Mat& origImg) {
    vector<int> param = vector<int>(2);
    param[0] = CV_IMWRITE_JPEG_QUALITY;
    param[1] = 95;//default(95) 0-100

    cv::imencode(".jpg", origImg, encodeImg, param);
}

AtlasError ClassifyProcess::SendImage(vector<DetectionResult>& detectionResults,
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
        ATLAS_LOG_ERROR("PresentImage failed %d", static_cast<int>(errorCode));
        return ATLAS_ERROR;
    }

    return ATLAS_OK;
}

void ClassifyProcess::ConstructClassifyResult(vector<DetectionResult>& result, 
                                              int classIdx, float score) {
    DetectionResult dr;

    dr.lt.x = 0;
    dr.lt.y = 0;
    dr.rb.x = 0;
    dr.rb.y = 0;

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
    if (!isReleased_) {
        aclrtFree(inputBuf_);
        inputBuf_ = nullptr;
        model_.DestroyResource();
        isReleased_ = true;
    }
    ATLAS_LOG_INFO("end to finalize acl");

}
