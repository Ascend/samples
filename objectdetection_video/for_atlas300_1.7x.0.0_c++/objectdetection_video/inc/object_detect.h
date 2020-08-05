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

* File sample_process.h
* Description: handle acl resource
*/
#pragma once

#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs/legacy/constants_c.h"
#include "opencv2/imgproc/types_c.h"

#include "utils.h"
#include "acl/acl.h"
#include "model_process.h"
#include <memory>
#include "ascenddk/presenter/agent/presenter_channel.h"

using namespace std;
using namespace ascend::presenter;

/**
* ObjectDetect
*/
class ObjectDetect {
public:
    ObjectDetect(const char* modelPath, uint32_t modelWidth,
    uint32_t modelHeight);
    ~ObjectDetect();
    //推理初始化
    Result Init();
    //推理帧图片预处理
    Result Preprocess(cv::Mat& frame);
    //推理帧图片
    Result Inference(aclmdlDataset*& inferenceOutput);
    //推理输出后处理
    Result Postprocess(cv::Mat& frame, aclmdlDataset* modelOutput);
    
private:
    //初始化acl资源
    Result InitResource();
    //加载推理模型
    Result InitModel(const char* omModelPath);
    Result CreateModelInputdDataset();
    //与presenter server建立连接
    Result OpenPresenterChannel();
    //从模型推理输出aclmdlDataset中获取数据到本地
    void* GetInferenceOutputItem(uint32_t& itemDataSize,
    aclmdlDataset* inferenceOutput,
    uint32_t idx);
    //将帧图像序列化为数据流
    void EncodeImage(vector<uint8_t>& encodeImg, cv::Mat& origImg);
    Result SendImage(std::vector<DetectionResult>& detectionResults,
                     cv::Mat& frame);
    //释放申请的资源
    void DestroyResource();

private:
    int32_t deviceId_;  //设备id,默认为0
    ModelProcess model_; //推理模型实例

    const char* modelPath_; //离线模型文件路径
    uint32_t modelWidth_;   //模型要求的输入宽
    uint32_t modelHeight_;  //模型要求的输入高
    uint32_t imageDataSize_; //模型输入数据大小
    void*    imageDataBuf_;      //模型输入数据缓存
    uint32_t imageInfoSize_;
    void*    imageInfoBuf_;
    aclrtRunMode runMode_;   //运行模式,即当前应用运行在atlas200dk还是AI1

    Channel* channel_;  //连接presenter server的通道
    bool isInited_;     //初始化标记,防止推理实例多次初始化
};

