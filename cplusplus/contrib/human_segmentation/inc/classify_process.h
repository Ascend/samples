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
#include "acl/acl.h"
#include "atlasutil/atlas_utils.h"
#include "atlasutil/atlas_model.h"
#include <memory>
#include <opencv2/opencv.hpp>
#include "ascenddk/presenter/agent/presenter_channel.h"

/**
* ClassifyProcess
*/
class ClassifyProcess {
public:
    ClassifyProcess(const std::string& modelPath, uint32_t modelWidth, uint32_t modelHeight);
    ~ClassifyProcess();
    //推理初始化
    AtlasError Init();
    //推理帧图片预处理
    AtlasError Preprocess(cv::Mat& frame);
    //推理帧图片
    AtlasError Inference(std::vector<InferenceOutput>& inferOutputs);
    //推理输出后处理
    AtlasError Postprocess(cv::Mat& frame, std::vector<InferenceOutput>& modelOutput);
    
private:
    //加载推理模型
    AtlasError InitModel();
    //与presenter server建立连接
    AtlasError OpenPresenterChannel();

    //使用解析后的推理数据,构造发送给presenter server的推理结果数据结构
    void ConstructClassifyResult(std::vector<ascend::presenter::DetectionResult>& result,
                                 int classIdx, float score);
    //将帧图像序列化为数据流
    void EncodeImage(std::vector<uint8_t>& encodeImg, cv::Mat& origImg);
    AtlasError SendImage(std::vector<ascend::presenter::DetectionResult>& detectionResults,
                     cv::Mat& frame);
    //释放申请的资源
    void DestroyResource();

private:
    int32_t deviceId_;  //设备id,默认为0
    AtlasModel model_; //推理模型实例

    const char* modelPath_; //离线模型文件路径
    uint32_t modelWidth_;   //模型要求的输入宽
    uint32_t modelHeight_;  //模型要求的输入高
    uint32_t inputDataSize_; //模型输入数据大小
    void*    inputBuf_;      //模型输入数据缓存
    aclrtRunMode runMode_;   //运行模式,即当前应用运行在atlas200dk还是AI1

    ascend::presenter::Channel* channel_;  //连接presenter server的通道
    bool isInited_;     //初始化标记,防止推理实例多次初始化
    bool isReleased_;
};

