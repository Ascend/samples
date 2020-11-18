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

#include <iostream>
#include <mutex>
#include <unistd.h>
#include "presenter/agent/presenter_channel.h"
#include "presenter/agent/presenter_types.h"

#include "utils.h"
#include "acl/acl.h"
#include "model_process.h"
#include "dvpp_process.h"

#include "face_detection.h"
#include "face_feature_mask.h"
#include "face_recognition.h"
#include "face_post_process.h"
#include "face_register.h"
#include "mind_camera.h"


using namespace std;
using namespace ascend::presenter;

struct ModelInfoParams {
    const char* modelPath1;
    uint32_t modelWidth1;
    uint32_t modelHeight1;
    const char* modelPath2;
    uint32_t modelWidth2;
    uint32_t modelHeight2;
    const char* modelPath3;
    uint32_t modelWidth3;
    uint32_t modelHeight3;
};

class ResourceLoad {
public:

    static ResourceLoad& GetInstance() {
        static ResourceLoad instance;
        return instance;
    }

    ~ResourceLoad();

    Result Init(const ModelInfoParams& param);

    void* GetInferenceOutputItem(uint32_t& itemDataSize,
            aclmdlDataset* inferenceOutput, uint32_t idx);

    ModelProcess& GetModel(int model);

    DvppProcess& GetDvpp();

    Result SendNextModelProcess(const string objStr, std::shared_ptr<FaceRecognitionInfo> &image_handle);

    void DestroyResource();

    static FaceDetection faceDetection;
    static FaceFeatureMaskProcess faceFeatureMask;
    static FaceRecognition faceRecognition;
    static FacePostProcess facePostProcess;

private:
    void InitModelInfo(const ModelInfoParams& param);
    Result InitResource();
    //Result InitNormlizedData();
    Result OpenPresenterChannel();
    Result InitModel(const char* omModelPath1, const char* omModelPath2, const char* omModelPath3);
    Result InitComponent();



private:
    int32_t deviceId_;
    aclrtContext context_;
    aclrtStream stream_;
    uint32_t imageInfoSize_;
    void* imageInfoBuf_;

    const char* modelPath1_;
    uint32_t modelWidth1_;
    uint32_t modelHeight1_;
    uint32_t inputDataSize1_;

    const char* modelPath2_;
    uint32_t modelWidth2_;
    uint32_t modelHeight2_;
    uint32_t inputDataSize2_;

    const char* modelPath3_;
    uint32_t modelWidth3_;
    uint32_t modelHeight3_;
    uint32_t inputDataSize3_;

    ModelProcess model1_;
    ModelProcess model2_;
    ModelProcess model3_;
    DvppProcess dvpp_;

    aclrtRunMode runMode_;

    bool isInited_;

    /*cv::Mat train_mean_;
    cv::Mat train_std_;*/
};

