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
#include "utils.h"
#include <iostream>
#include <mutex>
#include <unistd.h>

#include "acl/acl.h"
#include "model_process.h"
#include "dvpp_process.h"
#include "engine_post_dange.h"
#include "engine_post_follow.h"
#include "engine_post_road.h"

#include "engine_handle.h"
#include "common.h"
#include "common_post.h"
#include "i2c.h"
#include "wheel.h"

using namespace std;
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
    uint32_t PreviewWidth;
    uint32_t PreviewHeight;

};
//using namespace ascend::presenter;
/**
* ObjectDetect
*/

class ObjectDetect {
public:
    static int _s_init_flag;
    static int _s_init_flag_2;
    static int _s_init_flag_3;

    static int _s_work_mode;
    ObjectDetect();
    ~ObjectDetect();

    Result init(const ModelInfoParams& param);

    void init_model_info(const ModelInfoParams& param);
    Result preprocess(ImageData& resizedImage, ImageData& srcImage);
    Result inference(aclmdlDataset*& inferenceOutput, ImageData& resizedImage);
    Result postprocess(ImageData& image, aclmdlDataset* modelOutput);
    Result propreview(ImageData& previewImage, ImageData& srcImage);
    EngineHandle EngineHan;
private:
    Result init_resource();
    Result init_model1(const char* omModelPath);
    Result init_model2(const char* omModelPath);
    Result init_model3(const char* omModelPath);

    Result create_imageinfobuffer();
    void* get_inference_output_item(uint32_t& itemDataSize, aclmdlDataset* inferenceOutput,  uint32_t idx);
    void destroy_resource();
private:
    int32_t deviceId_;
    aclrtContext context_;
    aclrtStream stream_;
    uint32_t imageInfoSize_;
    void* imageInfoBuf_;

    ModelProcess model1_;
    ModelProcess model2_;
    ModelProcess model3_;

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

    uint32_t inputDataSize_;

    uint32_t previewWidth_;
    uint32_t previewHeight_;

    DvppProcess dvpp_;
    aclrtRunMode runMode_;

    EnginePostDange PostDange;
    EnginePostFollow PostFollow;
    EnginePostRoad PostRoad;
    bool isInited_;
};

