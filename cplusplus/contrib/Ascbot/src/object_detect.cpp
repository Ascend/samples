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
#include <cstdio>
#include <iostream>

#include "acl/acl.h"
#include "model_process.h"
#include "utils.h"
#include "object_detect.h"

using namespace std;
namespace {
    const static std::vector<std::string> ssdLabel = { "backgroud", "cup", "car", "box", "person",
 "hand","bottle", "phone", "book", "line",
 "left_round", "right_round", "edge", "light"};

}
int ObjectDetect::_s_init_flag = 0;
int ObjectDetect::_s_init_flag_3 = 0;
int ObjectDetect::_s_init_flag_2 = 0;

int ObjectDetect::_s_work_mode = ASCBOT_ROAD_MODE;

ObjectDetect::ObjectDetect()
:deviceId_(0), context_(nullptr), stream_(nullptr), isInited_(false){
    imageInfoSize_ = 0;
    imageInfoBuf_ = nullptr;
    //init_flag = 0; //flag
}
void ObjectDetect::init_model_info(const ModelInfoParams& param) {
    modelPath1_ = param.modelPath1;
    modelWidth1_ = param.modelWidth1;
    modelHeight1_ = param.modelHeight1;
    modelPath2_ = param.modelPath2;
    modelWidth2_ = param.modelWidth2;
    modelHeight2_ = param.modelHeight2;
    modelPath3_ = param.modelPath3;
    modelWidth3_ = param.modelWidth3;
    modelHeight3_ = param.modelHeight3;
    previewWidth_ = param.PreviewWidth;
    previewHeight_ = param.PreviewHeight;

}
ObjectDetect::~ObjectDetect() {
    destroy_resource();
}

Result ObjectDetect::init_resource() {

    // ACL init
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl init failed");
        return FAILED;
    }
    INFO_LOG("acl init success");

    // open device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl open device %d failed", deviceId_);
        return FAILED;
    }
    INFO_LOG("open device %d success", deviceId_);

    // create context (set current)
    ret = aclrtCreateContext(&context_, deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl create context failed");
        return FAILED;
    }
    INFO_LOG("create context success");

    // create stream
    ret = aclrtCreateStream(&stream_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl create stream failed");
        return FAILED;
    }
    INFO_LOG("create stream success");

    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("acl get run mode failed");
        return FAILED;
    }
    return SUCCESS;
}

Result ObjectDetect::init_model1(const char* omModelPath) {
    Result ret = model1_.LoadModelFromFileWithMem(omModelPath);
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadModelFromFileWithMem failed");
        return FAILED;
    }

    ret = model1_.CreateDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateDesc failed");
        return FAILED;
    }

    ret = model1_.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed");
        return FAILED;
    }
    return SUCCESS;
}
Result ObjectDetect::init_model2(const char* omModelPath) {
    Result ret = model2_.LoadModelFromFileWithMem(omModelPath);
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadModelFromFileWithMem failed");
        return FAILED;
    }

    ret = model2_.CreateDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateDesc failed");
        return FAILED;
    }

    ret = model2_.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed");
        return FAILED;
    }
    return SUCCESS;
}
Result ObjectDetect::init_model3(const char* omModelPath) {
    Result ret = model3_.LoadModelFromFileWithMem(omModelPath);
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadModelFromFileWithMem failed");
        return FAILED;
    }

    ret = model3_.CreateDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateDesc failed");
        return FAILED;
    }

    ret = model3_.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed");
        return FAILED;
    }
    return SUCCESS;
}
Result ObjectDetect::init(const ModelInfoParams& param) {
    if (isInited_) {
        INFO_LOG("Classify instance is initied already!");
        return SUCCESS;
    }

    init_model_info(param);

    Result ret = init_resource();
    if (ret != SUCCESS) {
        ERROR_LOG("Init acl resource failed");
        return FAILED;
    }

    ret = init_model1(modelPath1_);
    if (ret != SUCCESS) {
        ERROR_LOG("Init model failed");
        return FAILED;
    }

    ret = init_model2(modelPath2_);
    if (ret != SUCCESS) {
        ERROR_LOG("Init model_2 failed");
        return FAILED;
    }

    ret = init_model3(modelPath3_);
    if (ret != SUCCESS) {
        ERROR_LOG("Init model_3 failed");
        return FAILED;
    }

    ret = dvpp_.init_resource(stream_);
    if (ret != SUCCESS) {
        ERROR_LOG("Init dvpp failed");
        return FAILED;
    }

    if (PostDange.Init() != 0) {
        ERROR_LOG("error PostDange Init");
        return FAILED;
    }
    if (PostFollow.Init() != 0) {
        ERROR_LOG("error PostFollow Init");
        return FAILED;
    }
    if (PostRoad.Init() != 0) {
        ERROR_LOG("error PostRoad Init");
        return FAILED;
    }
    if (EngineHan.Init() != 0){
        ERROR_LOG("error EngineHan Init");
        return FAILED;
    }

    isInited_ = true;
    return SUCCESS;
}

Result ObjectDetect::preprocess(ImageData& resizedImage, ImageData& srcImage) {
    ImageData imageDevice;
    uint32_t modelWidth = 0;
    uint32_t modelHeight = 0;
    switch(_s_work_mode)
    {
        case ASCBOT_OFF:
        case ASCBOT_VOID_DANGER_MODE:
        case ASCBOT_REMOTE_MODE:
            modelWidth = modelWidth1_;
            modelHeight = modelHeight1_;
        break;
        case ASCBOT_OBJECT_MODE:
            modelWidth = modelWidth2_ ;
            modelHeight = modelHeight2_;
        break;
        case ASCBOT_ROAD_MODE:
            modelWidth = modelWidth3_;
            modelHeight = modelHeight3_;
        break;
    }
    Utils::CopyImageDataToDVPP(imageDevice, srcImage, runMode_);
    //resize
    Result ret = dvpp_.resize(resizedImage, imageDevice, modelWidth, modelHeight);
    if (ret == FAILED) {
        ERROR_LOG("Resize image failed\n");
        return FAILED;
    }
    return SUCCESS;
}
Result ObjectDetect::propreview(ImageData& previewImage, ImageData& srcImage)
{
    ImageData imageDevice;
    Utils::CopyImageDataToDVPP(imageDevice, srcImage, runMode_);
    Result ret = dvpp_.resize(previewImage, imageDevice, previewWidth_, previewHeight_);
    if (ret == FAILED) {
        ERROR_LOG("Resize image failed\n");
        return FAILED;
    }
    switch(_s_work_mode)
    {
        case ASCBOT_OFF:
        case ASCBOT_VOID_DANGER_MODE:
        case ASCBOT_REMOTE_MODE:
            PostDange.handle_preview(previewImage, _s_work_mode);
        break;
        case ASCBOT_OBJECT_MODE:
            PostFollow.handle_preview(previewImage, _s_work_mode);
        break;
        case ASCBOT_ROAD_MODE:
            PostRoad.handle_preview(previewImage, _s_work_mode);
        break;
    }
    return SUCCESS;
}

Result ObjectDetect::inference(aclmdlDataset*& inferenceOutput,
                               ImageData& resizedImage) {

    switch(_s_work_mode)
    {
        case ASCBOT_OFF:
        case ASCBOT_VOID_DANGER_MODE:
        case ASCBOT_REMOTE_MODE:
        {
            Result ret = model1_.CreateInput(resizedImage.data.get(), resizedImage.size);
            if (ret != SUCCESS) {
                ERROR_LOG("Create mode input dataset failed");
                return FAILED;
            }

            ret = model1_.Execute();
            if (ret != SUCCESS) {
              ERROR_LOG("Execute model inference failed");
              return FAILED;
            }
            inferenceOutput = model1_.GetModelOutputData();
        }
        break;
        case ASCBOT_OBJECT_MODE:
        {
            Result ret = model2_.CreateInput(resizedImage.data.get(), resizedImage.size);
            if (ret != SUCCESS) {
                ERROR_LOG("Create mode input dataset failed");
                return FAILED;
            }

            ret = model2_.Execute();
            if (ret != SUCCESS) {
              ERROR_LOG("Execute model inference failed");
              return FAILED;
            }

            inferenceOutput = model2_.GetModelOutputData();
        }
        break;
        case ASCBOT_ROAD_MODE:
        {
            Result ret = model3_.CreateInput(resizedImage.data.get(), resizedImage.size);
            if (ret != SUCCESS) {
                ERROR_LOG("Create mode input dataset failed");
                return FAILED;
            }

            ret = model3_.Execute();
            if (ret != SUCCESS) {
              ERROR_LOG("Execute model inference failed");
              return FAILED;
            }
            inferenceOutput = model3_.GetModelOutputData();
        }
        break;
        default:
        break;

    }
    return SUCCESS;
}

Result ObjectDetect::postprocess(ImageData& image, aclmdlDataset* modelOutput) {
    uint32_t dataSize = 0;
    float* detectData = NULL;
    uint32_t uOutNum = 0;
    int size_data = 0;
    INFO_LOG(" get work_mode:%d\n",_s_work_mode);
    switch(_s_work_mode){
        case ASCBOT_VOID_DANGER_MODE:
        case ASCBOT_OFF:
        case ASCBOT_REMOTE_MODE:
            uOutNum = 0;
            detectData = (float *)get_inference_output_item(dataSize, modelOutput, uOutNum);
            size_data = dataSize/sizeof(float);
            PostDange.HandleResults(detectData,size_data, _s_work_mode);
        break;
        case ASCBOT_OBJECT_MODE:
            uOutNum = 1;
            detectData = (float *)get_inference_output_item(dataSize, modelOutput, uOutNum);
            size_data = dataSize/sizeof(float);
            PostFollow.HandleResults(detectData,size_data, _s_work_mode);
        break;
        case ASCBOT_ROAD_MODE:
            uOutNum = 0;
            detectData = (float *)get_inference_output_item(dataSize, modelOutput, uOutNum);
            size_data = dataSize/sizeof(float);
            PostRoad.HandleResults(detectData,size_data, _s_work_mode);
        break;
    }
    EngineHan.HandleResults();
    return SUCCESS;
}

void* ObjectDetect::get_inference_output_item(uint32_t& itemDataSize,
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

void ObjectDetect::destroy_resource()
{
    aclError ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("reset device failed");
    }
    INFO_LOG("end to reset device is %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_SUCCESS) {
        ERROR_LOG("finalize acl failed");
    }
    INFO_LOG("end to finalize acl");
}
