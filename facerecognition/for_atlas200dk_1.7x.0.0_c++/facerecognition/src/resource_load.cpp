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
#include "model_process.h"
#include "utils.h"
#include "resource_load.h"
#include "face_feature_train_mean.h"
#include "face_feature_train_std.h"

using namespace std;

namespace {
 const static std::vector<std::string> ssdLabel = { "background", "face"};

}

FaceDetection ResourceLoad::faceDetection;
FaceFeatureMaskProcess ResourceLoad::faceFeatureMask;
FaceRecognition ResourceLoad::faceRecognition;
FacePostProcess ResourceLoad::facePostProcess;


ResourceLoad::~ResourceLoad() {
    //DestroyResource();
}

void ResourceLoad::InitModelInfo(const ModelInfoParams& param) {
    modelPath1_ = param.modelPath1;
    modelWidth1_ = param.modelWidth1;
    modelHeight1_ = param.modelHeight1;
    modelPath2_ = param.modelPath2;
    modelWidth2_ = param.modelWidth2;
    modelHeight2_ = param.modelHeight2;
    modelPath3_ = param.modelPath3;
    modelWidth3_ = param.modelWidth3;
    modelHeight3_ = param.modelHeight3;

}

Result ResourceLoad::InitResource() {
    // ACL init
    //const char *aclConfigPath = "../src/acl.json";
    char aclConfigPath[32] = {'\0'};
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl init failed\n");
        return FAILED;
    }

    // open device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl open device %d failed\n", deviceId_);
        return FAILED;
    }

    // create context (set current)
    ret = aclrtCreateContext(&context_, deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl create context failed\n");
        return FAILED;
    }

    // create stream
    ret = aclrtCreateStream(&stream_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl create stream failed\n");
        return FAILED;
    }

    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl get run mode failed\n");
        return FAILED;
    }

    return SUCCESS;
}

Result ResourceLoad::InitModel(const char* omModelPath1, const char* omModelPath2, const char* omModelPath3) {
    Result ret = model1_.LoadModelFromFileWithMem(omModelPath1);
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadModelFromFileWithMem failed\n");
        return FAILED;
    }

    ret = model1_.CreateDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateDesc failed\n");
        return FAILED;
    }

    ret = model1_.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed\n");
        return FAILED;
    }

    ret = model2_.LoadModelFromFileWithMem(omModelPath2);
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadModelFromFileWithMem failed\n");
        return FAILED;
    }

    ret = model2_.CreateDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateDesc failed\n");
        return FAILED;
    }

    ret = model2_.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed\n");
        return FAILED;
    }

    ret = model3_.LoadModelFromFileWithMem(omModelPath3);
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadModelFromFileWithMem failed\n");
        return FAILED;
    }

    ret = model3_.CreateDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateDesc failed\n");
        return FAILED;
    }

    ret = model3_.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed\n");
        return FAILED;
    }
    return SUCCESS;
}

Result ResourceLoad::InitComponent() {
	
	ResourceLoad::faceFeatureMask.Init();
    return SUCCESS;
}

Result ResourceLoad::OpenPresenterChannel() {
    const char* configFile = "./param.conf";
    map<string, string> config;
    PresenterChannels::GetInstance().ReadConfig(config, configFile);

    PresenterServerParams register_param;
    register_param.app_type = "facial_recognition";
    map<string, string>::const_iterator mIter = config.begin();
    for (; mIter != config.end(); ++mIter) {
        if (mIter->first == "presenter_server_ip") {
            register_param.host_ip = mIter->second;
        }
        else if (mIter->first == "presenter_server_port") {
            register_param.port = std::stoi(mIter->second);
        }
        else if (mIter->first == "channel_name") {
            register_param.app_id = mIter->second;
        }
        /*else if (mIter->first == "content_type") {
            register_param.app_type = mIter->second;
        }*/
    }

    INFO_LOG("host_ip = %s,port = %d,app_name = %s,app_type %s",
                  register_param.host_ip.c_str(), register_param.port,
                  register_param.app_id.c_str(), register_param.app_type.c_str());
    INFO_LOG("Init Presenter Channel \n");
    // Init class PresenterChannels by configs
    PresenterChannels::GetInstance().Init(register_param);

    // create agent channel and register app
    Channel *channel= PresenterChannels::GetInstance().GetChannel();
    if (channel == nullptr) {
        ERROR_LOG("register app failed.");
        return FAILED;
    }

    return SUCCESS;
}

ModelProcess& ResourceLoad::GetModel(int model) {
    switch(model) {
        case 1:
            return model1_;
        case 2:
            return model2_;
        case 3:
            return model3_;
        defult:
            ERROR_LOG("ResourceLoad get model error model\n");
            return model1_;
    }
}

DvppProcess& ResourceLoad::GetDvpp() {
     return dvpp_;
}

Result ResourceLoad::Init(const ModelInfoParams& param) {
    if (isInited_) {
        return SUCCESS;
    }
    InitModelInfo(param);

    Result ret = InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("Init acl resource failed\n");
        return FAILED;
    }

    ret = InitModel(modelPath1_, modelPath2_, modelPath3_);
    if (ret != SUCCESS) {
        ERROR_LOG("Init model failed\n");
        return FAILED;
    }

    ret = dvpp_.InitResource(stream_);
    if (ret != SUCCESS) {
        ERROR_LOG("Init dvpp failed\n");
        return FAILED;
    }

    ret = OpenPresenterChannel();
    if (ret != SUCCESS) {
        ERROR_LOG("Open presenter channel failed\n");
        return FAILED;
    }

    ret = InitComponent();
    if (ret != SUCCESS) {
        ERROR_LOG("Init component failed\n");
        return FAILED;
    }
    INFO_LOG("Init ResourceLoad ResourceLoad\n");
    isInited_ = true;
    return SUCCESS;
}

void* ResourceLoad::GetInferenceOutputItem(uint32_t& itemDataSize,
                                           aclmdlDataset* inferenceOutput,
                                           uint32_t idx) {
    aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(inferenceOutput, idx);
    if (dataBuffer == nullptr) {
        ERROR_LOG("Get the %dth dataset buffer from model "
                  "inference output failed\n", idx);
        return nullptr;
    }

    void* dataBufferDev = aclGetDataBufferAddr(dataBuffer);
    if (dataBufferDev == nullptr) {
        ERROR_LOG("Get the %dth dataset buffer address "
                  "from model inference output failed\n", idx);
        return nullptr;
    }

    size_t bufferSize = aclGetDataBufferSize(dataBuffer);
    if (bufferSize == 0) {
        ERROR_LOG("The %d   th dataset buffer size of "
                  "model inference output is 0\n", idx);
        return nullptr;
    }

    void* data = nullptr;
    data = dataBufferDev;

    itemDataSize = bufferSize;
    return data;
}

void ResourceLoad::DestroyResource()
{
    model1_.DestroyResource();
    model2_.DestroyResource();
    model3_.DestroyResource();

    aclError ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("reset device failed\n");
    }

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("finalize acl failed\n");
    }
}

Result ResourceLoad::SendNextModelProcess(const string objStr,  std::shared_ptr<FaceRecognitionInfo> &image_handle) {

    if (objStr == "FaceRegister") {
        faceDetection.Process(image_handle);
    } else if (objStr == "MindCamera") {
        faceDetection.Process(image_handle);
    } else if (objStr == "FaceDetection") {
        faceFeatureMask.Process(image_handle);
    } else if (objStr == "FaceFeatureMaskProcess") {
        faceRecognition.Process(image_handle);
    } else if (objStr == "FaceRecognition") {
        facePostProcess.Process(image_handle);
    } else {
        ERROR_LOG("SendNextModelProcess error Object ");
        return FAILED;
    }
    return SUCCESS;
}
