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
#include "data_receiver.h"
#include "painting_process.h"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <sys/types.h>
#include "model_process.h"
#include "acl/acl.h"
#include "utils.h"

using namespace std;

#define MODEL_INPUT_SIZE(N, H, W, C, bytes) ((N) * (H) * (W) * (C)* bytes)

extern bool g_isDevice;

PaintingProcess::PaintingProcess(const char* modelPath, uint32_t ModelInputSize[][4])
:deviceId_(0), context_(nullptr), stream_(nullptr),
layoutDataBuffer_(nullptr), objectInfoBuffer_(nullptr),
channel_(nullptr), isInited_(false){
    modelPath_ = modelPath;
    objectInfoSize_ = MODEL_INPUT_SIZE(ModelInputSize[0][0], ModelInputSize[0][1], \
                                        ModelInputSize[0][2], ModelInputSize[0][3], 8);
    layoutDataSize_ = MODEL_INPUT_SIZE(ModelInputSize[1][0], ModelInputSize[1][1], \
                                        ModelInputSize[1][2], ModelInputSize[1][3], 4);

    INFO_LOG("Objet info data (the first model input) size: %d", objectInfoSize_);
    INFO_LOG("Layout data (the second model input) size: %d", layoutDataSize_);
}

PaintingProcess::~PaintingProcess() {
    DestroyResource();
}

Result PaintingProcess::InitResource() {
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

    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl get run mode failed");
        return FAILED;
    }

    if(runMode_ == ACL_DEVICE){
        INFO_LOG("run mode : device");
        g_isDevice = true;
    }else{
        INFO_LOG("run mode : host");
    }
    INFO_LOG("get run mode success");
    return SUCCESS;
}

Result PaintingProcess::InitModel(const char* omModelPath) {
    //Load Model
    Result ret = model_.LoadModelFromFileWithMem(omModelPath);

    if (ret != SUCCESS) {
        ERROR_LOG("execute load model from file with memory failed");
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


    aclrtMalloc(&objectInfoBuffer_, (size_t)(objectInfoSize_), ACL_MEM_MALLOC_HUGE_FIRST); //申请device上的内存
    if (objectInfoBuffer_ == nullptr) {
        ERROR_LOG("Acl malloc object info data buffer failed.");
        return FAILED;
    }

    aclrtMalloc(&layoutDataBuffer_, (size_t)(layoutDataSize_), ACL_MEM_MALLOC_HUGE_FIRST); //申请device上的内存
    if (layoutDataBuffer_ == nullptr) {
        ERROR_LOG("Acl malloc layout data buffer failed.");
        return FAILED;
    }

    ret = model_.CreateInput(objectInfoBuffer_, objectInfoSize_, layoutDataBuffer_, layoutDataSize_);
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateInput failed");
        aclrtFree(objectInfoBuffer_);
        aclrtFree(layoutDataBuffer_);
        return FAILED;
    }

    return SUCCESS;
}

Result PaintingProcess::OpenPresenterChannel() {
    INFO_LOG("OpenChannel start");
    PresenterErrorCode errorCode = OpenChannelByConfig(channel_, "./param.conf");
    INFO_LOG("OpenChannel param");
    if (errorCode != PresenterErrorCode::kNone) {
        ERROR_LOG("OpenChannel failed %d", static_cast<int>(errorCode));
//        return FAILED;
    }

    return SUCCESS;
}

Result PaintingProcess::Init() {
    if (isInited_) {
        INFO_LOG("Painting process instance is initied already!");
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

    ret = OpenPresenterChannel();
    if (ret != SUCCESS) {
        ERROR_LOG("Open presenter channel failed");
        return FAILED;
    }

    ret = dataReceiver_.Init();
    if (ret != SUCCESS) {
        ERROR_LOG("Data receiver register app failed");
        return FAILED;
    }

    isInited_ = true;
    return SUCCESS;
}

Result PaintingProcess::Preprocess() {

    Result ret = dataReceiver_.DoReceiverProcess(objectInfoBuffer_, layoutDataBuffer_);
    if (ret != SUCCESS) {
        INFO_LOG("No message received");
        return FAILED;
    }

    return SUCCESS;
}

Result PaintingProcess::Inference(aclmdlDataset*& inferenceOutput) {
    Result ret = model_.Execute();
    if (ret != SUCCESS) {
        ERROR_LOG("Execute model inference failed");
        return FAILED;
    }

    inferenceOutput = model_.GetModelOutputData();

    return SUCCESS;
}

Result PaintingProcess::Postprocess(aclmdlDataset* modelOutput){

    for (size_t index = 0; index < aclmdlGetDatasetNumBuffers(modelOutput); ++index) {
        aclDataBuffer* dataBuffer = aclmdlGetDatasetBuffer(modelOutput, index);
        void* data = aclGetDataBufferAddr(dataBuffer);
        uint32_t len = aclGetDataBufferSize(dataBuffer);

        void *outHostData = NULL;
        aclError ret = ACL_ERROR_NONE;
        float *outData = NULL;
        if (!g_isDevice) {
            aclError ret = aclrtMallocHost(&outHostData, len);
            if (ret != ACL_ERROR_NONE) {
                ERROR_LOG("aclrtMallocHost failed, ret[%d]", ret);
                return FAILED;
            }

            ret = aclrtMemcpy(outHostData, len, data, len, ACL_MEMCPY_DEVICE_TO_HOST);
            if (ret != ACL_ERROR_NONE) {
                ERROR_LOG("aclrtMemcpy failed, ret[%d]", ret);
                return FAILED;
            }

            outData = reinterpret_cast<float*>(outHostData);
        } else {
            outData = reinterpret_cast<float*>(data);
        }

        if(layoutMap == index){
            uint32_t size = static_cast<uint32_t>(len) / sizeof(float);
            INFO_LOG("The size of layoutMap: %d", size);
            cv::Mat layoutMap(256, 256, CV_32FC3, const_cast<float_t*>((float_t*)outData));
            cv::cvtColor(layoutMap, layoutMap, CV_RGB2BGR);
            SendImage(layoutMap);
            INFO_LOG("Sending layoutMap...");
        }

        if(resultImg == index){
            uint32_t size = static_cast<uint32_t>(len) / sizeof(float);
            INFO_LOG("The size of resultImg: %d", size);
            cv::Mat resultImg(448, 448, CV_32FC3, const_cast<float*>((float*)outData));
            cv::cvtColor(resultImg, resultImg, CV_RGB2BGR);
            resultImg = (resultImg * 0.5 + 0.5) * 255;
            SendImage(resultImg);
            INFO_LOG("Sending resultImg...");
        }

        if (!g_isDevice) {
            ret = aclrtFreeHost(outHostData);
            if (ret != ACL_ERROR_NONE) {
                ERROR_LOG("aclrtFreeHost failed, ret[%d]", ret);
                return FAILED;
            }
        }
    }

    INFO_LOG("output data success");
    INFO_LOG("---------------------------------");
    return SUCCESS;
}

void* PaintingProcess::GetInferenceOutputItem(uint32_t& itemDataSize,
                                              aclmdlDataset* inferenceOutput) {
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

void PaintingProcess::SaveImage(const string& filename, cv::Mat& image) {

    stringstream sstream;
    sstream.str("");
    sstream << "./output/" << filename;

    string outputPath = sstream.str();
    cv::imwrite(outputPath, image);
}

void PaintingProcess::EncodeImage(vector<uint8_t>& encodeImg, cv::Mat& origImg) {
    vector<int> param = vector<int>(2);
    param[0] = CV_IMWRITE_JPEG_QUALITY;
    param[1] = 95;//default(95) 0-100

    cv::imencode(".jpg", origImg, encodeImg, param);
}

Result PaintingProcess::SendImage(cv::Mat& image) {
    vector<uint8_t> encodeImg;
    EncodeImage(encodeImg, image);

    ImageFrame imageParam;
    imageParam.format = ImageFormat::kJpeg;
    imageParam.width = image.cols;
    imageParam.height = image.rows;
    imageParam.size = encodeImg.size();
    imageParam.data = reinterpret_cast<uint8_t*>(encodeImg.data());

    std::vector<DetectionResult> detectionResults;
    imageParam.detection_results = detectionResults;

    INFO_LOG("The width of the image is %d, the height of the image is %d", imageParam.width, imageParam.height);
    PresenterErrorCode errorCode = PresentImage(channel_, imageParam);
    if (errorCode != PresenterErrorCode::kNone) {
        ERROR_LOG("PresentImage failed %d", static_cast<int>(errorCode));
        return FAILED;
    }

    return SUCCESS;
}

void PaintingProcess::DestroyResource()
{   
    aclrtFree(layoutDataBuffer_);
    aclrtFree(objectInfoBuffer_);
    layoutDataBuffer_ = nullptr;
    objectInfoBuffer_ = nullptr;

    delete channel_;
    
    model_.DestroyResource();

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
