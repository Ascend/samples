/**
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2022. All rights reserved.
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
#include <sys/timeb.h>
#include <cmath>
#include <algorithm>
#include "acl/acl.h"
#include "AclLiteApp.h"
#include "AclLiteModel.h"
#include "CarParams.h"
#include "inference.h"

using namespace std;

namespace {
uint32_t kDetectModelWidth = 416;
uint32_t kDetectModelHeight = 416;
const char* kDetectModelPath = "../model/yolov3.om";
uint32_t kClassifyModelWidth = 224;
uint32_t kClassifyModelHeight = 224;
const char* kClassifyModelPath = "../model/color_dynamic_batch.om";
const int32_t kBatch = 8;
const int kInvalidSize = -1;
}

InferenceThread::InferenceThread(aclrtRunMode& runMode)
    :detectModel_(kDetectModelPath), classifyModel_(kClassifyModelPath),
    batchSize_(kBatch), runMode_(runMode), imageInfoSize_(0),
    imageInfoBuf_(nullptr), classifyInputSize_(0), classifyInputBuf_(nullptr)
{
}

InferenceThread::~InferenceThread()
{
    detectModel_.DestroyResource();
    classifyModel_.DestroyResource();
}

AclLiteError InferenceThread::InitModelInput()
{
    classifyInputSize_ = YUV420SP_SIZE(kClassifyModelWidth, kClassifyModelHeight) * kBatch;
    void* buf = nullptr;
    aclError aclRet = aclrtMalloc(&buf, classifyInputSize_,
                                  ACL_MEM_MALLOC_HUGE_FIRST);
    if ((buf == nullptr) || (aclRet != ACL_ERROR_NONE)) {
        ACLLITE_LOG_ERROR("Malloc classify inference input buffer failed, "
                          "error %d", aclRet);
        return ACLLITE_ERROR;
    }
    classifyInputBuf_ = (uint8_t *)buf;
    return ACLLITE_OK;
}

AclLiteError InferenceThread::Init()
{
    AclLiteError ret = detectModel_.Init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("detect Model init failed, error:%d", ret);
        return ret;
    }

    ret = classifyModel_.Init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("classify Model init failed, error:%d", ret);
        return ret;
    }

    ret = InitModelInput();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Model Input init failed, error:%d", ret);
        return ret;
    }

    return ACLLITE_OK;
}

AclLiteError InferenceThread::DetectModelExecute(shared_ptr<CarDetectDataMsg> carDetectDataMsg)
{
    if (carDetectDataMsg->isLastFrame == 1) {
        ACLLITE_LOG_INFO("it is channel %d's lastframe in Detect Inference", carDetectDataMsg->channelId);
        return ACLLITE_OK;
    }
    aclFloat16 new_shapeHeight = aclFloatToFloat16((float)kDetectModelHeight);
    aclFloat16 new_shapeWidth = aclFloatToFloat16((float)kDetectModelWidth);
    aclFloat16 orig_shapeHeight = aclFloatToFloat16((float)carDetectDataMsg->imageFrame.height);
    aclFloat16 orig_shapeWidth = aclFloatToFloat16((float)carDetectDataMsg->imageFrame.width);
    const aclFloat16 imageInfo[4] = {new_shapeHeight, new_shapeWidth,
                                     orig_shapeHeight, orig_shapeWidth};
    imageInfoSize_ = aclDataTypeSize(ACL_FLOAT16) * 4;
    imageInfoBuf_ = CopyDataToDevice((void *)imageInfo, imageInfoSize_,
                                     runMode_, MEMORY_DEVICE);
    if (imageInfoBuf_ == nullptr) {
        ACLLITE_LOG_ERROR("Copy image info to device failed");
        return ACLLITE_ERROR;
    }
    AclLiteError ret = detectModel_.CreateInput(carDetectDataMsg->resizedFrame.data.get(),
                                                carDetectDataMsg->resizedFrame.size,
                                                imageInfoBuf_, imageInfoSize_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create detect model input dataset failed");
        return ACLLITE_ERROR;
    }

    ret = detectModel_.Execute(carDetectDataMsg->detectInferData);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Execute detect model inference failed, error: %d", ret);
        return ACLLITE_ERROR;
    }
    detectModel_.DestroyInput();
    aclrtFree(imageInfoBuf_);
    imageInfoBuf_ = nullptr;
    return ACLLITE_OK;
}

AclLiteError InferenceThread::DetectMsgSend(shared_ptr<CarDetectDataMsg> carDetectDataMsg)
{
    while (1) {
        AclLiteError ret = SendMessage(carDetectDataMsg->detectPostThreadId, MSG_DETECT_INFER_OUTPUT, carDetectDataMsg);
        if (ret == ACLLITE_ERROR_ENQUEUE) {
            usleep(500);
            continue;
        } else if(ret == ACLLITE_OK) {
            break;
        } else {
            ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
            return ret;
        }
    }

    return ACLLITE_OK;
}

int InferenceThread::CopyOneBatchImages(uint8_t* buffer, uint32_t bufferSize,
                                        vector<CarInfo> &carImgs, int batchIdx)
{
    uint32_t j = 0;
    int dataLen = 0;
    int totalSize = 0;

    for (uint32_t i = batchIdx * batchSize_;
         i < carImgs.size() && j < batchSize_ && bufferSize > totalSize;
         i++, j++) {
        dataLen = CopyImageData(buffer + totalSize,
                                bufferSize - totalSize, carImgs[i].resizedImgs);
        if (dataLen == kInvalidSize) {
            return kInvalidSize;
        }
        totalSize += dataLen;
    }
    
    if (j < batchSize_) {
        for (uint32_t k = 0;
             k < batchSize_ - j && bufferSize > totalSize;
             k++) {
            dataLen = CopyImageData(buffer + totalSize,
                                    bufferSize - totalSize,
                                    carImgs[carImgs.size() - 1].resizedImgs);
            if (dataLen == kInvalidSize) {
                return kInvalidSize;
            }
            totalSize += dataLen;
        }
    }

    return j;
}

int InferenceThread::CopyImageData(uint8_t *buffer,
                                   uint32_t bufferSize, ImageData& image)
{
    uint32_t dataSize = YUV420SP_SIZE(kClassifyModelWidth, kClassifyModelHeight);
    AclLiteError ret = CopyDataToDeviceEx(buffer, bufferSize, image.data.get(),
                                          dataSize, runMode_);
    if (ret) {
        ACLLITE_LOG_ERROR("Copy face data to device failed");
        return kInvalidSize;
    }

    return dataSize;
}

AclLiteError InferenceThread::ClassifyModelExecute(shared_ptr<CarDetectDataMsg> carDetectDataMsg)
{
    if (carDetectDataMsg->isLastFrame == 1) {
        ACLLITE_LOG_INFO("it is channel %d's lastframe in Classify Inference", carDetectDataMsg->channelId);
        return ACLLITE_OK;
    }
    
    if (carDetectDataMsg->flag == 1) {
        return ACLLITE_OK;
    }

    int carcnt = carDetectDataMsg->carInfo.size();
    int batchNum = 0;
    if (carcnt == 1) {
        batchNum = 1;
        batchSize_ = 1;
    } else if (carcnt == 2) {
        batchNum = 1;
        batchSize_ = 2;
    } else if (carcnt > 2  && carcnt <= 4) {
        batchNum = 1;
        batchSize_ = 4;
    } else if (carcnt > 4  && carcnt <= 8) {
        batchNum = 1;
        batchSize_ = 8;
    } else if (carcnt > 8) {
        batchSize_ = 8;
        batchNum =  carcnt / batchSize_;
        if (carcnt % batchSize_ != 0)
            batchNum++;
    }
    
    for (int i = 0; i < batchNum; i++) {
        // Copy one batch preprocessed image data to device
        int carNum = CopyOneBatchImages(classifyInputBuf_, classifyInputSize_,
                                        carDetectDataMsg->carInfo, i);
        if (carNum < 0) {
            ACLLITE_LOG_ERROR("Copy the %dth batch images failed", i);
            break;
        }
        // Inference one batch data
        AclLiteError ret = classifyModel_.Execute(carDetectDataMsg->classifyInferData,
                                                  classifyInputBuf_, classifyInputSize_, batchSize_);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Execute model inference failed\n");
            break;
        }
    }

    return ACLLITE_OK;
}

AclLiteError InferenceThread::ClassifyMsgSend(shared_ptr<CarDetectDataMsg> carDetectDataMsg)
{
    while (1) {
        AclLiteError ret = SendMessage(carDetectDataMsg->classifyPostThreadId,
            MSG_CLASSIFY_INFER_OUTPUT, carDetectDataMsg);
        if (ret == ACLLITE_ERROR_ENQUEUE) {
            usleep(500);
            continue;
        } else if (ret == ACLLITE_OK) {
            break;
        } else {
            ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
            return ret;
        }
    }

    return ACLLITE_OK;
}

AclLiteError InferenceThread::Process(int msgId, shared_ptr<void> data)
{
    switch (msgId) {
        case MSG_DETECT_PREPROC_DATA:
            DetectModelExecute(static_pointer_cast<CarDetectDataMsg>(data));
            DetectMsgSend(static_pointer_cast<CarDetectDataMsg>(data));
            break;
        case MSG_CLASSIFY_PREPROC_DATA:
            ClassifyModelExecute(static_pointer_cast<CarDetectDataMsg>(data));
            ClassifyMsgSend(static_pointer_cast<CarDetectDataMsg>(data));
            break;
        default:
            ACLLITE_LOG_INFO("Inference thread ignore msg %d", msgId);
            break;
    }

    return ACLLITE_OK;
}

