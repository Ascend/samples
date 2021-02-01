/*
 * Copyright(C) 2020. Huawei Technologies Co.,Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <iostream>
#include <algorithm>

#include "TextDetection/TextDetection.h"
#include "DetectPost/DetectPost.h"
#include "ErrorCode/ErrorCode.h"
#include "Log/Log.h"
#include "FileManager/FileManager.h"
#include "PointerDeleter/PointerDeleter.h"
#include "CommonDataType/CommonDataType.h"
#include "Common/CommonType.h"

using namespace ascend_base_module;

TextDetection::TextDetection() {}

TextDetection::~TextDetection() {}

APP_ERROR TextDetection::parse_config(ConfigParser &configParser)
{
    std::string itemCfgStr = moduleName_ + std::string(".modelPath");
    APP_ERROR ret = configParser.GetStringValue(itemCfgStr, modelPath_);
    if (ret != APP_ERR_OK) {
        LogFatal << "TextDetection[" << instanceId_ << "]: Failed to get " << itemCfgStr << ", ret = " << ret << ".";
        return ret;
    }
    itemCfgStr = moduleName_ + std::string(".modelName");
    ret = configParser.GetStringValue(itemCfgStr, modelName_);
    if (ret != APP_ERR_OK) {
        LogFatal << "TextDetection[" << instanceId_ << "]: Failed to get " << itemCfgStr << ", ret = " << ret << ".";
        return ret;
    }
    itemCfgStr = std::string("SystemConfig.deviceId");
    ret = configParser.GetUnsignedIntValue(itemCfgStr, deviceId_);
    if (ret != APP_ERR_OK) {
        LogFatal << "TextDetection[" << instanceId_ << "]: Failed to get " << itemCfgStr << ", ret = " << ret << ".";
        return ret;
    }

    return ret;
}

APP_ERROR TextDetection::init(ConfigParser &configParser, ModuleInitArgs &initArgs)
{
    assign_initargs(initArgs);
    LogDebug << "TextDetection[" << instanceId_ << "]: TextDetection begin to init instance " \
             << initArgs.instanceId << ".";
    APP_ERROR ret = parse_config(configParser);
    if (ret != APP_ERR_OK) {
        LogFatal << "TextDetection[" << instanceId_ << "]: Failed to parse config params: " \
                 << get_app_errcodeinfo(ret) << ".";
        return ret;
    }

    modelProcess_ = new ModelProcess(deviceId_, modelName_);
    char detectModelPath[PATH_MAX];
    if (realpath(modelPath_.c_str(), detectModelPath) == nullptr) {
        LogError << "TextDetection[" << instanceId_ << "]: Failed to get the real path of " << modelPath_.c_str();
        return APP_ERR_COMM_NO_EXIST;
    }
    ret = modelProcess_->init(std::string(detectModelPath));
    if (ret != APP_ERR_OK) {
        LogFatal << "TextDetection[" << instanceId_ << "]: Failed to execute init model, ret = " << ret << ".";
        delete modelProcess_;
        return ret;
    }

    LogDebug << "TextDetection[" << instanceId_ << "]: TextDetection init successfully.";
    return APP_ERR_OK;
}

APP_ERROR TextDetection::deinit(void)
{
    LogDebug << "TextDetection[" << instanceId_ << "]: TextDetection begin to deinit.";
    APP_ERROR ret = APP_ERR_OK;

    // Release ModelProcess object resource
    if (modelProcess_ != nullptr) {
        modelProcess_->deinit();
        delete modelProcess_;
        modelProcess_ = nullptr;
    }

    LogDebug << "TextDetection[" << instanceId_ << "]: TextDetection deinit successfully.";
    return ret;
}

APP_ERROR TextDetection::PrepareModelBuffer(std::vector<void *> &inputBuffers, std::vector<size_t> &inputSizes,
                                            std::vector<void *> &outputBuffers, std::vector<size_t> &outputSizes)
{
    // Get model description
    aclmdlDesc *modelDesc = modelProcess_->GetModelDesc();
    size_t outputSize = aclmdlGetNumOutputs(modelDesc);
    for (size_t i = 0; i < outputSize; i++) {
        size_t bufferSize = aclmdlGetOutputSizeByIndex(modelDesc, i);
        void *outputBuffer = nullptr;
        APP_ERROR ret = aclrtMalloc(&outputBuffer, bufferSize, ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != APP_ERR_OK) {
            LogError << "TextDetection[" << instanceId_ << "]: Failed to malloc model output buffer, ret = " << ret;
            // Free the buffer malloced successfully before return error
            modelProcess_->ReleaseModelBuffer(outputBuffers);
            return ret;
        }
        outputBuffers.push_back(outputBuffer);
        outputSizes.push_back(bufferSize);
    }

    size_t inputNum = aclmdlGetNumInputs(modelDesc);
    for (size_t i = 1; i < inputNum; ++i) {
        void *buffer = nullptr;
        size_t size = aclmdlGetInputSizeByIndex(modelDesc, i);
        APP_ERROR ret = aclrtMalloc(&buffer, size, ACL_MEM_MALLOC_NORMAL_ONLY);
        if (ret != ACL_ERROR_NONE) {
            LogError << "TextDetection[" << instanceId_ << "]: Failed to malloc model input buffer, ret = " \
                     << ret << ".";
            // inputBuffers[0] will be freed by shared point, just set it to nullptr here
            inputBuffers[0] = nullptr;
            // Free the buffer malloced successfully before return error
            modelProcess_->ReleaseModelBuffer(inputBuffers);
            modelProcess_->ReleaseModelBuffer(outputBuffers);
            return ret;
        }
        inputBuffers.push_back(buffer);
        inputSizes.push_back(size);
    }
    return APP_ERR_OK;
}

APP_ERROR TextDetection::process(std::shared_ptr<void> inputData)
{
    textDetectStatic_.RunTimeStatisticStart("TextDetectModel_Execute_Time", instanceId_, true);
    std::shared_ptr<SendInfo> sendData = std::static_pointer_cast<SendInfo>(inputData);
    LogDebug << "TextDetection[" << instanceId_ << "]: [" << sendData->imageName << "] process start.";
    modelProcess_->SetModelWH(sendData->resizedData.width, sendData->resizedData.height);
    // Prepare output buffer for text detection model
    std::vector<void *> outputBuffers;
    std::vector<size_t> outputSizes;
    // Prepare input buffer for text detection model
    std::vector<void *> inputBuffers({(uint8_t *)sendData->resizedData.data.get()});
    std::vector<size_t> inputSizes({sendData->resizedData.lenOfByte});
    APP_ERROR ret = PrepareModelBuffer(inputBuffers, inputSizes, outputBuffers, outputSizes);
    if (ret != APP_ERR_OK) {
        return ret;
    }
    // Execute text detection model
    ret = modelProcess_->ModelInferDynamicHW(inputBuffers, inputSizes, outputBuffers, outputSizes);
    // Release input buffer after model inference completed
    inputBuffers[0] = nullptr;
    // Free the buffer malloced successfully after model inference completed
    modelProcess_->ReleaseModelBuffer(inputBuffers);
    if (ret != APP_ERR_OK) {
        LogError << "TextDetection[" << instanceId_ << "]: [" << sendData->imageName \
                 << "]: Failed to execute the model: " << modelName_ << ", ret = " << ret << ".";
        // Release output buffer before return error
        modelProcess_->ReleaseModelBuffer(outputBuffers);;
        return ret;
    }
    textDetectStatic_.RunTimeStatisticStop();

    for (size_t i = 0; i < outputBuffers.size(); i++) {
        RawData rawDevData = RawData();
        rawDevData.data.reset(outputBuffers[i], [](void*) {});
        rawDevData.lenOfByte = outputSizes[i];
        sendData->textDetectOutput.push_back(rawDevData);
    }

    SendToNextModule(MT_DetectPost, sendData, instanceId_);
    LogDebug << "TextDetection[" << instanceId_ << "]: [" << sendData->imageName << "]: process end.";
    return APP_ERR_OK;
}