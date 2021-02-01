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
#include <fstream>

#include "TextRecognition/TextRecognition.h"
#include "ResultProcess/ResultProcess.h"
#include "ErrorCode/ErrorCode.h"
#include "Log/Log.h"
#include "FileManager/FileManager.h"
#include "PointerDeleter/PointerDeleter.h"
#include "CommonDataType/CommonDataType.h"

using namespace ascend_base_module;

TextRecognition::TextRecognition() {}

TextRecognition::~TextRecognition() {}

APP_ERROR TextRecognition::parse_config(ConfigParser &configParser)
{
    std::string itemCfgStr = moduleName_ + std::string(".modelPath");
    APP_ERROR ret = configParser.GetStringValue(itemCfgStr, modelPath_);
    if (ret != APP_ERR_OK) {
        LogFatal << "TextRecognition[" << instanceId_ << "]: Failed to get " << itemCfgStr << ", ret = " << ret << ".";
        return ret;
    }
    itemCfgStr = moduleName_ + std::string(".modelName");
    ret = configParser.GetStringValue(itemCfgStr, modelName_);
    if (ret != APP_ERR_OK) {
        LogFatal << "TextRecognition[" << instanceId_ << "]: Failed to get " << itemCfgStr << ", ret = " << ret << ".";
        return ret;
    }
    itemCfgStr = moduleName_ + std::string(".modelHeight");
    ret = configParser.GetUnsignedIntValue(itemCfgStr, modelHeight_);
    if (ret != APP_ERR_OK) {
        LogFatal << "TextRecognition[" << instanceId_ << "]: Failed to get " << itemCfgStr << ", ret = " << ret << ".";
        return ret;
    }
    itemCfgStr = std::string("SystemConfig.deviceId");
    ret = configParser.GetUnsignedIntValue(itemCfgStr, deviceId_);
    if (ret != APP_ERR_OK) {
        LogFatal << "TextRecognition[" << instanceId_ << "]: Failed to get " << itemCfgStr << ", ret = " << ret << ".";
        return ret;
    }
    itemCfgStr = std::string("SystemConfig.debugMode");
    ret = configParser.GetUnsignedIntValue(itemCfgStr, debugMode_);
    if (ret != APP_ERR_OK) {
        LogFatal << "TextRecognition[" << instanceId_ << "]: Failed to get " << itemCfgStr << ", ret = " << ret << ".";
        return ret;
    }
    itemCfgStr = moduleName_ + ".dynamicWidthList";
    ret = configParser.GetVectorUint32Value(itemCfgStr, dynamicWidthVec_);
    if (ret != APP_ERR_OK) {
        LogFatal << "TextRecognition[" << instanceId_ << "]: Failed to get " << itemCfgStr << ", ret = " << ret << ".";
        return ret;
    }
    modelMaxWidth_ = dynamicWidthVec_[dynamicWidthVec_.size() - 1];
    itemCfgStr = moduleName_ + std::string(".keysFilePath");
    ret = configParser.GetStringValue(itemCfgStr, keysFilePath_);
    if (ret != APP_ERR_OK) {
        LogFatal << "TextRecognition[" << instanceId_ << "]: Failed to get " << itemCfgStr << ", ret = " << ret << ".";
        return ret;
    }
    return ret;
}

APP_ERROR TextRecognition::init(ConfigParser &configParser, ModuleInitArgs &initArgs)
{
    assign_initargs(initArgs);
    LogDebug << "TextRecognition[" << instanceId_ << "]: TextDetection begin to init instance " \
             << initArgs.instanceId << ".";
    APP_ERROR ret = parse_config(configParser);
    if (ret != APP_ERR_OK) {
        LogFatal << "TextRecognition[" << instanceId_ << "]: Failed to parse config params: " \
                 << get_app_errcodeinfo(ret) << ".";
        return ret;
    }

    modelProcess_ = new ModelProcess(deviceId_, modelName_);
    char tempPath[PATH_MAX];
    if (realpath(modelPath_.c_str(), tempPath) == nullptr) {
        LogError << "TextRecognition[" << instanceId_ << "]: Failed to get the real path of " << modelPath_.c_str();
        return APP_ERR_COMM_NO_EXIST;
    }
    ret = modelProcess_->init(std::string(tempPath));
    if (ret != APP_ERR_OK) {
        LogFatal << "TextRecognition[" << instanceId_ << "]: Failed to execute init model, ret = " << ret << ".";
        return ret;
    }

    // Create dvpp stream and dvpp object for cropping the text
    ret = aclrtCreateStream(&dvppStream_);
    if (ret != APP_ERR_OK) {
        LogFatal << "TextRecognition[" << instanceId_ << "]: Failed to execute aclrtCreateStream, ret=" << ret << ".";
        return ret;
    }

    dvppObjPtr_ = new DvppCommon(dvppStream_);
    ret = dvppObjPtr_->init();
    if (ret != APP_ERR_OK) {
        LogFatal << "TextRecognition[" << instanceId_ << "]: Failed to create dvpp channel, ret = " << ret << ".";
        return ret;
    }

    if (realpath(keysFilePath_.c_str(), tempPath) == nullptr) {
        LogError << "Failed to get the real path of " << keysFilePath_.c_str();
        return APP_ERR_COMM_NO_EXIST;
    }
    ret = LoadKeysUTF8File(std::string(tempPath), keysVec_);
    if (ret != APP_ERR_OK) {
        LogFatal << "TextRecognition[" << instanceId_ << "]: Failed to load " << tempPath <<", ret = " \
                 << ret << ".";
        return ret;
    }

    LogDebug << "TextRecognition[" << instanceId_ << "]: TextRecognition init successfully.";
    return APP_ERR_OK;
}

APP_ERROR TextRecognition::deinit(void)
{
    LogDebug << "TextRecognition[" << instanceId_ << "]: TextRecognition begin to deinit.";

    APP_ERROR ret = APP_ERR_OK;

    if (dvppObjPtr_ != nullptr) {
        ret = dvppObjPtr_->deinit();
        if (ret != APP_ERR_OK) {
            LogFatal << "TextRecognition[" << instanceId_ << "]: Faild to execute dvpp deinit, ret = " << ret << ".";
            return ret;
        }
        delete dvppObjPtr_;
        dvppObjPtr_ = nullptr;
    }

    ret = aclrtDestroyStream(dvppStream_);
    if (ret != APP_ERR_OK) {
        LogFatal << "TextRecognition[" << instanceId_ << "]: Failed to destroy dvpp stream, ret = " \
                 << ret << ".";
        return ret;
    }
    dvppStream_ = nullptr;

    if (modelProcess_ != nullptr) {
        delete modelProcess_;
    }

    LogDebug << "TextRecognition[" << instanceId_ << "]: TextRecognition deinit successfully.";
    return ret;
}

APP_ERROR TextRecognition::process(std::shared_ptr<void> inputData)
{
    textRecognitionStatic_.RunTimeStatisticStart("TextRecognitionModel_Execute_Time", instanceId_, true);
    std::shared_ptr<SendInfo> sendData = std::static_pointer_cast<SendInfo>(inputData);

    LogDebug << "TextRecognition[" << instanceId_ << "]: [" << sendData->imageName << "-" \
             << sendData->itemInfo->itemId << "]: process start.";
    APP_ERROR ret = Preprocess(sendData);
    if (ret != APP_ERR_OK) {
        return ret;
    }

    if (debugMode_ != 0) {
        SaveResizedImage(sendData);
    }
    std::vector<void *> outputBuffers;
    std::vector<size_t> outputSizes;
    std::vector<void *> inputBuffers;
    std::vector<size_t> inputSizes;
    inputBuffers.push_back(sendData->itemInfo->resizeData.data.get());
    inputSizes.push_back(sendData->itemInfo->resizeData.lenOfByte);
    ret = PrepareModelBuffer(inputBuffers, inputSizes, outputBuffers, outputSizes);
    if (ret != APP_ERR_OK) {
        return ret;
    }

    ret = modelProcess_->ModelInferDynamicHW(inputBuffers, inputSizes, outputBuffers, outputSizes);

    // Release input buffer after model inference completed
    inputBuffers[0] = nullptr;
    // Free the buffer malloced successfully after model inference completed
    modelProcess_->ReleaseModelBuffer(inputBuffers);
    if (ret != APP_ERR_OK) {
        LogError << "TextRecognition[" << instanceId_ << "]: [" << sendData->imageName << "-" \
                 << sendData->itemInfo->itemId << "]: Failed to execute ModelInference, ret = " << ret << ".";
        modelProcess_->ReleaseModelBuffer(outputBuffers);
        return ret;
    }

    // PostProcess
    ret = RecognizePostProcess(outputBuffers, outputSizes, sendData->itemInfo->textContent);
    modelProcess_->ReleaseModelBuffer(outputBuffers);
    if (ret != APP_ERR_OK) {
        LogError << "TextRecognition[" << instanceId_ << "]: [" << sendData->imageName << "-" \
                 << sendData->itemInfo->itemId << "]: Failed to execute ModelInference, ret = " << ret << ".";
        return ret;
    }

    textRecognitionStatic_.RunTimeStatisticStop();
    SendToNextModule(MT_ResultProcess, sendData, 0);
    LogDebug << "TextRecognition[" << instanceId_ << "]: [" << sendData->imageName << "-" \
             << sendData->itemInfo->itemId << "]: process end.";
    return APP_ERR_OK;
}

APP_ERROR TextRecognition::Preprocess(std::shared_ptr<SendInfo> sendData)
{
    FindTheBestHWSize(modelWidth_, modelHeight_, sendData->itemInfo->perspectiveData.width,
                      sendData->itemInfo->perspectiveData.height);
    modelProcess_->SetModelWH(modelWidth_, modelHeight_);

    // Prepare the input info for resize
    DvppDataInfo resizeInput;
    resizeInput.width = sendData->itemInfo->perspectiveData.width;
    resizeInput.height = sendData->itemInfo->perspectiveData.height;
    resizeInput.widthStride = DVPP_ALIGN_UP(sendData->itemInfo->perspectiveData.width, VPC_STRIDE_WIDTH);
    resizeInput.heightStride = DVPP_ALIGN_UP(sendData->itemInfo->perspectiveData.height, VPC_STRIDE_HEIGHT);
    resizeInput.dataSize = sendData->itemInfo->perspectiveData.lenOfByte;
    resizeInput.data = sendData->itemInfo->perspectiveData.data.get();

    // Begin to resize the cropped image to adapte the model
    DvppDataInfo resizeOutput;
    resizeOutput.width = modelWidth_;
    resizeOutput.height = modelHeight_;
    resizeOutput.format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    APP_ERROR ret = dvppObjPtr_->CombineResizeProcess(resizeInput, resizeOutput, true);
    if (ret != APP_ERR_OK) {
        LogError << "TextRecognition[" << instanceId_ << "]: [" << sendData->imageName << "-" \
                 << sendData->itemInfo->itemId << "]: Failed to process resize, ret = " << ret << ".";
        return ret;
    }

    std::shared_ptr<DvppDataInfo> itemResizeImg = dvppObjPtr_->GetResizedImage();
    sendData->itemInfo->resizeData.data.reset(itemResizeImg->data, acldvppFree);
    sendData->itemInfo->resizeData.lenOfByte = itemResizeImg->dataSize;

    if (debugMode_ != 0) {
        LogDebug << "TextRecognition[" << instanceId_ << "]: [" << sendData->imageName << "-" \
                 << sendData->itemInfo->itemId << "]: Input: " << resizeInput.width << ", " << resizeInput.height;
        LogDebug << "TextRecognition[" << instanceId_ << "]: [" << sendData->imageName << "-" \
                 << sendData->itemInfo->itemId << "]: Output: " << resizeOutput.width << ", " << resizeOutput.height;
    }
    return APP_ERR_OK;
}

void TextRecognition::SaveResizedImage(std::shared_ptr<SendInfo> sendData)
{
    void *hostPtr = nullptr;
    APP_ERROR ret = aclrtMallocHost(&hostPtr, sendData->itemInfo->resizeData.lenOfByte);
    if (ret != APP_ERR_OK) {
        LogWarn << "TextRecognition[" << instanceId_ << "]: [" << sendData->imageName << "-" \
                << sendData->itemInfo->itemId << "]: Failed to malloc on host, ret = " << ret << ".";
        return;
    }
    std::shared_ptr<void> hostSharedPtr(hostPtr, aclrtFreeHost);
    ret = aclrtMemcpy(hostPtr, sendData->itemInfo->resizeData.lenOfByte, sendData->itemInfo->resizeData.data.get(),
                      sendData->itemInfo->resizeData.lenOfByte, ACL_MEMCPY_DEVICE_TO_HOST);
    if (ret != APP_ERR_OK) {
        LogWarn << "TextRecognition[" << instanceId_ << "]: [" << sendData->imageName << "-" \
                << sendData->itemInfo->itemId << "]: Failed to memcpy from device to host, ret = " \
                << ret << ".";
        return;
    }
    // Save the resized result
    std::stringstream fileName;
    fileName << "resize_image_" << sendData->imageName << "_" << sendData->itemInfo->itemId;
    SaveFileWithTimeStamp(hostSharedPtr, sendData->itemInfo->resizeData.lenOfByte, moduleName_, fileName.str(), ".yuv");
}

void TextRecognition::FindTheBestHWSize(uint32_t &bestWidth, uint32_t bestHight, uint32_t inWidth, uint32_t inHeight)
{
    uint32_t widthInit = static_cast<uint32_t>(static_cast<float>(inWidth) * (bestHight / inHeight));
    for (auto widthTmp: dynamicWidthVec_) {
        bestWidth = widthTmp;
        if (widthInit <= widthTmp) {
            break;
        }
    }
}

APP_ERROR TextRecognition::PrepareModelBuffer(std::vector<void *> &inputBuffers, std::vector<size_t> &inputSizes,
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
            LogError << "TextRecognition[" << instanceId_ << "]: Failed to malloc model output buffer, ret = " << ret;
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
            LogError << "TextRecognition[" << instanceId_ << "]: Failed to malloc model input buffer, ret = " \
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

APP_ERROR TextRecognition::RecognizePostProcess(const std::vector<void *> &outputBuffers,
                                                const std::vector<size_t> &outputSizes, std::string &recognizeStr)
{
    const size_t outputLen = outputSizes.size();
    if (outputLen <= 0) {
        LogError << "TextRecognition[" << instanceId_ << "]: Failed to get model output data.";
        return APP_ERR_INFER_GET_OUTPUT_FAIL;
    }

    APP_ERROR ret;
    std::vector<std::shared_ptr<void>> singleResult;
    for (size_t j = 0; j < outputLen; j++) {
        void *hostPtrBuffer = nullptr;
        ret = (APP_ERROR)aclrtMallocHost(&hostPtrBuffer, outputSizes[j]);
        if (ret != APP_ERR_OK) {
            LogError << "TextRecognition[" << instanceId_ << "]: Failed to malloc host memory, ret = " << ret << ".";
            return ret;
        }
        std::shared_ptr<void> hostPtrBufferManager(hostPtrBuffer, aclrtFreeHost);
        ret = (APP_ERROR)aclrtMemcpy(hostPtrBuffer, outputSizes[j], outputBuffers[j], outputSizes[j],
                                     ACL_MEMCPY_DEVICE_TO_HOST);
        if (ret != APP_ERR_OK) {
            LogError << "TextRecognition[" << instanceId_ << "]: Failed to memcpy device to host, ret = " << ret << ".";
            return ret;
        }

        singleResult.push_back(hostPtrBufferManager);
    }
    RecognizeOutput(singleResult, outputSizes, recognizeStr);
    return APP_ERR_OK;
}

void TextRecognition::RecognizeOutput(const std::vector<std::shared_ptr<void>> &featLayerData,
                                      const std::vector<size_t> &outputSizes, std::string &recognizeStr)
{
    float* res = static_cast<float *>(featLayerData[0].get());
    size_t outputSize = outputSizes[0] * modelWidth_ / modelMaxWidth_;
    const int bytesPerCharacter = 4;
    int charactersNum  = outputSize / bytesPerCharacter / keysNum_;
    int cnt = 0;
    std::vector<int> maxList;
    for (int i = 0; i < charactersNum; i++) {
        int max = 0;
        for (int j = 0; j < keysNum_; j++) {
            if (res[i * keysNum_ + max] < res[cnt++]) {
                max = j;
            }
        }
        maxList.push_back(max);
    }

    const uint32_t preOneIndex = 1;
    const uint32_t preTwoIndex = 2;
    for (size_t k = 0; k < maxList.size(); k++) {
        if ((maxList[k] != keysNum_ - 1) && (!(k > preTwoIndex && maxList[k] == maxList[k - preOneIndex]) \
            || (k > preTwoIndex && maxList[k] == maxList[k - preTwoIndex]))) {
            recognizeStr += keysVec_[maxList[k]];
        }
    }
}

APP_ERROR TextRecognition::LoadKeysUTF8File(const std::string& fileName, std::vector<std::string>& keysVector)
{
    std::ifstream ifile(fileName, std::ios::binary);
    if (!ifile) {
        return APP_ERR_COMM_OPEN_FAIL;
    }
    char key;
    while (ifile >> key) {
        std::string strTmp(1, key);
        unsigned char mask = 0x80;
        // Calculate the bytes number of utf-8 character
        int bytesNum = 0;
        while (mask & key) {
            bytesNum++;
            mask >>= 1;
        }
        if (bytesNum > 0) {
            bytesNum--;
        }
        while (bytesNum--) {
            ifile >> key;
            strTmp += key;
        }
        keysVector.push_back(strTmp);
        keysNum_++;
    }
    keysVector.push_back("");
    keysNum_++;
    return APP_ERR_OK;
}
