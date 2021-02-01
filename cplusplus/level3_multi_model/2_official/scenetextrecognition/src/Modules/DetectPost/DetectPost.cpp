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

#include "DetectPost/DetectPost.h"
#include "DetectPost/ModelPostProcess.h"
#include "WarpPerspective/WarpPerspective.h"
#include "ErrorCode/ErrorCode.h"
#include "Log/Log.h"
#include "FileManager/FileManager.h"
#include "PointerDeleter/PointerDeleter.h"
#include "CommonDataType/CommonDataType.h"
#include "Common/CommonType.h"

using namespace ascend_base_module;

DetectPost::DetectPost() {}

DetectPost::~DetectPost() {}

APP_ERROR DetectPost::init(ConfigParser &configParser, ModuleInitArgs &initArgs)
{
    assign_initargs(initArgs);
    LogDebug << "DetectPost[" << instanceId_ << "]: DetectPost begin to init instance " \
             << initArgs.instanceId << ".";
    LogDebug << "DetectPost[" << instanceId_ << "]: DetectPost init successfully.";
    return APP_ERR_OK;
}

APP_ERROR DetectPost::deinit(void)
{
    LogDebug << "DetectPost[" << instanceId_ << "]: DetectPost begin to deinit.";
    LogDebug << "DetectPost[" << instanceId_ << "]: DetectPost deinit successfully.";

    return APP_ERR_OK;
}

APP_ERROR DetectPost::process(std::shared_ptr<void> inputData)
{
    textDetectPostStatic_.RunTimeStatisticStart("TextDetectPost_Execute_Time", instanceId_, true);
    // Prepare input buffer for text detection model
    std::shared_ptr<SendInfo> sendData = std::static_pointer_cast<SendInfo>(inputData);
    LogDebug << "DetectPost[" << instanceId_ << "]: [" << sendData->imageName << "]: process start.";
    std::vector<RawData> &modelOutput = sendData->textDetectOutput;
    std::vector<std::shared_ptr<void>> singleResult;
    for (size_t j = 0; j < modelOutput.size(); j++) {
        void *hostPtrBuffer = nullptr;
        APP_ERROR ret = aclrtMallocHost(&hostPtrBuffer, modelOutput[j].lenOfByte);
        if (ret != APP_ERR_OK) {
            LogError << "DetectPost[" << instanceId_ << "]: [" << sendData->imageName \
                     << "]: Failed to malloc host memory, ret=" << ret << ".";
            return ret;
        }
        std::shared_ptr<void> hostPtrBufferManager(hostPtrBuffer, aclrtFreeHost);
        ret = aclrtMemcpy(hostPtrBuffer, modelOutput[j].lenOfByte, modelOutput[j].data.get(),
                          modelOutput[j].lenOfByte, ACL_MEMCPY_DEVICE_TO_HOST);
        if (ret != APP_ERR_OK) {
            LogError << "DetectPost[" << instanceId_ << "]: [" << sendData->imageName \
                     << "]: Failed to memcpy device to host, ret=" << ret << ".";
            return ret;
        }
        singleResult.push_back(hostPtrBufferManager);
    }

    std::vector<std::vector<int>> textInfo;
    detect_postprocess(singleResult, sendData->resizedData.width, sendData->resizedData.height, textInfo);
    textDetectPostStatic_.RunTimeStatisticStop();

    // Release output buffer
    for (size_t i = 0; i < modelOutput.size(); i++) {
        if (modelOutput[i].data != nullptr) {
            aclrtFree(modelOutput[i].data.get());
            modelOutput[i].data = nullptr;
        }
    }

    sendData->itemNum = textInfo.size();
    for (size_t i = 0; i < sendData->itemNum; i++) {
        auto sendDataCur = std::make_shared<SendInfo>();
        *sendDataCur = *sendData;
        auto itemInfoCur = std::make_shared<ItemInfo>();
        itemInfoCur->itemId = i;
        std::copy(textInfo[i].begin(), textInfo[i].end(), itemInfoCur->boxCoordinate.begin());
        for (uint32_t j = 0; j < TEXT_BOX_COORDINATES_NUM; j += TEXT_BOX_COORDINATES_DIM) {
            CalcOriginCoordinates(sendDataCur, textInfo[i][j], textInfo[i][j + 1], itemInfoCur->orgCoordinate[j],
                                  itemInfoCur->orgCoordinate[j + 1]);
        }
        sendDataCur->itemInfo = itemInfoCur;
        SendToNextModule(MT_WarpPerspective, sendDataCur, instanceId_);
    }

    LogDebug << "DetectPost[" << instanceId_ << "]: [" << sendData->imageName << "]: process end.";
    return APP_ERR_OK;
}

void DetectPost::CalcOriginCoordinates(std::shared_ptr<SendInfo> sendData, int detectX, int detectY,
                                       int &originX, int &originY)
{
    // The image has been resized with VPC_PT_FIT, we should revert the coordinates on the original image
    bool widthRatioLarger = true;
    // The scaling ratio is based on the larger ratio to ensure the largest edge to fill the targe edge
    float resizeRatio = static_cast<float>(sendData->imageWidth) / sendData->resizedData.width;
    if (resizeRatio < (static_cast<float>(sendData->imageHeight) / sendData->resizedData.height)) {
        resizeRatio = static_cast<float>(sendData->imageHeight) / sendData->resizedData.height;
        widthRatioLarger = false;
    }

    const int halfValue = 2;
    if (widthRatioLarger) {
        uint32_t paddingUp = (sendData->resizedData.height - (sendData->imageHeight / resizeRatio)) / halfValue;
        originX = static_cast<uint32_t>(detectX * resizeRatio);
        originY = static_cast<uint32_t>((detectY - paddingUp) * resizeRatio);
        return;
    }

    uint32_t paddingLeft = (sendData->resizedData.width - (sendData->imageWidth / resizeRatio)) / halfValue;
    originX = static_cast<uint32_t>((detectX - paddingLeft) * resizeRatio);
    originY = static_cast<uint32_t>(detectY * resizeRatio);
    return;
}