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

#include "ResultProcess/ResultProcess.h"
#include "ErrorCode/ErrorCode.h"
#include "Log/Log.h"
#include "FileManager/FileManager.h"
#include "PointerDeleter/PointerDeleter.h"
#include "CommonDataType/CommonDataType.h"
#include "Common/CommonType.h"
#include "ImageReader/ImageReader.h"

extern bool g_signal_recieved;
using namespace ascend_base_module;

bool* ResultProcess::finish_ = nullptr;
FinalResult* ResultProcess::finalResult_ = nullptr;
CallBack ResultProcess::callback_ = nullptr;

ResultProcess::ResultProcess() {}

ResultProcess::~ResultProcess() {}

void ResultProcess::RegisterCallBack(CallBack callback, bool& flag, FinalResult& result)
{
    callback_ = callback;
    finish_ = &flag;
    finalResult_ = &result;
}

APP_ERROR ResultProcess::init(ConfigParser &configParser, ModuleInitArgs &initArgs)
{
    assign_initargs(initArgs);
    LogDebug << "ResultProcess[" << instanceId_ << "]: ResultProcess begin to init instance "
             << initArgs.instanceId << ".";
    std::string itemCfgStr = moduleName_ + std::string(".savePath");
    APP_ERROR ret = configParser.GetStringValue(itemCfgStr, savePath_);
    if (ret != APP_ERR_OK) {
        LogFatal << "ResultProcess[" << instanceId_ << "]: Failed to get " << itemCfgStr << ", ret = " << ret << ".";
        return ret;
    }
    itemCfgStr = moduleName_ + std::string(".enableCallback");
    ret = configParser.GetUnsignedIntValue(itemCfgStr, enableCallback_);
    if (ret != APP_ERR_OK) {
        LogFatal << "ResultProcess[" << instanceId_ << "]: Failed to get " << itemCfgStr << ", ret = " << ret << ".";
        return ret;
    }
    ret = CreateDir(savePath_);
    if (ret != APP_ERR_OK) {
        LogFatal << "ResultProcess[" << instanceId_ << "]: Failed to create result directory " << savePath_ \
                 << ", ret = " << ret << ".";
        return ret;
    }

    LogDebug << "ResultProcess[" << instanceId_ << "]: ResultProcess init successfully.";
    return APP_ERR_OK;
}

APP_ERROR ResultProcess::deinit(void)
{
    LogDebug << "ResultProcess[" << instanceId_ << "]: ResultProcess begin to deinit.";
    LogDebug << "ResultProcess[" << instanceId_ << "]: ResultProcess deinit successfully.";
    return APP_ERR_OK;
}

APP_ERROR ResultProcess::process(std::shared_ptr<void> inputData)
{
    resultProcessStatic_.RunTimeStatisticStart("ResultProcess_Execute_Time", instanceId_, true);
    LogDebug << "ResultProcess[" << instanceId_ << "]: ResultProcess process start.";
    std::shared_ptr<SendInfo> sendData = std::static_pointer_cast<SendInfo>(inputData);
    size_t imgId = sendData->imageId;
    textInfoMap_[imgId].insert({sendData->itemInfo->itemId, sendData->itemInfo->textContent});
    textCoordinatesMap_[imgId].insert({sendData->itemInfo->itemId, sendData->itemInfo->orgCoordinate});

    if (textInfoMap_[imgId].size() == sendData->itemNum) {
        WriteRecognizeResult(sendData->imageName, textCoordinatesMap_[imgId], textInfoMap_[imgId]);
        textInfoMap_.erase(imgId);
        textCoordinatesMap_.erase(imgId);
        imageNum_++;
        // When all the image is processed completely, set the flag g_signal_recieved to notify the main program to exit
        if (imageNum_ == ImageReader::GetTotalImageNum()) {
            g_signal_recieved = true;
        }
        resultProcessStatic_.RunTimeStatisticStop();
        Statistic::GlobalTimeStatisticStop();
    }
    LogDebug << "ResultProcess[" << instanceId_ << "]: ResultProcess process end.";
    return APP_ERR_OK;
}

void ResultProcess::WriteRecognizeResult(std::string imageName,
                                         std::map<uint32_t, std::array<int, TEXT_BOX_COORDINATES_NUM>> &boxMap,
                                         std::map<uint32_t, std::string> &textMap)
{
    // Result file name use the time stamp as a suffix
    struct timeval time = {0, 0};
    gettimeofday(&time, nullptr);
    const int TIME_DIFF_S = 28800; // 8 hour time difference
    const int TIME_STRING_SIZE = 32;
    char timeString[TIME_STRING_SIZE] = {0};
    time_t timeVal = time.tv_sec + TIME_DIFF_S;
    struct tm *ptm = gmtime(&timeVal);
    if (ptm != nullptr) {
        strftime(timeString, sizeof(timeString), "%Y%m%d%H%M%S", ptm);
    }
    // Create result file under result directory
    std::string resultFileName = savePath_ + "/" + imageName + '_' + timeString + ".txt";
    std::ofstream tfile(resultFileName);
    // Check result file validity
    if (tfile.fail()) {
        LogWarn << "ResultProcess[" << instanceId_ << "]: [" << imageName \
                << "]: Failed to save recognition result, errno = " << errno << ".";
        return;
    }

    tfile << "Image: " << imageName << std::endl;
    for (size_t i = 0; i < textMap.size(); i++) {
        tfile << "Item" << i << "[";
        for (size_t j = 0; j < TEXT_BOX_COORDINATES_NUM; j += TEXT_BOX_COORDINATES_DIM) {
            tfile << "(" << std::to_string(boxMap[i][j]) << ", " << std::to_string(boxMap[i][j + 1]) << ")";
            if (j < TEXT_BOX_COORDINATES_NUM - TEXT_BOX_COORDINATES_DIM) {
                tfile << ", ";
            }
        }
        tfile << "]: " << textMap[i] << std::endl;
    }
    if (enableCallback_ && callback_) {
        FinalResult temp{boxMap, textMap};
        callback_(temp, *finalResult_, *finish_);
    }
}