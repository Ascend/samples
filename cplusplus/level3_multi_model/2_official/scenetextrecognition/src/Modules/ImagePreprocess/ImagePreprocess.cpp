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

#include "ImagePreprocess/ImagePreprocess.h"
#include "TextDetection/TextDetection.h"
#include "ErrorCode/ErrorCode.h"
#include "Log/Log.h"
#include "FileManager/FileManager.h"
#include "PointerDeleter/PointerDeleter.h"
#include "CommonDataType/CommonDataType.h"
#include "Common/CommonType.h"

using namespace ascend_base_module;

ImagePreprocess::ImagePreprocess() {}

ImagePreprocess::~ImagePreprocess() {}

APP_ERROR ImagePreprocess::parse_config(ConfigParser &configParser)
{
    std::string itemCfgStr = std::string("TextDetection.dynamicHWList");
    std::vector<uint32_t> dynamicHWList;
    APP_ERROR ret = configParser.GetVectorUint32Value(itemCfgStr, dynamicHWList);
    if (ret != APP_ERR_OK) {
        LogFatal << "ImagePreprocess[" << instanceId_ << "]: Failed to get " << itemCfgStr << ", ret = " << ret << ".";
        return ret;
    }
    const size_t dynamicListLen = 2;
    if (dynamicHWList.size() != dynamicListLen) {
        LogFatal << "ImagePreprocess[" << instanceId_ << "]: Invalid value of " << itemCfgStr \
                 << ", it need to have 2 values.";
        return APP_ERR_COMM_INVALID_PARAM;
    }
    for (auto HWTmp: dynamicHWList) {
        if ((HWTmp % VPC_WIDTH_ALIGN) != 0) {
            LogFatal << "ImagePreprocess[" << instanceId_ << "]: Invalid value of " << itemCfgStr
                     << ", it has to be a multiple of " << VPC_WIDTH_ALIGN << ".";
            return APP_ERR_COMM_INVALID_PARAM;
        }
        if ((HWTmp % VPC_HEIGHT_ALIGN) != 0) {
            LogFatal << "ImagePreprocess[" << instanceId_ << "]: Invalid value of " << itemCfgStr
                     << ", it has to be a multiple of " << VPC_HEIGHT_ALIGN << ".";
            return APP_ERR_COMM_INVALID_PARAM;
        }
    }
    HWmin_ = std::min(dynamicHWList[0], dynamicHWList[1]);
    HWmax_ = std::max(dynamicHWList[0], dynamicHWList[1]);

    itemCfgStr = std::string("SystemConfig.debugMode");
    ret = configParser.GetUnsignedIntValue(itemCfgStr, debugMode_);
    if (ret != APP_ERR_OK) {
        LogFatal << "ImagePreprocess[" << instanceId_ << "]: Failed to get " << itemCfgStr << ", ret = " << ret << ".";
        return ret;
    }

    return ret;
}

APP_ERROR ImagePreprocess::init(ConfigParser &configParser, ModuleInitArgs &initArgs)
{
    assign_initargs(initArgs);
    LogDebug << "ImagePreprocess[" << instanceId_ << "]: ImagePreprocess begin to init instance " \
             << initArgs.instanceId << ".";
    APP_ERROR ret = parse_config(configParser);
    if (ret != APP_ERR_OK) {
        LogFatal << "ImagePreprocess[" << instanceId_ << "]: Failed to parse config params: " \
                 << get_app_errcodeinfo(ret) << ".";
        return ret;
    }

    ret = aclrtCreateStream(&dvppStream_);
    if (ret != APP_ERR_OK) {
        LogFatal << "ImagePreprocess[" << instanceId_ << "]: Failed to execute aclrtCreateStream, ret=" << ret << ".";
        return ret;
    }

    dvppObjPtr_ = new DvppCommon(dvppStream_);
    ret = dvppObjPtr_->init();
    if (ret != APP_ERR_OK) {
        LogFatal << "ImagePreprocess[" << instanceId_ << "]: Failed to create dvpp channel, ret = " << ret << ".";
        return ret;
    }

    LogDebug << "ImagePreprocess[" << instanceId_ << "]: ImagePreprocess init successfully.";
    return APP_ERR_OK;
}

APP_ERROR ImagePreprocess::deinit(void)
{
    LogDebug << "ImagePreprocess[" << instanceId_ << "]: ImagePreprocess begin to deinit.";
    APP_ERROR ret = APP_ERR_OK;

    if (dvppObjPtr_ != nullptr) {
        ret = dvppObjPtr_->deinit();
        if (ret != APP_ERR_OK) {
            LogFatal << "ImagePreprocess[" << instanceId_ << "]: Faild to execute dvpp deinit, ret = " << ret << ".";
            return ret;
        }
        delete dvppObjPtr_;
        dvppObjPtr_ = nullptr;
    }

    ret = aclrtDestroyStream(dvppStream_);
    if (ret != APP_ERR_OK) {
        LogFatal << "ImagePreprocess[" << instanceId_ << "]: Failed to destroy dvpp stream, ret = " << ret << ".";
        return ret;
    }
    dvppStream_ = nullptr;

    LogDebug << "ImagePreprocess[" << instanceId_ << "]: ImagePreprocess deinit successfully.";
    return ret;
}

void ImagePreprocess::FindTheBestHWSize(uint32_t &bestWidth, uint32_t &bestHight, uint32_t inWidth, uint32_t inHeight)
{
    const int halfValue = 2;
    float threshold = (HWmax_ + HWmin_) / halfValue;
    // If the maximum value of inWidth and inHeight is less than the middle value of HWmin_ and HWmax_,
    // we select the minimum value as the model width and height without considering the ratio of inWidth and inHeight
    if (std::max(inWidth, inHeight) < threshold) {
        bestWidth = HWmin_;
        bestHight = HWmin_;
        return;
    }

    // If the minimum value of inWidth and inHeight is larger than the middle value of HWmin_ and HWmax_,
    // we select the maximum value as the model width and height without considering the ratio of inWidth and inHeight
    if (std::min(inWidth, inHeight) > threshold) {
        bestWidth = HWmax_;
        bestHight = HWmax_;
        return;
    }

    float ratio = static_cast<float>(inWidth / inHeight);
    const float width_gt_height = 1.5;
    const float height_gt_widht = 0.67;
    if (ratio > width_gt_height) {
        bestWidth = HWmax_;
        bestHight = HWmin_;
        return;
    }

    if (ratio < height_gt_widht) {
        bestWidth = HWmin_;
        bestHight = HWmax_;
        return;
    }

    bestWidth = HWmax_;
    bestHight = HWmax_;
    return;
}

APP_ERROR ImagePreprocess::process(std::shared_ptr<void> inputData)
{
    jpegDecodeStatic_.RunTimeStatisticStart("JpegDecode_Execute_Time", instanceId_, true);
    std::shared_ptr<SendInfo> sendData = std::static_pointer_cast<SendInfo>(inputData);
    LogDebug << "ImagePreprocess[" << instanceId_ << "]: [" << sendData->imageName << "] process start.";
    // Begin to decode jpeg image
    APP_ERROR ret = dvppObjPtr_->CombineJpegdProcess(*(sendData->imageData.get()), PIXEL_FORMAT_YUV_SEMIPLANAR_420,
                                                     true);
    if (ret != APP_ERR_OK) {
        LogError << "ImagePreprocess[" << instanceId_ << "]: [" << sendData->imageName \
                 << "] Failed to process decode, ret = " << ret << ".";
        return ret;
    }
    jpegDecodeStatic_.RunTimeStatisticStop();

    // Release input inmage buffer on dvpp
    std::shared_ptr<DvppDataInfo> inputImg = dvppObjPtr_->GetInputImage();
    RELEASE_DVPP_DATA(inputImg->data);

    // Get output of decoded jpeg image
    std::shared_ptr<DvppDataInfo> decodeImg = dvppObjPtr_->GetDecodedImage();
    sendData->decodedData.data.reset((void *)decodeImg->data, acldvppFree);
    sendData->decodedData.lenOfByte = decodeImg->dataSize;
    sendData->imageWidth = decodeImg->width;
    sendData->imageHeight = decodeImg->height;
    jpegResizeStatic_.RunTimeStatisticStart("JpegResize_Execute_Time", instanceId_);
    // Begin to resize the decoded image
    DvppDataInfo resizeInfo;
    FindTheBestHWSize(resizeInfo.width, resizeInfo.height, sendData->imageWidth, sendData->imageHeight);
    resizeInfo.format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    ret = dvppObjPtr_->CombineResizeProcess(*(decodeImg.get()), resizeInfo, true, VPC_PT_FIT);
    if (ret != APP_ERR_OK) {
        LogError << "ImagePreprocess[" << instanceId_ << "]: [" << sendData->imageName \
                 << "] Failed to process resize, ret = " << ret << ".";
        return ret;
    }
    jpegResizeStatic_.RunTimeStatisticStop();

    std::shared_ptr<DvppDataInfo> resizeImg = dvppObjPtr_->GetResizedImage();
    sendData->resizedData.data.reset(resizeImg->data, acldvppFree);
    sendData->resizedData.lenOfByte = resizeImg->dataSize;
    sendData->resizedData.width = resizeImg->width;
    sendData->resizedData.height = resizeImg->height;

    if (debugMode_) {
        SaveResizedImage(sendData);
        LogInfo << "ImagePreprocess[" << instanceId_ << "]: [" << sendData->imageName << "] Resize WH: " \
                << sendData->resizedData.width << ", " << sendData->resizedData.height << ".";
    }
    SendToNextModule(MT_TextDetection, sendData, instanceId_);
    LogDebug << "ImagePreprocess[" << instanceId_ << "]: [" << sendData->imageName << "] process end.";
    return APP_ERR_OK;
}

void ImagePreprocess::SaveResizedImage(std::shared_ptr<SendInfo> sendData)
{
    void *hostPtr = nullptr;
    APP_ERROR ret = aclrtMallocHost(&hostPtr, sendData->resizedData.lenOfByte);
    if (ret != APP_ERR_OK) {
        LogWarn << "ImagePreprocess[" << instanceId_ << "]: [" << sendData->imageName \
                << "]: Failed to malloc on host, ret = " << ret << ".";
        return;
    }
    std::shared_ptr<void> hostSharedPtr(hostPtr, aclrtFreeHost);
    ret = aclrtMemcpy(hostPtr, sendData->resizedData.lenOfByte, sendData->resizedData.data.get(),
                      sendData->resizedData.lenOfByte, ACL_MEMCPY_DEVICE_TO_HOST);
    if (ret != APP_ERR_OK) {
        LogWarn << "ImagePreprocess[" << instanceId_ << "]: [" << sendData->imageName \
                << "]: Failed to memcpy from device to host, ret = " << ret << ".";
        return;
    }
    // Save the resized result
    std::stringstream fileName;
    fileName << "resize_image_" << sendData->imageName;
    SaveFileWithTimeStamp(hostSharedPtr, sendData->resizedData.lenOfByte, moduleName_, fileName.str(), ".yuv");
}