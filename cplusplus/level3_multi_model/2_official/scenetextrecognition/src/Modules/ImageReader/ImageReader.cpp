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

#include <chrono>
#include <iostream>
#include <string>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include "ImageReader/ImageReader.h"
#include "Log/Log.h"
#include "CommonDataType/CommonDataType.h"
#include "Common/CommonType.h"
#include "ConfigParser/ConfigParser.h"
#include "FileManager/FileManager.h"
#include "ImagePreprocess/ImagePreprocess.h"

using namespace ascend_base_module;
extern bool g_signal_recieved;

namespace {
    const int LOW_THRESHOLD = 128;
    const int MAX_THRESHOLD = 4096;
    const std::string IMAGE_FILE_SUFFIX = ".jpg";
    const int IMAGE_SUFFIX_LEN = 4;
}

using Time = std::chrono::high_resolution_clock;
using fsec = std::chrono::duration<float>;

// totalImageNum_ is the total number of images, which will be set after all the image process completed
// It will be used by the last module to judge whether to notify the main program to exit
size_t ImageReader::totalImageNum_ = 0;

ImageReader::ImageReader()
{
    withoutInputQueue_ = true;
}

ImageReader::~ImageReader() {}

APP_ERROR ImageReader::init(ConfigParser &configParser, ModuleInitArgs &initArgs)
{
    // initialize member variables
    assign_initargs(initArgs);
    LogDebug << "ImageReader[" << instanceId_ << "]: ImageReader init start.";
    std::string itemCfgStr = moduleName_ + std::string(".inputPath");
    APP_ERROR ret = configParser.GetStringValue(itemCfgStr, inputPath_);
    if (ret != APP_ERR_OK) {
        LogFatal << "ImageReader[" << instanceId_ << "]: Failed to get " << itemCfgStr << ", ret = " << ret << ".";
        return ret;
    }
    char absPath[PATH_MAX];
    if (realpath(inputPath_.c_str(), absPath) == nullptr) {
        LogError << "Failed to get the real path of " << inputPath_.c_str();
        return APP_ERR_COMM_NO_EXIST;
    }
    inputPath_ = absPath;
    // check the validity of input path
    if (access(inputPath_.c_str(), APP_ERR_OK) == -1) {
        LogFatal << "ImageReader[" << instanceId_ << "]: Failed to access input path[" \
                 << inputPath_ << "] set in setup.config.";
        return APP_ERR_COMM_NO_EXIST;
    }
    itemCfgStr = "SystemConfig.runTimes";
    ret = configParser.GetUnsignedIntValue(itemCfgStr, runTimes_);
    if (ret != APP_ERR_OK) {
        LogFatal << "ImageReader[" << instanceId_ << "]: Failed to get " << itemCfgStr << ", ret = " << ret << ".";
        return ret;
    }
    if (runTimes_ == 0) {
        longTimeTest_ = true;
    }
    isStop_ = false;

    LogDebug << "ImageReader[" << instanceId_ << "]: ImageReader init successfully.";
    return APP_ERR_OK;
}

APP_ERROR ImageReader::deinit(void)
{
    LogDebug << "ImageReader[" << instanceId_ << "]: ImageReader deinit start.";
    isStop_ = true;

    LogDebug << "ImageReader[" << instanceId_ << "]: ImageReader deinit successfully.";
    return APP_ERR_OK;
}

APP_ERROR ImageReader::ScanFolder(std::string &inputPath)
{
    LogDebug << "ImageReader[" << instanceId_ << "]: ScanFolder " << inputPath << ".";
    APP_ERROR ret = APP_ERR_OK;
    struct dirent *ent = nullptr;
    DIR *dir = nullptr;

    if ((dir = opendir(inputPath.c_str())) == nullptr) {
        LogError << "ImageReader[" << instanceId_ << "]: Can't open path " << inputPath << ".";
        return APP_ERR_COMM_NO_EXIST;
    }

    while ((ent = readdir(dir)) != nullptr) {
        std::string fileName = ent->d_name;
        std::string imageFile = inputPath + "/" + fileName;
        PushFileToVector(imageFile);
    }
    closedir(dir);
    return ret;
}

inline void ImageReader::PushFileToVector(const std::string &imageFile)
{
    if ((imageFile.size() > IMAGE_SUFFIX_LEN) &&
        (imageFile.substr(imageFile.size() - IMAGE_SUFFIX_LEN) == IMAGE_FILE_SUFFIX)) {
        LogInfo << "ImageReader[" << instanceId_ << "]: Push jpeg: " << imageFile << ".";
        fileNameSet_.push_back(imageFile);
    } else {
        LogInfo << "ImageReader[" << instanceId_ << "]: " << imageFile << " is not suffix with jpg.";
    }
}

APP_ERROR ImageReader::process(std::shared_ptr<void> inputData)
{
    Statistic::GlobalTimeStatisticStart("TotalRunTime");
    LogDebug << "ImageReader[" << instanceId_ << "]: ImageReader process start.";
    enum PathType type;
    APP_ERROR ret = CheckPathType(inputPath_, type);
    if (ret != APP_ERR_OK) {
        LogError << "ImageReader[" << instanceId_ << "]: Failed to check type of input path " << inputPath_ \
                 << ", ret = " << ret << ".";
        return ret;
    }
    if (type >= NONE_TYPE) {
        LogError << "ImageReader[" << instanceId_ << "]: Type of input path " << inputPath_ \
                 << " is not directory nor regular file.";
        return APP_ERR_COMM_INVALID_PARAM;
    }

    // 1. Get file list, save to fileNameSet_
    if (type == FILE_TYPE) {
        PushFileToVector(inputPath_);
    } else {
        ret = ScanFolder(inputPath_);
        if (ret != APP_ERR_OK) {
            LogError << "ImageReader[" << instanceId_ << "]: Failed to scan folder " << inputPath_ << ".";
            return ret;
        }
    }

    // 2. Read files and send to next modules
    size_t imgId = 0;
    while ((!isStop_) && (longTimeTest_ || (runTimes_ > 0))) {
        ReadAndSendImages(imgId);
        runTimes_--;
    }
    isStop_ = true;
    ImageReader::totalImageNum_ += imgId;
    if (ImageReader::totalImageNum_ == 0) {
        g_signal_recieved = true;
    }
    LogDebug << "ImageReader[" << instanceId_ << "]: ImageReader process end.";
    return ret;
}

APP_ERROR ImageReader::CheckPathType(std::string &path, enum PathType &type)
{
    struct stat s = {0};
    APP_ERROR ret = stat(path.c_str(), &s);
    if (ret != APP_ERR_OK) {
        LogError << "ImageReader[" << instanceId_ << "]: Failed to get stat of " << path << ", ret = " << ret \
                 << ", errno = " << errno << ".";
        return ret;
    }
    if (s.st_mode & S_IFDIR) {
        type = FOLDER_TYPE;
    } else if (s.st_mode & S_IFREG) {
        type = FILE_TYPE;
    } else {
        type = NONE_TYPE;
    }
    LogInfo << "ImageReader[" << instanceId_ << "]: File type of " << path << " is " << type << ".";
    return APP_ERR_OK;
}

void ImageReader::ReadAndSendImages(size_t &imgId)
{
    const int readImageInterval = 20; // The uint is ms
    std::vector<std::string>::iterator it = fileNameSet_.begin();
    while (it != fileNameSet_.end()) {
        jpegReaderStatic_.RunTimeStatisticStart("JpegReader_Execute_Time", instanceId_, true);
        std::shared_ptr<SendInfo> sendData = std::make_shared<SendInfo>();
        std::shared_ptr<RawData> imageData = std::make_shared<RawData>();
        // Read image data from input image file
        APP_ERROR ret = ReadFile(*it, *imageData);
        if (ret != APP_ERR_OK) {
            LogError << "ImageReader[" << instanceId_ << "]: Failed to read file " << (*it) \
                     << ", ret = " << ret << ".";
            ++it;
            continue;
        }
        sendData->imageData = imageData;
        jpegReaderStatic_.RunTimeStatisticStop();
        size_t nameStartPos = (*it).rfind('/') + 1;
        sendData->imageId = imgId;
        size_t nameLen = (*it).size() - nameStartPos - IMAGE_SUFFIX_LEN;
        sendData->imageName = (*it).substr(nameStartPos, nameLen) + '_' \
                              + std::to_string(imgId);
        SendToNextModule(MT_ImagePreprocess, sendData, instanceId_);
        imgId++;
        ++it;
        // Sleep readImageInterval ms for reading image
        std::this_thread::sleep_for(std::chrono::milliseconds(readImageInterval));
    }
}

size_t ImageReader::GetTotalImageNum()
{
    return ImageReader::totalImageNum_;
}
