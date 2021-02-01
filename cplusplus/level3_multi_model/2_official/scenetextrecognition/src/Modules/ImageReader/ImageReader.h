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

#ifndef INC_IMAGE_READER_H
#define INC_IMAGE_READER_H

#include "acl/acl.h"
#include "ErrorCode/ErrorCode.h"
#include "ConfigParser/ConfigParser.h"
#include "Framework/ModuleManager/ModuleManager.h"
#include "Statistic/Statistic.h"

enum PathType {
    FOLDER_TYPE = 0,
    FILE_TYPE = 1,
    NONE_TYPE
};

class ImageReader : public ascend_base_module::ModuleBase {
public:
    ImageReader();
    ~ImageReader();
    APP_ERROR init(ConfigParser &configParser, ascend_base_module::ModuleInitArgs &initArgs);
    APP_ERROR deinit(void);
    static size_t GetTotalImageNum();

protected:
    APP_ERROR process(std::shared_ptr<void> inputData);

private:
    APP_ERROR ScanFolder(std::string &inputPath);
    APP_ERROR parse_config(ConfigParser &configParser);
    inline void PushFileToVector(const std::string &imageFile);
    APP_ERROR CheckPathType(std::string &path, enum PathType &type);
    void ReadAndSendImages(size_t &imgId);

private:
    std::string inputPath_;
    uint32_t runTimes_ = 1;
    bool longTimeTest_ = false;
    static size_t totalImageNum_;
    std::vector<std::string> fileNameSet_ = {};
    Statistic jpegReaderStatic_ = {};
};

MODULE_REGIST(ImageReader)
#endif
