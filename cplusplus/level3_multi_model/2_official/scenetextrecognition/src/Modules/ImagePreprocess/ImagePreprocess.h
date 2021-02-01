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

#ifndef INC_IMAGE_PREPROCESS_H
#define INC_IMAGE_PREPROCESS_H

#include "DvppCommon/DvppCommon.h"
#include "ConfigParser/ConfigParser.h"
#include "Framework/ModuleManager/ModuleManager.h"
#include "Statistic/Statistic.h"
#include "Common/CommonType.h"

class ImagePreprocess : public ascend_base_module::ModuleBase {
public:
    ImagePreprocess();
    ~ImagePreprocess();
    APP_ERROR init(ConfigParser &configParser, ascend_base_module::ModuleInitArgs &initArgs);
    APP_ERROR deinit(void);

protected:
    APP_ERROR process(std::shared_ptr<void> inputData);

private:
    APP_ERROR parse_config(ConfigParser &configParser);
    void SaveResizedImage(std::shared_ptr<SendInfo> sendData);
    void FindTheBestHWSize(uint32_t &bestWidth, uint32_t &bestHight, uint32_t inWidth, uint32_t inHeight);

    DvppCommon *dvppObjPtr_ = nullptr;
    uint32_t HWmin_ = 0;
    uint32_t HWmax_ = 0;
    uint32_t debugMode_ = false;
    aclrtStream dvppStream_ = nullptr;
    Statistic jpegDecodeStatic_ = {};
    Statistic jpegResizeStatic_ = {};
};

MODULE_REGIST(ImagePreprocess)
#endif
