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

#ifndef INC_WARP_PERSPECTIVE_H
#define INC_WARP_PERSPECTIVE_H

#include "DvppCommon/DvppCommon.h"
#include "ConfigParser/ConfigParser.h"
#include "Framework/ModuleManager/ModuleManager.h"
#include "Statistic/Statistic.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "Common/CommonType.h"

class WarpPerspective : public ascend_base_module::ModuleBase {
public:
    WarpPerspective();
    ~WarpPerspective();
    APP_ERROR init(ConfigParser &configParser, ascend_base_module::ModuleInitArgs &initArgs);
    APP_ERROR deinit(void);

protected:
    APP_ERROR process(std::shared_ptr<void> inputData);

private:
    APP_ERROR parse_config(ConfigParser &configParser);
    APP_ERROR ApplyWarpPerspective(std::shared_ptr<SendInfo> sendData, cv::Mat &imgRGB888);
    APP_ERROR CropTextBox(std::shared_ptr<SendInfo> sendData);

    uint32_t leftTopX_ = 0;
    uint32_t leftTopY_ = 0;
    uint32_t rightBotX_ = 0;
    uint32_t rightBotY_ = 0;
    DvppCommon *dvppObjPtr_ = nullptr;
    aclrtStream dvppStream_ = nullptr;
    Statistic warpAffineStatic_ = {};
    uint32_t dstWidth_ = 0;
    uint32_t dstHeight_ = 0;
    uint32_t debugMode_ = false;
};

MODULE_REGIST(WarpPerspective)
#endif
