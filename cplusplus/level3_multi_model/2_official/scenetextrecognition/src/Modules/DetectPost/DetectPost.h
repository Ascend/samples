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

#ifndef INC_DETECT_POST_H
#define INC_DETECT_POST_H

#include "DvppCommon/DvppCommon.h"
#include "ConfigParser/ConfigParser.h"
#include "Framework/ModelProcess/ModelProcess.h"
#include "Framework/ModuleManager/ModuleManager.h"
#include "Statistic/Statistic.h"
#include "Common/CommonType.h"

class DetectPost : public ascend_base_module::ModuleBase {
public:
    DetectPost();
    ~DetectPost();
    APP_ERROR init(ConfigParser &configParser, ascend_base_module::ModuleInitArgs &initArgs);
    APP_ERROR deinit(void);

protected:
    APP_ERROR process(std::shared_ptr<void> inputData);

private:
    void CalcOriginCoordinates(std::shared_ptr<SendInfo> sendData, int detectX, int detectY,
                               int &originX, int &originY);

    uint32_t deviceId_ = 0;
    uint32_t modelWidth_ = 0;
    uint32_t modelHeight_ = 0;
    std::string modelName_ = "";
    std::string modelPath_ = "";
    ModelProcess* modelProcess_ = nullptr;
    DvppCommon *dvppObjPtr_ = nullptr;
    aclrtStream dvppStream_ = nullptr;
    Statistic textDetectPostStatic_ = {};
};

MODULE_REGIST(DetectPost)
#endif
