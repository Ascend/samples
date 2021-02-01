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

#ifndef INC_RESULT_PROCESS_H
#define INC_RESULT_PROCESS_H

#include "DvppCommon/DvppCommon.h"
#include "ConfigParser/ConfigParser.h"
#include "Framework/ModuleManager/ModuleManager.h"
#include "Statistic/Statistic.h"
#include "Common/CommonType.h"

using CallBack = void (*)(FinalResult&, FinalResult&, bool&);

class ResultProcess : public ascend_base_module::ModuleBase {
public:
    ResultProcess();
    ~ResultProcess();
    APP_ERROR init(ConfigParser &configParser, ascend_base_module::ModuleInitArgs &initArgs);
    APP_ERROR deinit(void);
    static void RegisterCallBack(CallBack callback, bool& flag, FinalResult& result);
protected:
    APP_ERROR process(std::shared_ptr<void> inputData);

private:
    APP_ERROR parse_config(ConfigParser &configParser);
    void WriteRecognizeResult(std::string imageName,
                              std::map<uint32_t, std::array<int, TEXT_BOX_COORDINATES_NUM>> &boxMap,
                              std::map<uint32_t, std::string> &textMap);
    std::string savePath_ = "./result";
    std::map<size_t, std::map<uint32_t, std::string>> textInfoMap_;
    std::map<size_t, std::map<uint32_t, std::array<int, TEXT_BOX_COORDINATES_NUM>>> textCoordinatesMap_;
    Statistic resultProcessStatic_ = {};
    size_t imageNum_ = 0;
    uint32_t enableCallback_ = 0;
    static bool* finish_;
    static FinalResult* finalResult_;
    static CallBack callback_;
};

MODULE_REGIST(ResultProcess)
#endif
