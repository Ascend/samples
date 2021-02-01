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

#ifndef INC_TEXT_RECOGNITION_H
#define INC_TEXT_RECOGNITION_H

#include "DvppCommon/DvppCommon.h"
#include "ConfigParser/ConfigParser.h"
#include "Framework/ModuleManager/ModuleManager.h"
#include "Statistic/Statistic.h"
#include "Framework/ModelProcess/ModelProcess.h"
#include "Common/CommonType.h"

class TextRecognition : public ascend_base_module::ModuleBase {
public:
    TextRecognition();
    ~TextRecognition();
    APP_ERROR init(ConfigParser &configParser, ascend_base_module::ModuleInitArgs &initArgs);
    APP_ERROR deinit(void);

protected:
    APP_ERROR process(std::shared_ptr<void> inputData);

private:
    APP_ERROR parse_config(ConfigParser &configParser);
    APP_ERROR Preprocess(std::shared_ptr<SendInfo> sendData);
    void SaveResizedImage(std::shared_ptr<SendInfo> sendData);
    void FindTheBestHWSize(uint32_t &bestHeight, uint32_t bestWidth, uint32_t inWidth, uint32_t inHeight);
    APP_ERROR PrepareModelBuffer(std::vector<void *> &inputBuffers, std::vector<size_t> &inputSizes,
                                 std::vector<void *> &outputBuffers, std::vector<size_t> &outputSizes);
    void RecognizeOutput(const std::vector<std::shared_ptr<void>> &featLayerData,
                         const std::vector<size_t> &outputSizes, std::string &recognizeStr);
    APP_ERROR RecognizePostProcess(const std::vector<void *> &outputBuffers, const std::vector<size_t> &outputSizes,
                                   std::string &recognizeStr);
    APP_ERROR LoadKeysUTF8File(const std::string& fileName, std::vector<std::string>& keysVector);

    uint32_t deviceId_ = 0;
    std::string modelName_ = "";
    std::string modelPath_ = "";
    uint32_t modelMaxWidth_ = 0;
    uint32_t modelWidth_ = 0;
    uint32_t modelHeight_ = 0;
    ModelProcess* modelProcess_ = nullptr;
    DvppCommon *dvppObjPtr_ = nullptr;
    aclrtStream dvppStream_ = nullptr;
    uint32_t debugMode_ = false;
    Statistic textRecognitionStatic_ = {};
    std::vector<uint32_t> dynamicWidthVec_ = {0};
    std::vector<std::string> keysVec_ = {};
    std::string keysFilePath_ = "";
    int keysNum_ = 0;
};

MODULE_REGIST(TextRecognition)
#endif
