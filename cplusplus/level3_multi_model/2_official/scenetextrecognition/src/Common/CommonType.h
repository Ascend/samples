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

#ifndef INC_COMMON_TYPE_H
#define INC_COMMON_TYPE_H

#include <unistd.h>
#include "CommonDataType/CommonDataType.h"
#include "CommandParser/CommandParser.h"
#include "ErrorCode/ErrorCode.h"
#include "Log/Log.h"

const uint32_t TEXT_BOX_COORDINATES_NUM = 8;
const uint32_t TEXT_BOX_COORDINATES_DIM = 2;
const uint32_t TEXT_BOX_X0_INDEX = 0;
const uint32_t TEXT_BOX_Y0_INDEX = 1;
const uint32_t TEXT_BOX_X1_INDEX = 2;
const uint32_t TEXT_BOX_Y1_INDEX = 3;
const uint32_t TEXT_BOX_X2_INDEX = 4;
const uint32_t TEXT_BOX_Y2_INDEX = 5;
const uint32_t TEXT_BOX_X3_INDEX = 6;
const uint32_t TEXT_BOX_Y3_INDEX = 7;

struct FinalResult {
    std::map<uint32_t, std::array<int, TEXT_BOX_COORDINATES_NUM>> boxMap;
    std::map<uint32_t, std::string> textMap;
};

struct ItemInfo {
    uint32_t itemId;
    ImageInfo cropData;
    ImageInfo perspectiveData;
    RawData resizeData;
    // Coordinates for text box on resized image
    std::array<int, TEXT_BOX_COORDINATES_NUM> boxCoordinate;
    // Coordinates for text box on original image
    std::array<int, TEXT_BOX_COORDINATES_NUM> orgCoordinate;
    std::string textContent;
};

struct TextInfo {
    uint32_t itemId;
    std::string textContent;
    std::array<int, TEXT_BOX_COORDINATES_NUM> textCoordinate;
};

struct SendInfo {
    std::string imageName;
    size_t imageId;
    uint32_t imageWidth;
    uint32_t imageHeight;
    uint32_t itemNum;
    std::shared_ptr<RawData> imageData;
    RawData decodedData;
    ImageInfo resizedData;
    std::vector<RawData> textDetectOutput;
    std::shared_ptr<ItemInfo> itemInfo;
};

#endif
