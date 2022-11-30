/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at

* http://www.apache.org/licenses/LICENSE-2.0

* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#ifndef YOLOV3_COCO_DETECTION_PICTURE_WITH_FREETYPE_INC_DRAWING_H
#define YOLOV3_COCO_DETECTION_PICTURE_WITH_FREETYPE_INC_DRAWING_H

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <string>
#include "freetype_helper.h"

void DrawText(ImageData& image, int x, int y, const std::string &text, const YUVColor &color);
void DrawRect(ImageData& image, int x1, int y1, int x2, int y2, const YUVColor &color,
              int lineWidth);

#endif