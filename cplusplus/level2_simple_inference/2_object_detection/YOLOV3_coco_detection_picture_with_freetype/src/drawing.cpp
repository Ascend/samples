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

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <string>
#include <stdio.h>
#include "drawing.h"

void DrawText(ImageData& image, int x, int y, const std::string &text, const YUVColor &color)
{
    RenderText(image, x, y, text, &color);
}

void DrawRect(ImageData& image, int x1, int y1, int x2, int y2, const YUVColor &color,
              int lineWidth)
{
    if (x1 > x2) {
        std::swap(x1, x2);
    }

    if (y1 > y2) {
        std::swap(y1, y2);
    }

    int i, j;
    int iBound, jBound;
    int iStart, jStart;
    int width = (int)image.width;
    int height = (int)image.height;

    jBound = std::min(height, y1 + lineWidth);
    iBound = std::min(width - 1, x2);

    for (j = y1; j < jBound; ++j) {
        for (i = x1; i <= iBound; ++i) {
            SetPixel(image, i, j, color);
        }
    }

    jStart = std::max(0, y2 - lineWidth + 1);
    jBound = std::min(height - 1, y2);

    iStart = std::max(0, x1);
    iBound = std::min(width - 1, x2);

    for (j = jStart; j <= jBound; ++j) {
        for (i = iStart; i <= iBound; ++i) {
            SetPixel(image, i, j, color);
        }
    }

    iBound = std::min(width, x1 + lineWidth);
    jBound = std::min(height - 1, y2);

    for (i = x1; i < iBound; ++i) {
        for (j = y1; j <= jBound; ++j) {
            SetPixel(image, i, j, color);
        }
    }

    iStart = std::max(0, (x2 - lineWidth + 1));
    iBound = std::min(width - 1, x2);

    jStart = std::max(0, y1);
    jBound = std::min(height - 1, y2);

    for (i = iStart; i <= iBound; ++i) {
        for (j = jStart; j <= jBound; ++j) {
            SetPixel(image, i, j, color);
        }
    }
}