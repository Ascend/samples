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

#ifndef YOLOV4_COCO_DETECTION_PICTURE_INC_POST_PROCESS_H
#define YOLOV4_COCO_DETECTION_PICTURE_INC_POST_PROCESS_H

#pragma once

#include <iostream>
#include <vector>
#include "utils.h"

typedef struct BBox {
    float x;
    float y;
    float w;
    float h;
    float score;
    size_t classIndex;
    size_t index; // index of output buffer
} BBox;

class PostProcess {
public:
    /**
    * @brief Constructor
    * @param [in] outputClass: ouput buffer for class
    * @param [in] outputBox: ouput buffer for box
    * @param [in] xScale: ouput buffer for x scale
    * @param [in] yScale: ouput buffer for y scale
    * @param [in] originImage: origin image path
    */
    PostProcess(void *outputClass, void *outputBox, float xScale, float yScale, std::string &originImage)
    : g_outputClass_(outputClass), g_outputBox_(outputBox), g_xScale_(xScale), g_yScale_(yScale),
      g_originImage_(originImage)
    {
    }

    /**
    * @brief Constructor
    */
    PostProcess() : g_outputClass_(nullptr), g_outputBox_(nullptr), g_xScale_(0), g_yScale_(0), g_originImage_("") {}

    /**
    * @brief Destructor
    */
    virtual ~PostProcess()
    {
    }

    /**
    * @brief sort box
    * @param [in] box1: left box
    * @param [in] box2: right box
    * @return result, true when score of box1 large than box2's
    */
    static bool SortScore(BBox box1, BBox box2);

    /**
    * @brief sort box
    * @param [in] b1: box1
    * @param [in] b2: box2
    * @return result, iou value
    */
    static float IOU(const BBox &b1, const BBox &b2);

    /**
    * @brief post process
    * @return result
    */
    Result Process();

private:
    void SetBoxInfo(size_t index, BBox &box);

    void NMS(std::vector <BBox> &boxes, std::vector <BBox> &result);

    void DrawBoundBoxToImage(const std::vector <BBox> &result);

    void *g_outputClass_;
    void *g_outputBox_;
    float g_xScale_;
    float g_yScale_;
    std::string g_originImage_;
};

#endif