/**
* @file post_process.h
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
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
    PostProcess(void *outputClass, void *outputBox, float xScale, float yScale, std::string &originImage) :
            outputClass_(outputClass), outputBox_(outputBox), xScale_(xScale), yScale_(yScale),
            originImage_(originImage) {}

    /**
    * @brief Constructor
    */
    PostProcess() : outputClass_(nullptr), outputBox_(nullptr), xScale_(0), yScale_(0), originImage_("") {}

    /**
    * @brief Destructor
    */
    virtual ~PostProcess() {}

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

    void *outputClass_;
    void *outputBox_;
    float xScale_;
    float yScale_;
    std::string originImage_;
};

