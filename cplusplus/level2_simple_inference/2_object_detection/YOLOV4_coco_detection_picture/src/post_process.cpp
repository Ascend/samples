/**
* @file post_process.cpp
*
* Copyright (C) 2020. Huawei Technologies Co., Ltd. All rights reserved.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
#include "post_process.h"
#include <algorithm>
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs/legacy/constants_c.h"
#include "opencv2/imgproc/types_c.h"

using namespace std;

namespace {
    const static std::vector <std::string> kLabels = {"person", "bicycle", "car", "motorbike",
                                                      "aeroplane", "bus", "train", "truck", "boat",
                                                      "traffic light", "fire hydrant", "stop sign", "parking meter",
                                                      "bench", "bird", "cat", "dog", "horse",
                                                      "sheep", "cow", "elephant", "bear", "zebra",
                                                      "giraffe", "backpack", "umbrella", "handbag", "tie",
                                                      "suitcase", "frisbee", "skis", "snowboard", "sports ball",
                                                      "kite", "baseball bat", "baseball glove", "skateboard",
                                                      "surfboard",
                                                      "tennis racket", "bottle", "wine glass", "cup",
                                                      "fork", "knife", "spoon", "bowl", "banana",
                                                      "apple", "sandwich", "orange", "broccoli", "carrot",
                                                      "hot dog", "pizza", "donut", "cake", "chair",
                                                      "sofa", "potted plant", "bed", "dining table", "toilet",
                                                      "TV monitor", "laptop", "mouse", "remote", "keyboard",
                                                      "cell phone", "microwave", "oven", "toaster", "sink",
                                                      "refrigerator", "book", "clock", "vase", "scissors",
                                                      "teddy bear", "hair drier", "toothbrush"};

    // bounding box line solid
    const uint32_t kLineSolid = 2;

    // opencv draw label params.
    const double kFountScale = 0.5;
    const cv::Scalar kFontColor(0, 0, 255);
    const uint32_t kLabelOffset = 11;

    const size_t kClassNum = 80;
    const size_t kModelOutputBoxNum = 10647;
    const float kNMSThreshold = 0.8;
    const float kScoreThreshold = 0.4;

    // opencv color list for boundingbox
    const vector <cv::Scalar> kColors{
            cv::Scalar(237, 149, 100), cv::Scalar(0, 215, 255), cv::Scalar(50, 205, 50),
            cv::Scalar(139, 85, 26)};
}

void PostProcess::SetBoxInfo(size_t index, BBox &box) {
    float *boxBuff = static_cast<float *>(outputBox_);
    boxBuff += (index * sizeof(float));
    box.x = boxBuff[0] * xScale_;
    box.y = boxBuff[1] * yScale_;
    box.w = boxBuff[2] * xScale_;
    box.h = boxBuff[3] * yScale_;
}

bool PostProcess::SortScore(BBox box1, BBox box2) {
    return box1.score > box2.score;
}

float PostProcess::IOU(const BBox &b1, const BBox &b2) {
    float x1 = max(b1.x, b2.x);
    float y1 = max(b1.y, b2.y);
    float x2 = min(b1.x + b1.w, b2.x + b2.w);
    float y2 = min(b1.y + b1.h, b2.y + b2.h);
    float w = max(0.0f, x2 - x1 + 1);
    float h = max(0.0f, y2 - y1 + 1);
    float area = w * h;
    return area / (b1.w * b1.h + b2.w * b2.h - area);
}

void PostProcess::NMS(vector <BBox> &boxes, vector <BBox> &result) {
    result.clear();
    std::sort(boxes.begin(), boxes.end(), SortScore);

    while (boxes.size() != 0) {
        result.push_back(boxes[0]);
        size_t index = 1;
        while (boxes.size() > index) {
            float iou = IOU(boxes[0], boxes[index]);
            if (iou > kNMSThreshold) {
                boxes.erase(boxes.begin() + index);
                continue;
            }
            ++index;
        }
        boxes.erase(boxes.begin());
    }
}

void PostProcess::DrawBoundBoxToImage(const vector <BBox> &result) {
    cv::Mat image = cv::imread(originImage_, CV_LOAD_IMAGE_UNCHANGED);
    for (size_t i = 0; i < result.size(); ++i) {
        cv::Point p1, p2;
        p1.x = result[i].x - result[i].w / 2;
        p1.y = result[i].y - result[i].h / 2;
        p2.x = result[i].x + result[i].w / 2;
        p2.y = result[i].y + result[i].h / 2;
        cv::rectangle(image, p1, p2, kColors[i % kColors.size()], kLineSolid);
        string className = kLabels[result[i].classIndex]; // classIndex is valid for kLabels
        cv::putText(image, className, cv::Point(p1.x, p1.y + kLabelOffset),
                    cv::FONT_HERSHEY_COMPLEX, kFountScale, kFontColor);
    }

    size_t pos = originImage_.find_last_of("/");
    string fileName(originImage_.substr(pos + 1));
    stringstream sstream;
    sstream.str("");
    sstream << "../out/out_" << fileName;
    cv::imwrite(sstream.str(), image);
    INFO_LOG("running success, you can get the running result from the %s", sstream.str().c_str());
}

Result PostProcess::Process() {
    vector <BBox> boxes;
    float *classBuff = static_cast<float *>(outputClass_);
    for (size_t i = 0; i < kModelOutputBoxNum; ++i) {
        float maxValue = 0;
        float maxIndex = 0;
        for (size_t j = 0; j < kClassNum; ++j) {
            float value = classBuff[i * kClassNum + j];
            if (value > maxValue) {
                maxIndex = j;
                maxValue = value;
            }
        }
        if (maxValue >= kScoreThreshold) {
            BBox b;
            SetBoxInfo(i, b);
            b.score = maxValue;
            b.classIndex = maxIndex;
            b.index = i;
            if (maxIndex < kClassNum) {
                boxes.push_back(b);
            }
        }
    }

    vector <BBox> result;
    NMS(boxes, result);
    DrawBoundBoxToImage(result);
    return SUCCESS;
}
