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
#include "post_process.h"
#include "opencv2/opencv.hpp"
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
    const uint32_t g_lineSolid = 2;

    // opencv draw label params.
    const double g_fountScale = 0.5;
    const cv::Scalar g_fontColor(0, 0, 255);
    const uint32_t g_labelOffset = 11;
    const size_t g_classNum = 80;
    const size_t g_modelOutputBoxNum = 10647;
    const float g_NMSThreshold = 0.8;
    const float g_scoreThreshold = 0.4;

    // opencv color list for boundingbox
    const vector <cv::Scalar> g_colors{
            cv::Scalar(237, 149, 100), cv::Scalar(0, 215, 255), cv::Scalar(50, 205, 50),
            cv::Scalar(139, 85, 26)};
}

void PostProcess::SetBoxInfo(size_t index, BBox &box)
{
    float *boxBuff = static_cast<float *>(g_outputBox_);
    boxBuff += (index * sizeof(float));
    box.x = boxBuff[0] * g_xScale_;
    box.y = boxBuff[1] * g_yScale_;
    box.w = boxBuff[2] * g_xScale_;
    box.h = boxBuff[3] * g_yScale_;
}

bool PostProcess::SortScore(BBox box1, BBox box2)
{
    return box1.score > box2.score;
}

float PostProcess::IOU(const BBox &b1, const BBox &b2)
{
    float x1 = max(b1.x, b2.x);
    float y1 = max(b1.y, b2.y);
    float x2 = min(b1.x + b1.w, b2.x + b2.w);
    float y2 = min(b1.y + b1.h, b2.y + b2.h);
    float w = max(0.0f, x2 - x1 + 1);
    float h = max(0.0f, y2 - y1 + 1);
    float area = w * h;
    return area / (b1.w * b1.h + b2.w * b2.h - area);
}

void PostProcess::NMS(vector <BBox> &boxes, vector <BBox> &result)
{
    result.clear();
    std::sort(boxes.begin(), boxes.end(), SortScore);

    while (boxes.size() != 0) {
        result.push_back(boxes[0]);
        size_t index = 1;
        while (boxes.size() > index) {
            float iou = IOU(boxes[0], boxes[index]);
            if (iou > g_NMSThreshold) {
                boxes.erase(boxes.begin() + index);
                continue;
            }
            ++index;
        }
        boxes.erase(boxes.begin());
    }
}

void PostProcess::DrawBoundBoxToImage(const vector <BBox> &result)
{
    cv::Mat image = cv::imread(g_originImage_, CV_LOAD_IMAGE_UNCHANGED);
    int half = 2;
    for (size_t i = 0; i < result.size(); ++i) {
        cv::Point p1, p2;
        p1.x = result[i].x - result[i].w / half;
        p1.y = result[i].y - result[i].h / half;
        p2.x = result[i].x + result[i].w / half;
        p2.y = result[i].y + result[i].h / half;
        cv::rectangle(image, p1, p2, g_colors[i % g_colors.size()], g_lineSolid);
        string className = kLabels[result[i].classIndex]; // classIndex is valid for kLabels
        cv::putText(image, className, cv::Point(p1.x, p1.y + g_labelOffset),
                    cv::FONT_HERSHEY_COMPLEX, g_fountScale, g_fontColor);
    }

    size_t pos = g_originImage_.find_last_of("/");
    string fileName(g_originImage_.substr(pos + 1));
    stringstream sstream;
    sstream.str("");
    sstream << "../out/output/out_" << fileName;
    cv::imwrite(sstream.str(), image);
    INFO_LOG("running success, you can get the running result from the %s", sstream.str().c_str());
}

Result PostProcess::Process()
{
    vector <BBox> boxes;
    float *classBuff = static_cast<float *>(g_outputClass_);
    for (size_t i = 0; i < g_modelOutputBoxNum; ++i) {
        float maxValue = 0;
        float maxIndex = 0;
        for (size_t j = 0; j < g_classNum; ++j) {
            float value = classBuff[i * g_classNum + j];
            if (value > maxValue) {
                maxIndex = j;
                maxValue = value;
            }
        }
        if (maxValue >= g_scoreThreshold) {
            BBox b;
            SetBoxInfo(i, b);
            b.score = maxValue;
            b.classIndex = maxIndex;
            b.index = i;
            if (maxIndex < g_classNum) {
                boxes.push_back(b);
            }
        }
    }

    vector <BBox> result;
    NMS(boxes, result);
    DrawBoundBoxToImage(result);
    return SUCCESS;
}
