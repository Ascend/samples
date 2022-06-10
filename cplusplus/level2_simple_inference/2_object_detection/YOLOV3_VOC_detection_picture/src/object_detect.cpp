/**
* Copyright 2020 Huawei Technologies Co., Ltd
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

* File sample_process.cpp
* Description: handle acl resource
*/
#include "object_detect.h"
#include <iostream>
#include <cmath>
#include "acl/acl.h"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/types_c.h"

using namespace std;

namespace {
 const static std::vector<std::string> yolov3Label = { "person", "bicycle", "car", "motorbike",
 "aeroplane","bus", "train", "truck", "boat",
 "traffic light", "fire hydrant", "stop sign", "parking meter",
 "bench", "bird", "cat", "dog", "horse",
 "sheep", "cow", "elephant", "bear", "zebra",
 "giraffe", "backpack", "umbrella", "handbag","tie",
 "suitcase", "frisbee", "skis", "snowboard", "sports ball",
 "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
 "tennis racket", "bottle", "wine glass", "cup",
 "fork", "knife", "spoon", "bowl", "banana",
 "apple", "sandwich", "orange", "broccoli", "carrot",
 "hot dog", "pizza", "donut", "cake", "chair",
 "sofa", "potted plant", "bed", "dining table", "toilet",
 "TV monitor", "laptop", "mouse", "remote", "keyboard",
 "cell phone", "microwave", "oven", "toaster", "sink",
 "refrigerator", "book", "clock", "vase","scissors",
 "teddy bear", "hair drier", "toothbrush" };

const uint32_t kOutputTensorSize  = 3;

uint32_t kModelWidth = 416;
uint32_t kModelHeight = 416;
const char* kModelPath = "../model/yolov3.om";

enum BBoxIndex { TOPLEFTX = 0, TOPLEFTY, BOTTOMRIGHTX, BOTTOMRIGHTY, SCORE, LABEL };
// bounding box line solid
const uint32_t kLineSolid = 2;

// output image prefix
const string kOutputFilePrefix = "out_";
// opencv draw label params.
const double kFountScale = 0.5;
const cv::Scalar kFontColor(0, 0, 255);
const uint32_t kLabelOffset = 11;

const uint numClasses = 80;
const uint BoxTensorLabel  = 85;

const uint numBBoxes = 3;
const uint  BoxTensorLength = (BoxTensorLabel * numBBoxes);
const float nmsThresh = 0.45;
const float MaxBoxClassThresh = 0.25;
const float MaxClassThresh = 0.6;

const string kFileSperator = "/";

// opencv color list for boundingbox
const vector<cv::Scalar> kColors{
  cv::Scalar(237, 149, 100), cv::Scalar(0, 215, 255), cv::Scalar(50, 205, 50),
  cv::Scalar(139, 85, 26) };

const static std::vector<uint32_t>  anchors = {10,13,16,30,33,23,30,61,62,45,59,119,116,90,156,198,373,326};
const static std::vector<uint32_t>  kGridSize = {13,26,52};
}

ObjectDetect::ObjectDetect():
model_(kModelPath),
isInited_(false), 
isReleased_(false){
}

ObjectDetect::~ObjectDetect() {
    DestroyResource();
}

AclLiteError ObjectDetect::Init() {
    if (isInited_) {
        ACLLITE_LOG_INFO("Object detection is initied already");
        return ACLLITE_OK;
    }

    AclLiteError atlRet = dvpp_.Init();
    if (atlRet) {
        ACLLITE_LOG_ERROR("Dvpp init failed, error %d", atlRet);
        return ACLLITE_ERROR;
    }

    atlRet = model_.Init();
    if (atlRet) {
        ACLLITE_LOG_ERROR("Model init failed, error %d", atlRet);
        return ACLLITE_ERROR;
    }

    isInited_ = true;
    return ACLLITE_OK;
}

AclLiteError ObjectDetect::Preprocess(ImageData& resizedImage, ImageData& srcImage, aclrtRunMode RunMode) {
    ImageData imageDevice;

    AclLiteError ret = CopyImageToDevice(imageDevice, srcImage, RunMode, MEMORY_DVPP);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Copy image to device failed");
        return ACLLITE_ERROR;
    }

    ImageData yuvImage;
    ret = dvpp_.JpegD(yuvImage, imageDevice);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Convert jpeg to yuv failed");
        return ACLLITE_ERROR;
    }

    //resize
    ret = dvpp_.Resize(resizedImage, yuvImage, kModelWidth, kModelHeight);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Resize image failed");
        return ACLLITE_ERROR;
    }
    
    return ACLLITE_OK;
}

AclLiteError ObjectDetect::Inference(std::vector<InferenceOutput>& inferenceOutput,
                               ImageData& resizedImage) {
    AclLiteError ret = model_.CreateInput(resizedImage.data.get(),
                                    resizedImage.size);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed");
        return ACLLITE_ERROR;
    }

    ret = model_.Execute(inferenceOutput);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Execute model inference failed");
        return ACLLITE_ERROR;
    }
    model_.DestroyInput();
    return ACLLITE_OK;
}

AclLiteError ObjectDetect::Postprocess(ImageData& image, std::vector<InferenceOutput>& modelOutput,
                                 const string& origImagePath) {
    std::vector<BoundingBox> binfo;

    for (uint ImgIndex = 0; ImgIndex < kOutputTensorSize; ImgIndex++) {
        uint gridSize = kGridSize[ImgIndex];
        uint32_t dataSize = 0;
        float* detectData = (float *)modelOutput[kOutputTensorSize - ImgIndex - 1].data.get();
        for (uint cx = 0; cx < gridSize; cx++)
        {
            for(uint cy = 0; cy < gridSize; cy++)
            {
                float tx = 0, ty = 0, tw = 0, th = 0, cf = 0;

                for (uint i = 0; i  < numBBoxes; ++i)
                {
                    const int bbindex = BoxTensorLength*(cx * gridSize + cy);
                    tx =  detectData[bbindex+i * BoxTensorLabel + 0];
                    ty =  detectData[bbindex+i * BoxTensorLabel + 1];
                    tw =  detectData[bbindex+i * BoxTensorLabel + 2];
                    th =  detectData[bbindex+i * BoxTensorLabel + 3];
                    cf =  detectData[bbindex+i * BoxTensorLabel + 4];

                    float MaxClass =0.0f;
                    uint32_t MaxClass_Loc = 0;
                    for (int j = 5;j< BoxTensorLabel; j++)
                    {
                        float class_prob =  (float)detectData[bbindex+ (i * BoxTensorLabel + j)];
                        if(MaxClass < class_prob)
                        {
                            MaxClass = class_prob;
                            MaxClass_Loc = j - 5;
                        }
                    }

                    if( ( cf * MaxClass > MaxBoxClassThresh)&&( MaxClass > MaxClassThresh ))
                    {
                        uint32_t x1 = (tx-tw/2);
                        uint32_t y1 = (ty-th/2);
                        uint32_t x2 = (tx+tw/2);
                        uint32_t y2 = (ty+th/2);
                        binfo.push_back({x1, y1,x2, y2, MaxClass_Loc, MaxClass});
                    }

                }
            }
        }
    }

    std::vector<BoundingBox> result;
    std::vector<std::vector<BoundingBox>> splitBoxes(numClasses);
    for (auto& box : binfo)
    {
        splitBoxes.at(box.attribute).push_back(box);
    }
    for (auto& boxes : splitBoxes)
    {
        auto overlap1D = [](float x1min, float x1max, float x2min, float x2max) -> float {
            if (x1min > x2min)
            {
                std::swap(x1min, x2min);
                std::swap(x1max, x2max);
            }
            return x1max < x2min ? 0 : std::min(x1max, x2max) - x2min;
        };
        auto computeIoU = [&overlap1D](BoundingBox& bbox1, BoundingBox& bbox2) -> float {
            float overlapX = overlap1D(bbox1.lt_x, bbox1.rb_x, bbox2.lt_x, bbox2.rb_x);
            float overlapY = overlap1D(bbox1.lt_y, bbox1.rb_y, bbox2.lt_y, bbox2.rb_y);
            float area1 = (bbox1.rb_x - bbox1.lt_x) * (bbox1.rb_y - bbox1.lt_y);
            float area2 = (bbox2.rb_x - bbox2.lt_x) * (bbox2.rb_y - bbox2.lt_y);
            float overlap2D = overlapX * overlapY;
            float u = area1 + area2 - overlap2D;
            return u == 0 ? 0 : overlap2D / u;
        };
        std::stable_sort(binfo.begin(), binfo.end(),
        [](const BoundingBox& b1, const BoundingBox& b2) { return b1.score > b2.score; });
        std::vector<BoundingBox> out;
        for (auto& i : binfo)
        {
            bool keep = true;
            for (auto& j : out)
            {
                if (keep)
                {
                    float overlap = computeIoU(i, j);
                    keep = (overlap <= nmsThresh);
                }
                else
                    break;
            }
            if (keep) out.push_back(i);
        }
        boxes = out;
        result.insert(result.end(), boxes.begin(), boxes.end());
    }

    vector<BBox> detectResults;
    float widthScale = (float)(image.width) / kModelWidth;
    float heightScale = (float)(image.height) / kModelHeight;

    for (uint32_t i = 0; i < result.size(); i++) {
        BBox boundBox;
        uint32_t score = uint32_t(result[i].score * 100);
        if (score < 90) continue;

        boundBox.rect.ltX = result[i].lt_x * widthScale;
        boundBox.rect.ltY = result[i].lt_y * heightScale;
        boundBox.rect.rbX = result[i].rb_x * widthScale;
        boundBox.rect.rbY = result[i].rb_y * heightScale;
        boundBox.text = yolov3Label[result[i].attribute] + std::to_string(score) + "\%";
        detectResults.emplace_back(boundBox);
    }
    DrawBoundBoxToImage(detectResults, origImagePath);

    return ACLLITE_OK;
}
void ObjectDetect::DrawBoundBoxToImage(vector<BBox>& detectionResults,
                                       const string& origImagePath) {
    cv::Mat image = cv::imread(origImagePath, CV_LOAD_IMAGE_UNCHANGED);
    for (int i = 0; i < detectionResults.size(); ++i) {
        cv::Point p1, p2;
        p1.x = detectionResults[i].rect.ltX;
        p1.y = detectionResults[i].rect.ltY;
        p2.x = detectionResults[i].rect.rbX;
        p2.y = detectionResults[i].rect.rbY;
        cv::rectangle(image, p1, p2, kColors[i % kColors.size()], kLineSolid);
        cv::putText(image, detectionResults[i].text, cv::Point(p1.x, p1.y + kLabelOffset),
                    cv::FONT_HERSHEY_COMPLEX, kFountScale, kFontColor);
    }

    int pos = origImagePath.find_last_of("/");
    string filename(origImagePath.substr(pos + 1));
    stringstream sstream;
    sstream.str("");
    sstream << "./output/out_" << filename;
    cv::imwrite(sstream.str(), image);
}

void ObjectDetect::DestroyResource()
{
    if (!isReleased_) {
        dvpp_.DestroyResource();
        model_.DestroyResource();
        isReleased_ = true;
    }

}
