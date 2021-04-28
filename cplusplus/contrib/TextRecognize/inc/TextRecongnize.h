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

* File sample_process.h
* Description: handle acl resource
*/
#pragma once

#include <iostream>
#include <mutex>
#include <unistd.h>
#include <vector>
#include <map>
#include <memory>
#include <jsoncpp/json/json.h>
#include "presenter/agent/presenter_channel.h"
#include "presenter/agent/presenter_types.h"

#include "acl/acl.h"
#include "atlasutil/atlas_model.h"
#include "atlasutil/dvpp_process.h"

#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs/legacy/constants_c.h"
#include "opencv2/imgproc/types_c.h"
#include "clipper.hpp"

using namespace std;
using namespace ascend::presenter;
using namespace cv;
using namespace ClipperLib;

class TextRecongnize {
public:
    TextRecongnize();
    ~TextRecongnize();

    AtlasError Init();
    AtlasError Process(ImageData& image, aclrtRunMode RunMode);
    void DestroyResource();

private:
    AtlasError FirstModelPreprocess(cv::Mat &camera_rgb, ImageData &srcImage, cv::Mat &modelInputMat, aclrtRunMode RunMode);

    AtlasError FirstModelInference(std::vector<InferenceOutput>& inferOutputs,
                         cv::Mat &modelInputMat);
    
    AtlasError FirstModelPostprocess(std::vector<InferenceOutput>& firstmodelinferOutputs, 
                        cv::Mat rgbImg, cv::Mat &detectResImg, vector<cv::Mat> &cropArea,
                        vector<vector<Point2f>> &boxes, vector<Mat> &HMatrix);
    
    AtlasError SecondModelPreprocess(cv::Mat &srcImage, cv::Mat &modelInputMat);
    
    AtlasError SecondModelInference(std::vector<InferenceOutput>& inferOutputs, cv::Mat &detectResImg);

    AtlasError SecondModelPostprocess(std::vector<InferenceOutput>& inferOutputs, string &TextRes, cv::Mat &detectResImg,
                                  vector<cv::Point2f> &box);

    void PostProcessDBNet(float *outData, cv::Mat rgbImg, vector<vector<cv::Point2f>> &boxes);

    void bboxFromBitmap(Mat outputMap, Mat bitmap,
                    uint32_t destHeight, uint32_t destWidth,
                    vector<vector<Point2f>> &boxes,
                    vector<float> &scores,
                    uint32_t maxCandidates,
                    float box_thresh);
    
    float boxScoreFast(Mat outputMap, Point2f *vtx);

    vector<cv::Point> unclip(RotatedRect boundingBox, Point2f *vtx, float unclipRatio);

    void sortVertices(vector<Point2f> &bbox);

    void fourPointsTransform(const cv::Mat &frame, cv::Point2f *vertices, cv::Mat &result, vector<Mat> &HMatrix);

    AtlasError SendImage(cv::Mat& jpegImage);

    AtlasError CopyImageToDvpp(ImageData &srcImage, 
                            aclrtRunMode runMode);

    AtlasError CopyMatToDevice(cv::Mat &srcMat, 
                            aclrtRunMode runMode);

    AtlasError CopyImageFromDvpp(ImageData &srcImage, 
                            aclrtRunMode runMode);

    void EncodeImage(vector<uint8_t>& encodeImg, cv::Mat& origImg);
                             
private:
    AtlasModel FirstModel_;
    AtlasModel SecondModel_;
    uint32_t firstModelWidth_;
    uint32_t firstModelHeight_;
    uint32_t secondModelWidth_;
    uint32_t secondModelHeight_;
    DvppProcess dvpp_;    
    ascend::presenter::Channel* presenterChannel_;

    bool isInited_;
    bool isReleased_;
    
    cv::Mat frame_rgb;
    cv::Mat detectResImg = cv::Mat(cv::Size(firstModelWidth_, firstModelHeight_), CV_8UC3, cv::Scalar(255, 255, 255));
    vector<cv::Mat> cropAreas;
    vector<vector<cv::Point2f>> boxes;
    string textRes;
    vector<cv::Mat> hMatrix;
};