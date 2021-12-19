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
#include "acllite/AclLiteModel.h"
#include "acllite/AclLiteImageProc.h"

#include "opencv2/opencv.hpp"

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

    AclLiteError Init();
    AclLiteError Process(ImageData& image, aclrtRunMode RunMode);
    void DestroyResource();

private:
    AclLiteError FirstModelPreprocess(cv::Mat &camera_rgb, ImageData &srcImage, cv::Mat &modelInputMat, aclrtRunMode RunMode);

    AclLiteError FirstModelInference(std::vector<InferenceOutput>& inferOutputs,
                         cv::Mat &modelInputMat);
    
    AclLiteError FirstModelPostprocess(std::vector<InferenceOutput>& firstmodelinferOutputs, 
                        cv::Mat rgbImg, cv::Mat &detectResImg, vector<cv::Mat> &cropArea,
                        vector<vector<Point2f>> &boxes, vector<Mat> &HMatrix);
    
    AclLiteError SecondModelPreprocess(cv::Mat &srcImage, cv::Mat &modelInputMat);
    
    AclLiteError SecondModelInference(std::vector<InferenceOutput>& inferOutputs, cv::Mat &detectResImg);

    AclLiteError SecondModelPostprocess(std::vector<InferenceOutput>& inferOutputs, string &TextRes, cv::Mat &detectResImg,
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

    AclLiteError SendImage(cv::Mat& jpegImage);

    AclLiteError CopyImageToDvpp(ImageData &srcImage, 
                            aclrtRunMode runMode);

    AclLiteError CopyMatToDevice(cv::Mat &srcMat, 
                            aclrtRunMode runMode);

    AclLiteError CopyImageFromDvpp(ImageData &srcImage, 
                            aclrtRunMode runMode);

    void EncodeImage(vector<uint8_t>& encodeImg, cv::Mat& origImg);
                             
private:
    AclLiteModel FirstModel_;
    AclLiteModel SecondModel_;
    uint32_t firstModelWidth_;
    uint32_t firstModelHeight_;
    uint32_t secondModelWidth_;
    uint32_t secondModelHeight_;
    AclLiteImageProc dvpp_;    
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