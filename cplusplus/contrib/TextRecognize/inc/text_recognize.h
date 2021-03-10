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

#include "utils.h"
#include <iostream>
#include <mutex>
#include <unistd.h>
#include "acl/acl.h"
#include "model_process.h"
#include "dvpp_process.h"

#include <map>
#include <memory>
#include <jsoncpp/json/json.h>
//#include "presenter_channel.h"
//#include "HaweiUser/presenter/agent/presenter_types.h"
#include "presenter/agent/presenter_channel.h"
#include "presenter/agent/presenter_types.h"

//#include<fstream>
//#include<string>
//#include<sstream>


using namespace std;
using namespace ascend::presenter;
using namespace cv;

//enum outputImg {
//    layoutMap,
//    resultImg
//};
/**
* TextRecognizeProcess
*/
class TextRecongnize {
public:
    TextRecongnize(const char *OpenPose_modelPath, const char *Gesture_modelPath,
                   uint32_t firstModelWidth, uint32_t firstModelHeight,
                   uint32_t secondModelWidth, uint32_t secondModelHeight);

    ~TextRecongnize();



    Result Init();

    Result FirstModelPreprocess(cv::Mat& camera_rgb, ImageData & srcImage, cv::Mat &modelInputMat);



    Result Resize(ImageData& src, ImageData& dest,
                  uint32_t width, uint32_t height);
    Result CvtYuv420spToJpeg(ImageData& dest, ImageData& src);

    Result FirstModelInference(aclmdlDataset *&inferenceOutput, ImageData &resizedImg);

    Result FirstModelInference(aclmdlDataset *&inferenceOutput, cv::Mat &modelInputMat);

    Result
    FirstModelPostprocess(aclmdlDataset *modelOutput, cv::Mat camImg, cv::Mat &detectResImg, vector<cv::Mat> &cropArea,
                          vector<vector<Point2f>> &boxes, vector<Mat> &HMatrix);

    Result SecondModelPreprocess(cv::Mat &srcImage, cv::Mat &modelInputMat);

    Result SecondModelInference(aclmdlDataset *&inferenceOutput, cv::Mat &detectResImg);

    Result SecondModelPostprocess(aclmdlDataset *modelOutput, string &TextRes, cv::Mat &detectResImg,
                                  vector<cv::Point2f> &box);

    void getUnionBoxes(vector<vector<Point2f>> &unionBoxes, vector<vector<Point2f>> boxes);

    void fourPointsTransform(const cv::Mat &frame, cv::Point2f *vertices, cv::Mat &result, vector<Mat> &HMatrix);

    void PostProcessDBNet(float *outData, cv::Mat camImg, vector<vector<cv::Point2f>> &boxes);

    void sortVertices(vector<Point2f> &bbox);


    void bboxFromBitmap(Mat outputMap, Mat bitmap,
                        uint32_t destHeight, uint32_t destWidth,
                        vector<vector<Point2f>> &boxes,
                        vector<float> &scores,
                        uint32_t maxCandidates,
                        float box_thresh);

    vector<cv::Point> unclip(RotatedRect boundingBox, Point2f *vtx, float unclipRatio);

    float boxScoreFast(Mat outputMap, Point2f *vtx);
    void EncodeImage(vector<uint8_t>& encodeImg, cv::Mat& origImg);

    Result SendImage( cv::Mat& jpegImage);

private:
    Result InitResource();

    Result InitModel(const char *FirstModelPath, const char *SecondModelPath);
    Result OpenPresenterChannel();
    Result CreateImageInfoBuffer();
    void *GetInferenceOutputItem(uint32_t &itemDataSize,
                                 aclmdlDataset *inferenceOutput,
                                 uint32_t idx);

    void DrowBoundBoxToImage(vector<BBox> &detectionResults,
                             const string &origImagePath);

    void DestroyResource();

private:

    int32_t deviceId_;
    aclrtContext context_;
    aclrtStream stream_;
    uint32_t imageInfoSize_;
    void* imageInfoBuf_;
    ModelProcess firstModel_;
    ModelProcess secondModel_;

    const char *firstModelPath_;
    const char *secondModelPath_;
    uint32_t firstModelWidth_;
    uint32_t firstModelHeight_;
    uint32_t secondModelWidth_;
    uint32_t secondModelHeight_;
    uint32_t inputDataSize_;
    DvppProcess dvpp_;
    aclrtRunMode runMode_;

    Channel* channel_;
    bool isInited_;
    std::shared_ptr<Channel> chan_;

    //DataReceiver dataReceiver_;


};

