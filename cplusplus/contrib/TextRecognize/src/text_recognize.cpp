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
#include "text_recognize.h"
#include <cstddef>
#include <iostream>
#include <fstream>
#include <cstdint>
#include <sys/types.h>
#include "acl/acl.h"
#include "model_process.h"
#include "utils.h"
#include <cmath>
#include "clipper.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include<string.h>
#include<sstream>
#include <string>
#include "presenter/agent/presenter_channel.h"
#include "presenter/agent/presenter_types.h"


using namespace cv;
using namespace ClipperLib;
using namespace std;

TextRecongnize::TextRecongnize(const char *FirstModelPath,
                               const char *SecondModelPath,
                               uint32_t firstModelWidth,
                               uint32_t firstModelHeight,
                               uint32_t secondModelWidth,
                               uint32_t secondModelHeight)
        : deviceId_(0), context_(nullptr), stream_(nullptr), firstModelWidth_(firstModelWidth),
          firstModelHeight_(firstModelHeight), secondModelWidth_(secondModelWidth),
          secondModelHeight_(secondModelHeight), channel_(nullptr),isInited_(false) {
    imageInfoSize_ = 0;
    imageInfoBuf_ = nullptr;
    firstModelPath_ = FirstModelPath;
    secondModelPath_ = SecondModelPath;
    OpenPresenterChannel();
}

TextRecongnize::~TextRecongnize() {
    DestroyResource();
}

Result TextRecongnize::InitResource() {
    // ACL init
    const char *aclConfigPath = "../src/acl.json";
    aclError ret = aclInit(aclConfigPath);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl init failed");
        return FAILED;
    }
//    INFO_LOG("acl init success");

    // open device
    ret = aclrtSetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl open device %d failed", deviceId_);
        return FAILED;
    }
//    INFO_LOG("open device %d success", deviceId_);

    // create context (set current)
    ret = aclrtCreateContext(&context_, deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl create context failed");
        return FAILED;
    }
//    INFO_LOG("create context success");

    // create stream
    ret = aclrtCreateStream(&stream_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl create stream failed");
        return FAILED;
    }
//    INFO_LOG("create stream success");

    ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("acl get run mode failed");
        return FAILED;
    }

    return SUCCESS;
}

Result TextRecongnize::InitModel(const char *FirstModelPath, const char *SecondModelPath) {
    // load two om model
    Result ret = firstModel_.LoadModelFromFileWithMem(FirstModelPath);
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadFirstModelFromFileWithMem failed");
        return FAILED;
    }

    ret = secondModel_.LoadModelFromFileWithMem(SecondModelPath);
    if (ret != SUCCESS) {
        ERROR_LOG("execute LoadSecondModelFromFileWithMem failed");
        return FAILED;
    }

    ret = firstModel_.CreateDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateDesc failed");
        return FAILED;
    }

    ret = secondModel_.CreateDesc();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateDesc failed");
        return FAILED;
    }

    ret = firstModel_.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed");
        return FAILED;
    }

    ret = secondModel_.CreateOutput();
    if (ret != SUCCESS) {
        ERROR_LOG("execute CreateOutput failed");
        return FAILED;
    }
    return SUCCESS;
}


Result TextRecongnize::CreateImageInfoBuffer() {
    // 128 128 128 128

    cout << "firstModelWidth_ " << firstModelWidth_ << "firstModelHeight_ " << firstModelHeight_ << "firstModelWidth_ "
         << firstModelWidth_ << "firstModelHeight_ " << firstModelHeight_ << endl;
    const float imageInfo[4] = {(float) firstModelWidth_, (float) firstModelHeight_,
                                (float) firstModelWidth_, (float) firstModelHeight_};
    imageInfoSize_ = sizeof(imageInfo);
    if (runMode_ == ACL_HOST)
        imageInfoBuf_ = Utils::CopyDataHostToDevice((void *) imageInfo, imageInfoSize_);
    else
        imageInfoBuf_ = Utils::CopyDataDeviceToDevice((void *) imageInfo, imageInfoSize_);
    if (imageInfoBuf_ == nullptr) {
        ERROR_LOG("Copy image info to device failed");
        return FAILED;
    }

    return SUCCESS;
}

Result TextRecongnize::OpenPresenterChannel() {
    INFO_LOG("OpenChannel start");
    PresenterErrorCode errorCode = OpenChannelByConfig(channel_, "../data/param.conf");
    INFO_LOG("OpenChannel param");
    if (errorCode != PresenterErrorCode::kNone) {
        ERROR_LOG("OpenChannel failed %d", static_cast<int>(errorCode));
        return FAILED;
    }

    return SUCCESS;
}

Result TextRecongnize::Init() {
    if (isInited_) {
//        INFO_LOG("Classify instance is initied already!");
        return SUCCESS;
    }

    Result ret = InitResource();
    if (ret != SUCCESS) {
        ERROR_LOG("Init acl resource failed");
        return FAILED;
    }

    ret = InitModel(firstModelPath_, secondModelPath_);
    if (ret != SUCCESS) {
        ERROR_LOG("Init model failed");
        return FAILED;
    }
    ret = dvpp_.InitResource(stream_);

    if (ret != SUCCESS) {
        ERROR_LOG("Init dvpp failed\n");
        return FAILED;
    }

    //socket
    ret = CreateImageInfoBuffer();
    if (ret != SUCCESS) {
        ERROR_LOG("Create image info buf failed\n");
        return FAILED;
    }

    isInited_ = true;
    return SUCCESS;
}

// camre procrss
Result TextRecongnize::FirstModelPreprocess(cv::Mat &camera_rgb, ImageData &srcImage, cv::Mat &modelInputMat) {

    INFO_LOG("srcImage size is :%d * %d", srcImage.width, srcImage.height);
    //dvpp resize to model input
    ImageData resizedImage;
    Result ret = dvpp_.Resize(resizedImage, srcImage, firstModelWidth_, firstModelHeight_);

    // resize camre image
    //ImageData resizedImage;
    //cv::resize(srcImage, resizedImage, Size(firstModelWidth_, firstModelHeight_));
    if (ret == FAILED) {
        ERROR_LOG("Resize image failed\n");
        return FAILED;
    }
    INFO_LOG("Resize image success\n");
    INFO_LOG("resizedImage size is :%d * %d", resizedImage.width, resizedImage.height);

    //get yuv image
    cv::Mat yuvImg(resizedImage.height * 3 / 2, resizedImage.width, CV_8UC1, resizedImage.data.get());
    INFO_LOG(" yuvImg height: %d ; yuvImg width: %d", resizedImage.height, resizedImage.width);

    //convert to rgb image
    cv::Mat rgbImg;
    cv::cvtColor(yuvImg, rgbImg, CV_YUV420sp2RGB);
    INFO_LOG(" rgbImg height: %d ; rgbImg width: %d", rgbImg.rows, rgbImg.cols);
    INFO_LOG("rgbImg CV_IMAGE_TYPE: %d", rgbImg.type());
    rgbImg.copyTo(camera_rgb);

//    Utils::SaveCVImage("input_rgb_img.png", rgbImg);      // save rgb image to local

    cv::Mat tempImg;
    rgbImg.convertTo(tempImg, CV_32FC3);

    std::vector<float> mean_value{123.68, 116.78, 103.94};
    std::vector<float> std_value{1, 1, 1};
    std::vector<cv::Mat> bgrChannels(3);
    cv::split(tempImg, bgrChannels);
    for (auto i = 0; i < bgrChannels.size(); i++) {
        bgrChannels[i].convertTo(bgrChannels[i], CV_32FC1, 1.0 / std_value[i], (0.0 - mean_value[i]) / std_value[i]);
    }
    cv::merge(bgrChannels, modelInputMat);


    return SUCCESS;
}

// detect推理函数
Result TextRecongnize::FirstModelInference(aclmdlDataset *&inferenceOutput, ImageData &resizedImg) {
    Result ret = firstModel_.CreateInput(resizedImg.data.get(), resizedImg.size);
    if (ret != SUCCESS) {
        ERROR_LOG("Create mode input dataset failed");
        return FAILED;
    }

    ret = firstModel_.Execute();
    if (ret != SUCCESS) {
        ERROR_LOG("Execute model inference failed");
        return FAILED;
    }

    inferenceOutput = firstModel_.GetModelOutputData();

    return SUCCESS;
}

// detect推理函数
Result TextRecongnize::FirstModelInference(aclmdlDataset *&inferenceOutput, cv::Mat &modelInputMat) {
    Result ret = firstModel_.CreateInput(modelInputMat.data,
                                         modelInputMat.rows * modelInputMat.cols * (int) modelInputMat.elemSize());
    //cout << "create input sucess" << endl;
    if (ret != SUCCESS) {
        ERROR_LOG("Create mode input dataset failed");
        return FAILED;
    }

    ret = firstModel_.Execute();
    if (ret != SUCCESS) {
        ERROR_LOG("Execute model inference failed");
        return FAILED;
    }

    inferenceOutput = firstModel_.GetModelOutputData();
    //cout << "inference output sucess" << endl;
    return SUCCESS;
}

// 处理输出的结果
Result TextRecongnize::FirstModelPostprocess(aclmdlDataset *modelOutput, Mat camImg, Mat &detectResImg,
                                             vector<Mat> &cropAreas, vector<vector<Point2f>> &boxes,
                                             vector<Mat> &HMatrix) {
    for (size_t index = 0; index < aclmdlGetDatasetNumBuffers(modelOutput); ++index) {
        aclDataBuffer *dataBuffer = aclmdlGetDatasetBuffer(modelOutput, index);
        void *data = aclGetDataBufferAddr(dataBuffer);
        uint32_t len = aclGetDataBufferSize(dataBuffer);
        printf ("model 1 buffer length: %d \n",len);

        void *outHostData = NULL;
        aclError ret = ACL_ERROR_NONE;
        float *outData = NULL;
        if (runMode_ == ACL_HOST) {
            aclError ret = aclrtMallocHost(&outHostData, len);
            if (ret != ACL_ERROR_NONE) {
                ERROR_LOG("aclrtMallocHost failed, ret[%d]", ret);
                return FAILED;
            }

            ret = aclrtMemcpy(outHostData, len, data, len, ACL_MEMCPY_DEVICE_TO_HOST);
            if (ret != ACL_ERROR_NONE) {
                ERROR_LOG("aclrtMemcpy failed, ret[%d]", ret);
                return FAILED;
            }

            outData = reinterpret_cast<float *>(outHostData);
        } else {
            outData = reinterpret_cast<float *>(data);
        }
        if (index == 0) {
            camImg.copyTo(detectResImg);
            PostProcessDBNet(outData, camImg, boxes);

            for (auto bbox : boxes) {
                Point2f pts[bbox.size()];
                memcpy(pts, &bbox[0], bbox.size() * sizeof(bbox[0]));

                line(detectResImg, bbox[0], bbox[1], Scalar(0, 0, 255), 2);
                line(detectResImg, bbox[1], bbox[2], Scalar(0, 0, 255), 2);
                line(detectResImg, bbox[2], bbox[3], Scalar(0, 0, 255), 2);
                line(detectResImg, bbox[3], bbox[0], Scalar(0, 0, 255), 2);

                Mat result;
                fourPointsTransform(camImg, pts, result, HMatrix);

                cropAreas.push_back(result);
            }

        } else if (index == 1) {
            cout << "ERROR" << endl;
        }

        if (runMode_ == ACL_HOST) {
            ret = aclrtFreeHost(outHostData);
            if (ret != ACL_ERROR_NONE) {
                ERROR_LOG("aclrtFreeHost failed, ret[%d]", ret);
                return FAILED;
            }
        }
    }

    INFO_LOG(" First model output data success");
//    INFO_LOG("---------------------------------");
    return SUCCESS;
}

Result TextRecongnize::SecondModelPreprocess(cv::Mat &srcImage, cv::Mat &modelInputMat) {
    Result ret = srcImage.empty() ? FAILED : SUCCESS;
    if (ret != SUCCESS) {
        ERROR_LOG("Source image is empty!");
        return FAILED;
    }

//    imwrite("./output/srcImage.bmp", srcImage);

    cv::Mat resizedImg;
    cv::resize(srcImage, resizedImg, Size(secondModelWidth_, secondModelHeight_));
    cout << "secondModelWidth" << endl;
    cout << secondModelWidth_ << endl;
    cout << secondModelHeight_ << endl;
    cout << "resizeimage" << endl;
    cout << resizedImg.size().width << resizedImg.size().height << endl;
//    imwrite("./output/resizedImg.bmp", resizedImg);

    cv::Mat bgr8Img, tempImg;
    cv::cvtColor(resizedImg, bgr8Img, COLOR_RGB2BGR);
    bgr8Img.convertTo(tempImg, CV_32FC3);
//    imwrite("./output/tempImg.bmp", tempImg);
    modelInputMat = tempImg / 127.5 - 1;

    cout << bgr8Img.size().width << bgr8Img.size().height << endl;
    cout << tempImg.size().width << tempImg.size().height << endl;
    cout << modelInputMat.size().width << modelInputMat.size().height << endl;
//    imwrite("./output/modelInputMat.bmp", modelInputMat);

    return SUCCESS;
}

// recognize推理函数
Result TextRecongnize::SecondModelInference(aclmdlDataset *&inferenceOutput, Mat &secondModelInputMat) {

    Result ret = secondModel_.CreateInput(secondModelInputMat.data,
                                          secondModelInputMat.rows * secondModelInputMat.cols *
                                          (int) secondModelInputMat.elemSize());
    if (ret != SUCCESS) {
        ERROR_LOG("Create mode input dataset failed");
        return FAILED;
    }

    ret = secondModel_.Execute();
    if (ret != SUCCESS) {
        ERROR_LOG("Execute model inference failed");
        return FAILED;
    }

    inferenceOutput = secondModel_.GetModelOutputData();

    return SUCCESS;
}

Result TextRecongnize::SecondModelPostprocess(aclmdlDataset *modelOutput, string &TextRes, cv::Mat &detectResImg,
                                              vector<cv::Point2f> &box) {
    for (size_t index = 0; index < aclmdlGetDatasetNumBuffers(modelOutput); ++index) {
        aclDataBuffer *dataBuffer = aclmdlGetDatasetBuffer(modelOutput, index);
        void *data = aclGetDataBufferAddr(dataBuffer);
        uint32_t len = aclGetDataBufferSize(dataBuffer);

        void *outHostData = NULL;
        aclError ret = ACL_ERROR_NONE;
        int64_t *outData = NULL;
        if (runMode_ == ACL_HOST) {
            aclError ret = aclrtMallocHost(&outHostData, len);
            if (ret != ACL_ERROR_NONE) {
                ERROR_LOG("aclrtMallocHost failed, ret[%d]", ret);
                return FAILED;
            }

            ret = aclrtMemcpy(outHostData, len, data, len, ACL_MEMCPY_DEVICE_TO_HOST);
            if (ret != ACL_ERROR_NONE) {
                ERROR_LOG("aclrtMemcpy failed, ret[%d]", ret);
                return FAILED;
            }

            outData = reinterpret_cast<int64_t *>(outHostData);
        } else {
            outData = reinterpret_cast<int64_t *>(data);
        }

        if (index == 0) {
            std:
            string text_res;
            // load json dict
            Json::Value ord_map, char_dict;
            Json::Reader reader;
            std::ifstream ifs_ord ("../data/ord_map_en.json");

            if (!reader.parse(ifs_ord, ord_map)) {
                std::cout << "Read ord_map failed..." << std::endl;
            }
            std::ifstream ifs_char ("../data/char_dict_en.json");
            if (!reader.parse(ifs_char, char_dict)) {
                std::cout << "Read char_dict failed..." << std::endl;
            }

            // seq decode
            int64_t *seq_prob = outData;
            size_t seq_len = len / 4;
            int64_t last_char = -1;
            for (size_t seq_idx = 0; seq_idx < seq_len; ++seq_idx) {
                int64_t cur_char = *(seq_prob + seq_idx);
                if (cur_char == 36) {
                    last_char = -1;
                    continue;
                } else {
                    if (last_char == cur_char) {
                        continue;
                    } else {
                        std::string tmp_str = ord_map[std::to_string(cur_char) + "_index"].asString();
                        text_res += char_dict[tmp_str + "_ord"].asString();
                        last_char = cur_char;
                    }
                }
            }
            TextRes = text_res;
            cv::putText(detectResImg, TextRes, cv::Point(box[3].x, box[3].y + 30), CV_FONT_HERSHEY_SIMPLEX, 0.5,
                        Scalar(0, 0, 255),
                        2, LINE_8);
        }
        if (runMode_ == ACL_HOST) {
            ret = aclrtFreeHost(outHostData);
            if (ret != ACL_ERROR_NONE) {
                ERROR_LOG("aclrtFreeHost failed, ret[%d]", ret);
                return FAILED;
            }
        }
    }

    INFO_LOG("Second model output data success");
//    INFO_LOG("---------------------------------");
    return SUCCESS;
}

int find_index(vector<int>::iterator begin, vector<int>::iterator end, int element) {
    auto temp = begin;
    while (temp != end) {
        if (*temp == element) {
            return element;
        }
        temp += 1;
    }
    return -1;
}

void TextRecongnize::getUnionBoxes(vector<vector<Point2f>> &unionBoxes, vector<vector<Point2f>> boxes) {
    while (boxes.size() > 0) {
        vector<Point2f> curr = boxes[boxes.size() - 1];
        boxes.pop_back();
        Point2f p1BottomRight = curr[2];
        Point2f p1BottomLeft = curr[3];
        for (int i = 0; i < boxes.size(); i++) {
            vector<Point2f> bbox = boxes[i];
            Point2f p2BottomRight = bbox[2];
            Point2f p2BottomLeft = bbox[3];
            float distance1 = sqrtf(
                    powf((p1BottomRight.x - p2BottomLeft.x), 2) + powf((p1BottomRight.y - p2BottomLeft.y), 2));
            float distance2 = sqrtf(
                    powf((p1BottomLeft.x - p2BottomRight.x), 2) + powf((p1BottomLeft.y - p2BottomRight.y), 2));
            if (distance1 / distance2 < 0.1 || distance2 / distance1 < 0.1) {
                vector<cv::Point> contour(curr.begin(), curr.end());
                contour.insert(contour.end(), bbox.begin(), bbox.end());
                Point2f vtx[4];

                RotatedRect boundingBox = minAreaRect(contour);
                boundingBox.points(vtx);
                vector<Point2f> newBbox(vtx, vtx + 4);

                sortVertices(newBbox);
                curr = newBbox;
                boxes.erase(boxes.begin() + i);
                i--;
            }
        }
        unionBoxes.push_back(curr);
    }
}

bool LessSort(Point2f a, Point2f b) {
    return (a.x < b.x);
}

void TextRecongnize::sortVertices(vector<Point2f> &bbox) {
    std::sort(bbox.begin(), bbox.end(), LessSort);
    int index_1 = 0;
    int index_2 = 1;
    int index_3 = 2;
    int index_4 = 3;

    if (bbox[1].y > bbox[0].y) {
        index_1 = 0;
        index_4 = 1;
    } else {
        index_1 = 1;
        index_4 = 0;
    }
    if (bbox[3].y > bbox[2].y) {
        index_2 = 2;
        index_3 = 3;
    } else {
        index_2 = 3;
        index_3 = 2;
    }

    vector<Point2f> orderedBox;
    orderedBox.push_back(bbox[index_1]);
    orderedBox.push_back(bbox[index_2]);
    orderedBox.push_back(bbox[index_3]);
    orderedBox.push_back(bbox[index_4]);
    bbox = orderedBox;
}

void TextRecongnize::fourPointsTransform(const Mat &frame, Point2f *vertices, Mat &result, vector<Mat> &HMatrix) {
    const Size outputSize = Size(secondModelWidth_, secondModelHeight_);
    Point2f targetVertices[4] = {cv::Point(0, 0), cv::Point(outputSize.width - 1, 0),
                                 cv::Point(outputSize.width - 1, outputSize.height - 1),
                                 cv::Point(0, outputSize.height - 1)};

    Mat rotationMatrix = getPerspectiveTransform(vertices, targetVertices);
    HMatrix.push_back(rotationMatrix);
    warpPerspective(frame, result, rotationMatrix, outputSize);
}

void TextRecongnize::PostProcessDBNet(float *outData, Mat camImg, vector<vector<Point2f>> &boxes) {
    Mat outputMap(firstModelHeight_, firstModelWidth_, CV_32FC1, const_cast<float_t *>((float_t *) outData));
    Mat bitmap = Mat(outputMap > 0.3);
    vector<float> scores;
    bboxFromBitmap(outputMap, bitmap, camImg.rows, camImg.cols, boxes, scores, 1000, 0.7f);
}


void TextRecongnize::bboxFromBitmap(Mat outputMap, Mat bitmap,
                                    uint32_t destHeight, uint32_t destWidth,
                                    vector<vector<Point2f>> &boxes,
                                    vector<float> &scores,
                                    uint32_t maxCandidates = 1000,
                                    float box_thresh = 0.5) {
    uint32_t width = outputMap.cols;
    uint32_t height = outputMap.rows;
    bitmap = bitmap * 255;
    bitmap.convertTo(bitmap, CV_8UC1);
//    imwrite("output/bitmap.bmp",bitmap);
    vector<vector<cv::Point>> contours;
    findContours(bitmap, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

    uint32_t num_contours = min(uint32_t(contours.size()), maxCandidates);

    for (uint32_t index = 0; index < num_contours; index++) {
        vector<cv::Point> contour = contours[index];
        Point2f vtx[4];
        RotatedRect boundingBox = minAreaRect(contour);
        boundingBox.points(vtx);

        float sside = min(boundingBox.size.height, boundingBox.size.width);
        if (sside < 3) {
            continue;
        }
        float score = boxScoreFast(outputMap, vtx);
        if (score < box_thresh) {
            continue;
        }
        vector<cv::Point> tmp = unclip(boundingBox, vtx, 1.5);
        boundingBox = minAreaRect(tmp);
        boundingBox.points(vtx);


        sside = min(boundingBox.size.height, boundingBox.size.width);
        if (sside < 5) {
            continue;
        }

        for (int i = 0; i < 4; i++) {
            vtx[i].x = min(max(round(vtx[i].x / width * destWidth), 0.0f), destWidth - 1.0f);
            vtx[i].y = min(max(round(vtx[i].y / height * destHeight), 0.0f), destHeight - 1.0f);
        }

        vector<Point2f> ans(vtx, vtx + 4);
        sortVertices(ans);

        boxes.push_back(ans);
        scores.push_back(score);
    }
}

vector<cv::Point> TextRecongnize::unclip(RotatedRect boundingBox, Point2f *vtx, float unclipRatio = 1.5) {
    float area = boundingBox.size.height * boundingBox.size.width;
    float length = 2 * (boundingBox.size.height + boundingBox.size.width);
    float distance = area * unclipRatio / length;

    ClipperOffset co;
    Path path;
    Paths expand;
    for (int i = 0; i < 4; i++) {
        IntPoint ip;
        ip.X = vtx[i].x;
        ip.Y = vtx[i].y;
        path.push_back(ip);
    }

    co.AddPath(path, jtRound, etClosedPolygon);
    co.Execute(expand, distance);

    vector<cv::Point> ans;
    for (auto ip: expand[0]) {
        cv::Point pt = cv::Point(ip.X, ip.Y);
        ans.push_back(pt);
    }
    return ans;
}

float TextRecongnize::boxScoreFast(Mat outputMap, Point2f *vtx) {
    uint32_t height = outputMap.rows;
    uint32_t width = outputMap.cols;

    Mat mask = Mat::zeros(outputMap.size(), CV_8UC1);
    vector<cv::Point> pts(vtx, vtx + 4);
    vector<vector<cv::Point>> tmp;
    tmp.push_back(pts);

    fillPoly(mask, tmp, 1);
    return cv::mean(outputMap, mask)[0];
}

void *TextRecongnize::GetInferenceOutputItem(uint32_t &itemDataSize, aclmdlDataset *inferenceOutput, uint32_t idx) {
//    printf("get output id %d\n", idx);
    aclDataBuffer *dataBuffer = aclmdlGetDatasetBuffer(inferenceOutput, idx);
    if (dataBuffer == nullptr) {
        ERROR_LOG("Get the %dth dataset buffer from model "
                  "inference output failed", idx);
        return nullptr;
    }

    void *dataBufferDev = aclGetDataBufferAddr(dataBuffer);
    if (dataBufferDev == nullptr) {
        ERROR_LOG("Get the %dth dataset buffer address "
                  "from model inference output failed", idx);
        return nullptr;
    }

    size_t bufferSize = aclGetDataBufferSize(dataBuffer);
    if (bufferSize == 0) {
        ERROR_LOG("The %dth dataset buffer size of "
                  "model inference output is 0", idx);
        return nullptr;
    }

    void *data = nullptr;
    if (runMode_ == ACL_HOST) {
        data = Utils::CopyDataDeviceToLocal(dataBufferDev, bufferSize);
        if (data == nullptr) {
            ERROR_LOG("Copy inference output to host failed");
            return nullptr;
        }
    } else {
        data = dataBufferDev;
    }

    itemDataSize = bufferSize;
    return data;
}

void TextRecongnize::DestroyResource() {

    firstModel_.DestroyResource();

    aclError ret;
    if (stream_ != nullptr) {
        ret = aclrtDestroyStream(stream_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy stream failed");
        }
        stream_ = nullptr;
    }
//    INFO_LOG("end to destroy stream");

    if (context_ != nullptr) {
        ret = aclrtDestroyContext(context_);
        if (ret != ACL_ERROR_NONE) {
            ERROR_LOG("destroy context failed");
        }
        context_ = nullptr;
    }
//    INFO_LOG("end to destroy context");

    ret = aclrtResetDevice(deviceId_);
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("reset device failed");
    }
//    INFO_LOG("end to reset device is %d", deviceId_);

    ret = aclFinalize();
    if (ret != ACL_ERROR_NONE) {
        ERROR_LOG("finalize acl failed");
    }
//    INFO_LOG("end to finalize acl");
//    aclrtFree(imageInfoBuf_);
}


void TextRecongnize::EncodeImage(vector<uint8_t> &encodeImg, cv::Mat &origImg) {
    vector<int> param = vector<int>(2);
    param[0] = cv::IMWRITE_JPEG_QUALITY;
    param[1] = 100;//default(95) 0-100

    cv::imencode(".jpg", origImg, encodeImg, param);
}


Result TextRecongnize::SendImage( cv::Mat &image) {

    //add
    vector<uint8_t> encodeImg;
    EncodeImage(encodeImg, image);

    //ascend::presenter::ImageFrame frame;
    ImageFrame imageParam;
    imageParam.format = ImageFormat::kJpeg;
    imageParam.width = image.cols;
    imageParam.height = image.rows;
    imageParam.size = encodeImg.size();
    imageParam.data = reinterpret_cast<uint8_t*>(encodeImg.data());

    ascend::presenter::PresenterErrorCode ret = PresentImage(channel_, imageParam);
    // send to presenter failed
    if (ret != PresenterErrorCode::kNone) {
        ERROR_LOG("Send JPEG image to presenter failed, error %d\n", (int) ret);
        return FAILED;
    }

//    INFO_LOG("Send JPEG image to presenter success, ret %d,num =%d \n", (int)ret,gSendNum++);

    return SUCCESS;
}

