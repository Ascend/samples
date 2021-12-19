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
#include <iostream>
#include <fstream>
#include "acl/acl.h"
#include "acllite/AclLiteModel.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "TextRecongnize.h"

using namespace std;
using namespace ascend::presenter;
using namespace cv;

namespace {
const uint32_t firstModelWidth = 1312;
const uint32_t firstModelHeight = 736;
const uint32_t secondModelWidth = 100;
const uint32_t secondModelHeight = 32;
const string firstModelPath = "../model/dbnet.om";
const string secondModelPath = "../model/crnn_static.om";
const string kConfigFile = "../scripts/TextRecongnize.conf";
}

TextRecongnize::TextRecongnize(): 
FirstModel_(firstModelPath),
SecondModel_(secondModelPath),
presenterChannel_(nullptr),
isInited_(false), 
isReleased_(false),
firstModelWidth_(firstModelWidth),
firstModelHeight_(firstModelHeight),
secondModelWidth_(secondModelWidth),
secondModelHeight_(secondModelHeight){
}

TextRecongnize::~TextRecongnize() {
    DestroyResource();
}

AclLiteError TextRecongnize::Init() {
    if (isInited_) {
        ACLLITE_LOG_INFO("Text Recongnize is initied already");
        return ACLLITE_OK;
    }
    ACLLITE_LOG_INFO("Text Recongnize start initied ");

    PresenterErrorCode ret = OpenChannelByConfig(presenterChannel_, 
                                                 kConfigFile.c_str());
    if (ret != PresenterErrorCode::kNone) {
        ACLLITE_LOG_ERROR("Open channel failed, error %d", (int)ret);
        return ACLLITE_ERROR;
    }
    ACLLITE_LOG_INFO("Text Recongnize Open Channel ");

    AclLiteError atlRet = dvpp_.Init();
    if (atlRet) {
        ACLLITE_LOG_ERROR("Dvpp init failed, error %d", atlRet);
        return ACLLITE_ERROR;
    }
    ACLLITE_LOG_INFO("dvpp intialiaze ");

    atlRet = FirstModel_.Init();
    if (atlRet) {
        ACLLITE_LOG_ERROR("Model dbnet init failed, error %d", atlRet);
        return ACLLITE_ERROR;
    }
    ACLLITE_LOG_INFO("Model dbnet intialiaze ");

    atlRet = SecondModel_.Init();
    if (atlRet) {
        ACLLITE_LOG_ERROR("Model crnn_static init failed, error %d", atlRet);
        return ACLLITE_ERROR;
    }
    ACLLITE_LOG_INFO("Model crnn_static intialiaze ");

    isInited_ = true;
    return ACLLITE_OK;
}

AclLiteError TextRecongnize::Process(ImageData& image, aclrtRunMode runMode) {
    cv::Mat firstModelInputMat;
    AclLiteError ret = CopyImageToDvpp(image, runMode);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Copy Image To Dvpp failed");
        return ACLLITE_ERROR;
    }
    ret = FirstModelPreprocess(frame_rgb, image, firstModelInputMat, runMode);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("First model preprocess failed, continue to read next\n");
        return ACLLITE_ERROR;
    }

    CopyMatToDevice(firstModelInputMat, runMode);
    std::vector<InferenceOutput> firstModelInferOutputs;
    ret = FirstModelInference(firstModelInferOutputs, firstModelInputMat);
    if (ret) {
        ACLLITE_LOG_ERROR("First model inference image failed");
        return ACLLITE_ERROR;        
    }
    
    ret = FirstModelPostprocess(firstModelInferOutputs, frame_rgb, detectResImg, cropAreas, boxes, hMatrix);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Process first model inference output data failed");
        return ACLLITE_ERROR;
    }
    for (int index = 0; index < cropAreas.size(); index++) {
        cv::Mat secondModelInputMat;
        ret = SecondModelPreprocess(cropAreas[index], secondModelInputMat);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("SecondModelPreprocess failed, continue to process next");
            return ACLLITE_ERROR;
        }
        CopyMatToDevice(secondModelInputMat, runMode);
        std::vector<InferenceOutput> secondModelInferOutputs;
        ret = SecondModelInference(secondModelInferOutputs, secondModelInputMat); 
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Inference second model failed");
            return ACLLITE_ERROR;
        }
        ret = SecondModelPostprocess(secondModelInferOutputs, textRes, detectResImg, boxes[index]);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Process second model inference output data failed");
            return ACLLITE_ERROR;
        }
    }
    ret = SendImage(detectResImg);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("SendImage failed");
        return ACLLITE_ERROR;
    }
    cropAreas.clear();
    boxes.clear();
    hMatrix.clear();
    return ACLLITE_OK;
}

AclLiteError TextRecongnize::FirstModelPreprocess(cv::Mat &camera_rgb, ImageData &srcImage, 
                                            cv::Mat &modelInputMat, aclrtRunMode runMode) {
    ImageData resizedImage;
    AclLiteError ret = dvpp_.Resize(resizedImage, srcImage, firstModelWidth_, firstModelHeight_);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Resize image failed\n");
        return ACLLITE_ERROR;
    }
    ret = CopyImageFromDvpp(resizedImage, runMode);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Execute model inference failed");
        return ACLLITE_ERROR;
    }
    cv::Mat yuvImg(resizedImage.height * 3 / 2, resizedImage.width, CV_8UC1, resizedImage.data.get());
    cv::Mat rgbImg;
    cv::cvtColor(yuvImg, rgbImg, CV_YUV420sp2RGB);
    rgbImg.copyTo(camera_rgb);
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
    return ACLLITE_OK;
}

AclLiteError TextRecongnize::FirstModelInference(std::vector<InferenceOutput>& inferOutputs,
                                 cv::Mat &modelInputMat) {
    AclLiteError ret = FirstModel_.CreateInput(modelInputMat.data,
                                         modelInputMat.rows * modelInputMat.cols * 
                                         (int) modelInputMat.elemSize());
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed\n");
        return ACLLITE_ERROR;
    }

    ret = FirstModel_.Execute(inferOutputs);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Execute model inference failed\n");
    }
    FirstModel_.DestroyInput();
    return ret;
}

AclLiteError TextRecongnize::FirstModelPostprocess(std::vector<InferenceOutput>& firstmodelinferOutputs, 
                                             Mat rgbImg, Mat &detectResImg,
                                             vector<Mat> &cropAreas, vector<vector<Point2f>> &boxes,
                                             vector<Mat> &hMatrix) {
        for (size_t index = 0; index < firstmodelinferOutputs.size(); ++index) {
        if (index == 0) {
            rgbImg.copyTo(detectResImg);
            float * tmpdata = (float *)firstmodelinferOutputs[index].data.get();
            PostProcessDBNet(tmpdata, rgbImg, boxes);
            for (auto bbox : boxes) {
                Point2f pts[bbox.size()];
                memcpy(pts, &bbox[0], bbox.size() * sizeof(bbox[0]));
                line(detectResImg, bbox[0], bbox[1], Scalar(0, 0, 255), 2);
                line(detectResImg, bbox[1], bbox[2], Scalar(0, 0, 255), 2);
                line(detectResImg, bbox[2], bbox[3], Scalar(0, 0, 255), 2);
                line(detectResImg, bbox[3], bbox[0], Scalar(0, 0, 255), 2);
                Mat result;
                fourPointsTransform(rgbImg, pts, result, hMatrix);
                cropAreas.push_back(result);
            }

        } else if (index == 1) {
            cout << "ERROR" << endl;
        }
    }
    return ACLLITE_OK;
}

AclLiteError TextRecongnize::SecondModelPreprocess(cv::Mat &srcImage, cv::Mat &modelInputMat) {
    AclLiteError ret = srcImage.empty() ? ACLLITE_ERROR : ACLLITE_OK;
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Source image is empty!");
        return ACLLITE_ERROR;
    }

    cv::Mat resizedImg;
    cv::resize(srcImage, resizedImg, Size(secondModelWidth_, secondModelHeight_));
    cv::Mat bgr8Img, tempImg;
    cv::cvtColor(resizedImg, bgr8Img, COLOR_RGB2BGR);
    bgr8Img.convertTo(tempImg, CV_32FC3);
    modelInputMat = tempImg / 127.5 - 1;
    return ACLLITE_OK;
}


// recognize推理函数
AclLiteError TextRecongnize::SecondModelInference(std::vector<InferenceOutput>& inferOutputs, Mat &secondModelInputMat) {
    AclLiteError ret = SecondModel_.CreateInput(secondModelInputMat.data,
                                          secondModelInputMat.rows * secondModelInputMat.cols *
                                          (int) secondModelInputMat.elemSize());
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed");
        return ACLLITE_ERROR;
    }
    ret = SecondModel_.Execute(inferOutputs);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Execute model inference failed");
        return ACLLITE_ERROR;
    }
    return ACLLITE_OK;
}

AclLiteError TextRecongnize::SecondModelPostprocess(std::vector<InferenceOutput>& inferOutputs, string &textRes, 
                                                cv::Mat &detectResImg, vector<cv::Point2f> &box) {
    size_t outputnum = inferOutputs.size();
    for (size_t index = 0; index < outputnum; ++index) {
        if (index == 0) {
            std:
            string text_res;
            // load json dict
            Json::Value ord_map, char_dict;
            Json::Reader reader;
            std::ifstream ifs_ord("../data/ord_map_en.json");
            if (!reader.parse(ifs_ord, ord_map)) {
                std::cout << "Read ord_map failed..." << std::endl;
            }
            std::ifstream ifs_char("../data/char_dict_en.json");
            if (!reader.parse(ifs_char, char_dict)) {
                std::cout << "Read char_dict failed..." << std::endl;
            }

            // seq decode
            int64_t *seq_prob = (int64_t *)inferOutputs[index].data.get();
            size_t seq_len = inferOutputs[index].size / 8;
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
            textRes = text_res;
            cv::putText(detectResImg, textRes, cv::Point(box[3].x, box[3].y + 30), CV_FONT_HERSHEY_SIMPLEX, 0.5,
                        Scalar(0, 0, 255),
                        2, LINE_8);
        }
    }
    return ACLLITE_OK;
}

void TextRecongnize::PostProcessDBNet(float *outData, Mat rgbImg, vector<vector<Point2f>> &boxes) {
    Mat outputMap(firstModelHeight_, firstModelWidth_, CV_32FC1, const_cast<float_t *>((float_t *) outData));
    Mat bitmap = Mat(outputMap > 0.3);
    vector<float> scores;
    bboxFromBitmap(outputMap, bitmap, rgbImg.rows, rgbImg.cols, boxes, scores, 1000, 0.7f);
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

void TextRecongnize::fourPointsTransform(const Mat &frame, Point2f *vertices, Mat &result, vector<Mat> &hMatrix) {
    const Size outputSize = Size(secondModelWidth_, secondModelHeight_);
    Point2f targetVertices[4] = {cv::Point(0, 0), cv::Point(outputSize.width - 1, 0),
                                 cv::Point(outputSize.width - 1, outputSize.height - 1),
                                 cv::Point(0, outputSize.height - 1)};
    Mat rotationMatrix = getPerspectiveTransform(vertices, targetVertices);
    hMatrix.push_back(rotationMatrix);
    warpPerspective(frame, result, rotationMatrix, outputSize);
}

AclLiteError TextRecongnize::SendImage(cv::Mat &image) {
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

    ascend::presenter::PresenterErrorCode ret = PresentImage(presenterChannel_, imageParam);
    // send to presenter failed
    if (ret != PresenterErrorCode::kNone) {
        ACLLITE_LOG_ERROR("Send JPEG image to presenter failed, error %d\n", (int) ret);
        return ACLLITE_ERROR;
    }
    return ACLLITE_OK;
}

AclLiteError TextRecongnize::CopyImageToDvpp(ImageData &srcImage, 
                            aclrtRunMode runMode) {
    if (runMode == ACL_HOST) {
        void * buffer = nullptr;
        aclError ret = acldvppMalloc(&buffer, srcImage.size);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("Malloc dvpp memory failed, error No:%d", ret);
            return ACLLITE_ERROR;
        }
        ret = aclrtMemcpy(buffer, srcImage.size, srcImage.data.get(), 
                        srcImage.size, ACL_MEMCPY_HOST_TO_DEVICE);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("Copy data to device failed, aclRet is %d", ret);
            return ACLLITE_ERROR;
        }
        srcImage.data = SHARED_PTR_DVPP_BUF(buffer);
    }
    return ACLLITE_OK;                            
}

AclLiteError TextRecongnize::CopyImageFromDvpp(ImageData &srcImage, 
                            aclrtRunMode runMode) {
    if (runMode == ACL_HOST) {
        void * buffer = new uint8_t[srcImage.size];
        aclError ret = aclrtMemcpy(buffer, srcImage.size, srcImage.data.get(), 
                                    srcImage.size, ACL_MEMCPY_DEVICE_TO_HOST);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("Copy data from dvpp to host failed, aclRet is %d", ret);
            return ACLLITE_ERROR;
        }
        srcImage.data = SHARED_PTR_U8_BUF(buffer);
    }
    return ACLLITE_OK;                            
}

AclLiteError TextRecongnize::CopyMatToDevice(cv::Mat &srcMat, 
                            aclrtRunMode runMode) {
    if (runMode == ACL_HOST) {
        void * buffer = nullptr;
        size_t bufferSize = srcMat.rows * srcMat.cols * (int) srcMat.elemSize();
        aclError ret = aclrtMalloc(&buffer, bufferSize, ACL_MEM_MALLOC_HUGE_FIRST);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("Malloc device memory failed, error No:%d", ret);
            return ACLLITE_ERROR;
        }
        ret = aclrtMemcpy(buffer, bufferSize, srcMat.data, bufferSize, ACL_MEMCPY_HOST_TO_DEVICE);
        if (ret != ACL_SUCCESS) {
            ACLLITE_LOG_ERROR("Copy data to device failed, aclRet is %d", ret);
            return ACLLITE_ERROR;
        }
        srcMat.data = (uchar*)buffer;
    }
    return ACLLITE_OK;                            
}

void TextRecongnize::EncodeImage(vector<uint8_t> &encodeImg, cv::Mat &origImg) {
    vector<int> param = vector<int>(2);
    param[0] = cv::IMWRITE_JPEG_QUALITY;
    param[1] = 100;//default(95) 0-100
    cv::imencode(".jpg", origImg, encodeImg, param);
}

void TextRecongnize::DestroyResource() {
    if (!isReleased_) {
        dvpp_.DestroyResource();
        SecondModel_.DestroyResource();
        FirstModel_.DestroyResource();
        isReleased_ = true;
    }
}