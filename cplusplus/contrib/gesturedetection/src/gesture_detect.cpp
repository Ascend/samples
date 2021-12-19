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
#include "gesture_detect.h"
#include <cstddef>
#include <iostream>

#include "acl/acl.h"
#include "acllite/AclLiteModel.h"
#include "acllite/AclLiteUtils.h"
#include <cmath>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "opencv2/opencv.hpp"

#include "opencv2/imgproc/types_c.h"
#include "opencv2/core/hal/interface.h"
#include "opencv2/core/types.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace cv;

int gLastGes = -1;
//Eigen::MatrixXf gLeftMatrix(16, 16);
//Eigen::MatrixXf gRightMatrix(16, 16);
//Eigen::MatrixXf gTopMatrix(16, 16);
//Eigen::MatrixXf gBottomMatrix(16, 16);

int gLimbSeq[13][2] = {{2,3}, {2,6}, {3,4}, {4, 5}, {6, 7}, {7, 8}, {2, 9}, {9, 10}, {10, 11}, {2, 12},
{12, 13}, {13, 14}, {2, 1}};

int gMapIdx[19][2] = {{31,32}, {39,40}, {33,34}, {35,36}, {41,42}, {43,44}, {19,20}, {21,22}, {23,24},
{25,26}, {27,28}, {29,30}, {47,48}};
std::shared_ptr<EngineTransNewT> gMotionDataOld = std::make_shared<EngineTransNewT>();

GestureDetect::GestureDetect(const char* openPoseModelPath,
                           const char* gestureModelPath,
                           uint32_t modelWidth, 
                           uint32_t modelHeight)
:deviceId_(0), context_(nullptr), stream_(nullptr), modelWidth_(modelWidth),
 modelHeight_(modelHeight), isInited_(false){
    imageInfoSize_ = 0;
    imageInfoBuf_ = nullptr;
    modelPathOpenPose_ = openPoseModelPath;
    modelPathGesture_ = gestureModelPath;
}

GestureDetect::~GestureDetect() {
    DestroyResource();
}

AclLiteError GestureDetect::InitModel(const char* omModelPathOpenPose, const char* omModelPathGesture) {
    // Load model files
    AclLiteError ret = modelOpenPose_.Init(omModelPathOpenPose);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("execute LoadModelFromFileWithMem failed, model openpose");
        return ACLLITE_ERROR;
    }

    ret = modelGesture_.Init(omModelPathGesture);

    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("execute LoadModelFromFileWithMem failed, model gesture");
        return ACLLITE_ERROR;
    }
    return ACLLITE_OK;
}

AclLiteError GestureDetect::Init() {
    if (isInited_) {
        ACLLITE_LOG_INFO("Classify instance is initied already!");
        return ACLLITE_OK;
    }

    AclLiteError ret = aclrtGetRunMode(&runMode_);
    if (ret != ACL_SUCCESS) {
        ACLLITE_LOG_ERROR("acl get run mode failed");
        return ACLLITE_ERROR;
    }

    ret = InitModel(modelPathOpenPose_, modelPathGesture_);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Init model failed");
        return ACLLITE_ERROR;
    }

    ret = dvpp_.Init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Init dvpp failed");
        return ACLLITE_ERROR;
    }

    isInited_ = true;
    return ACLLITE_OK;
}

AclLiteError GestureDetect::Preprocess(ImageData& resizedImage, ImageData& srcImage) {
 
    ImageData imageDevice;
    void* buffer = CopyDataToDevice(srcImage.data.get(), srcImage.size,
                                runMode_, MEMORY_DVPP);
    if (buffer == nullptr) {
        return ACLLITE_ERROR_COPY_DATA;
    }

    imageDevice.width = srcImage.width;
    imageDevice.height = srcImage.height;
    imageDevice.size = srcImage.size;
    imageDevice.data.reset((uint8_t*)buffer, [](uint8_t* p) { aclrtFree((void *)p); });
  
    ImageData yuvImage;
    AclLiteError ret = dvpp_.JpegD(yuvImage, imageDevice);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Convert jpeg to yuv failed");
        return ACLLITE_ERROR;
    }

    //resize
    ret = dvpp_.Resize(resizedImage, yuvImage, modelWidth_, modelHeight_);
    if (ret == ACLLITE_ERROR) {
        ACLLITE_LOG_ERROR("Resize image failed");
        return ACLLITE_ERROR;
    }
    
    return ACLLITE_OK;
}

// OpenPose inference function
AclLiteError GestureDetect::OpenPoseInference(std::vector<InferenceOutput>& inferOutputs, ImageData& resizedImage) {
    AclLiteError ret = modelOpenPose_.CreateInput(resizedImage.data.get(), resizedImage.size);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed");
        return ACLLITE_ERROR;
    }

    ret = modelOpenPose_.Execute(inferOutputs);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Execute model inference failed");
        return ACLLITE_ERROR;
    }

    modelOpenPose_.DestroyInput();

    return ACLLITE_OK;
}

// gesture inference function
AclLiteError GestureDetect::GestureInference(std::vector<InferenceOutput>& inferOutputs,
                                        std::shared_ptr<EngineTransNewT> motionDataNew){

    motionDataNew->bufferSize = 2 * FRAME_LENGTH * 14 * sizeof(float);

    imageInfoBuf_ = CopyDataToDevice((void*) motionDataNew->data,
                                        motionDataNew->bufferSize, runMode_, MEMORY_DEVICE);
    if (imageInfoBuf_ == nullptr) {
        ACLLITE_LOG_ERROR("Copy image info to device failed");
        return ACLLITE_ERROR;
    }

    AclLiteError ret = modelGesture_.CreateInput((void*) imageInfoBuf_, motionDataNew->bufferSize);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Create mode input dataset failed");
        return ACLLITE_ERROR;
    }

    ret = modelGesture_.Execute(inferOutputs);
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Execute model inference failed");
        return ACLLITE_ERROR;
    }

    modelGesture_.DestroyInput();

    return ACLLITE_OK;
}

int find_index(vector<int>::iterator begin, vector<int>::iterator end, int element){
    auto temp = begin;
    while(temp != end){
        if(*temp == element){
            return element;
        }
        temp += 1;
    }
    return -1;
}

// Custom collation rules
bool cmp2(connectionT a,connectionT b) {
    return a.score>b.score;
}

// Process the results of the output
AclLiteError GestureDetect::Postprocess(ImageData& image, vector<InferenceOutput>& modelOutput, 
                                        std::shared_ptr<EngineTransNewT> motionDataNew, int &successNum) {

    uint32_t dataSize = 0;
    float* newResult = (float *)modelOutput[1].data.get();

    cv::Mat tempMat;
    cv::Mat tempMat_0;
    cv::Mat tempMat_1;
    Eigen::Matrix <float, 128, 128> resizedMatrix;
    Eigen::Matrix <float, 128, 128> scoreMid_0;
    Eigen::Matrix <float, 128, 128> scoreMid_1;
    Eigen::MatrixXd::Index maxRow, maxCol;
    Eigen::MatrixXd::Index maxRowF, maxColF;
    Eigen::MatrixXd::Index maxRowNew, maxColNew;
    Eigen::MatrixXd::Index tempMaxRow, tempMaxCol;

    Eigen::MatrixXf gLeftMatrix(16, 16);
    Eigen::MatrixXf gRightMatrix(16, 16);
    Eigen::MatrixXf gTopMatrix(16, 16);
    Eigen::MatrixXf gBottomMatrix(16, 16);

    vector <keyPointsT> onePicKeyPoints;
    vector <vector<keyPointsT>> allKeyPoints;
    vector <float> onePicPeaks;
    float gTempKeyPoints[2][14];
    int allPeakIndex = 0;
    float tempA = 0.0;
    bool ifValid = true;
    // Generates an all-white graph that you can use to draw the resulting graph
    Mat out1(cv::Size(128,128), CV_8UC3, cv::Scalar(255, 255, 255));
    // Find 14 key points (the first 14 points are enough for the action recognition sequence)
    for (int picNum = 0; picNum < 14; picNum++){
        float *v = newResult+picNum*256;
        // Map to Matrix by column
        Eigen::Map<Eigen::MatrixXf> matrQQ(v, 16, 16);

        Eigen::Matrix <float, 16, 16> m = matrQQ;

        tempA = m.maxCoeff(&maxRowF, &maxColF);
        // If the maximum value is not greater than 0.1, this frame is considered invalid
        if (tempA < 0.1){
            ifValid = false;
            break;
        }

        // Gets the matrix for the left shift of the matrix
        gLeftMatrix.leftCols(15) = m.rightCols(15);
        gLeftMatrix.col(15) = Eigen::MatrixXf::Zero(16, 1);
        // Moves to the right
        gRightMatrix.rightCols(15) = m.leftCols(15);
        gRightMatrix.col(0) = Eigen::MatrixXf::Zero(16, 1);
        // Move up
        gTopMatrix.topRows(15) = m.bottomRows(15);
        gTopMatrix.row(15) = Eigen::MatrixXf::Zero(1, 16);
        // Move down
        gBottomMatrix.bottomRows(15) = m.topRows(15);
        gBottomMatrix.row(0) = Eigen::MatrixXf::Zero(1, 16);

        gLeftMatrix = m - gLeftMatrix;
        gRightMatrix = m - gRightMatrix;
        gTopMatrix = m - gTopMatrix;
        gBottomMatrix = m - gBottomMatrix;

        for (int aa = 0; aa < 16; aa++){
            for(int bb = 0; bb < 16; bb++){
                if(gLeftMatrix(aa, bb) > 0 && gRightMatrix(aa, bb) > 0 
                    && gBottomMatrix(aa, bb) > 0 && gTopMatrix(aa, bb) > 0 && m(aa, bb) > 0.1){
                    onePicPeaks.push_back(aa);
                    onePicPeaks.push_back(bb);
                }
            }
        }
        // Extend 16x16 to 128x128 size
        cv::eigen2cv(m, tempMat);
        cv::resize(tempMat, tempMat, cv::Size(128, 128), cv::INTER_CUBIC);
        cv::GaussianBlur(tempMat, tempMat, cv::Size(3,3), 5);
        cv::cv2eigen(tempMat, resizedMatrix);

        // The local maximum value of each 128x128 image was found according to the local maximum value found in the 16x16 image
        for (int aa = 0; aa < onePicPeaks.size(); aa += 2){
            tempMaxRow = onePicPeaks[aa] * 8 - 6;
            tempMaxCol = onePicPeaks[aa+1] * 8 - 6;
            if(tempMaxRow < 0){
                tempMaxRow = 0;
            }
            if(tempMaxCol < 0){
                tempMaxCol = 0;
            }
            if(tempMaxRow > 121){
                tempMaxRow = 121;
            }
            if(tempMaxCol > 121){
                tempMaxCol = 121;
            }

            // Take a 12x12 submatrix and find the maximum value
            Eigen::MatrixXf smallMatrix = resizedMatrix.block<12, 12>(tempMaxRow, tempMaxCol);
            tempA = smallMatrix.maxCoeff(&maxRowNew, &maxColNew);
            // The maximum in a submatrix is considered a local maximum (a key point)
            keyPointsT temp = {float(tempMaxRow + maxRowNew), float(tempMaxCol + maxColNew), allPeakIndex};
            allPeakIndex++;
            onePicKeyPoints.push_back(temp);
        }

        // If you don't find a point in one part of the body, the body is missing a key point, and you don't need to go on looking
        if(onePicKeyPoints.size() == 0){
            return ACLLITE_ERROR;
        }
        // The calculated keypoints for each graph are stored in a vector, which then stores the total keypoints
        allKeyPoints.push_back(onePicKeyPoints);
        onePicPeaks.clear();
        onePicKeyPoints.clear();
    }

    if(!ifValid){
        cout << "invalid image!!" << endl;
        return ACLLITE_ERROR;
    }

    // =======================================================================
    // ===============Look for relationships between key points===============
    // =======================================================================
    // Gets the first output data
    vector <connectionT> connectionCandidate;
    vector <vector<connectionT>> connectionAll;
    float* newResult_0 = (float *)modelOutput[0].data.get();

    // Traverse mapIdx
    for (int kk = 0; kk < 13; kk++){
        float *v = newResult_0 + (gMapIdx[kk][0] - 19)*256;
        // Map to Matrix by column
        Eigen::Map<Eigen::MatrixXf> matrQQ_0(v, 16, 16);

        Eigen::Map<Eigen::MatrixXf> matrQQ_1(v + 256, 16, 16);

        Eigen::Matrix <float, 16, 16> m_0 = matrQQ_0; // score_mid
        Eigen::Matrix <float, 16, 16> m_1 = matrQQ_1; // score_mid

        cv::eigen2cv(m_0, tempMat_0);
        cv::eigen2cv(m_1, tempMat_1);
        cv::resize(tempMat_0, tempMat_0, cv::Size(128, 128), cv::INTER_CUBIC);
        cv::resize(tempMat_1, tempMat_1, cv::Size(128, 128), cv::INTER_CUBIC);
        cv::GaussianBlur(tempMat_0, tempMat_0, cv::Size(3,3), 3);
        cv::GaussianBlur(tempMat_1, tempMat_1, cv::Size(3,3), 3);
        cv::cv2eigen(tempMat_0, scoreMid_0); // score_mid

        cv::cv2eigen(tempMat_1, scoreMid_1); // score_mid

        vector <keyPointsT> temp_A = allKeyPoints[gLimbSeq[kk][0] -1];

        vector <keyPointsT> temp_B = allKeyPoints[gLimbSeq[kk][1] -1];

        int LA = temp_A.size();
        int LB = temp_B.size();

        if(LA != 0 && LB != 0){
            // Find the possibility of a connection between each point in La and the key points in LB
            for (int aa = 0; aa < LA; aa++){
                for(int bb = 0; bb < LB; bb++){
                    float vec[2] = {temp_B[bb].point_x - temp_A[aa].point_x, temp_B[bb].point_y - temp_A[aa].point_y};
                    float norms = sqrt(vec[0]*vec[0] + vec[1]*vec[1]);

                    vec[0] /= norms;
                    vec[1] /= norms;
                    Eigen::Matrix <float ,10 ,2> startEnd;
                    startEnd.col(0) = Eigen::ArrayXf::LinSpaced(10, temp_A[aa].point_x, temp_B[bb].point_x);
                    startEnd.col(1) = Eigen::ArrayXf::LinSpaced(10, temp_A[aa].point_y, temp_B[bb].point_y);

                    Eigen::Matrix <float, 10, 1> vec_x;
                    Eigen::Matrix <float, 10, 1> vec_y;

                    // TODO transformed
                    vec_x << scoreMid_0(int(round(startEnd(0 ,0))), int(round(startEnd(0 ,1))))
                    , scoreMid_0(int(round(startEnd(1 ,0))), int(round(startEnd(1 ,1))))
                    , scoreMid_0(int(round(startEnd(2 ,0))), int(round(startEnd(2 ,1))))
                    , scoreMid_0(int(round(startEnd(3 ,0))), int(round(startEnd(3 ,1))))
                    , scoreMid_0(int(round(startEnd(4 ,0))), int(round(startEnd(4 ,1))))
                    , scoreMid_0(int(round(startEnd(5 ,0))), int(round(startEnd(5 ,1))))
                    , scoreMid_0(int(round(startEnd(6 ,0))), int(round(startEnd(6 ,1))))
                    , scoreMid_0(int(round(startEnd(7 ,0))), int(round(startEnd(7 ,1))))
                    , scoreMid_0(int(round(startEnd(8 ,0))), int(round(startEnd(8 ,1))))
                    , scoreMid_0(int(round(startEnd(9 ,0))), int(round(startEnd(9 ,1))));

                    vec_y << scoreMid_1(int(round(startEnd(0 ,0))), int(round(startEnd(0 ,1))))
                    , scoreMid_1(int(round(startEnd(1 ,0))), int(round(startEnd(1 ,1))))
                    , scoreMid_1(int(round(startEnd(2 ,0))), int(round(startEnd(2 ,1))))
                    , scoreMid_1(int(round(startEnd(3 ,0))), int(round(startEnd(3 ,1))))
                    , scoreMid_1(int(round(startEnd(4 ,0))), int(round(startEnd(4 ,1))))
                    , scoreMid_1(int(round(startEnd(5 ,0))), int(round(startEnd(5 ,1))))
                    , scoreMid_1(int(round(startEnd(6 ,0))), int(round(startEnd(6 ,1))))
                    , scoreMid_1(int(round(startEnd(7 ,0))), int(round(startEnd(7 ,1))))
                    , scoreMid_1(int(round(startEnd(8 ,0))), int(round(startEnd(8 ,1))))
                    , scoreMid_1(int(round(startEnd(9 ,0))), int(round(startEnd(9 ,1))));

                    Eigen::Matrix <float, 10, 1>scoreMidpts = vec_x * vec[0] + vec_y * vec[1];

                    float scoreWithDistPrior = scoreMidpts.sum() / (10.001) + min(64 / (norms - 1 + 1e-3), 0.0);

                    if (scoreWithDistPrior > 0){
                        int badNum = 0;
                        for (int fff = 0; fff < 10; fff++){
                            if(scoreMidpts(fff) < 0.05){
                                badNum++;
                            }
                        }
                        if(badNum < 2){
                            connectionT tempConnection{aa, bb, scoreWithDistPrior};
                            connectionCandidate.push_back(tempConnection);
                        }
                    }
                }
            }

            // If there is a set of key points that cannot be connected to each other, it is considered invalid
            if(connectionCandidate.size() == 0){
                return ACLLITE_ERROR;
            }
            // In order of likelihood of connection, from most to least
            sort(connectionCandidate.begin(), connectionCandidate.end(), cmp2);
            vector<int> temp_1;
            vector<int> temp_2;
            temp_1.push_back(33);
            temp_2.push_back(33);

            int p_i = 0;
            int p_j = 0;
            // Get all non-repeating connections
            vector<connectionT> oneConnection;
            for (int tt = 0; tt < connectionCandidate.size(); tt++){
                int i = connectionCandidate[tt].point_1;
                int j = connectionCandidate[tt].point_2;
                float s = connectionCandidate[tt].score;

                p_i = find_index(temp_1.begin(), temp_1.end(), i);

                p_j = find_index(temp_2.begin(), temp_2.end(), j);
                if(p_i != i && p_j != j){
                    temp_1.push_back(i);
                    temp_2.push_back(j);
                    connectionT temp{temp_A[i].num, temp_B[j].num};
                    oneConnection.push_back(temp);
                    if (oneConnection.size() >= min(LA, LB)){
                        break;
                    }
                }
            }
            connectionCandidate.clear();
            connectionAll.push_back(oneConnection);
            oneConnection.clear();
        }
    }

    // =======================================================================
    // ==============Get the full key point of the middle person==============
    // =======================================================================
    int midIndex = -1;
    int minDis = 200;
    int tempIndex[14] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
    // Find the key point of the middle person
    int keySize_0 = allKeyPoints[0].size();
    for (int aa = 0; aa < connectionAll[0].size(); aa++){
        int thisPointX = allKeyPoints[1][connectionAll[0][aa].point_1 % keySize_0].point_x;
        if (abs(thisPointX - 64) < minDis){
            minDis = abs(thisPointX - 64);
            midIndex = aa;
        }
    }

    // the key points 1 and 2 of the middle person
    tempIndex[1] = connectionAll[0][midIndex].point_1;
    tempIndex[2] = connectionAll[0][midIndex].point_2;
    // 0 is the connection between 1 and 2, which we found up there, so we start with 1
    for (int aa = 1; aa < 13; aa++){
        int index_A = gLimbSeq[aa][0] - 1;
        int index_B = gLimbSeq[aa][1] - 1;
        if(tempIndex[index_A] == -1){
            return ACLLITE_ERROR;
        }

        for (int bb = 0; bb < connectionAll[aa].size(); bb++){
            if(connectionAll[aa][bb].point_1 == tempIndex[index_A]){
                tempIndex[index_B] = connectionAll[aa][bb].point_2;
            }
        }
        // If one connection point is not found, the image is deemed invalid and returned directly
        if(tempIndex[index_B] == -1){
            return ACLLITE_ERROR;
        }
    }

    // The 14 key points for the most middle person found are saved in temp_key_points
    for (int aa = 0; aa < 14; aa++){
        gTempKeyPoints[0][aa] = allKeyPoints[aa][tempIndex[aa]].point_x;
        gTempKeyPoints[1][aa] = allKeyPoints[aa][tempIndex[aa]].point_y;
        cv::Point p(gTempKeyPoints[0][aa], gTempKeyPoints[1][aa]);
        cv::circle(out1, p, 1, cv::Scalar(0, 0, 0), -1); 
        for(int bb = aa + 1; bb < 14; bb++){
            tempIndex[bb] -= allKeyPoints[aa].size();
        }
    }

    cv::Point x0(gTempKeyPoints[0][0], gTempKeyPoints[1][0]);
    cv::Point x1(gTempKeyPoints[0][1], gTempKeyPoints[1][1]);
    cv::Point x2(gTempKeyPoints[0][2], gTempKeyPoints[1][2]);
    cv::Point x3(gTempKeyPoints[0][3], gTempKeyPoints[1][3]);
    cv::Point x4(gTempKeyPoints[0][4], gTempKeyPoints[1][4]);
    cv::Point x5(gTempKeyPoints[0][5], gTempKeyPoints[1][5]);
    cv::Point x6(gTempKeyPoints[0][6], gTempKeyPoints[1][6]);
    cv::Point x7(gTempKeyPoints[0][7], gTempKeyPoints[1][7]);
    cv::Point x8(gTempKeyPoints[0][8], gTempKeyPoints[1][8]);
    cv::Point x9(gTempKeyPoints[0][9], gTempKeyPoints[1][9]);
    cv::Point x10(gTempKeyPoints[0][10], gTempKeyPoints[1][10]);
    cv::Point x11(gTempKeyPoints[0][11], gTempKeyPoints[1][11]);
    cv::Point x12(gTempKeyPoints[0][12], gTempKeyPoints[1][12]);
    cv::Point x13(gTempKeyPoints[0][13], gTempKeyPoints[1][13]);
    cv::line(out1, x0, x1, cv::Scalar(255, 0, 0), 1);
    cv::line(out1, x1, x2, cv::Scalar(255, 0, 0), 1);
    cv::line(out1, x2, x3, cv::Scalar(255, 0, 0), 1);
    cv::line(out1, x3, x4, cv::Scalar(255, 0, 0), 1);
    cv::line(out1, x1, x5, cv::Scalar(255, 0, 0), 1);
    cv::line(out1, x5, x6, cv::Scalar(255, 0, 0), 1);
    cv::line(out1, x6, x7, cv::Scalar(255, 0, 0), 1);
    cv::line(out1, x1, x8, cv::Scalar(255, 0, 0), 1);
    cv::line(out1, x8, x9, cv::Scalar(255, 0, 0), 1);
    cv::line(out1, x9, x10, cv::Scalar(255, 0, 0), 1);
    cv::line(out1, x1, x11, cv::Scalar(255, 0, 0), 1);
    cv::line(out1, x11, x12, cv::Scalar(255, 0, 0), 1);
    cv::line(out1, x12, x13, cv::Scalar(255, 0, 0), 1);

    cv::imwrite("../result.jpg", out1);

    memcpy(motionDataNew->data[0][0][FRAME_LENGTH-1], gTempKeyPoints[0], sizeof(float)*14);
    memcpy(motionDataNew->data[0][1][FRAME_LENGTH-1], gTempKeyPoints[1], sizeof(float)*14);
    // x
    memcpy(motionDataNew->data[0][0][0], gMotionDataOld->data[0][0][1], sizeof(float)*14*(FRAME_LENGTH - 1));
    // y
    memcpy(motionDataNew->data[0][1][0], gMotionDataOld->data[0][1][1], sizeof(float)*14*(FRAME_LENGTH - 1));
    memcpy(gMotionDataOld->data, motionDataNew->data, sizeof(float)*2*FRAME_LENGTH*14);

    //Find the coordinates of the center points and center them
    float skeletonCenter[2][FRAME_LENGTH] = {0.0};
    for (int c = 0; c < 2; c++)
    {
        for (int t = 0; t < FRAME_LENGTH; t++)
        {
            skeletonCenter[c][t] = float((motionDataNew->data[0][c][t][1] + motionDataNew->data[0][c][t][8] 
                                    + motionDataNew->data[0][c][t][11]) / float(3.0));
            for (int v = 0; v < 14; v++)
            {
                motionDataNew->data[0][c][t][v] = motionDataNew->data[0][c][t][v] - skeletonCenter[c][t];
            }
        }
    }
    successNum++;
    return ACLLITE_OK;
}

void GestureDetect::DestroyResource() {
    modelOpenPose_.DestroyResource();
    modelGesture_.DestroyResource();
    dvpp_.DestroyResource();
    aclrtFree(imageInfoBuf_);
}

AclLiteError GestureDetect::PostGestureProcess(vector<InferenceOutput>& modelOutput){
    uint32_t dataSize = 0;
    float* newResult = (float *)modelOutput[0].data.get();
    int maxPosition = max_element(newResult, newResult+5) - newResult;

    float down = 1.4;
    float resultTotal = pow(down, newResult[0]) + pow(down, newResult[1]) + pow(down, newResult[2])
                        + pow(down, newResult[3]) + pow(down, newResult[4]);
    newResult[0] = pow(down, newResult[0]) / resultTotal;
    newResult[1] = pow(down, newResult[1]) / resultTotal;
    newResult[2] = pow(down, newResult[2]) / resultTotal;
    newResult[3] = pow(down, newResult[3]) / resultTotal;
    newResult[4] = pow(down, newResult[4]) / resultTotal;

    bool ifNeedPub = false;

    if (newResult[maxPosition] >= 0.5){
        switch (maxPosition){
            case 0:
                if(newResult[maxPosition] > 0.9){
                    if (gLastGes != 0){
                        cout << " 鼓掌 " << newResult[0] << endl;
                        cout << " 挥手 " << newResult[1] << endl;
                        cout << " 站立 " << newResult[2] << endl;
                        cout << " 双手平举 " << newResult[3] << endl;
                        cout << " 踢腿 " << newResult[4] << endl;
                        cout << "=============================鼓掌" << endl;
                        ifNeedPub = false;
                        gLastGes = 10;
                    }
                }
                break;
            case 1:
                if(newResult[maxPosition] > 0.8){
                    if (gLastGes != 1){
                        cout << " 鼓掌 " << newResult[0] << endl;
                        cout << " 挥手 " << newResult[1] << endl;
                        cout << " 站立 " << newResult[2] << endl;
                        cout << " 双手平举 " << newResult[3] << endl;
                        cout << " 踢腿 " << newResult[4] << endl;
                        cout << "=============================挥手" << endl;
                        ifNeedPub = false;
                        gLastGes = 11;
                    }
                }
                break;
            case 2:
                if(newResult[maxPosition] > 0.5){
                    if (gLastGes != 2){
                        cout << " 鼓掌 " << newResult[0] << endl;
                        cout << " 挥手 " << newResult[1] << endl;
                        cout << " 站立 " << newResult[2] << endl;
                        cout << " 双手平举 " << newResult[3] << endl;
                        cout << " 踢腿 " << newResult[4] << endl;
                        cout << "=============================站立" << endl;
                        ifNeedPub = false;
                        gLastGes = 12;
                    }
                }
                break;
            case 3:
                if(newResult[maxPosition] > 0.95){
                    if (gLastGes != 3){
                        cout << " 鼓掌 " << newResult[0] << endl;
                        cout << " 挥手 " << newResult[1] << endl;
                        cout << " 站立 " << newResult[2] << endl;
                        cout << " 双手平举 " << newResult[3] << endl;
                        cout << " 踢腿 " << newResult[4] << endl;
                        cout << "=============================双手平举" << endl;
                        ifNeedPub = false;
                        gLastGes = 13;
                    }
                }
                break;
            case 4:
                if(newResult[maxPosition] > 0.9){
                    if (gLastGes != 4){
                        cout << " 鼓掌 " << newResult[0] << endl;
                        cout << " 挥手 " << newResult[1] << endl;
                        cout << " 站立 " << newResult[2] << endl;
                        cout << " 双手平举 " << newResult[3] << endl;
                        cout << " 踢腿 " << newResult[4] << endl;
                        cout << "==============================踢腿" << endl;
                        ifNeedPub = false;
                        gLastGes = 14;
                    }
                }
                break;
            default:
                cout << "max element==nothing  " << maxPosition << "     " << newResult[maxPosition] << endl;
                break;
        }
    }
    return ACLLITE_OK;
}

