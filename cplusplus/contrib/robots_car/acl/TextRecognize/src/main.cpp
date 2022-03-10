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

* File main.cpp
* Description: dvpp sample main func
*/

#include <bits/types/clock_t.h>
#include <ctime>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdlib.h>
#include <dirent.h>
#include <string.h>
#include <unistd.h>

#include "text_recognize.h"
#include "utils.h"

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

using namespace std;

namespace {
    uint32_t FirstModelWidth = 1312;
    uint32_t FirstModelHeight = 736;
    uint32_t SecondModelWidth = 100;
    uint32_t SecondModelHeight = 32;
    const char *FirstModelPath = "../model/dbnet.om";
    const char *SecondModelPath = "../model/crnn_static.om";
}

bool imgLoaded = false;
cv::Mat camImg, resizedImg;
cv::Mat detectResImg = cv::Mat(cv::Size(FirstModelWidth, FirstModelHeight), CV_8UC3, cv::Scalar(255, 255, 255));
vector<cv::Mat> cropAreas;
vector<vector<Point2f>> boxes;
string TextRes;
vector<string> textResults;
vector<cv::Mat> HMatrix;

vector<string> aimTextVec = {"voltage", "high", "caution"};

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        ROS_INFO("New Image Received, encoding: %s", msg->encoding.c_str());
        camImg = cv_bridge::toCvShare(msg, "bgr8")->image;
        ROS_INFO("convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        imgLoaded = true;
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "Ascend");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, imageCallback);
    // image_transport::Publisher tex_detect_pub = it.advertise("/TextDetect/detectResImg", 1);
    ros::Publisher tex_recognize_pub = nh.advertise<std_msgs::String>("/TextRecognize/recognizeResStr", 1);
    ros::Publisher area_pub = nh.advertise<std_msgs::String>("/TextDetect/detectRes", 1);

    ros::Rate loop_rate(10);

    //实例模型,模型路径,模型输入要求的宽和高 加载两个模型文件
    TextRecongnize recognize(FirstModelPath, SecondModelPath, FirstModelWidth, FirstModelHeight, SecondModelWidth,
                             SecondModelHeight);

    //init acl inference resource, model and memory.
    clock_t init_time = clock();
    Result ret = recognize.Init();
    cout << "recognize.Init() time " << double(clock() - init_time) / CLOCKS_PER_SEC << endl;
    if (ret != SUCCESS) {
        ERROR_LOG("ACL init resource failed");
        return FAILED;
    }

    ros::spinOnce();

    while (ros::ok() && imgLoaded) {

        clock_t detection_start_time = clock();
        //1\预处理图片:读取图片,将图片缩放到模型输入要求的尺寸
        cv::Mat firstModelInputMat;
        Result ret = recognize.FirstModelPreprocess(resizedImg, camImg, firstModelInputMat);
        if (ret != SUCCESS) {
            ERROR_LOG("FirstModelPreprocess failed, continue to process next");
            continue;
        }
//        cout << "recognize.FirstModelPreprocess() time " << double(clock() - detection_start_time) / CLOCKS_PER_SEC
//             << endl;

        //2\将预处理的图片送入detection模型推理,并获取detection推理结果
        aclmdlDataset *firstModelInferenceOutput = nullptr;
        // 推理时间
        ret = recognize.FirstModelInference(firstModelInferenceOutput, firstModelInputMat);
        if ((ret != SUCCESS) || (firstModelInferenceOutput == nullptr)) {
            ERROR_LOG("Inference first model failed");
            return FAILED;
        }
//        cout << "recognize.FirstModelInference() time " << double(clock() - detection_start_time) / CLOCKS_PER_SEC
//             << endl;

        // 3\解析detection推理输出
        ret = recognize.FirstModelPostprocess(firstModelInferenceOutput, camImg, detectResImg, cropAreas, boxes, HMatrix);
        if (ret != SUCCESS) {
            ERROR_LOG("Process first model inference output data failed");
            return FAILED;
        }

        ROS_INFO("Text detection :  %f ms", double (clock() -detection_start_time) * 1000 / CLOCKS_PER_SEC);
        ROS_INFO("Detected areas :  %ld ", cropAreas.size());


        for (int index = 0; index < cropAreas.size(); index++) {

            clock_t recognize_start_time = clock();
            cv::Mat secondModelInputMat;
            ret = recognize.SecondModelPreprocess(cropAreas[index], secondModelInputMat);
            if (ret != SUCCESS) {
                ERROR_LOG("SecondModelPreprocess failed, continue to process next");
                return FAILED;
            }

            aclmdlDataset *secondModelInferenceOutput = nullptr;
            //4\recognization模型推理,并获取recognization推理结果
            ret = recognize.SecondModelInference(secondModelInferenceOutput, secondModelInputMat);
            if ((ret != SUCCESS) || (secondModelInferenceOutput == nullptr)) {
                ERROR_LOG("Inference second model failed");
                return FAILED;
            }

            // 5\解析recognization推理输出
            ret = recognize.SecondModelPostprocess(secondModelInferenceOutput, TextRes, detectResImg, boxes[index]);
            if (ret != SUCCESS) {
                ERROR_LOG("Process second model inference output data failed");
                return FAILED;
            }
            ROS_INFO("Text recognization :  %f ms", double (clock() -recognize_start_time) * 1000 / CLOCKS_PER_SEC);

            std_msgs::String recognizeResStrMsg;

            std::stringstream ss;
            ss << "TextRes :  " << TextRes;
            recognizeResStrMsg.data = ss.str();
            tex_recognize_pub.publish(recognizeResStrMsg);
            ROS_INFO("TextRes :  %s ", recognizeResStrMsg.data.c_str());
            textResults.push_back(TextRes);
        }


        //////////////////////////////publish//////////////////////////////
        // sensor_msgs::ImagePtr detectResImgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
        //                                                            detectResImg).toImageMsg();
        // detectResImgMsg->header.stamp = ros::Time::now();
        // detectResImgMsg->header.frame_id = "camera_frame";
        // tex_detect_pub.publish(detectResImgMsg);


        Json::Value root, area, box, pt, H;
        Json::FastWriter JsonWriter;
        std::stringstream h;

        for (int index = 0; index < textResults.size(); index++) {
            vector<string>::iterator iter = find(aimTextVec.begin(), aimTextVec.end(), textResults[index]);
            if (iter != aimTextVec.end()) {
                for (int i = 0; i < boxes[index].size(); i++) {
                    pt["x"] = boxes[index][i].x;
                    pt["y"] = boxes[index][i].y;
                    box["pt" + to_string(i)] = pt;
                }
                area["Box"] = box;

                // H["h0"] = HMatrix[index].at<double>(0);H["h1"] = HMatrix[index].at<double>(1);H["h2"] = HMatrix[index].at<double>(2);
                // H["h3"] = HMatrix[index].at<double>(3);H["h4"] = HMatrix[index].at<double>(4);H["h5"] = HMatrix[index].at<double>(5);
                // H["h6"] = HMatrix[index].at<double>(6);H["h7"] = HMatrix[index].at<double>(7);H["h8"] = HMatrix[index].at<double>(8);
                // area["H"] = H;

                root[textResults[index]] = area;
            }
        }

        std_msgs::String areaMsg;
        std::stringstream ss;
        ss << root;
        areaMsg.data = ss.str();
        if (areaMsg.data != "null"){
            area_pub.publish(areaMsg);
            // cout << areaMsg.data << endl;
        }
        else{
            areaMsg.data = " ";
            area_pub.publish(areaMsg);
        }

        cropAreas.clear();
        boxes.clear();
        textResults.clear();
        HMatrix.clear();

        INFO_LOG("------------------------------------------------------------------");
        loop_rate.sleep();
        ros::spinOnce();
    }
    return SUCCESS;
}
