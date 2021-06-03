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
#include <time.h>
#include <thread>
#include "atlasutil/atlas_utils.h"
#include "atlasutil/acl_device.h"
#include "queue.h"
#include "acl/acl.h"
#include <iostream>
#include <stdlib.h>
#include <dirent.h>
#include <unistd.h>
#include <pthread.h>
#include "object_detect.h"
#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include <bits/stdint-uintn.h>
#include "opencv2/imgcodecs/legacy/constants_c.h"
using namespace std;

struct timespec time1 = {0, 0};
struct timespec time2 = {0, 0};
struct timespec time3 = {0, 0};
struct timespec time4 = {0, 0};

namespace {
    uint32_t kModelWidth = 416;
    uint32_t kModelHeight = 416;
    const char* kModelPath = "../model/yolov3.om";
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
    const uint32_t kBBoxDataBufId = 0;
    const uint32_t kBoxNumDataBufId = 1;
    enum BBoxIndex { TOPLEFTX = 0, TOPLEFTY, BOTTOMRIGHTX, BOTTOMRIGHTY, SCORE, LABEL };
    const uint32_t kLineSolid = 2;
    const string kOutputFilePrefix = "out_";
    const double kFountScale = 0.5;
    const cv::Scalar kFontColor(0, 0, 255);
    const uint32_t kLabelOffset = 11;
    const string kFileSperator = "/";
    const vector<cv::Scalar> kColors{
        cv::Scalar(237, 149, 100), cv::Scalar(0, 215, 255), cv::Scalar(50, 205, 50),
        cv::Scalar(139, 85, 26) };
}
static int gPostEnd = 0;
void Preprocess(cv::VideoCapture capture, aclrtContext context, 
                BlockingQueue<message_pre>* queue_pre) {
    message_pre premsg;
    aclrtSetCurrentContext(context);

    while(1){
        clock_gettime(CLOCK_REALTIME, &time1);
        cv::Mat frame;
        if (!capture.read(frame)){
            ATLAS_LOG_ERROR("Video capture return false");
            break;
        }
        premsg.frame = frame;
        cv::Mat reiszeMat;
        cv::resize(frame, reiszeMat, cv::Size(kModelWidth, kModelHeight));
        if (reiszeMat.empty()) {
            ATLAS_LOG_ERROR("Resize image failed");
            break;
        }
        premsg.number = 1;
        premsg.reiszeMat = reiszeMat;
        while(1) {
            if (queue_pre->Push(premsg) != 0) {
                usleep(1000);
            }
            else
                break;
        }
        usleep(1000);
        clock_gettime(CLOCK_REALTIME, &time2);
        cout << "pre time passed is: " << (time2.tv_sec - time1.tv_sec)*1000
            + (time2.tv_nsec - time1.tv_nsec)/1000000 << "ms" << endl;
    }

    premsg.number = -1;
    while(1) {
        if (queue_pre->Push(premsg) != 0) {
            usleep(1000);
        }
        else {
            ATLAS_LOG_INFO("preprocess end");
            break;
        }
    }
}

void Postprocess(cv::VideoWriter& outputVideo, aclrtContext context, BlockingQueue<message>* queue_post){

    message postmsg;
    uint32_t* boxNum = nullptr;
    float* detectData = nullptr;
    aclrtSetCurrentContext(context);

    while(1) {
        clock_gettime(CLOCK_REALTIME, &time1);
        if(queue_post->Pop(postmsg) != 0) {
            usleep(1000);
            continue;
        }

        if (postmsg.number == -1){
            break;
        }
        cv::Mat frame = postmsg.frame;
        detectData = (float*)postmsg.detectData.get();
        boxNum = (uint32_t*)postmsg.boxNum.get();
        if (detectData == nullptr || boxNum == nullptr){
            break;
        }
        uint32_t totalBox = boxNum[0];
        vector<BBox> detectResults;
        float widthScale = (float)(frame.cols) / kModelWidth;
        float heightScale = (float)(frame.rows) / kModelHeight;

        for (uint32_t i = 0; i < totalBox; i++) {
            BBox boundBox;
            uint32_t score = uint32_t(detectData[totalBox * SCORE + i] * 100);
            if (score < 90){
                continue;
            }
            boundBox.rect.ltX = detectData[totalBox * TOPLEFTX + i] * widthScale;
            boundBox.rect.ltY = detectData[totalBox * TOPLEFTY + i] * heightScale;
            boundBox.rect.rbX = detectData[totalBox * BOTTOMRIGHTX + i] * widthScale;
            boundBox.rect.rbY = detectData[totalBox * BOTTOMRIGHTY + i] * heightScale;

            uint32_t objIndex = (uint32_t)detectData[totalBox * LABEL + i];
            boundBox.text = yolov3Label[objIndex] + std::to_string(score) + "\%";
            detectResults.emplace_back(boundBox);
        }

        cout << "this frame's coordinates is :" << endl;
        for (int i = 0; i < detectResults.size(); ++i) {
            cout << detectResults[i].rect.ltX << "     " << detectResults[i].rect.ltY << endl;
            cout << detectResults[i].rect.rbX << "     " << detectResults[i].rect.rbY << endl;
            cout << detectResults[i].text << endl;
        }
    }
    gPostEnd++;
    outputVideo.release();
    ATLAS_LOG_INFO("postprocess end");
}

int main(int argc, char *argv[]) {
    //init acl resource
    AclDevice aclDev;
    AtlasError ret = aclDev.Init();
    if (ret) {
        ATLAS_LOG_ERROR("Init resource failed, error %d", ret);
        return ATLAS_ERROR;
    }

    ObjectDetect detect(kModelPath, kModelWidth, kModelHeight);

    ret = detect.Init();
    if (ret != ATLAS_OK) {
        ATLAS_LOG_ERROR("ObjectDetect Init resource failed");
        return 1;
    }

    aclrtContext context;
    aclrtGetCurrentContext(&context);

    string videoFile1 = "../data/video1.mp4";
    string videoFile2 = "../data/video2.mp4";
    string videoFile3 = "../data/video3.mp4";
    string videoFile4 = "../data/video4.mp4";
    cv::VideoCapture capture1(videoFile1);
    cv::VideoCapture capture2(videoFile2);
    cv::VideoCapture capture3(videoFile3);
    cv::VideoCapture capture4(videoFile4);

    if (!capture1.isOpened() || !capture2.isOpened() || !capture3.isOpened() || !capture4.isOpened()) {
        ATLAS_LOG_ERROR("Movie open Error");
        return 1;
    }
    long totalFrameNumber1 = capture1.get(cv::CAP_PROP_FRAME_COUNT);
    cout << "totalFrameNumber1 is :" << totalFrameNumber1 << endl;
    int rate1 = capture1.get(cv::CAP_PROP_FPS);
    int rate2 = capture2.get(cv::CAP_PROP_FPS);
    int rate3 = capture3.get(cv::CAP_PROP_FPS);
    int rate4 = capture4.get(cv::CAP_PROP_FPS);
    cout << "CAP_PROP_FPS is :" << rate1 << endl;
    int height1 = static_cast<int>(capture1.get(cv::CAP_PROP_FRAME_HEIGHT));
    int height2 = static_cast<int>(capture2.get(cv::CAP_PROP_FRAME_HEIGHT));
    int height3 = static_cast<int>(capture3.get(cv::CAP_PROP_FRAME_HEIGHT));
    int height4 = static_cast<int>(capture4.get(cv::CAP_PROP_FRAME_HEIGHT));
    int width1 = static_cast<int>(capture1.get(cv::CAP_PROP_FRAME_WIDTH));
    int width2 = static_cast<int>(capture2.get(cv::CAP_PROP_FRAME_WIDTH));
    int width3 = static_cast<int>(capture3.get(cv::CAP_PROP_FRAME_WIDTH));
    int width4 = static_cast<int>(capture4.get(cv::CAP_PROP_FRAME_WIDTH));

    cv::VideoWriter outputVideo1;
    cv::VideoWriter outputVideo2;
    cv::VideoWriter outputVideo3;
    cv::VideoWriter outputVideo4;
    string outputVideoPath1 = "./output/test1.mp4";
    string outputVideoPath2 = "./output/test2.mp4";
    string outputVideoPath3 = "./output/test3.mp4";
    string outputVideoPath4 = "./output/test4.mp4";

    outputVideo1.open(outputVideoPath1, cv::VideoWriter::fourcc('M', 'P', '4', '2'), rate1, cv::Size(width1, height1));
    outputVideo2.open(outputVideoPath2, cv::VideoWriter::fourcc('M', 'P', '4', '2'), rate2, cv::Size(width2, height2));
    outputVideo3.open(outputVideoPath3, cv::VideoWriter::fourcc('M', 'P', '4', '2'), rate3, cv::Size(width3, height3));
    outputVideo4.open(outputVideoPath4, cv::VideoWriter::fourcc('M', 'P', '4', '2'), rate4, cv::Size(width4, height4));

    BlockingQueue<message>queue_post1;
    BlockingQueue<message>queue_post2;
    BlockingQueue<message>queue_post3;
    BlockingQueue<message>queue_post4;
    BlockingQueue<message_pre>queue_pre1;
    BlockingQueue<message_pre>queue_pre2;
    BlockingQueue<message_pre>queue_pre3;
    BlockingQueue<message_pre>queue_pre4;

    thread task1(Preprocess, ref(capture1), ref(context), &queue_pre1);
    thread task2(Preprocess, ref(capture2), ref(context), &queue_pre2);
    thread task3(Preprocess, ref(capture3), ref(context), &queue_pre3);
    thread task4(Preprocess, ref(capture4), ref(context), &queue_pre4);
    
    thread task5(Postprocess, ref(outputVideo1), ref(context), &queue_post1);
    thread task6(Postprocess, ref(outputVideo2), ref(context), &queue_post2);
    thread task7(Postprocess, ref(outputVideo3), ref(context), &queue_post3);
    thread task8(Postprocess, ref(outputVideo4), ref(context), &queue_post4);

    task1.detach();
    task2.detach();
    task3.detach();
    task4.detach();
    task5.detach();
    task6.detach();
    task7.detach();
    task8.detach();

    message_pre premsg;
    message msg;
    int flag = 0;
    
    clock_gettime(CLOCK_REALTIME, &time3);
    while(1) {
        if(queue_pre1.Pop(premsg) != 0) {
            usleep(1000);
            continue;
        }

        msg.number = premsg.number;
        if(premsg.number == -1) {
            break;
        }

        msg.frame = premsg.frame;
        std::vector<InferenceOutput> inferenceOutput;
        ret = detect.Inference(inferenceOutput, premsg.reiszeMat);
        if (ret != ATLAS_OK) {
            ATLAS_LOG_ERROR("Inference model inference output data failed");
            return 1;
        }
        msg.detectData = inferenceOutput[kBBoxDataBufId].data;
        msg.boxNum = inferenceOutput[kBoxNumDataBufId].data;

        while (1) {
            if (queue_post1.Push(msg) != 0) {
                usleep(1000);
            }
            else {
                break;
            }
        }

        if(queue_pre2.Pop(premsg) != 0) {
            usleep(1000);
            continue;
        }

        msg.number = premsg.number;
        if(premsg.number == -1){
            break;
        }

        msg.frame = premsg.frame;

        ret = detect.Inference(inferenceOutput, premsg.reiszeMat);
        if (ret != ATLAS_OK) {
            ATLAS_LOG_ERROR("Inference model inference output data failed");
            return 1;
        }
        msg.detectData = inferenceOutput[kBBoxDataBufId].data;
        msg.boxNum = inferenceOutput[kBoxNumDataBufId].data;

        while (1) {
            if (queue_post2.Push(msg) != 0) {
                usleep(1000);
            }
            else {
                break;
            }
        }
        
        if(queue_pre3.Pop(premsg) != 0)
        {
            usleep(1000);
            continue;
        }

        msg.number = premsg.number;
        if(premsg.number == -1) {
            break;
        }

        msg.frame = premsg.frame;
        ret = detect.Inference(inferenceOutput, premsg.reiszeMat);
        if (ret != ATLAS_OK) {
            ATLAS_LOG_ERROR("Inference model inference output data failed");
            return 1;
        }
        msg.detectData = inferenceOutput[kBBoxDataBufId].data;
        msg.boxNum = inferenceOutput[kBoxNumDataBufId].data;

        while (1) {
            if (queue_post3.Push(msg) != 0) {
                usleep(1000);
            }
            else {
                break;
            }
        }
        
        if(queue_pre4.Pop(premsg) != 0) {
            usleep(1000);
            continue;
        }

        msg.number = premsg.number;
        if(premsg.number == -1) {
            break;
        }

        msg.frame = premsg.frame;
        ret = detect.Inference(inferenceOutput, premsg.reiszeMat);
        if (ret != ATLAS_OK) {
            ATLAS_LOG_ERROR("Inference model inference output data failed");
            return 1;
        }
        msg.detectData = inferenceOutput[kBBoxDataBufId].data;
        msg.boxNum = inferenceOutput[kBoxNumDataBufId].data;

        while (1) {
            if (queue_post4.Push(msg) != 0) {
                usleep(1000);
            }
            else {
                break;
            }
        }
    }
    clock_gettime(CLOCK_REALTIME, &time4);
    cout << "Execute time passed is: " << (time4.tv_sec - time3.tv_sec)*1000 
            + (time4.tv_nsec - time3.tv_nsec)/1000000 << "ms" << endl;
    while (1) {
        if (queue_post1.Push(msg) != 0) {
            usleep(1000);
        }
        else {
            break;
        }
    }
    while (1) {
        if (queue_post2.Push(msg) != 0) {
            usleep(1000);
        }
        else {
            break;
        }
    }
    while (1) {
        if (queue_post3.Push(msg) != 0) {
            usleep(1000);
        }
        else {
            break;
        }
    }
    while (1) {
        if (queue_post4.Push(msg) != 0) {
            usleep(1000);
        }
        else {
            break;
        }
    }

    while(1){
        if (gPostEnd == 4)
        {
            break;
        }

    }

    usleep(1000);
    ATLAS_LOG_INFO("Execute sample success");
    return ATLAS_OK;
}

