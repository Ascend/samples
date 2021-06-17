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
#define VIDEONUM 2
using namespace std;

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
int g_video_need_postprocess = VIDEONUM;
void Preprocess(cv::VideoCapture capture, aclrtContext context, 
                BlockingQueue<message_pre>* queue_pre, int threadNum) {
    message_pre premsg;
    aclrtSetCurrentContext(context);

    while(1){
        cv::Mat frame;
        if (!capture.read(frame)){
            ATLAS_LOG_ERROR("Video capture return false");
            break;
        }
        premsg.frame = frame;
        premsg.videoIndex = threadNum;
        premsg.isLastFrame = 1;
        cv::Mat reiszeMat;
        cv::resize(frame, reiszeMat, cv::Size(kModelWidth, kModelHeight));
        if (reiszeMat.empty()) {
            ATLAS_LOG_ERROR("Resize image failed");
            break;
        }
        premsg.reiszeMat = reiszeMat;
        while(1) {
            if (queue_pre->Push(premsg) != 0) {
                usleep(1000);
            }
            else
                break;
        }
    }

    premsg.videoIndex = threadNum;
    premsg.isLastFrame = 0;
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

    while(1)
    {
        if(queue_post->Pop(postmsg) != 0)
        {
            usleep(1000);
            continue;
        }

        if (!postmsg.isLastFrame){
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
        for (int i = 0; i < detectResults.size(); ++i) {
            cv::Point p1, p2;
            p1.x = detectResults[i].rect.ltX;
            p1.y = detectResults[i].rect.ltY;
            p2.x = detectResults[i].rect.rbX;
            p2.y = detectResults[i].rect.rbY;
            cv::rectangle(frame, p1, p2, kColors[i % kColors.size()], kLineSolid);
            cv::putText(frame, detectResults[i].text, cv::Point(p1.x, p1.y + kLabelOffset),
            cv::FONT_HERSHEY_COMPLEX, kFountScale, kFontColor);
        }

        outputVideo << frame;

    }
    g_video_need_postprocess--;
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

    cv::VideoCapture capture1("../data/video1.mp4");
    cv::VideoCapture capture2("../data/video2.mp4");
    if (!capture1.isOpened() || !capture2.isOpened())
    {
        ATLAS_LOG_ERROR("Movie open Error");
        return 1;
    }
    cv::VideoWriter outputVideo1;
    cv::VideoWriter outputVideo2;
    outputVideo1.open("./output/test1.mp4", cv::VideoWriter::fourcc('M', 'P', '4', 'v'), 
                        capture1.get(cv::CAP_PROP_FPS), 
                        cv::Size(static_cast<int>(capture1.get(cv::CAP_PROP_FRAME_WIDTH)), 
                        static_cast<int>(capture1.get(cv::CAP_PROP_FRAME_HEIGHT))));
    outputVideo2.open("./output/test2.mp4", cv::VideoWriter::fourcc('M', 'P', '4', 'v'), 
                        capture2.get(cv::CAP_PROP_FPS), 
                        cv::Size(static_cast<int>(capture2.get(cv::CAP_PROP_FRAME_WIDTH)), 
                        static_cast<int>(capture2.get(cv::CAP_PROP_FRAME_HEIGHT))));

    BlockingQueue<message_pre>queue_pre;
    BlockingQueue<message>queue_post[VIDEONUM];

    thread task1(Preprocess, ref(capture1), ref(context), &queue_pre, 0);
    thread task2(Preprocess, ref(capture2), ref(context), &queue_pre, 1);
    thread task3(Postprocess, ref(outputVideo1), ref(context), &queue_post[0]);
    thread task4(Postprocess, ref(outputVideo2), ref(context), &queue_post[1]);

    task1.detach();
    task2.detach();
    task3.detach();
    task4.detach();

    message_pre premsg;
    message msg;
    int video_need_preprocess = VIDEONUM;
    while(1) {
        if(queue_pre.Pop(premsg) != 0)
        {
            usleep(1000);
            continue;
        }
        msg.videoIndex = premsg.videoIndex;
        msg.isLastFrame = premsg.isLastFrame;
        if(msg.isLastFrame == 0)
        {
            video_need_preprocess = video_need_preprocess - 1;
        }
        if (video_need_preprocess == 0)
        {
            while (1) {
                int i = msg.videoIndex;
                if (queue_post[i].Push(msg) != 0) {
                    usleep(1000);
                }
                else {
                    break;
                }
            }
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

        int i = msg.videoIndex;
        while (1) {
            if (queue_post[i].Push(msg) != 0) {
                usleep(1000);
            }
            else {
                break;
            }
        }
    }
    while(1){
        if(!g_video_need_postprocess){
            break;
        }
    }
    usleep(5000);
    ATLAS_LOG_INFO("Execute sample success");
    return ATLAS_OK;
}
