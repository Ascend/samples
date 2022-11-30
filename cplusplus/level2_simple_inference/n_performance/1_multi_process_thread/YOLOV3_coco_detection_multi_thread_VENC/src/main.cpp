/*
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2020. All rights reserved.
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
*/

#include <thread>
#include "acllite/AclLiteUtils.h"
#include "acllite/AclLiteResource.h"
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

#define VIDEONUM 2
using namespace std;

namespace {
    uint32_t g_modelWidth = 416;
    uint32_t g_modelHeight = 416;
    const char* g_modelPath = "../model/yolov3.om";
    const static std::vector<std::string> g_yolov3Label = { "person", "bicycle", "car", "motorbike",
        "aeroplane", "bus", "train", "truck", "boat",
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
        "refrigerator", "book", "clock", "vase", "scissors",
        "teddy bear", "hair drier", "toothbrush" };
    const uint32_t g_bBoxDataBufId = 0;
    const uint32_t g_boxNumDataBufId = 1;
    enum BBoxIndex { TOPLEFTX = 0, TOPLEFTY, BOTTOMRIGHTX, BOTTOMRIGHTY, SCORE, LABEL };
    const uint32_t g_lineSolid = 2;
    const int timep1 = 1000;
    const int timep2 = 5000;
    const string g_outputFilePrefix = "out_";
    const double FOUNT_SCALE = 0.5;
    const cv::Scalar g_fontColor(0, 0, 255);
    const uint32_t g_labelOffset = 11;
    const string g_fileSperator = "/";
    const vector<cv::Scalar> g_colors {
        cv::Scalar(237, 149, 100), cv::Scalar(0, 215, 255), cv::Scalar(50, 205, 50),
        cv::Scalar(139, 85, 26) };
}
int g_videoNeedPostprocess = VIDEONUM;
void Preprocess(cv::VideoCapture capture, aclrtContext context, 
                Queue<message>* queue_pre, int threadNum)
{
    message premsg;
    aclrtSetCurrentContext(context);

    while (1) {
        cv::Mat frame;
        if (!capture.read(frame)) {
            ACLLITE_LOG_INFO("Video capture return false");
            break;
        }
        premsg.frame = frame;
        premsg.videoIndex = threadNum;
        premsg.isLastFrame = 1;
        cv::Mat reiszeMat;
        cv::resize(frame, reiszeMat, cv::Size(g_modelWidth, g_modelHeight));
        if (reiszeMat.empty()) {
            ACLLITE_LOG_ERROR("Resize image failed");
            break;
        }
        premsg.reiszeMat = reiszeMat;
        while (1) {
            if (queue_pre->Push(premsg) != 0) {
                usleep(timep1);
            } else
                break;
        }
    }

    premsg.videoIndex = threadNum;
    premsg.isLastFrame = 0;
    while (1) {
        if (queue_pre->Push(premsg) != 0) {
            usleep(timep1);
        }
        else {
            ACLLITE_LOG_INFO("preprocess end");
            break;
        }
    }
}

void Postprocess(cv::VideoWriter& outputVideo, aclrtContext context, Queue<message>* queue_post) {
    message postmsg;
    uint32_t* boxNum = nullptr;
    float* detectData = nullptr;
    aclrtSetCurrentContext(context);

    while (1)
    {
        if (queue_post->Pop(postmsg) != 0) {
            usleep(timep1);
            continue;
        }

        if (!postmsg.isLastFrame) {
            break;
        }

        cv::Mat frame = postmsg.frame;
        detectData = (float*)postmsg.detectData.get();
        boxNum = (uint32_t*)postmsg.boxNum.get();
        if (detectData == nullptr || boxNum == nullptr) {
            break;
        }
        uint32_t totalBox = boxNum[0];
        vector<BBox> detectResults;
        float widthScale = (float)(frame.cols) / g_modelWidth;
        float heightScale = (float)(frame.rows) / g_modelHeight;
        for (uint32_t i = 0; i < totalBox; i++) {
            BBox boundBox;
            uint32_t score = uint32_t(detectData[totalBox * SCORE + i] * 100);
            uint32_t scoreLine = 90;
            if (score < scoreLine) {
                continue;
            }
            boundBox.rect.ltX = detectData[totalBox * TOPLEFTX + i] * widthScale;
            boundBox.rect.ltY = detectData[totalBox * TOPLEFTY + i] * heightScale;
            boundBox.rect.rbX = detectData[totalBox * BOTTOMRIGHTX + i] * widthScale;
            boundBox.rect.rbY = detectData[totalBox * BOTTOMRIGHTY + i] * heightScale;

            uint32_t objIndex = (uint32_t)detectData[totalBox * LABEL + i];
            boundBox.text = g_yolov3Label[objIndex] + std::to_string(score) + "\%";
            detectResults.emplace_back(boundBox);
        }
        for (int i = 0; i < detectResults.size(); ++i) {
            cv::Point p1, p2;
            p1.x = detectResults[i].rect.ltX;
            p1.y = detectResults[i].rect.ltY;
            p2.x = detectResults[i].rect.rbX;
            p2.y = detectResults[i].rect.rbY;
            cv::rectangle(frame, p1, p2, g_colors[i % g_colors.size()], g_lineSolid);
            cv::putText(frame, detectResults[i].text, cv::Point(p1.x, p1.y + g_labelOffset),
            cv::FONT_HERSHEY_COMPLEX, FOUNT_SCALE, g_fontColor);
        }

        outputVideo << frame;

    }
    g_videoNeedPostprocess--;
    outputVideo.release();
    ACLLITE_LOG_INFO("postprocess end");
}

int main(int argc, char *argv[]) {
    // init acl resource
    AclLiteResource aclDev;
    AclLiteError ret = aclDev.Init();
    if (ret) {
        ACLLITE_LOG_ERROR("Init resource failed, error %d", ret);
        return ACLLITE_ERROR;
    }
    ObjectDetect detect(g_modelPath, g_modelWidth, g_modelHeight);

    ret = detect.Init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("ObjectDetect Init resource failed");
        return 1;
    }

    aclrtContext context;
    aclrtGetCurrentContext(&context);

    cv::VideoCapture capture1("../data/video1.mp4");
    cv::VideoCapture capture2("../data/video2.mp4");
    if (!capture1.isOpened() || !capture2.isOpened())
    {
        ACLLITE_LOG_ERROR("Movie open Error");
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

    Queue<message>queue_pre;
    Queue<message>queue_post[VIDEONUM];

    thread task1(Preprocess, ref(capture1), ref(context), &queue_pre, 0);
    thread task2(Preprocess, ref(capture2), ref(context), &queue_pre, 1);
    thread task3(Postprocess, ref(outputVideo1), ref(context), &queue_post[0]);
    thread task4(Postprocess, ref(outputVideo2), ref(context), &queue_post[1]);

    task1.detach();
    task2.detach();
    task3.detach();
    task4.detach();
    message msg;
    int video_need_preprocess = VIDEONUM;
    while (1) {
        if (queue_pre.Pop(msg) != 0)
        {
            usleep(timep1);
            continue;
        }
        if (msg.isLastFrame == 0)
        {
            video_need_preprocess = video_need_preprocess - 1;
        }
        if (video_need_preprocess == 0)
        {
            while (1) {
                int i = msg.videoIndex;
                if (queue_post[i].Push(msg) != 0) {
                    usleep(timep1);
                } else {
                    break;
                }
            }
            break;
        }
        std::vector<InferenceOutput> inferenceOutput;
        ret = detect.Inference(inferenceOutput, msg.reiszeMat);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Inference model inference output data failed");
            return 1;
        }
        msg.detectData = inferenceOutput[g_bBoxDataBufId].data;
        msg.boxNum = inferenceOutput[g_boxNumDataBufId].data;

        int i = msg.videoIndex;
        while (1) {
            if (queue_post[i].Push(msg) != 0) {
                usleep(timep1);
            } else {
                break;
            }
        }
    }
    while (1) {
        if (!g_videoNeedPostprocess) {
            break;
        }
    }
    usleep(timep2);
    ACLLITE_LOG_INFO("Execute sample success");
    return ACLLITE_OK;
}
