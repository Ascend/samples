/**
* Copyright (c) Huawei Technologies Co., Ltd. 2020-2022. All rights reserved.
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
#include "acl/acl.h"
#include "Params.h"
#include "detectPostprocess.h"
#include "AclLiteUtils.h"
#include "AclLiteApp.h"
#include "label.h"

using namespace std;

namespace {
    const uint32_t kSleepTime = 500;
    const double kFountScale = 0.5;
    const cv::Scalar kFountColor(0, 0, 255);
    const uint32_t kLabelOffset = 11;
    const uint32_t kLineSolid = 2;
    const vector <cv::Scalar> kColors{
        cv::Scalar(237, 149, 100), cv::Scalar(0, 215, 255),
        cv::Scalar(50, 205, 50), cv::Scalar(139, 85, 26)};
    typedef struct BoundBox {
        float x;
        float y;
        float width;
        float height;
        float score;
        size_t classIndex;
        size_t index;
    } BoundBox;

    bool sortScore(BoundBox box1, BoundBox box2)
    {
        return box1.score > box2.score;
    }
}

DetectPostprocessThread::DetectPostprocessThread(uint32_t modelWidth, uint32_t modelHeight,
    aclrtRunMode& runMode, uint32_t batch)
    :modelWidth_(modelWidth), modelHeight_(modelHeight), runMode_(runMode),
    sendLastBatch_(false), batch_(batch)
{
}

DetectPostprocessThread::~DetectPostprocessThread() {
}

AclLiteError DetectPostprocessThread::Init()
{
    return ACLLITE_OK;
}

AclLiteError DetectPostprocessThread::Process(int msgId, shared_ptr<void> data)
{
    AclLiteError ret = ACLLITE_OK;
    switch (msgId) {
        case MSG_POSTPROC_DETECTDATA:
            InferOutputProcess(static_pointer_cast<DetectDataMsg>(data));
            MsgSend(static_pointer_cast<DetectDataMsg>(data));
            break;
        default:
            ACLLITE_LOG_INFO("Detect PostprocessThread thread ignore msg %d", msgId);
            break;
    }

    return ret;
}

AclLiteError DetectPostprocessThread::InferOutputProcess(shared_ptr<DetectDataMsg> detectDataMsg)
{
    size_t pos = 0;
    for (int n = 0; n < detectDataMsg->decodedImg.size(); n++) {
        void* dataBuffer = CopyDataToHost(detectDataMsg->inferenceOutput[0].data.get() + pos,
            detectDataMsg->inferenceOutput[0].size / batch_, runMode_, MEMORY_NORMAL);
        if (dataBuffer == nullptr) {
            ACLLITE_LOG_ERROR("Copy inference output to host failed");
            return ACLLITE_ERROR_COPY_DATA;
        }
        pos = pos + detectDataMsg->inferenceOutput[0].size / batch_;
        float* detectBuff = static_cast<float*>(dataBuffer);

        // confidence threshold
        float confidenceThreshold = 0.25;

        // class number
        size_t classNum = 80;

        // number of (x, y, width, hight, confidence)
        size_t offset = 5;

        // total number = class number + (x, y, width, hight, confidence)
        size_t totalNumber = classNum + offset;

        // total number of boxs
        size_t modelOutputBoxNum = 25200;

        // top 5 indexes correspond (x, y, width, hight, confidence),
        // and 5~85 indexes correspond object's confidence
        size_t startIndex = 5;

        // get srcImage width height
        int srcWidth = detectDataMsg->decodedImg[n].width;
        int srcHeight = detectDataMsg->decodedImg[n].height;

        // filter boxes by confidence threshold
        vector <BoundBox> boxes;
        size_t yIndex = 1;
        size_t widthIndex = 2;
        size_t heightIndex = 3;
        size_t classConfidenceIndex = 4;
        for (size_t i = 0; i < modelOutputBoxNum; ++i) {
            float maxValue = 0;
            float maxIndex = 0;
            for (size_t j = startIndex; j < totalNumber; ++j) {
                float value = detectBuff[i * totalNumber + j] * detectBuff[i * totalNumber + classConfidenceIndex];
                    if (value > maxValue) {
                    // index of class
                    maxIndex = j - startIndex;
                    maxValue = value;
                }
            }
            float classConfidence = detectBuff[i * totalNumber + classConfidenceIndex];
            if (classConfidence >= confidenceThreshold) {
                // index of object's confidence
                size_t index = i * totalNumber + maxIndex + startIndex;

                // finalConfidence = class confidence * object's confidence
                float finalConfidence =  classConfidence * detectBuff[index];
                BoundBox box;
                box.x = detectBuff[i * totalNumber] * srcWidth / modelWidth_;
                box.y = detectBuff[i * totalNumber + yIndex] * srcHeight / modelHeight_;
                box.width = detectBuff[i * totalNumber + widthIndex] * srcWidth/modelWidth_;
                box.height = detectBuff[i * totalNumber + heightIndex] * srcHeight / modelHeight_;
                box.score = finalConfidence;
                box.classIndex = maxIndex;
                box.index = i;
                if (maxIndex < classNum) {
                    boxes.push_back(box);
                }
            }
            }

        // filter boxes by NMS
        vector <BoundBox> result;
        result.clear();
        float NMSThreshold = 0.45;
        int32_t maxLength = modelWidth_ > modelHeight_ ? modelWidth_ : modelHeight_;
        std::sort(boxes.begin(), boxes.end(), sortScore);
        BoundBox boxMax;
        BoundBox boxCompare;
        while (boxes.size() != 0) {
            size_t index = 1;
            result.push_back(boxes[0]);
            while (boxes.size() > index) {
                boxMax.score = boxes[0].score;
                boxMax.classIndex = boxes[0].classIndex;
                boxMax.index = boxes[0].index;

                // translate point by maxLength * boxes[0].classIndex to
                // avoid bumping into two boxes of different classes
                boxMax.x = boxes[0].x + maxLength * boxes[0].classIndex;
                boxMax.y = boxes[0].y + maxLength * boxes[0].classIndex;
                boxMax.width = boxes[0].width;
                boxMax.height = boxes[0].height;

                boxCompare.score = boxes[index].score;
                boxCompare.classIndex = boxes[index].classIndex;
                boxCompare.index = boxes[index].index;

                // translate point by maxLength * boxes[0].classIndex to
                // avoid bumping into two boxes of different classes
                boxCompare.x = boxes[index].x + boxes[index].classIndex * maxLength;
                boxCompare.y = boxes[index].y + boxes[index].classIndex * maxLength;
                boxCompare.width = boxes[index].width;
                boxCompare.height = boxes[index].height;

                // the overlapping part of the two boxes
                float xLeft = max(boxMax.x, boxCompare.x);
                float yTop = max(boxMax.y, boxCompare.y);
                float xRight = min(boxMax.x + boxMax.width, boxCompare.x + boxCompare.width);
                float yBottom = min(boxMax.y + boxMax.height, boxCompare.y + boxCompare.height);
                float width = max(0.0f, xRight - xLeft);
                float hight = max(0.0f, yBottom - yTop);
                float area = width * hight;
                float iou =  area / (boxMax.width * boxMax.height + boxCompare.width * boxCompare.height - area);

                // filter boxes by NMS threshold
                if (iou > NMSThreshold) {
                    boxes.erase(boxes.begin() + index);
                    continue;
                }
                ++index;
            }
            boxes.erase(boxes.begin());
        }

        // opencv draw label params
        int half = 2;

        cv::Point leftTopPoint;  // left top
        cv::Point rightBottomPoint;  // right bottom
        string className;  // yolo detect output

        // calculate framenum
        int frameCnt = (detectDataMsg->msgNum) * batch_ + n + 1;

        stringstream sstream;
        sstream.str("");
        sstream << "Channel-" << detectDataMsg->channelId << "-Frame-" << to_string(frameCnt) << "-result: ";
        
        string textHead = "";
        sstream >> textHead;
        string textMid = "[";
        for (size_t i = 0; i < result.size(); ++i) {
            leftTopPoint.x = result[i].x - result[i].width / half;
            leftTopPoint.y = result[i].y - result[i].height / half;
            rightBottomPoint.x = result[i].x + result[i].width / half;
            rightBottomPoint.y = result[i].y + result[i].height / half;
            className = label[result[i].classIndex] + ":" + to_string(result[i].score);
            cv::rectangle(detectDataMsg->frame[n], leftTopPoint,
                rightBottomPoint, kColors[i % kColors.size()], kLineSolid);
            cv::putText(detectDataMsg->frame[n], className,
                cv::Point(leftTopPoint.x, leftTopPoint.y + kLabelOffset),
                cv::FONT_HERSHEY_COMPLEX, kFountScale, kFountColor);
            textMid = textMid + className + " ";            
        }
        string textPrint = textHead + textMid + "]";
        detectDataMsg->textPrint.push_back(textPrint);
        free(detectBuff);
        detectBuff = nullptr;
    }
    return ACLLITE_OK;
}
    
AclLiteError DetectPostprocessThread::MsgSend(shared_ptr<DetectDataMsg> detectDataMsg)
{
    if (!sendLastBatch_) {
        while (1) {
            AclLiteError ret = SendMessage(detectDataMsg->dataOutputThreadId, MSG_OUTPUT_FRAME, detectDataMsg);
            if (ret == ACLLITE_ERROR_ENQUEUE) {
                usleep(kSleepTime);
                continue;
            } else if(ret == ACLLITE_OK) {
                break;
            } else {
                ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
                return ret;
            }
        }
    }
    if (detectDataMsg->isLastFrame && sendLastBatch_) {
        while (1) {
            AclLiteError ret = SendMessage(detectDataMsg->dataOutputThreadId, MSG_ENCODE_FINISH, detectDataMsg);
            if (ret == ACLLITE_ERROR_ENQUEUE) {
                usleep(kSleepTime);
                continue;
            } else if(ret == ACLLITE_OK) {
                break;
            } else {
                ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
                return ret;
            }
        }
    }
    if (detectDataMsg->isLastFrame && !sendLastBatch_) {
        while (1) {
            AclLiteError ret = SendMessage(detectDataMsg->dataOutputThreadId, MSG_ENCODE_FINISH, detectDataMsg);
            if (ret == ACLLITE_ERROR_ENQUEUE) {
                usleep(kSleepTime);
                continue;
            } else if(ret == ACLLITE_OK) {
                break;
            } else {
                ACLLITE_LOG_ERROR("Send read frame message failed, error %d", ret);
                return ret;
            }
        }
        sendLastBatch_ = true;
    }

    return ACLLITE_OK;
}