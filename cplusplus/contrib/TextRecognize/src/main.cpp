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
#include "camera.h"
#include "text_recognize.h"
#include "utils.h"

using namespace std;

namespace {
    uint32_t FirstModelWidth = 1312;
    uint32_t FirstModelHeight = 736;
    uint32_t SecondModelWidth = 100;
    uint32_t SecondModelHeight = 32;
    const char *FirstModelPath = "../model/dbnet.om";
    //const char *SecondModelPath = "../model/crnn_static_rnn_tune.om";
    const char *SecondModelPath = "../model/crnn_static.om";
}
shared_ptr<ImageData> g_imagedata = make_shared<ImageData>();
//cv::Mat camImg, resizedImg;
cv::Mat fp32Image, camera_rgb, camera_depth;
cv::Mat detectResImg = cv::Mat(cv::Size(FirstModelWidth, FirstModelHeight), CV_8UC3, cv::Scalar(255, 255, 255));
vector<cv::Mat> cropAreas;
vector<vector<Point2f >> boxes;
string TextRes;
vector<string> textResults;
vector<cv::Mat> HMatrix;

vector<string> aimTextVec = {
        "shipments", "inspection", "packing", "tools", "stopping"
};

int main(int argc, char *argv[]) {


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

    //cv::Mat fp32Image, camera_rgb, camera_depth;
    aclmdlDataset *inferenceOutput = nullptr;
    int cameraChannelId = 0;

    Camera cameraDevice(cameraChannelId, 15);
    if (!cameraDevice.IsOpened(cameraChannelId)) {
        if (cameraDevice.Open(cameraChannelId)) {
            ERROR_LOG("Failed to open cameraChannelId =%d.\n", cameraChannelId);
            return FAILED;
        }
    }

    void *buffer = nullptr;
    int size = cameraDevice.GetCameraDataSize(cameraChannelId);

    aclError aclRet = acldvppMalloc(&buffer, size);
    g_imagedata->data.reset((uint8_t *) buffer, [](uint8_t *p) {
        acldvppFree((void *) p);
    });


    while(1){

        clock_t detection_start_time = clock();

        //Reasoning picture by picture
        cameraDevice.Read(cameraChannelId, *(g_imagedata.get()));
        if (g_imagedata->data == nullptr) {
            ERROR_LOG("Read image %d failed\n", cameraChannelId);
            return FAILED;
        }
//        cout << "get image" << endl;
        //Preprocess the picture: read the picture and zoom the picture to the size required by the model input
        cv::Mat firstModelInputMat;
        Result ret = recognize.FirstModelPreprocess(camera_rgb, *(g_imagedata.get()), firstModelInputMat);
        if (ret != SUCCESS) {
            ERROR_LOG("Preprocess image %d failed, continue to read next\n", cameraChannelId);
            return FAILED;
        }

        if (ret != SUCCESS) {
            ERROR_LOG("FirstModelPreprocess failed, continue to process next");
            continue;
        }
        cout << "recognize.FirstModelPreprocess() time " << double(clock() - detection_start_time) / CLOCKS_PER_SEC
             << endl;

        //2\将预处理的图片送入detection模型推理,并获取detection推理结果
        aclmdlDataset *firstModelInferenceOutput = nullptr;
        // 推理时间
        ret = recognize.FirstModelInference(firstModelInferenceOutput, firstModelInputMat);
        cout << "first model inference sucess" << endl;
        if ((ret != SUCCESS) || (firstModelInferenceOutput == nullptr)) {
            ERROR_LOG("Inference first model failed");
            return FAILED;
        }

        cout << "recognize.FirstModelInference() time " << double(clock() - detection_start_time) / CLOCKS_PER_SEC
             << endl;

        // 3\解析detection推理输出
        ret = recognize.FirstModelPostprocess(firstModelInferenceOutput, camera_rgb, detectResImg, cropAreas, boxes, HMatrix);
        if (ret != SUCCESS) {
            ERROR_LOG("Process first model inference output data failed");
            return FAILED;
        }

        for (int index = 0; index < cropAreas.size(); index++) {

            clock_t recognize_start_time = clock();
            cv::Mat secondModelInputMat;
            ret = recognize.SecondModelPreprocess(cropAreas[index], secondModelInputMat);
            if (ret != SUCCESS) {
                ERROR_LOG("SecondModelPreprocess failed, continue to process next");
                return FAILED;
            }
            cout << "SecondModelPreprocess sucess" << endl;

            aclmdlDataset *secondModelInferenceOutput = nullptr;
            //4\recognization模型推理,并获取recognization推理结果
            ret = recognize.SecondModelInference(secondModelInferenceOutput, secondModelInputMat);
            cout << "SecondModel inference sucess" << endl;
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

            std::stringstream ss;
            ss << "TextRes :  " << TextRes.c_str();

        }

        //////////////////////////////send image//////////////////////////////

        ret = recognize.SendImage(detectResImg);
        if (ret != SUCCESS) {
            ERROR_LOG("SendImage failed");
            return FAILED;
        }

        cropAreas.clear();
        boxes.clear();
        textResults.clear();
        HMatrix.clear();

        INFO_LOG("------------------------------------------------------------------");

    }
    return SUCCESS;
}

