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

#include <iostream>
#include <stdlib.h>
#include <dirent.h>

#include "colorize_process.h"
#include "utils.h"
#include<opencv2/opencv.hpp>

using namespace std;

using namespace cv;

namespace {
    const uint32_t kModelWidth = 224;
    const uint32_t kModelHeight = 224;
    const char* kModelPath = "../model/colorization.om";

}

int main(int argc, char *argv[]) {
    //Check inputs and parameters
    if((argc < 2) || (argv[1] == nullptr)){
        ERROR_LOG("Please input: ./main <image_dir>");
        return FAILED;
    }
    //Instantiation
    ColorizeProcess colorize(kModelPath, kModelWidth, kModelHeight);
    //acl init
    Result ret = colorize.Init();
    if (ret != SUCCESS) {
        ERROR_LOG("colorization Init resource failed");
        return FAILED;
    }

    //input video path
    string videoFile = string(argv[1]);
    cv::VideoCapture capture(videoFile);
    if (!capture.isOpened())
    {
        cout << "Movie open Error" << endl;
        return FAILED;
    }

    int totalFrames = capture.get(7);
    int currentFrames = 0;


    while(1) {
        //preprocess
        cv::Mat frame;
        if (!capture.read(frame))
        {
            INFO_LOG("Video capture return false");
            break;
        }

        if (currentFrames == totalFrames - 1)
        {
            currentFrames = 0;
            capture.set(1, 0);
        }

        currentFrames++;

        Result ret = colorize.Preprocess(frame);
        if (ret != SUCCESS) {
            ERROR_LOG("Read file %s failed, continue to read next",
            videoFile.c_str());
            continue;
        }
        //inference
        aclmdlDataset* inferenceOutput = nullptr;
        ret = colorize.Inference(inferenceOutput);
        if ((ret != SUCCESS) || (inferenceOutput == nullptr)) {
            ERROR_LOG("Inference model inference output data failed");
            return FAILED;
        }
        //postprocess
        ret = colorize.Postprocess(frame, inferenceOutput);
        if (ret != SUCCESS) {
            ERROR_LOG("Process model inference output data failed");
            return FAILED;
        }
    }

    INFO_LOG("Execute sample success");
    return SUCCESS;
}
