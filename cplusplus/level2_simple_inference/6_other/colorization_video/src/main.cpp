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
#include <opencv2/opencv.hpp>
#include "colorize_process.h"
#include "acllite/AclLiteUtils.h"
#include "acllite/AclLiteError.h"

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
        ACLLITE_LOG_ERROR("Please input: ./main <image_dir>");
        return 1;
    }
    //Instantiation
    ColorizeProcess colorize(kModelPath, kModelWidth, kModelHeight);
    //acl init
    AclLiteError ret = colorize.Init();
    if (ret != ACLLITE_OK) {
        ACLLITE_LOG_ERROR("Classification Init resource failed");
        return 1;
    }

    //input video path
    string videoFile = string(argv[1]);
    cv::VideoCapture capture(videoFile);
    if (!capture.isOpened())
    {
        ACLLITE_LOG_ERROR("Movie open Error");
        return 1;
    }

    int totalFrames = capture.get(7);
    int currentFrames = 0;

    ACLLITE_LOG_INFO("The sample starts to run");

    while(1) {
        //preprocess
        cv::Mat frame;
        if (!capture.read(frame))
        {
            ACLLITE_LOG_INFO("Video capture return false");
            break;
        }

        if (currentFrames == totalFrames - 1)
        {
            currentFrames = 0;
            capture.set(1, 0);
        }

        currentFrames++;

        ret = colorize.Preprocess(frame);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Read file %s failed, continue to read next",
            videoFile.c_str());
            continue;
        }
        //inference
        std::vector<InferenceOutput> inferenceOutput;
        ret = colorize.Inference(inferenceOutput);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Inference model inference output data failed");
            return 1;
        }
        //postprocess
        ret = colorize.Postprocess(frame, inferenceOutput);
        if (ret != ACLLITE_OK) {
            ACLLITE_LOG_ERROR("Process model inference output data failed");
            return 1;
        }
    }

    ACLLITE_LOG_INFO("Execute sample success");
    return ACLLITE_OK;
}
