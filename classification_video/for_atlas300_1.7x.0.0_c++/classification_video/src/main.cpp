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

#include "classify_process.h"
#include "utils.h"
using namespace std;

namespace {
const uint32_t kModelWidth = 224;
const uint32_t kModelHeight = 224;
const char* kModelPath = "../model/googlenet.om";
}

int main(int argc, char *argv[]) {
    //Check the input when the application is executed, the parameter of the program execution is the input video file path
    if((argc < 2) || (argv[1] == nullptr)){
        ERROR_LOG("Please input: ./main <image_dir>");
        return FAILED;
    }
    //Instantiate the classification reasoning object, the parameter is the classification model path, the width and height of the model input requirements
    ClassifyProcess classify(kModelPath, kModelWidth, kModelHeight);
    //Initialize the acl resource for classification reasoning, load the model and apply for the memory used for reasoning input
    Result ret = classify.Init();
    if (ret != SUCCESS) {
        ERROR_LOG("Classification Init resource failed");
        return FAILED;
    }

    //Use opencv to open the video file
    string videoFile = string(argv[1]);
    cv::VideoCapture capture(videoFile);
    if (!capture.isOpened()) {
        cout << "Movie open Error" << endl;
        return FAILED;
    }
    //Frame-by-frame reasoning
    while(1) {
        //Read a picture
        cv::Mat frame;
        if (!capture.read(frame))
        {
            INFO_LOG("Video capture return false");
            break;
        }
        //Preprocess the frame picture
        Result ret = classify.Preprocess(frame);
        if (ret != SUCCESS) {
            ERROR_LOG("Read file %s failed, continue to read next",
                      videoFile.c_str());
            continue;
        }
        //Send the preprocessed pictures to the model for inference and get the inference results
        aclmdlDataset* inferenceOutput = nullptr;
        ret = classify.Inference(inferenceOutput);
        if ((ret != SUCCESS) || (inferenceOutput == nullptr)) {
            ERROR_LOG("Inference model inference output data failed");
            return FAILED;
        }
        //Analyze the inference output, and send the object category, confidence and picture obtained by inference to the presenter server for display
        ret = classify.Postprocess(frame, inferenceOutput);
        if (ret != SUCCESS) {
            ERROR_LOG("Process model inference output data failed");
            return FAILED;
        }
    }

    INFO_LOG("Execute video classification success");
    return SUCCESS;
}
