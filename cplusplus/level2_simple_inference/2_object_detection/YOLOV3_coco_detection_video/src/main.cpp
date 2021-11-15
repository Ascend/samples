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
#include <sys/time.h>
#include "atlasutil/atlas_videocapture.h"
#include "atlasutil/atlas_error.h"
#include "atlasutil/atlas_utils.h"
#include "object_detect.h"
#include "utils.h"
using namespace std;

namespace {
uint32_t kModelWidth = 416;
uint32_t kModelHeight = 416;
const char* kModelPath = "../model/yolov3.om";
}

int main(int argc, char *argv[]) {
    //Check the input when the application executes, which takes the path to the input video file
    if((argc < 2) || (argv[1] == nullptr)){
        ERROR_LOG("Please input: ./main <image_dir>");
        return FAILED;
    }
    //Instantiate the target detection class with the parameters of the classification model path and the required width and height of the model input
    ObjectDetect detect(kModelPath, kModelWidth, kModelHeight);
    //Initializes the ACL resource for categorical reasoning, loads the model and requests the memory used for reasoning input
    Result ret = detect.Init();
    if (ret != SUCCESS) {
        ERROR_LOG("Classification Init resource failed");
        return FAILED;
    }
    string videoFile = string(argv[1]);
    AtlasVideoCapture cap = AtlasVideoCapture(videoFile);
    if(!cap.IsOpened()) {
        ATLAS_LOG_ERROR("Open camera failed");
        return ATLAS_ERROR;
    }

    while(1) {
        ImageData image,yuvImage;
        AtlasError ret = cap.Read(image);
        if (ret) {
            ATLAS_LOG_ERROR("Read image failed, error %d", ret);
            return ATLAS_ERROR;
        }
        ret = CopyImageToLocal(yuvImage, image, ACL_DEVICE);
        if (ret != ATLAS_OK){
            ATLAS_LOG_ERROR("Copy image to host failed, error %d", ret);
            return ATLAS_ERROR;
        }
        cv::Mat yuvMat(yuvImage.height * 3 / 2, yuvImage.width, CV_8UC1, yuvImage.data.get());
        cv::Mat frame;
        cv::cvtColor(yuvMat, frame, CV_YUV2BGR_NV12);

        //The frame image is preprocessed
        Result result = detect.Preprocess(frame);
        if (result != SUCCESS) {
            ERROR_LOG("Read file %s failed, continue to read next",
                      videoFile.c_str());
            continue;
        }
        //The preprocessed images are fed into model reasoning and the reasoning results are obtained
        aclmdlDataset* inferenceOutput = nullptr;
        result = detect.Inference(inferenceOutput);
        if ((result != SUCCESS) || (inferenceOutput == nullptr)) {
            ERROR_LOG("Inference model inference output data failed");
            return FAILED;
        }
        //Parses the inference output and sends the inference class, location, confidence, and image to the Presenter Server for display
        result = detect.Postprocess(frame, inferenceOutput);
        if (result != SUCCESS) {
            ERROR_LOG("Process model inference output data failed");
            return FAILED;
        }
    }

    INFO_LOG("Execute video object detection success");
    return SUCCESS;
}
